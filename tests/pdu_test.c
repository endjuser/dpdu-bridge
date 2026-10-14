/*
 * pdu_test.c - drives a D-PDU API DLL through a Tech2Win-like sequence.
 *
 *   pdu_test.exe [-c | -cs | path\to\dll] [connect-wait-ms]
 *
 * -c loads dpdu-bridge-c.dll, -cs dpdu-bridge-cs.dll, from the current
 * directory, else the project's build folder (..\build next to this exe).
 * Without either, dpdu-bridge-c.dll is used if it exists, else
 * dpdu-bridge-cs.dll.
 *
 * connect-wait-ms: pause after PDUModuleConnect, e.g. 1500 in TCP mode so
 * the simulator has time to connect to the DLL's TCP server.
 *
 * Start the simulator first (run_sim.bat, serial or TCP).  The test resolves
 * object ids, connects the module, reads VBATT, then sends one request each
 * on HS-CAN, SW-CAN (GMLAN) and VPW, prints the events that come back and
 * checks the responses against the simulator's answers.  Exit code 0 when
 * every check passed.  Build with "build.cmd test" (32-bit, like the DLLs).
 */
#include <windows.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pdu_api.h"

#define MAX_RESULT 4200     /* largest result the drivers deliver (4095 + CAN id, rounded up) */

/* Declares a function-pointer type and variable for each export (see LOAD). */
#define F(name, ret, args) typedef ret (PDU_API_CALL *pfn_##name) args; static pfn_##name name
F(PDUConstruct, T_PDU_ERROR, (CHAR8 *, void *));
F(PDUDestruct, T_PDU_ERROR, (void));
F(PDUGetModuleIds, T_PDU_ERROR, (PDU_MODULE_ITEM **));
F(PDUModuleConnect, T_PDU_ERROR, (UNUM32));
F(PDUModuleDisconnect, T_PDU_ERROR, (UNUM32));
F(PDURegisterEventCallback, T_PDU_ERROR, (UNUM32, UNUM32, CALLBACKFNC));
F(PDUGetObjectId, T_PDU_ERROR, (T_PDU_OBJT, CHAR8 *, UNUM32 *));
F(PDUCreateComLogicalLink, T_PDU_ERROR, (UNUM32, PDU_RSC_DATA *, UNUM32, void *, UNUM32 *, PDU_FLAG_DATA *));
F(PDUDestroyComLogicalLink, T_PDU_ERROR, (UNUM32, UNUM32));
F(PDUConnect, T_PDU_ERROR, (UNUM32, UNUM32));
F(PDUDisconnect, T_PDU_ERROR, (UNUM32, UNUM32));
F(PDUSetComParam, T_PDU_ERROR, (UNUM32, UNUM32, PDU_PARAM_ITEM *));
F(PDUSetUniqueRespIdTable, T_PDU_ERROR, (UNUM32, UNUM32, PDU_UNIQUE_RESP_ID_TABLE_ITEM *));
F(PDUStartComPrimitive, T_PDU_ERROR, (UNUM32, UNUM32, UNUM32, UNUM32, UNUM8 *, PDU_COP_CTRL_DATA *, void *, UNUM32 *));
F(PDUCancelComPrimitive, T_PDU_ERROR, (UNUM32, UNUM32, UNUM32));
F(PDUGetEventItem, T_PDU_ERROR, (UNUM32, UNUM32, PDU_EVENT_ITEM **));
F(PDUDestroyItem, T_PDU_ERROR, (PDU_ITEM *));
F(PDUIoCtl, T_PDU_ERROR, (UNUM32, UNUM32, UNUM32, PDU_DATA_ITEM *, PDU_DATA_ITEM **));
F(PDUGetVersion, T_PDU_ERROR, (UNUM32, PDU_VERSION_DATA *));
F(PDUGetStatus, T_PDU_ERROR, (UNUM32, UNUM32, UNUM32, T_PDU_STATUS *, UNUM32 *, UNUM32 *));

static HANDLE g_evt;
static volatile LONG g_callbacks;
static int g_failures;
static int g_heartbeats;    /* VPW node-alive messages received */

static void PDU_API_CALL on_event(T_PDU_EVT_DATA e, UNUM32 hMod, UNUM32 hCLL, void *cllTag, void *apiTag)
{
    (void)e; (void)hMod; (void)hCLL; (void)cllTag; (void)apiTag;
    InterlockedIncrement(&g_callbacks);
    SetEvent(g_evt);
}

#define CHECK(call) do { T_PDU_ERROR _e = (call); if (_e != PDU_STATUS_NOERROR) { \
    printf("FAIL %s -> 0x%X\n", #call, _e); g_failures++; } } while (0)

static UNUM32 obj(T_PDU_OBJT t, const char *name)
{
    UNUM32 id = 0;
    CHECK(PDUGetObjectId(t, (CHAR8 *)name, &id));
    return id;
}

/* Drains the event queue; returns the number of non-empty RESULT events other
 * than VPW node-alive messages.  The last one is copied to last (MAX_RESULT
 * bytes). */
static int drain(UNUM32 hCLL, UNUM8 *last, UNUM32 *lastLen, UNUM32 *lastUrid)
{
    PDU_EVENT_ITEM *ev;
    int results = 0;
    while (PDUGetEventItem(1, hCLL, &ev) == PDU_STATUS_NOERROR) {
        if (ev->ItemType == PDU_IT_STATUS) {
            printf("   event STATUS  hCoP=%08X status=%04X\n", ev->hCop, *(UNUM32 *)ev->pData);
        } else if (ev->ItemType == PDU_IT_RESULT &&
                   ((PDU_RESULT_DATA *)ev->pData)->NumDataBytes >= 4 &&
                   ((PDU_RESULT_DATA *)ev->pData)->pDataBytes[0] == 0x08 &&
                   ((PDU_RESULT_DATA *)ev->pData)->pDataBytes[1] == 0xFF) {
            /* unsolicited VPW node-alive message (08 FF <node> 03): not a response */
            g_heartbeats++;
        } else if (ev->ItemType == PDU_IT_RESULT) {
            PDU_RESULT_DATA *r = (PDU_RESULT_DATA *)ev->pData;
            UNUM32 i;
            printf("   event RESULT  hCoP=%08X urid=%08X acc=%u rxflag=%02X%02X%02X%02X data:",
                   ev->hCop, r->UniqueRespIdentifier, r->AcceptanceId, r->RxFlag.pFlagData[0],
                   r->RxFlag.pFlagData[1], r->RxFlag.pFlagData[2], r->RxFlag.pFlagData[3]);
            for (i = 0; i < r->NumDataBytes; i++)
                printf(" %02X", r->pDataBytes[i]);
            printf("\n");
            if (r->NumDataBytes) {
                *lastLen = r->NumDataBytes > MAX_RESULT ? MAX_RESULT : r->NumDataBytes;
                memcpy(last, r->pDataBytes, *lastLen);
                *lastUrid = r->UniqueRespIdentifier;
                results++;
            }
        }
        PDUDestroyItem((PDU_ITEM *)ev);
    }
    return results;
}

static int wait_result(UNUM32 hCLL, UNUM8 *buf, UNUM32 *len, UNUM32 *urid)
{
    DWORD t0 = GetTickCount();
    while (GetTickCount() - t0 < 2000) {
        if (drain(hCLL, buf, len, urid))
            return 1;
        WaitForSingleObject(g_evt, 100);
    }
    return 0;
}

static void param(UNUM32 hCLL, const char *name, UNUM32 value)
{
    PDU_PARAM_ITEM p;
    p.ItemType = PDU_IT_PARAM;
    p.ComParamId = obj(PDU_OBJT_COMPARAM, name);
    p.ComParamDataType = PDU_PT_UNUM32;
    p.ComParamClass = PDU_PC_COM;
    p.pComParamData = &value;
    CHECK(PDUSetComParam(1, hCLL, &p));
}

static void expect(const char *what, const UNUM8 *got, UNUM32 gotLen, const UNUM8 *want, UNUM32 wantLen)
{
    if (gotLen == wantLen && !memcmp(got, want, wantLen)) {
        printf("PASS %s\n", what);
    } else {
        printf("FAIL %s (unexpected response)\n", what);
        g_failures++;
    }
}

/* Opens a CAN CLL the way Tech2Win does (unique response id table, receive
 * ComPrimitive, then a transmit ComPrimitive), sends one request and checks
 * the response: respId followed by want. */
static void run_can(const char *bus, UNUM32 pin1, UNUM32 pin2, UNUM32 nPins, UNUM32 reqId, UNUM32 respId,
                    const UNUM8 *req, UNUM32 reqLen, const UNUM8 *want, UNUM32 wantLen)
{
    PDU_PIN_DATA pins[2];
    PDU_RSC_DATA rsc;
    UNUM8 flagByte[4] = {0};
    PDU_FLAG_DATA flags = {4, flagByte};
    UNUM32 hCLL = 0, hStart = 0, hRx = 0, hTx = 0, urid = 0, len = 0;
    UNUM32 cpReq = reqId, cpResp = respId, cpFmt = 5;
    PDU_PARAM_ITEM items[3];
    PDU_ECU_UNIQUE_RESP_DATA ecu;
    PDU_UNIQUE_RESP_ID_TABLE_ITEM table;
    UNUM8 mask[4] = {0, 0, 0, 0}, pattern[4] = {0, 0, 0, 0};
    PDU_EXP_RESP_DATA exp;
    PDU_COP_CTRL_DATA ctrl;
    UNUM8 msg[64], got[MAX_RESULT];
    char what[64];

    printf("--- %s ---\n", bus);
    pins[0].DLCPinNumber = pin1; pins[0].DLCPinTypeId = obj(PDU_OBJT_PINTYPE, nPins == 2 ? "HI" : "SINGLE");
    pins[1].DLCPinNumber = pin2; pins[1].DLCPinTypeId = obj(PDU_OBJT_PINTYPE, "LOW");
    rsc.BusTypeId = obj(PDU_OBJT_BUSTYPE, bus);
    rsc.ProtocolId = obj(PDU_OBJT_PROTOCOL, "ISO_15765_3_on_ISO_15765_2");
    rsc.NumPinData = nPins;
    rsc.pDLCPinData = pins;
    CHECK(PDUCreateComLogicalLink(1, &rsc, PDU_ID_UNDEF, (void *)0x1234, &hCLL, &flags));
    CHECK(PDURegisterEventCallback(1, hCLL, on_event));
    param(hCLL, "CP_P2Max", 100000);

    items[0].ItemType = items[1].ItemType = items[2].ItemType = PDU_IT_PARAM;
    items[0].ComParamId = obj(PDU_OBJT_COMPARAM, "CP_CanPhysReqId");      items[0].pComParamData = &cpReq;
    items[1].ComParamId = obj(PDU_OBJT_COMPARAM, "CP_CanRespUSDTId");     items[1].pComParamData = &cpResp;
    items[2].ComParamId = obj(PDU_OBJT_COMPARAM, "CP_CanRespUSDTFormat"); items[2].pComParamData = &cpFmt;
    items[0].ComParamDataType = items[1].ComParamDataType = items[2].ComParamDataType = PDU_PT_UNUM32;
    items[0].ComParamClass = items[1].ComParamClass = items[2].ComParamClass = PDU_PC_UNIQUE_ID;
    ecu.UniqueRespIdentifier = 0x11;
    ecu.NumParamItems = 3;
    ecu.pParams = items;
    table.ItemType = PDU_IT_UNIQUE_RESP_ID_TABLE;
    table.NumEntries = 1;
    table.pUniqueData = &ecu;
    CHECK(PDUSetUniqueRespIdTable(1, hCLL, &table));
    CHECK(PDUConnect(1, hCLL));

    memset(&ctrl, 0, sizeof ctrl);
    CHECK(PDUStartComPrimitive(1, hCLL, PDU_COPT_STARTCOMM, 0, NULL, &ctrl, NULL, &hStart));

    exp.ResponseType = 1; exp.AcceptanceId = 1; exp.NumMaskPatternBytes = 4;
    exp.pMaskData = mask; exp.pPatternData = pattern; exp.NumUniqueRespIds = 0; exp.pUniqueRespIds = NULL;
    ctrl.NumSendCycles = 0; ctrl.NumReceiveCycles = -1;
    ctrl.NumPossibleExpectedResponses = 1; ctrl.pExpectedResponseArray = &exp;
    CHECK(PDUStartComPrimitive(1, hCLL, PDU_COPT_SENDRECV, 0, NULL, &ctrl, (void *)0xAAAA, &hRx));

    msg[0] = (UNUM8)(reqId >> 24); msg[1] = (UNUM8)(reqId >> 16); msg[2] = (UNUM8)(reqId >> 8); msg[3] = (UNUM8)reqId;
    memcpy(msg + 4, req, reqLen);
    ctrl.NumSendCycles = 1; ctrl.NumReceiveCycles = 0; ctrl.NumPossibleExpectedResponses = 0;
    ctrl.pExpectedResponseArray = NULL;
    CHECK(PDUStartComPrimitive(1, hCLL, PDU_COPT_SENDRECV, reqLen + 4, msg, &ctrl, (void *)0xBBBB, &hTx));
    printf("   hCLL=%08X hStart=%08X hRx=%08X hTx=%08X\n", hCLL, hStart, hRx, hTx);

    _snprintf(what, sizeof what, "%s request/response", bus);
    if (wait_result(hCLL, got, &len, &urid)) {
        UNUM8 full[MAX_RESULT];
        full[0] = (UNUM8)(respId >> 24); full[1] = (UNUM8)(respId >> 16); full[2] = (UNUM8)(respId >> 8); full[3] = (UNUM8)respId;
        memcpy(full + 4, want, wantLen);
        expect(what, got, len, full, wantLen + 4);
        if (urid != 0x11) { printf("FAIL UniqueRespIdentifier %08X\n", urid); g_failures++; }
    } else {
        printf("FAIL %s: no response\n", what);
        g_failures++;
    }
    CHECK(PDUCancelComPrimitive(1, hCLL, hRx));
    CHECK(PDUDisconnect(1, hCLL));
    drain(hCLL, got, &len, &urid);
    CHECK(PDUDestroyComLogicalLink(1, hCLL));
}

static void run_vpw(void)
{
    PDU_RSC_DATA rsc;
    PDU_PIN_DATA pin = {2, 0};
    UNUM32 hCLL = 0, hStart = 0, hRx = 0, hTx = 0, len = 0, urid = 0;
    PDU_COP_CTRL_DATA ctrl;
    UNUM8 req[] = {0x6C, 0x10, 0xF1, 0x3C, 0x01};
    /* PCM block $01 read; the block data itself comes from ecu.json */
    UNUM8 want[] = {0x6C, 0xF1, 0x10, 0x7C, 0x01};
    UNUM8 got[MAX_RESULT];

    printf("--- SAE_J1850_VPW ---\n");
    pin.DLCPinTypeId = obj(PDU_OBJT_PINTYPE, "SINGLE");
    rsc.BusTypeId = obj(PDU_OBJT_BUSTYPE, "SAE_J1850_VPW");
    rsc.ProtocolId = obj(PDU_OBJT_PROTOCOL, "SAE_J2190_on_SAE_J1850_VPW");
    rsc.NumPinData = 1;
    rsc.pDLCPinData = &pin;
    CHECK(PDUCreateComLogicalLink(1, &rsc, PDU_ID_UNDEF, NULL, &hCLL, NULL));
    CHECK(PDURegisterEventCallback(1, hCLL, on_event));
    CHECK(PDUConnect(1, hCLL));
    memset(&ctrl, 0, sizeof ctrl);
    CHECK(PDUStartComPrimitive(1, hCLL, PDU_COPT_STARTCOMM, 0, NULL, &ctrl, NULL, &hStart));
    ctrl.NumReceiveCycles = -1;
    CHECK(PDUStartComPrimitive(1, hCLL, PDU_COPT_SENDRECV, 0, NULL, &ctrl, NULL, &hRx));
    ctrl.NumSendCycles = 1; ctrl.NumReceiveCycles = 0;
    CHECK(PDUStartComPrimitive(1, hCLL, PDU_COPT_SENDRECV, sizeof req, req, &ctrl, NULL, &hTx));
    if (wait_result(hCLL, got, &len, &urid))
        expect("VPW request/response", got, len > sizeof want ? sizeof want : len, want, sizeof want);
    else {
        printf("FAIL VPW: no response\n");
        g_failures++;
    }
    CHECK(PDUDisconnect(1, hCLL));
    drain(hCLL, got, &len, &urid);
    CHECK(PDUDestroyComLogicalLink(1, hCLL));
}

/* Finds a driver DLL in the current directory, else in the project's build
 * folder (..\build next to this exe); NULL when it is in neither. */
static const char *find_dll(const char *name)
{
    static char path[MAX_PATH];
    char *slash;
    if (GetFileAttributesA(name) != INVALID_FILE_ATTRIBUTES)
        return name;
    GetModuleFileNameA(NULL, path, sizeof path);
    slash = strrchr(path, '\\');
    if (!slash)
        return NULL;
    _snprintf(slash + 1, sizeof path - (slash + 1 - path), "..\\build\\%s", name);
    path[sizeof path - 1] = 0;
    return GetFileAttributesA(path) != INVALID_FILE_ATTRIBUTES ? path : NULL;
}

/* Resolves one export; main() returns 1 if it is missing. */
#define LOAD(name) name =(pfn_##name)GetProcAddress(h, #name); if (!name) { printf("missing export %s\n", #name); return 1; }

int main(int argc, char **argv)
{
    const char *dll = NULL;
    const char *wait = NULL;
    HMODULE h;
    PDU_MODULE_ITEM *mods = NULL;
    PDU_VERSION_DATA ver;
    PDU_DATA_ITEM *vb = NULL;
    UNUM8 got[MAX_RESULT];
    UNUM32 len = 0, urid = 0;
    static const UNUM8 req1a[] = {0x1A, 0x90};
    static const UNUM8 resp1a[] = {0x5A, 0x90, '1', 'G', '1', 'Y', 'Y', '2', '2', 'G', '9', '6', '5', '1', '0', '0', '0', '0', '1'};
    static const UNUM8 req22[] = {0x22, 0xF1, 0x87};
    static const UNUM8 resp22[] = {0x62, 0xF1, 0x87, '1', '2', '3', '4', '5', '6', '7', '8'};

    /* first argument: -c, -cs or a DLL path (optional); then connect-wait-ms */
    if (argc > 1 && !isdigit((unsigned char)argv[1][0])) {
        if (!_stricmp(argv[1], "-c") || !_stricmp(argv[1], "-cs")) {
            const char *name = !_stricmp(argv[1], "-c") ? "dpdu-bridge-c.dll" : "dpdu-bridge-cs.dll";
            dll = find_dll(name);
            if (!dll) {
                printf("%s is not in the current directory or the build folder - "
                       "build it first (build.cmd %s)\n", name, argv[1] + 1);
                return 1;
            }
        } else {
            dll = argv[1];
        }
        if (argc > 2)
            wait = argv[2];
    } else if (argc > 1) {
        wait = argv[1];
    }
    if (!dll) {
        dll = find_dll("dpdu-bridge-c.dll");
        if (!dll)
            dll = find_dll("dpdu-bridge-cs.dll");
        if (!dll) {
            printf("neither dpdu-bridge-c.dll nor dpdu-bridge-cs.dll is in the current directory or "
                   "the build folder - build one of the drivers first (build.cmd c or build.cmd cs)\n");
            return 1;
        }
    }
    printf("using %s\n", dll);
    h = LoadLibraryA(dll);
    if (!h) { printf("cannot load %s (%lu)\n", dll, GetLastError()); return 1; }
    LOAD(PDUConstruct) LOAD(PDUDestruct) LOAD(PDUGetModuleIds) LOAD(PDUModuleConnect) LOAD(PDUModuleDisconnect)
    LOAD(PDURegisterEventCallback) LOAD(PDUGetObjectId) LOAD(PDUCreateComLogicalLink) LOAD(PDUDestroyComLogicalLink)
    LOAD(PDUConnect) LOAD(PDUDisconnect) LOAD(PDUSetComParam) LOAD(PDUSetUniqueRespIdTable) LOAD(PDUStartComPrimitive)
    LOAD(PDUCancelComPrimitive) LOAD(PDUGetEventItem) LOAD(PDUDestroyItem) LOAD(PDUIoCtl) LOAD(PDUGetVersion)
    LOAD(PDUGetStatus)
    g_evt = CreateEventA(NULL, FALSE, FALSE, NULL);

    CHECK(PDUConstruct(NULL, (void *)0x5A5A));
    CHECK(PDUGetModuleIds(&mods));
    if (!mods || mods->NumEntries != 1) { printf("FAIL module list\n"); return 1; }
    printf("module: %s (%s) hMod=%u\n", mods->pModuleData[0].pVendorModuleName,
           mods->pModuleData[0].pVendorAdditionalInfo, mods->pModuleData[0].hMod);
    PDUDestroyItem((PDU_ITEM *)mods);
    CHECK(PDUGetVersion(1, &ver));
    printf("version: %s / %s, part2 %08X\n", ver.HwName, ver.PDUApiSwName, ver.MVCI_Part2StandardVersion);
    CHECK(PDUModuleConnect(1));
    if (wait)
        Sleep((DWORD)atoi(wait));
    CHECK(PDURegisterEventCallback(1, PDU_HANDLE_UNDEF, on_event));
    drain(PDU_HANDLE_UNDEF, got, &len, &urid);
    CHECK(PDUIoCtl(1, PDU_HANDLE_UNDEF, obj(PDU_OBJT_IO_CTRL, "PDU_IOCTL_READ_VBATT"), NULL, &vb));
    printf("VBATT %u mV\n", vb ? *(UNUM32 *)vb->pData : 0);
    if (vb) PDUDestroyItem((PDU_ITEM *)vb);

    run_can("ISO_11898_2_DWCAN", 6, 14, 2, 0x7E0, 0x7E8, req22, sizeof req22, resp22, sizeof resp22);
    run_can("SAE_J2411_SWCAN", 1, 0, 1, 0x241, 0x641, req1a, sizeof req1a, resp1a, sizeof resp1a);
    run_vpw();

    CHECK(PDUModuleDisconnect(1));
    CHECK(PDUDestruct());
    printf("callbacks received: %ld\n", g_callbacks);
    printf("VPW node-alive messages received: %d\n", g_heartbeats);
    printf(g_failures ? "\n%d FAILURE(S)\n" : "\nALL TESTS PASSED\n", g_failures);
    return g_failures != 0;
}
