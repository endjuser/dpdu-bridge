/*
 * j2534_test.c - drives the J2534 (PassThru 04.04) side of a bridge DLL.
 *
 *   j2534_test.exe [-c | -cs | path\to\dll] [connect-wait-ms]
 *
 * -c loads dpdu-bridge-c.dll, -cs dpdu-bridge-cs.dll, from the current
 * directory, else the project's build folder (..\build next to this exe).
 * Without either, dpdu-bridge-c.dll is used if it exists, else
 * dpdu-bridge-cs.dll.
 *
 * connect-wait-ms: pause after PassThruOpen, e.g. 1500 in TCP mode so the
 * simulator has time to connect to the DLL's TCP server.
 *
 * Start the simulator first (run_sim.bat, serial or TCP).  The test opens
 * the device, reads the version and VBATT, then sends one request each on
 * J1850 VPW, ISO 15765 (HS-CAN), SW_ISO15765_PS (GMLAN) and raw CAN (with
 * loopback and a periodic message), and checks the responses.  Exit code 0
 * when every check passed.  Build with "build.cmd test" (32-bit, like the DLL).
 */
#include <windows.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    unsigned long ProtocolID, RxStatus, TxFlags, Timestamp, DataSize, ExtraDataIndex;
    unsigned char Data[4128];
} PASSTHRU_MSG;

typedef struct { unsigned long Parameter, Value; } SCONFIG;
typedef struct { unsigned long NumOfParams; SCONFIG *ConfigPtr; } SCONFIG_LIST;

#define J1850VPW 1
#define J1850PWM 2
#define CAN 5
#define ISO15765 6
#define SW_ISO15765_PS 0x8007
#define SW_CAN_PS 0x8008
#define SW_CAN_HV_TX 0x400
#define PASS_FILTER 1
#define FLOW_CONTROL_FILTER 3
#define GET_CONFIG 1
#define SET_CONFIG 2
#define READ_VBATT 3
#define CLEAR_RX_BUFFER 8
#define DATA_RATE 1
#define LOOPBACK 3
#define TX_MSG_TYPE 0x01
#define TX_DONE 0x08
#define ERR_INVALID_PROTOCOL_ID 0x03
#define ERR_TIMEOUT 0x09
#define ERR_BUFFER_EMPTY 0x10

/* Declares a function-pointer type and variable for each export (see LOAD). */
#define F(name, args) typedef long (WINAPI *pfn_##name) args; static pfn_##name name
F(PassThruOpen, (void *, unsigned long *));
F(PassThruClose, (unsigned long));
F(PassThruConnect, (unsigned long, unsigned long, unsigned long, unsigned long, unsigned long *));
F(PassThruDisconnect, (unsigned long));
F(PassThruReadMsgs, (unsigned long, PASSTHRU_MSG *, unsigned long *, unsigned long));
F(PassThruWriteMsgs, (unsigned long, PASSTHRU_MSG *, unsigned long *, unsigned long));
F(PassThruStartPeriodicMsg, (unsigned long, PASSTHRU_MSG *, unsigned long *, unsigned long));
F(PassThruStopPeriodicMsg, (unsigned long, unsigned long));
F(PassThruStartMsgFilter, (unsigned long, unsigned long, PASSTHRU_MSG *, PASSTHRU_MSG *, PASSTHRU_MSG *, unsigned long *));
F(PassThruStopMsgFilter, (unsigned long, unsigned long));
F(PassThruSetProgrammingVoltage, (unsigned long, unsigned long, unsigned long));
F(PassThruReadVersion, (unsigned long, char *, char *, char *));
F(PassThruGetLastError, (char *));
F(PassThruIoctl, (unsigned long, unsigned long, void *, void *));

static int g_failures;
static int g_heartbeats;    /* VPW node-alive messages received */

static void check(const char *what, long got, long want)
{
    if (got != want) {
        char err[80] = "";
        PassThruGetLastError(err);
        printf("FAIL %s -> 0x%lX (%s)\n", what, got, err);
        g_failures++;
    }
}
#define CHECK(call) check(#call, (call), 0)

static void pass_fail(const char *what, int ok)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        g_failures++;
}

static void set_msg(PASSTHRU_MSG *m, unsigned long protocol, const unsigned char *data, unsigned long len)
{
    memset(m, 0, sizeof *m);
    m->ProtocolID = protocol;
    m->DataSize = len;
    memcpy(m->Data, data, len);
}

static void set_id(PASSTHRU_MSG *m, unsigned long protocol, unsigned long id)
{
    unsigned char b[4];
    b[0] = (unsigned char)(id >> 24); b[1] = (unsigned char)(id >> 16); b[2] = (unsigned char)(id >> 8); b[3] = (unsigned char)id;
    set_msg(m, protocol, b, 4);
}

static void print_msg(const PASSTHRU_MSG *m)
{
    unsigned long i;
    printf("   rx protocol=%lX status=%lX:", m->ProtocolID, m->RxStatus);
    for (i = 0; i < m->DataSize && i < 64; i++)
        printf(" %02X", m->Data[i]);
    printf("\n");
}

/* Reads messages for up to 2 s until one with RxStatus status (TX_MSG_TYPE,
 * TX_DONE or 0) starts with want; VPW node-alive messages are counted and
 * skipped.  Returns 1 when found. */
static int wait_for(unsigned long ch, unsigned long status, const unsigned char *want, unsigned long wantLen)
{
    static PASSTHRU_MSG m;
    DWORD t0 = GetTickCount();
    while (GetTickCount() - t0 < 2000) {
        unsigned long n = 1;
        long r = PassThruReadMsgs(ch, &m, &n, 100);
        if (r != 0 && r != ERR_TIMEOUT && r != ERR_BUFFER_EMPTY) {
            check("PassThruReadMsgs", r, 0);
            return 0;
        }
        if (n == 0)
            continue;
        if (m.DataSize >= 2 && m.Data[0] == 0x08 && m.Data[1] == 0xFF && m.RxStatus == 0) {
            g_heartbeats++;
            continue;
        }
        print_msg(&m);
        if (m.RxStatus == status && m.DataSize >= wantLen && !memcmp(m.Data, want, wantLen))
            return 1;
    }
    return 0;
}

static void write1(unsigned long ch, PASSTHRU_MSG *m)
{
    unsigned long n = 1;
    CHECK(PassThruWriteMsgs(ch, m, &n, 0));
    if (n != 1) { printf("FAIL WriteMsgs sent %lu messages\n", n); g_failures++; }
}

static void set_config(unsigned long ch, unsigned long param, unsigned long value)
{
    SCONFIG c;
    SCONFIG_LIST l;
    c.Parameter = param; c.Value = value;
    l.NumOfParams = 1; l.ConfigPtr = &c;
    CHECK(PassThruIoctl(ch, SET_CONFIG, &l, NULL));
}

static void run_vpw(unsigned long dev)
{
    unsigned long ch = 0, fid = 0;
    static PASSTHRU_MSG mask, pattern, msg;
    static const unsigned char zero[1] = {0};
    /* PCM block $01 read; the block data itself comes from ecu.json */
    static const unsigned char req[] = {0x6C, 0x10, 0xF1, 0x3C, 0x01};
    static const unsigned char want[] = {0x6C, 0xF1, 0x10, 0x7C, 0x01};
    SCONFIG c;
    SCONFIG_LIST l;

    printf("--- J1850VPW ---\n");
    CHECK(PassThruConnect(dev, J1850VPW, 0, 10400, &ch));
    set_msg(&mask, J1850VPW, zero, 1);
    set_msg(&pattern, J1850VPW, zero, 1);
    CHECK(PassThruStartMsgFilter(ch, PASS_FILTER, &mask, &pattern, NULL, &fid));
    CHECK(PassThruIoctl(ch, CLEAR_RX_BUFFER, NULL, NULL));
    set_config(ch, DATA_RATE, 41600);           /* 4x mode and back, as Tech2Win does */
    set_config(ch, DATA_RATE, 10400);
    c.Parameter = DATA_RATE; c.Value = 0;
    l.NumOfParams = 1; l.ConfigPtr = &c;
    CHECK(PassThruIoctl(ch, GET_CONFIG, &l, NULL));
    pass_fail("VPW GET_CONFIG DATA_RATE", c.Value == 10400);
    set_msg(&msg, J1850VPW, req, sizeof req);
    write1(ch, &msg);
    pass_fail("VPW request/response", wait_for(ch, 0, want, sizeof want));
    CHECK(PassThruStopMsgFilter(ch, fid));
    CHECK(PassThruDisconnect(ch));
}

/* ISO 15765 channel: flow control filter for respId / reqId, one request,
 * then the TX_DONE indication and the complete response respId + want. */
static void run_iso(unsigned long dev, const char *name, unsigned long protocol, unsigned long baud,
                    unsigned long reqId, unsigned long respId,
                    const unsigned char *req, unsigned long reqLen, const unsigned char *want, unsigned long wantLen)
{
    unsigned long ch = 0, fid = 0;
    static PASSTHRU_MSG mask, pattern, flow, msg;
    unsigned char buf[64];
    char what[64];

    printf("--- %s ---\n", name);
    CHECK(PassThruConnect(dev, protocol, 0, baud, &ch));
    set_id(&mask, protocol, 0xFFFFFFFF);
    set_id(&pattern, protocol, respId);
    set_id(&flow, protocol, reqId);
    CHECK(PassThruStartMsgFilter(ch, FLOW_CONTROL_FILTER, &mask, &pattern, &flow, &fid));
    set_id(&msg, protocol, reqId);
    memcpy(msg.Data + 4, req, reqLen);
    msg.DataSize = 4 + reqLen;
    write1(ch, &msg);
    _snprintf(what, sizeof what, "%s TX_DONE indication", name);
    pass_fail(what, wait_for(ch, TX_DONE, flow.Data, 4));
    memcpy(buf, pattern.Data, 4);
    memcpy(buf + 4, want, wantLen);
    _snprintf(what, sizeof what, "%s request/response", name);
    pass_fail(what, wait_for(ch, 0, buf, 4 + wantLen));
    CHECK(PassThruDisconnect(ch));
}

/* HS-CAN the way Tech2Win's emulator.exe sets it up: one ISO15765 channel in
 * mixed format, the default GMLAN filter table (32 UUDT pass filters with
 * protocol CAN and the 29-bit marker bit, 32 USDT flow control filters, each
 * followed by CLEAR_RX_BUFFER), a 5-byte flow control filter whose 5th byte
 * is garbage, and every transmit as protocol ISO15765. */
static void run_tech2win_can(unsigned long dev)
{
    unsigned long ch = 0, fid = 0, i;
    static PASSTHRU_MSG mask, pattern, flow, msg;
    SCONFIG c;
    SCONFIG_LIST l;
    static const unsigned char req1a[] = {0x00, 0x00, 0x02, 0x41, 0x1A, 0x90};
    static const unsigned char resp1a[] = {0x00, 0x00, 0x06, 0x41, 0x5A, 0x90, '1', 'G', '1'};
    static const unsigned char req22[] = {0x00, 0x00, 0x07, 0xE0, 0x22, 0xF1, 0x87};
    static const unsigned char resp22[] = {0x00, 0x00, 0x07, 0xE8, 0x62, 0xF1, 0x87};

    printf("--- ISO15765 the Tech2Win way (mixed format, 65 filters) ---\n");
    CHECK(PassThruConnect(dev, ISO15765, 0, 500000, &ch));
    c.Parameter = 0x8000; c.Value = 2;          /* CAN_MIXED_FORMAT = ALL_FRAMES */
    l.NumOfParams = 1; l.ConfigPtr = &c;
    CHECK(PassThruIoctl(ch, SET_CONFIG, &l, NULL));
    CHECK(PassThruIoctl(ch, 0x0A, NULL, NULL));  /* CLEAR_MSG_FILTERS */
    for (i = 0; i < 32; i++) {
        set_id(&mask, CAN, 0x3FFFFFFF);          /* bit 29: Tech2's 29-bit marker */
        set_id(&pattern, CAN, 0x20000540 + i);
        CHECK(PassThruStartMsgFilter(ch, PASS_FILTER, &mask, &pattern, NULL, &fid));
        CHECK(PassThruIoctl(ch, CLEAR_RX_BUFFER, NULL, NULL));
        set_id(&mask, ISO15765, 0xFFFFFFFF);
        set_id(&pattern, ISO15765, 0x640 + i);
        set_id(&flow, ISO15765, 0x240 + i);
        CHECK(PassThruStartMsgFilter(ch, FLOW_CONTROL_FILTER, &mask, &pattern, &flow, &fid));
        CHECK(PassThruIoctl(ch, CLEAR_RX_BUFFER, NULL, NULL));
    }
    set_id(&mask, ISO15765, 0xFFFFFFFF);
    set_id(&pattern, ISO15765, 0x7E8);
    set_id(&flow, ISO15765, 0x7E0);
    mask.Data[4] = 0xCC; pattern.Data[4] = 0x5A; flow.Data[4] = 0xA5;   /* uninitialised 5th byte */
    mask.DataSize = pattern.DataSize = flow.DataSize = 5;
    CHECK(PassThruStartMsgFilter(ch, FLOW_CONTROL_FILTER, &mask, &pattern, &flow, &fid));

    set_msg(&msg, ISO15765, req1a, sizeof req1a);
    write1(ch, &msg);
    pass_fail("Tech2Win-style USDT request/response (0x241)", wait_for(ch, 0, resp1a, sizeof resp1a));
    set_msg(&msg, ISO15765, req22, sizeof req22);
    write1(ch, &msg);
    pass_fail("Tech2Win-style 5-byte flow control filter (0x7E0)", wait_for(ch, 0, resp22, sizeof resp22));
    CHECK(PassThruDisconnect(ch));
}

/* Raw CAN with LOOPBACK: echo of the request, the response, then a periodic
 * tester present. */
static void run_raw_can(unsigned long dev)
{
    unsigned long ch = 0, fid = 0, pid = 0, n;
    static PASSTHRU_MSG mask, pattern, msg, rx;
    static const unsigned char tp[] = {0x00, 0x00, 0x07, 0xE0, 0x3E, 0x00};
    static const unsigned char tpResp[] = {0x00, 0x00, 0x07, 0xE8, 0x7E};
    int responses = 0;
    DWORD t0;

    printf("--- CAN (raw, loopback, periodic) ---\n");
    CHECK(PassThruConnect(dev, CAN, 0, 500000, &ch));
    set_id(&mask, CAN, 0x7FF);
    set_id(&pattern, CAN, 0x7E8);
    CHECK(PassThruStartMsgFilter(ch, PASS_FILTER, &mask, &pattern, NULL, &fid));
    set_config(ch, LOOPBACK, 1);
    set_msg(&msg, CAN, tp, sizeof tp);
    write1(ch, &msg);
    pass_fail("CAN loopback echo", wait_for(ch, TX_MSG_TYPE, tp, sizeof tp));
    pass_fail("CAN request/response", wait_for(ch, 0, tpResp, sizeof tpResp));
    set_config(ch, LOOPBACK, 0);

    CHECK(PassThruStartPeriodicMsg(ch, &msg, &pid, 100));
    t0 = GetTickCount();
    while (GetTickCount() - t0 < 550) {
        n = 1;
        if (PassThruReadMsgs(ch, &rx, &n, 50) == 0 && n == 1 && rx.DataSize == sizeof tpResp &&
            !memcmp(rx.Data, tpResp, sizeof tpResp))
            responses++;
    }
    CHECK(PassThruStopPeriodicMsg(ch, pid));
    printf("   periodic tester present: %d responses in 550 ms\n", responses);
    pass_fail("CAN periodic message", responses >= 3);
    CHECK(PassThruDisconnect(ch));
}

/* SW-CAN high-voltage wake-up: id 0x100 with SW_CAN_HV_TX (the DLL log should
 * show one WAKEUP SWCAN line, not two), then a frame with SW_CAN_HV_TX. */
static void run_sw_wakeup(unsigned long dev)
{
    unsigned long ch = 0;
    static PASSTHRU_MSG msg;
    static const unsigned char tp[] = {0x00, 0x00, 0x01, 0x01, 0xFE, 0x01, 0x3E};

    printf("--- SW_CAN_PS (high-voltage wake-up) ---\n");
    CHECK(PassThruConnect(dev, SW_CAN_PS, 0, 33333, &ch));
    set_id(&msg, SW_CAN_PS, 0x100);
    msg.TxFlags = SW_CAN_HV_TX;
    write1(ch, &msg);
    set_msg(&msg, SW_CAN_PS, tp, sizeof tp);
    msg.TxFlags = SW_CAN_HV_TX;
    write1(ch, &msg);
    CHECK(PassThruDisconnect(ch));
}

/* Finds the DLL in the current directory, else in the project's build folder
 * (..\build next to this exe); NULL when it is in neither. */
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
#define LOAD(name) name = (pfn_##name)GetProcAddress(h, #name); if (!name) { printf("missing export %s\n", #name); return 1; }

int main(int argc, char **argv)
{
    const char *dll = NULL;
    const char *wait = NULL;
    HMODULE h;
    unsigned long dev = 0, vbatt = 0, ch = 0, n;
    char fw[80], dllv[80], api[80], err[80];
    static PASSTHRU_MSG rx;
    static const unsigned char req1a[] = {0x1A, 0x90};
    static const unsigned char resp1a[] = {0x5A, 0x90, '1', 'G', '1', 'Y', 'Y', '2', '2', 'G', '9', '6', '5', '1', '0', '0', '0', '0', '1'};
    static const unsigned char req22[] = {0x22, 0xF1, 0x87};
    static const unsigned char resp22[] = {0x62, 0xF1, 0x87, '1', '2', '3', '4', '5', '6', '7', '8'};

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
    LOAD(PassThruOpen) LOAD(PassThruClose) LOAD(PassThruConnect) LOAD(PassThruDisconnect) LOAD(PassThruReadMsgs)
    LOAD(PassThruWriteMsgs) LOAD(PassThruStartPeriodicMsg) LOAD(PassThruStopPeriodicMsg) LOAD(PassThruStartMsgFilter)
    LOAD(PassThruStopMsgFilter) LOAD(PassThruSetProgrammingVoltage) LOAD(PassThruReadVersion)
    LOAD(PassThruGetLastError) LOAD(PassThruIoctl)

    CHECK(PassThruOpen(NULL, &dev));
    if (wait)
        Sleep((DWORD)atoi(wait));
    CHECK(PassThruReadVersion(dev, fw, dllv, api));
    printf("version: firmware '%s', DLL '%s', API '%s'\n", fw, dllv, api);
    CHECK(PassThruIoctl(dev, READ_VBATT, NULL, &vbatt));
    printf("VBATT %lu mV\n", vbatt);

    /* errors the host may rely on */
    check("PassThruConnect(J1850PWM)", PassThruConnect(dev, J1850PWM, 0, 41600, &ch), ERR_INVALID_PROTOCOL_ID);
    PassThruGetLastError(err);
    printf("   last error: %s\n", err);
    CHECK(PassThruConnect(dev, CAN, 0, 500000, &ch));
    n = 1;
    check("PassThruReadMsgs(empty, timeout 0)", PassThruReadMsgs(ch, &rx, &n, 0), ERR_BUFFER_EMPTY);
    CHECK(PassThruDisconnect(ch));

    run_vpw(dev);
    run_iso(dev, "ISO15765 (HS-CAN)", ISO15765, 500000, 0x7E0, 0x7E8, req22, sizeof req22, resp22, sizeof resp22);
    run_iso(dev, "SW_ISO15765_PS (GMLAN)", SW_ISO15765_PS, 33333, 0x241, 0x641, req1a, sizeof req1a, resp1a, sizeof resp1a);
    run_tech2win_can(dev);
    run_raw_can(dev);
    run_sw_wakeup(dev);

    CHECK(PassThruClose(dev));
    printf("VPW node-alive messages received: %d\n", g_heartbeats);
    printf(g_failures ? "\n%d FAILURE(S)\n" : "\nALL TESTS PASSED\n", g_failures);
    return g_failures != 0;
}
