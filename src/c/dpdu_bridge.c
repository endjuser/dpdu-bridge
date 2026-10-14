/*
 * dpdu_bridge.c - ISO 22900-2 D-PDU API driver (dpdu-bridge-c.dll) that
 * bridges to a line-based text protocol instead of a vehicle interface.
 *
 * Every message the host (e.g. Tech2Win) transmits is written as a text line
 * to the configured links: a serial port (e.g. one end of a com0com pair)
 * and/or the clients of a TCP server.  Lines received on any link are turned
 * into D-PDU receive events, so an ECU simulator (sim/ecu_sim.py) can answer
 * the diagnostic application.  Configuration comes from dpdu-bridge-c.ini or
 * dpdu-bridge.ini next to the DLL; README.md documents the ini keys and the
 * line protocol.  src/cs/ is a C# implementation with identical behaviour.
 *
 * The API only ever deals with complete messages (ISO 15765-2 segmentation is
 * not done here), so the links carry complete messages too.
 *
 * Threads: API calls run on the host's threads and share g_lock.  Each open
 * link adds worker threads: the serial reader and writer, and the TCP network
 * (accept/read) and writer threads.  Readers deliver received lines under
 * g_lock; writers drain per-link output queues guarded by g_txLock, so an API
 * call never blocks on I/O.  Host callbacks are always invoked with no lock
 * held.
 *
 * Build: 32-bit, /W4, see build.cmd.  Exports are listed in dpdu_bridge.def.
 */
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>

#include "pdu_api.h"

#pragma comment(lib, "user32.lib") /* MessageBoxA */
#pragma comment(lib, "ws2_32.lib") /* TCP mode */

/* ------------------------------------------------------------------------ */
/* Constants                                                                 */
/* ------------------------------------------------------------------------ */

#define DRIVER_NAME          "dpdu-bridge-c"   /* HELLO line, message box title, log */
#define MODULE_HANDLE        1u
#define MODULE_TYPE_ID       1u

/* Initial handle values (see g_nextCll). */
#define HCLL_FIRST           0x02000002u
#define HCOP_RX_FIRST        0x02100002u
#define HCOP_TX_FIRST        0x02200004u
#define HCOP_STARTCOMM_FIRST 0x02500001u
#define HCOP_UPDATEPARAM     0x02100007u /* constant handle used for UPDATEPARAM */

/* Bus type ids (PDUGetObjectId) that PDUCreateComLogicalLink accepts. */
#define BUSTYPE_ISO_11898_2_DWCAN 3u
#define BUSTYPE_SAE_J1850_VPW     12u
#define BUSTYPE_SAE_J2411_SWCAN   14u
#define BUSTYPE_SAE_J2740_UART    16u

/* ComParam ids the driver reacts to (same ids as in g_objects). */
#define CP_CANPHYSREQID            31u
#define CP_CANRESPUSDTFORMAT       33u
#define CP_CANRESPUSDTID           34u
#define CP_CANRESPUUDTID           37u
#define CP_TRANSMITINDENABLE       190u

#define IOCTL_READ_VBATT           6u
#define IOCTL_START_MSG_FILTER     12u
#define IOCTL_CLEAR_MSG_FILTER     14u

#define GMLAN_HV_WAKEUP_ID         0x100u

#define MAX_FRAME                  4200 /* ISO-TP max 4095 + 4-byte CAN id, rounded up */

/* ------------------------------------------------------------------------ */
/* PDUGetObjectId table                                                      */
/* ------------------------------------------------------------------------ */

/* Short names the host resolves with PDUGetObjectId; unlisted names are
 * rejected with PDU_ERR_INVALID_PARAMETERS.  The ComParam names are also used
 * in PARAM lines. */
typedef struct { T_PDU_OBJT type; const char *name; UNUM32 id; } OBJ_ENTRY;

static const OBJ_ENTRY g_objects[] = {
    /* protocols */
    {PDU_OBJT_PROTOCOL, "ISO_11898_RAW", 6},
    {PDU_OBJT_PROTOCOL, "ISO_14230_3_on_ISO_14230_2", 8},
    {PDU_OBJT_PROTOCOL, "ISO_15031_5_on_ISO_15765_4", 11},
    {PDU_OBJT_PROTOCOL, "ISO_15765_3_on_ISO_15765_2", 16},
    {PDU_OBJT_PROTOCOL, "KW309_on_KW_UART", 22},
    {PDU_OBJT_PROTOCOL, "KW81_on_KW_UART", 24},
    {PDU_OBJT_PROTOCOL, "KW82_on_KW_UART", 25},
    {PDU_OBJT_PROTOCOL, "SAE_J1939_73_on_SAE_J1939_21", 34},
    {PDU_OBJT_PROTOCOL, "SAE_J2190_on_ISO_9141_2", 38},
    {PDU_OBJT_PROTOCOL, "SAE_J2190_on_SAE_J1850_VPW", 40},
    {PDU_OBJT_PROTOCOL, "XDE_5024_on_XDE_5024_UART", 46},
    /* bus types */
    {PDU_OBJT_BUSTYPE, "ISO_11898_2_DWCAN", 3},
    {PDU_OBJT_BUSTYPE, "ISO_11898_3_DWFTCAN", 4},
    {PDU_OBJT_BUSTYPE, "ISO_14230_1_UART", 6},
    {PDU_OBJT_BUSTYPE, "ISO_9141_2_UART", 7},
    {PDU_OBJT_BUSTYPE, "KW_UART", 8},
    {PDU_OBJT_BUSTYPE, "SAE_J1850_VPW", 12},
    {PDU_OBJT_BUSTYPE, "SAE_J1939_11_DWCAN", 13},
    {PDU_OBJT_BUSTYPE, "SAE_J2411_SWCAN", 14},
    {PDU_OBJT_BUSTYPE, "SAE_J2740_UART", 16},
    /* IOCTLs */
    {PDU_OBJT_IO_CTRL, "PDU_IOCTL_READ_VBATT", 6},
    {PDU_OBJT_IO_CTRL, "PDU_IOCTL_START_MSG_FILTER", 12},
    {PDU_OBJT_IO_CTRL, "PDU_IOCTL_CLEAR_MSG_FILTER", 14},
    /* pin types */
    {PDU_OBJT_PINTYPE, "HI", 1},
    {PDU_OBJT_PINTYPE, "LOW", 2},
    {PDU_OBJT_PINTYPE, "K", 3},
    {PDU_OBJT_PINTYPE, "L", 4},
    {PDU_OBJT_PINTYPE, "PLUS", 7},
    {PDU_OBJT_PINTYPE, "SINGLE", 9},
    /* ComParams */
    {PDU_OBJT_COMPARAM, "CP_5BaudAddressPhys", 2},
    {PDU_OBJT_COMPARAM, "CP_5BaudCommBaudrateOverride", 3},
    {PDU_OBJT_COMPARAM, "CP_5BaudInitBaudrate", 4},
    {PDU_OBJT_COMPARAM, "CP_5BaudMode", 5},
    {PDU_OBJT_COMPARAM, "CP_As", 8},
    {PDU_OBJT_COMPARAM, "CP_Baudrate", 9},
    {PDU_OBJT_COMPARAM, "CP_BecomeMaster", 10},
    {PDU_OBJT_COMPARAM, "CP_BecomeSlave", 11},
    {PDU_OBJT_COMPARAM, "CP_BitSamplePoint", 12},
    {PDU_OBJT_COMPARAM, "CP_Bs", 17},
    {PDU_OBJT_COMPARAM, "CP_ByteCountOffset", 18},
    {PDU_OBJT_COMPARAM, "CP_CanFillerByte", 21},
    {PDU_OBJT_COMPARAM, "CP_CanFillerByteHandling", 22},
    {PDU_OBJT_COMPARAM, "CP_CanPhysReqExtAddr", 29},
    {PDU_OBJT_COMPARAM, "CP_CanPhysReqFormat", 30},
    {PDU_OBJT_COMPARAM, "CP_CanPhysReqId", 31},
    {PDU_OBJT_COMPARAM, "CP_CanRespUSDTExtAddr", 32},
    {PDU_OBJT_COMPARAM, "CP_CanRespUSDTFormat", 33},
    {PDU_OBJT_COMPARAM, "CP_CanRespUSDTId", 34},
    {PDU_OBJT_COMPARAM, "CP_CanRespUUDTExtAddr", 35},
    {PDU_OBJT_COMPARAM, "CP_CanRespUUDTFormat", 36},
    {PDU_OBJT_COMPARAM, "CP_CanRespUUDTId", 37},
    {PDU_OBJT_COMPARAM, "CP_Cr", 51},
    {PDU_OBJT_COMPARAM, "CP_Cs", 52},
    {PDU_OBJT_COMPARAM, "CP_HeaderFormatKW", 79},
    {PDU_OBJT_COMPARAM, "CP_InitializationSettings", 86},
    {PDU_OBJT_COMPARAM, "CP_K_L_LineInit", 104},
    {PDU_OBJT_COMPARAM, "CP_Loopback", 112},
    {PDU_OBJT_COMPARAM, "CP_MessageScheduler", 120},
    {PDU_OBJT_COMPARAM, "CP_P2Max", 126},
    {PDU_OBJT_COMPARAM, "CP_P2Min", 127},
    {PDU_OBJT_COMPARAM, "CP_P3Max", 130},
    {PDU_OBJT_COMPARAM, "CP_P3Min", 131},
    {PDU_OBJT_COMPARAM, "CP_P3Min_J2740", 132},
    {PDU_OBJT_COMPARAM, "CP_P3Phys", 133},
    {PDU_OBJT_COMPARAM, "CP_P4Max", 134},
    {PDU_OBJT_COMPARAM, "CP_P4Min", 135},
    {PDU_OBJT_COMPARAM, "CP_RepeatReqCountApp", 150},
    {PDU_OBJT_COMPARAM, "CP_RequestAddrMode", 152},
    {PDU_OBJT_COMPARAM, "CP_SamplesPerBit", 153},
    {PDU_OBJT_COMPARAM, "CP_SetPollResponse", 160},
    {PDU_OBJT_COMPARAM, "CP_SetRelinquishMastership", 161},
    {PDU_OBJT_COMPARAM, "CP_StartMsgIndEnable", 164},
    {PDU_OBJT_COMPARAM, "CP_SwCan_HighVoltage", 166},
    {PDU_OBJT_COMPARAM, "CP_SyncJumpWidth", 167},
    {PDU_OBJT_COMPARAM, "CP_TIdle", 169},
    {PDU_OBJT_COMPARAM, "CP_TInil", 170},
    {PDU_OBJT_COMPARAM, "CP_TWup", 171},
    {PDU_OBJT_COMPARAM, "CP_TerminationType", 177},
    {PDU_OBJT_COMPARAM, "CP_TesterPresentHandling", 183},
    {PDU_OBJT_COMPARAM, "CP_TransmitIndEnable", 190},
    {PDU_OBJT_COMPARAM, "CP_W1Max", 194},
    {PDU_OBJT_COMPARAM, "CP_W1Min", 195},
    {PDU_OBJT_COMPARAM, "CP_W2Max", 196},
    {PDU_OBJT_COMPARAM, "CP_W2Min", 197},
    {PDU_OBJT_COMPARAM, "CP_W3Max", 198},
    {PDU_OBJT_COMPARAM, "CP_W3Min", 199},
    {PDU_OBJT_COMPARAM, "CP_W4Max", 200},
    {PDU_OBJT_COMPARAM, "CP_W4Min", 201},
};

static const char *comparam_name(UNUM32 id)
{
    size_t i;
    for (i = 0; i < sizeof g_objects / sizeof g_objects[0]; i++)
        if (g_objects[i].type == PDU_OBJT_COMPARAM && g_objects[i].id == id)
            return g_objects[i].name;
    return NULL;
}

/* Reads a scalar ComParam value with the width its ComParamDataType gives
 * (signed types are sign-extended to 32 bits).  A scalar with no data reads
 * as 0.  Returns FALSE, without touching pComParamData, for any other data
 * type (byte/struct/long fields): those are not supported as values. */
static BOOL comparam_scalar(const PDU_PARAM_ITEM *p, UNUM32 *value)
{
    const void *d = p->pComParamData;
    *value = 0;
    switch (p->ComParamDataType) {
    case PDU_PT_UNUM8:  if (d) *value = *(const UNUM8 *)d;                    return TRUE;
    case PDU_PT_SNUM8:  if (d) *value = (UNUM32)(SNUM32)*(const SNUM8 *)d;    return TRUE;
    case PDU_PT_UNUM16: if (d) *value = *(const UNUM16 *)d;                   return TRUE;
    case PDU_PT_SNUM16: if (d) *value = (UNUM32)(SNUM32)*(const SNUM16 *)d;   return TRUE;
    case PDU_PT_UNUM32: if (d) *value = *(const UNUM32 *)d;                   return TRUE;
    case PDU_PT_SNUM32: if (d) *value = (UNUM32)*(const SNUM32 *)d;           return TRUE;
    default:            return FALSE;
    }
}

/* ------------------------------------------------------------------------ */
/* State                                                                     */
/* ------------------------------------------------------------------------ */

typedef enum { BUS_NONE = 0, BUS_HSCAN, BUS_SWCAN, BUS_VPW, BUS_ALDL } BUS;

static const char *bus_name(BUS b)
{
    switch (b) {
    case BUS_HSCAN: return "HSCAN";
    case BUS_SWCAN: return "SWCAN";
    case BUS_VPW:   return "VPW";
    case BUS_ALDL:  return "ALDL";
    default:        return "NONE";
    }
}

static int bus_is_can(BUS b) { return b == BUS_HSCAN || b == BUS_SWCAN; }

typedef struct {
    UNUM32 respType;
    UNUM32 mask;     /* big-endian 4 byte mask, 0 if NumMaskPatternBytes != 4 */
    UNUM32 pattern;
    UNUM32 acceptanceId;
} EXP_RESP;

typedef struct {
    UNUM32 uniqueRespId;
    UNUM32 physReqId;   /* CP_CanPhysReqId */
    UNUM32 respUsdtId;  /* CP_CanRespUSDTId */
    UNUM32 respFormat;  /* CP_CanRespUSDTFormat */
    UNUM32 respUudtId;  /* CP_CanRespUUDTId */
} URID_ENTRY;

typedef struct {
    UNUM32      hMod;
    UNUM32      hCLL;
    CALLBACKFNC fn;
} CB_REG;

/* ComLogicalLink.  Only the most recent receive / transmit / STARTCOMM
 * ComPrimitive of each kind is tracked. */
typedef struct {
    UNUM32       hCLL;
    BUS          bus;
    void        *pCllTag;
    UNUM32       txIndEnable;   /* CP_TransmitIndEnable */
    UNUM32       hStartComm;
    UNUM32       hRxCop;
    UNUM32       hTxCop;
    void        *rxCopTag;
    void        *txCopTag;
    T_PDU_STATUS rxCopStatus;
    T_PDU_STATUS cllStatus;
    EXP_RESP    *expResp;       /* expected responses of the receive CoP */
    UNUM32       numExpResp;
    URID_ENTRY  *urid;          /* unique response id table (CAN only) */
    UNUM32       numUrid;
    CB_REG       cb;
} CLL;

/* START_MSG_FILTER entry, applied to received VPW/ALDL frames.  Only the
 * header bytes are compared (3 for VPW, 1 for ALDL). */
typedef struct {
    BUS          bus;
    T_PDU_FILTER type;
    UNUM8        mask[4];
    UNUM8        pattern[4];
} SW_FILTER;

typedef struct EVT_NODE { PDU_EVENT_ITEM *item; struct EVT_NODE *next; } EVT_NODE;

/* Pending callback, invoked after the state lock has been released. */
typedef struct {
    CALLBACKFNC    fn;
    T_PDU_EVT_DATA evt;
    UNUM32         hMod;
    UNUM32         hCLL;
    void          *cllTag;
    void          *apiTag;
} CB_CALL;

/* Lock order: g_lock, g_txLock and g_clientLock are never nested in each
 * other; g_logLock may be taken while holding any of them. */
static CRITICAL_SECTION g_lock;       /* all API state below */
static CRITICAL_SECTION g_logLock;    /* log file */
static CRITICAL_SECTION g_txLock;     /* output queues, g_linkOpen */
static CRITICAL_SECTION g_clientLock; /* TCP client table */
static HMODULE          g_hInst;

static BOOL          g_constructed;
static void         *g_apiTag;
static T_PDU_STATUS  g_moduleStatus = PDU_MODST_AVAIL;
static UNUM32        g_connectedMod;     /* hMod given to PDUModuleConnect */
static BOOL          g_moduleListed;     /* port configured (and COM port present) at construct */
static BUS           g_curBus = BUS_NONE; /* bus of the last created CLL / last transmit */
static CB_REG        g_sysCb;            /* hMod = hCLL = undef: stored, never called */
static CB_REG        g_modCb;            /* module level */

static CLL         **g_clls;
static UNUM32        g_numClls;

static EVT_NODE     *g_evHead, *g_evTail;  /* one event queue for module and all CLLs */

static SW_FILTER    *g_filters;
static UNUM32        g_numFilters;

static void        **g_items;            /* every item handed to the host */
static UNUM32        g_numItems, g_capItems;

/* Handle counters: set at DLL load, never reset by PDUConstruct/PDUDestruct. */
static UNUM32        g_nextCll    = HCLL_FIRST;
static UNUM32        g_nextRxCop  = HCOP_RX_FIRST;
static UNUM32        g_nextTxCop  = HCOP_TX_FIRST;
static UNUM32        g_nextStart  = HCOP_STARTCOMM_FIRST;

static volatile LONG g_vbattMv = 13000;  /* [Device] VBattMilliVolts, or set by a VBATT line */

/* Configuration, read once at DLL load (load_config). */
static char  g_portName[64];          /* [Serial] Port; empty: no serial link */
static BOOL  g_serialBad;             /* [Serial] Port is a number, not a COM port name */
static char  g_tcpPortStr[16];        /* [Tcp] Port as written */
static int   g_tcpPort;               /* > 0: TCP server on this port */
static BOOL  g_tcpBad;                /* [Tcp] Port is not 1..65535 */
static char  g_tcpBind[64] = "127.0.0.1";
static char  g_tcpDesc[96];           /* "TCP 127.0.0.1:5000" or "TCP [::1]:5000" */
static char  g_linkDesc[160];         /* e.g. "COM8 + TCP 127.0.0.1:5000" */
static char  g_iniPath[MAX_PATH];
static DWORD g_baud = 115200;
static BOOL  g_logEnabled;           /* [Log] Enabled, default 0 (off) */
static char  g_logPath[MAX_PATH];
static char  g_moduleName[64] = "D-PDU Serial/TCP Bridge";
static BOOL  g_swFilters = TRUE;

static BOOL use_serial(void) { return g_portName[0] && !g_serialBad; }

/* ------------------------------------------------------------------------ */
/* Link state (serial port and/or TCP server)                                */
/* ------------------------------------------------------------------------ */

/* One queued output line, CR LF included (not NUL-terminated). */
typedef struct TX_NODE {
    struct TX_NODE *next;
    unsigned        skip;   /* TCP client (uid) that must not get this line; 0 = none */
    int             len;
    char            data[1];
} TX_NODE;

/* Output queue of one link.  Each link has its own queue and writer thread,
 * so a stalled serial port never delays the TCP clients (or vice versa). */
typedef struct {
    TX_NODE *head, *tail;               /* head, tail, count, active: guarded by g_txLock */
    UNUM32   count;
    BOOL     active;                    /* link open: lines are queued */
    HANDLE   evt;                       /* auto-reset: lines queued */
    HANDLE   thread;                    /* writer thread */
} TX_QUEUE;

static BOOL     g_linkOpen;           /* serial port open and/or TCP server listening */
static HANDLE   g_port = INVALID_HANDLE_VALUE;
static HANDLE   g_rxThread;           /* serial reader */
static HANDLE   g_netThread;          /* TCP accept/read */
static HANDLE   g_stopEvt;            /* manual-reset: stops all worker threads */
static TX_QUEUE g_serialQ, g_tcpQ;

#define TX_QUEUE_MAX      2000         /* lines; further lines are dropped */
#define TX_WRITE_TIMEOUT  250          /* ms; com0com stalls writes while the other end is closed */
#define TCP_MAX_CLIENTS   32

typedef struct {
    SOCKET        s;
    unsigned      uid;     /* unique per connection (echo suppression) */
    volatile BOOL dead;    /* send failed: closed by the network thread */
    char          peer[64];  /* "addr:port" label for echoes and notices */
    size_t        len;       /* bytes of the partial line in buf */
    char          buf[MAX_FRAME * 4];
} TCP_CLIENT;

static SOCKET      g_listenSock = INVALID_SOCKET;
static TCP_CLIENT *g_clients[TCP_MAX_CLIENTS];   /* slots written only by the network thread */
static unsigned     g_nextClientUid;

static LARGE_INTEGER g_qpf;              /* QueryPerformanceFrequency */

/* ------------------------------------------------------------------------ */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------ */

static UNUM32 now_us(void)
{
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    /* Microseconds since boot, truncated to 32 bits. */
    return (UNUM32)(unsigned __int64)((double)c.QuadPart * 1000000.0 / (double)g_qpf.QuadPart);
}

/* Appends one timestamped line to the log.  The file is opened per line so it
 * can be viewed, truncated or deleted while the DLL is loaded. */
static void log_msg(const char *fmt, ...)
{
    FILE *f;
    va_list ap;
    SYSTEMTIME st;
    if (!g_logEnabled || !g_logPath[0])
        return;
    EnterCriticalSection(&g_logLock);
    f = fopen(g_logPath, "a");
    if (f) {
        GetLocalTime(&st);
        fprintf(f, "[%02u:%02u:%02u.%03u] ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
        va_start(ap, fmt);
        vfprintf(f, fmt, ap);
        va_end(ap);
        fputc('\n', f);
        fclose(f);
    }
    LeaveCriticalSection(&g_logLock);
}

/* "01 02 0A"; output that does not fit in outSize is truncated. */
static void hex_bytes(char *out, size_t outSize, const UNUM8 *p, UNUM32 n)
{
    size_t pos = 0;
    UNUM32 i;
    out[0] = 0;
    for (i = 0; i < n && pos + 4 < outSize; i++)
        pos += sprintf(out + pos, i ? " %02X" : "%02X", p[i]);
}

static UNUM32 be32(const UNUM8 *p)
{
    return ((UNUM32)p[0] << 24) | ((UNUM32)p[1] << 16) | ((UNUM32)p[2] << 8) | p[3];
}

static void put_be32(UNUM8 *p, UNUM32 v)
{
    p[0] = (UNUM8)(v >> 24); p[1] = (UNUM8)(v >> 16); p[2] = (UNUM8)(v >> 8); p[3] = (UNUM8)v;
}

/* Every item returned to the host is registered so PDUDestroyItem only frees
 * memory that this DLL actually allocated. */
static void *item_alloc(size_t size)
{
    void *p = calloc(1, size);
    if (!p)
        return NULL;
    if (g_numItems == g_capItems) {
        UNUM32 cap = g_capItems ? g_capItems * 2 : 64;
        void **n = (void **)realloc(g_items, cap * sizeof *n);
        if (!n) { free(p); return NULL; }
        g_items = n;
        g_capItems = cap;
    }
    g_items[g_numItems++] = p;
    return p;
}

static BOOL item_free(void *p)
{
    UNUM32 i;
    for (i = 0; i < g_numItems; i++) {
        if (g_items[i] == p) {
            g_items[i] = g_items[--g_numItems];
            free(p);
            return TRUE;
        }
    }
    return FALSE;
}

static CLL *find_cll(UNUM32 hCLL)
{
    UNUM32 i;
    for (i = 0; i < g_numClls; i++)
        if (g_clls[i]->hCLL == hCLL)
            return g_clls[i];
    return NULL;
}

static void cll_free(CLL *c)
{
    free(c->expResp);
    free(c->urid);
    free(c);
}

/* Caller holds g_lock.  ev may be NULL (allocation failed): nothing is queued. */
static void enqueue_event(PDU_EVENT_ITEM *ev)
{
    EVT_NODE *n;
    if (!ev)
        return;
    n = (EVT_NODE *)malloc(sizeof *n);
    if (!n) {
        item_free(ev);
        return;
    }
    n->item = ev;
    n->next = NULL;
    if (g_evTail) g_evTail->next = n; else g_evHead = n;
    g_evTail = n;
}

/* Frees queued events the host has not fetched. */
static void clear_events(void)
{
    while (g_evHead) {
        EVT_NODE *n = g_evHead;
        g_evHead = n->next;
        item_free(n->item);
        free(n);
    }
    g_evTail = NULL;
}

/* PDU_IT_STATUS event: pData points at a single UNUM32 status code. */
static PDU_EVENT_ITEM *make_status_event(UNUM32 hCop, void *tag, UNUM32 ts, UNUM32 status)
{
    PDU_EVENT_ITEM *ev = (PDU_EVENT_ITEM *)item_alloc(sizeof(PDU_EVENT_ITEM) + sizeof(UNUM32));
    if (!ev)
        return NULL;
    ev->ItemType = PDU_IT_STATUS;
    ev->hCop = hCop;
    ev->pCoPTag = tag;
    ev->Timestamp = ts;
    ev->pData = ev + 1;
    *(UNUM32 *)ev->pData = status;
    return ev;
}

/* PDU_IT_RESULT event; the whole tree lives in one allocation. */
static PDU_EVENT_ITEM *make_result_event(UNUM32 hCop, void *tag, const UNUM8 rxFlag[4],
                                         UNUM32 urid, UNUM32 acceptanceId,
                                         const UNUM8 *data, UNUM32 len)
{
    size_t size = sizeof(PDU_EVENT_ITEM) + sizeof(PDU_RESULT_DATA) + sizeof(PDU_EXTRA_INFO) + 8 + len;
    UNUM8 *base = (UNUM8 *)item_alloc(size);
    PDU_EVENT_ITEM *ev;
    PDU_RESULT_DATA *res;
    PDU_EXTRA_INFO *extra;
    UNUM8 *flags;
    if (!base)
        return NULL;
    ev = (PDU_EVENT_ITEM *)base;
    res = (PDU_RESULT_DATA *)(ev + 1);
    extra = (PDU_EXTRA_INFO *)(res + 1);
    flags = (UNUM8 *)(extra + 1);
    memcpy(flags, rxFlag, 4);             /* RxFlag bytes        */
    memset(flags + 4, 0, 4);              /* TimestampFlags bytes */
    res->RxFlag.NumFlagBytes = 4;
    res->RxFlag.pFlagData = flags;
    res->UniqueRespIdentifier = urid;
    res->AcceptanceId = acceptanceId;
    res->TimestampFlags.NumFlagBytes = 4;
    res->TimestampFlags.pFlagData = flags + 4;
    res->TxMsgDoneTimestamp = 0;
    res->StartMsgTimestamp = 0;
    res->pExtraInfo = extra;
    res->NumDataBytes = len;
    res->pDataBytes = len ? flags + 8 : NULL;
    if (len)
        memcpy(flags + 8, data, len);
    ev->ItemType = PDU_IT_RESULT;
    ev->hCop = hCop;
    ev->pCoPTag = tag;
    ev->Timestamp = now_us();
    ev->pData = res;
    return ev;
}

/* Callback for a CLL event: registered hMod/hCLL, CLL tag and API tag.
 * Nothing is called when no callback is registered. */
static CB_CALL cll_callback(const CLL *c)
{
    CB_CALL cb;
    cb.fn = c->cb.fn;
    cb.evt = PDU_EVT_DATA_AVAILABLE;
    cb.hMod = c->cb.hMod;
    cb.hCLL = c->cb.hCLL;
    cb.cllTag = c->pCllTag;
    cb.apiTag = g_apiTag;
    return cb;
}

static void fire(const CB_CALL *cb)
{
    if (cb->fn)
        cb->fn(cb->evt, cb->hMod, cb->hCLL, cb->cllTag, cb->apiTag);
}

/* ------------------------------------------------------------------------ */
/* Configuration                                                             */
/* ------------------------------------------------------------------------ */

/* Reads the ini once, at DLL load.  The ini named after the DLL
 * (dpdu-bridge-c.dll -> dpdu-bridge-c.ini) is used if it exists, else the
 * dpdu-bridge.ini shared with the C# version, both next to the DLL.  The
 * default log file is named after the DLL (dpdu-bridge-c.log). */
static void load_config(void)
{
    char base[MAX_PATH], ini[MAX_PATH], buf[MAX_PATH];
    char *name, *dot;
    GetModuleFileNameA(g_hInst, base, sizeof base);
    base[sizeof base - 1] = 0;
    name = strrchr(base, '\\');
    name = name ? name + 1 : base;
    dot = strrchr(name, '.');
    if (dot) *dot = 0;
    _snprintf(ini, sizeof ini, "%s.ini", base);
    ini[sizeof ini - 1] = 0;
    if (GetFileAttributesA(ini) == INVALID_FILE_ATTRIBUTES) {
        _snprintf(ini, sizeof ini, "%.*sdpdu-bridge.ini", (int)(name - base), base);
        ini[sizeof ini - 1] = 0;
    }

    strncpy(g_iniPath, ini, sizeof g_iniPath - 1);
    /* A missing or empty value gives the default (same rules as the C#
     * version).  There are no default ports: [Serial] Port names a COM port
     * and [Tcp] Port a TCP port number; either or both can be used. */
    GetPrivateProfileStringA("Serial", "Port", "", g_portName, sizeof g_portName, ini);
    g_serialBad = g_portName[0] && strspn(g_portName, "0123456789") == strlen(g_portName);
    GetPrivateProfileStringA("Tcp", "Port", "", g_tcpPortStr, sizeof g_tcpPortStr, ini);
    g_tcpPort = 0;
    g_tcpBad = FALSE;
    if (g_tcpPortStr[0]) {
        long v = strspn(g_tcpPortStr, "0123456789") == strlen(g_tcpPortStr) && strlen(g_tcpPortStr) <= 5
                 ? atol(g_tcpPortStr) : 0;
        if (v >= 1 && v <= 65535)
            g_tcpPort = (int)v;
        else
            g_tcpBad = TRUE;
    }
    if (!GetPrivateProfileStringA("Tcp", "Bind", "", g_tcpBind, sizeof g_tcpBind, ini))
        strcpy(g_tcpBind, "127.0.0.1");
    /* IPv6 addresses are written as [addr]:port */
    _snprintf(g_tcpDesc, sizeof g_tcpDesc - 1, strchr(g_tcpBind, ':') ? "TCP [%s]:%d" : "TCP %s:%d",
              g_tcpBind, g_tcpPort);
    g_linkDesc[0] = 0;
    if (use_serial())
        strncpy(g_linkDesc, g_portName, sizeof g_linkDesc - 1);
    if (g_tcpPort)
        _snprintf(g_linkDesc + strlen(g_linkDesc), sizeof g_linkDesc - 1 - strlen(g_linkDesc),
                  "%s%s", g_linkDesc[0] ? " + " : "", g_tcpDesc);
    g_baud = GetPrivateProfileIntA("Serial", "Baud", 115200, ini);
    g_logEnabled = GetPrivateProfileIntA("Log", "Enabled", 0, ini) != 0;
    GetPrivateProfileStringA("Log", "File", "", buf, sizeof buf, ini);
    if (buf[0])
        strncpy(g_logPath, buf, sizeof g_logPath - 1);
    else
        _snprintf(g_logPath, sizeof g_logPath - 1, "%s.log", base);
    if (!GetPrivateProfileStringA("Device", "ModuleName", "", g_moduleName, sizeof g_moduleName, ini))
        strcpy(g_moduleName, "D-PDU Serial/TCP Bridge");
    g_vbattMv = GetPrivateProfileIntA("Device", "VBattMilliVolts", 13000, ini);
    g_swFilters = GetPrivateProfileIntA("Device", "SoftwareFilters", 1, ini) != 0;
}

/* ------------------------------------------------------------------------ */
/* Links: output queues, serial port, TCP server                             */
/* ------------------------------------------------------------------------ */

/* TRUE when a DOS device of that name (e.g. COM8) exists, without opening it. */
static BOOL port_exists(const char *name)
{
    char target[512];
    return QueryDosDeviceA(name, target, sizeof target) != 0;
}

/* Caller holds g_txLock.  The line is dropped when the link is closed or its
 * queue is full, so a link nobody reads cannot grow memory without bound. */
static void tx_enqueue(TX_QUEUE *q, const char *data, int n, unsigned skip)
{
    TX_NODE *node;
    if (!q->active || q->count >= TX_QUEUE_MAX || (node = (TX_NODE *)malloc(sizeof *node + n)) == NULL)
        return;
    node->next = NULL;
    node->skip = skip;
    node->len = n;
    memcpy(node->data, data, n);
    if (q->tail) q->tail->next = node; else q->head = node;
    q->tail = node;
    q->count++;
    SetEvent(q->evt);
}

static TX_NODE *tx_dequeue(TX_QUEUE *q)
{
    TX_NODE *node;
    EnterCriticalSection(&g_txLock);
    node = q->head;
    if (node) {
        q->head = node->next;
        if (!q->head) q->tail = NULL;
        q->count--;
    }
    LeaveCriticalSection(&g_txLock);
    return node;
}

/* Deactivates the queue and frees the lines still in it. */
static void tx_queue_clear(TX_QUEUE *q)
{
    TX_NODE *n;
    EnterCriticalSection(&g_txLock);
    q->active = FALSE;
    LeaveCriticalSection(&g_txLock);
    while ((n = tx_dequeue(q)) != NULL)
        free(n);
}

/* Queues a line for the serial port (if toSerial) and every TCP client except
 * the one with uid skipClient; never blocks the API caller. */
static void link_vwrite(BOOL toSerial, unsigned skipClient, const char *fmt, va_list ap)
{
    char line[MAX_FRAME * 3 + 128];
    int n = _vsnprintf(line, sizeof line - 3, fmt, ap);
    if (n < 0) {
        line[sizeof line - 3] = 0;
        n = (int)strlen(line);
    }
    log_msg(">> %.*s", n, line);
    line[n++] = '\r';
    line[n++] = '\n';

    EnterCriticalSection(&g_txLock);
    if (g_linkOpen) {
        if (toSerial)
            tx_enqueue(&g_serialQ, line, n, 0);
        tx_enqueue(&g_tcpQ, line, n, skipClient);
    }
    LeaveCriticalSection(&g_txLock);
}

/* Queues a line for every open link (serial port and TCP clients). */
static void link_write_line(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    link_vwrite(TRUE, 0, fmt, ap);
    va_end(ap);
}

/* Queues a line for every link except the client a received line came from:
 * fromSerial = the serial port, else the TCP client with uid fromClient. */
static void link_write_except(BOOL fromSerial, unsigned fromClient, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    link_vwrite(!fromSerial, fromSerial ? 0 : fromClient, fmt, ap);
    va_end(ap);
}

/* Sends to every connected client except the one with uid skip.  A client
 * whose send fails or times out is only marked dead here; the network thread
 * closes it. */
static void tcp_broadcast(const char *data, int len, unsigned skip)
{
    int i;
    EnterCriticalSection(&g_clientLock);
    for (i = 0; i < TCP_MAX_CLIENTS; i++) {
        TCP_CLIENT *c = g_clients[i];
        int sent = 0;
        if (!c || c->dead || (skip && c->uid == skip))
            continue;
        while (sent < len) {
            int r = send(c->s, data + sent, len - sent, 0);
            if (r <= 0) {
                c->dead = TRUE;
                log_msg("%s: send failed (%d) - disconnecting", c->peer, WSAGetLastError());
                break;
            }
            sent += r;
        }
    }
    LeaveCriticalSection(&g_clientLock);
}

/* Writer thread of the TCP clients.  The thread parameter of every worker is
 * the module reference taken by start_worker(). */
static DWORD WINAPI tcp_tx_thread(LPVOID arg)
{
    HMODULE self = (HMODULE)arg;
    HANDLE waits[2];
    waits[0] = g_stopEvt;
    waits[1] = g_tcpQ.evt;
    while (WaitForMultipleObjects(2, waits, FALSE, INFINITE) != WAIT_OBJECT_0) {
        TX_NODE *node;
        while ((node = tx_dequeue(&g_tcpQ)) != NULL) {
            tcp_broadcast(node->data, node->len, node->skip);
            free(node);
            if (WaitForSingleObject(g_stopEvt, 0) == WAIT_OBJECT_0)
                break;
        }
    }
    FreeLibraryAndExitThread(self, 0);
}

/* Writer thread of the serial port.  A write that does not complete within
 * TX_WRITE_TIMEOUT is cancelled and the line dropped; only the first drop of
 * a stall is logged. */
static DWORD WINAPI serial_tx_thread(LPVOID arg)
{
    HMODULE self = (HMODULE)arg;
    OVERLAPPED ov;
    HANDLE waits[2];
    BOOL stalled = FALSE;

    memset(&ov, 0, sizeof ov);
    ov.hEvent = CreateEventA(NULL, TRUE, FALSE, NULL);
    waits[0] = g_stopEvt;
    waits[1] = g_serialQ.evt;
    while (WaitForMultipleObjects(2, waits, FALSE, INFINITE) != WAIT_OBJECT_0) {
        TX_NODE *node;
        while ((node = tx_dequeue(&g_serialQ)) != NULL) {
            DWORD written = 0;
            BOOL done;
            ResetEvent(ov.hEvent);
            done = WriteFile(g_port, node->data, (DWORD)node->len, &written, &ov);
            if (!done && GetLastError() == ERROR_IO_PENDING) {
                if (WaitForSingleObject(ov.hEvent, TX_WRITE_TIMEOUT) == WAIT_OBJECT_0) {
                    done = GetOverlappedResult(g_port, &ov, &written, FALSE);
                } else {
                    /* Nobody is reading the other end of the pair: drop it. */
                    CancelIo(g_port);
                    GetOverlappedResult(g_port, &ov, &written, TRUE);
                    PurgeComm(g_port, PURGE_TXABORT | PURGE_TXCLEAR);
                    done = FALSE;
                }
            }
            if (!done && !stalled)
                log_msg("serial output dropped - is the simulator reading the other end of %s?", g_portName);
            stalled = !done;
            free(node);
            if (WaitForSingleObject(g_stopEvt, 0) == WAIT_OBJECT_0)
                break;
        }
    }
    CloseHandle(ov.hEvent);
    FreeLibraryAndExitThread(self, 0);
}

static void process_line(char *line);

/* Splits received bytes into lines; calls fn(ctx, line) for each non-empty
 * one.  buf/len carry a partial line across calls; bytes beyond cap - 1 of an
 * overlong line are discarded. */
static void split_lines(char *buf, size_t *len, size_t cap, const char *data, size_t n,
                        void (*fn)(void *, char *), void *ctx)
{
    size_t i;
    for (i = 0; i < n; i++) {
        char ch = data[i];
        if (ch == '\n' || ch == '\r') {
            if (*len) {
                buf[*len] = 0;
                fn(ctx, buf);
                *len = 0;
            }
        } else if (*len < cap - 1) {
            buf[(*len)++] = ch;
        }
    }
}

/* A line from the serial port: echoed as a comment to the other clients (the
 * TCP clients), so every client sees the complete traffic, then delivered to
 * the host. */
static void serial_line(void *ctx, char *line)
{
    (void)ctx;
    while (*line == ' ' || *line == '\t')
        line++;
    if (!*line)
        return;
    link_write_except(TRUE, 0, "# %s: %s", g_portName, line);
    process_line(line);
}

/* Reader thread of the serial port.  Overlapped reads so g_stopEvt can
 * interrupt a pending read.  lineBuf is static because there is at most one
 * serial reader at a time and it is too large for the stack. */
static DWORD WINAPI serial_rx_thread(LPVOID arg)
{
    static char lineBuf[MAX_FRAME * 4];
    size_t lineLen = 0;
    OVERLAPPED ov;
    HANDLE waits[2];
    HMODULE self = (HMODULE)arg;

    memset(&ov, 0, sizeof ov);
    ov.hEvent = CreateEventA(NULL, TRUE, FALSE, NULL);
    waits[0] = g_stopEvt;
    waits[1] = ov.hEvent;

    for (;;) {
        char buf[512];
        DWORD got = 0;
        ResetEvent(ov.hEvent);
        if (!ReadFile(g_port, buf, sizeof buf, &got, &ov)) {
            if (GetLastError() != ERROR_IO_PENDING) {
                if (WaitForSingleObject(g_stopEvt, 100) == WAIT_OBJECT_0)
                    break;
                continue;
            }
            if (WaitForMultipleObjects(2, waits, FALSE, INFINITE) == WAIT_OBJECT_0) {
                CancelIo(g_port);
                GetOverlappedResult(g_port, &ov, &got, TRUE);
                break;
            }
            if (!GetOverlappedResult(g_port, &ov, &got, FALSE))
                continue;
        }
        if (WaitForSingleObject(g_stopEvt, 0) == WAIT_OBJECT_0)
            break;
        split_lines(lineBuf, &lineLen, sizeof lineBuf, buf, got, serial_line, NULL);
    }
    CloseHandle(ov.hEvent);
    /* Release the module reference taken in start_worker(). */
    FreeLibraryAndExitThread(self, 0);
}

/* A line from a TCP client: echoed as a comment labelled with the client's
 * address to the other clients (the serial port and the other TCP clients),
 * so every client sees the complete traffic, then delivered to the host. */
static void tcp_client_line(void *ctx, char *line)
{
    TCP_CLIENT *c = (TCP_CLIENT *)ctx;
    while (*line == ' ' || *line == '\t')
        line++;
    if (!*line)
        return;
    link_write_except(FALSE, c->uid, "# %s: %s", c->peer, line);
    process_line(line);
}

/* "a.b.c.d:port" for IPv4 (including IPv4-mapped IPv6), "[v6]:port" for IPv6. */
static void format_peer(const struct sockaddr_storage *ss, char *out, size_t size)
{
    char addr[INET6_ADDRSTRLEN] = "?";
    if (ss->ss_family == AF_INET6) {
        const struct sockaddr_in6 *a6 = (const struct sockaddr_in6 *)ss;
        if (IN6_IS_ADDR_V4MAPPED(&a6->sin6_addr)) {
            inet_ntop(AF_INET, &a6->sin6_addr.s6_addr[12], addr, sizeof addr);
            _snprintf(out, size - 1, "%s:%u", addr, ntohs(a6->sin6_port));
        } else {
            inet_ntop(AF_INET6, &a6->sin6_addr, addr, sizeof addr);
            _snprintf(out, size - 1, "[%s]:%u", addr, ntohs(a6->sin6_port));
        }
    } else {
        const struct sockaddr_in *a4 = (const struct sockaddr_in *)ss;
        inet_ntop(AF_INET, &a4->sin_addr, addr, sizeof addr);
        _snprintf(out, size - 1, "%s:%u", addr, ntohs(a4->sin_port));
    }
    out[size - 1] = 0;
}

/* Network thread only.  The send timeout keeps a client that stops reading
 * from stalling the TCP writer thread for long. */
static void tcp_accept_client(void)
{
    struct sockaddr_storage peer;
    int peerLen = sizeof peer, i, slot = -1;
    DWORD timeout = TX_WRITE_TIMEOUT;
    BOOL on = TRUE;
    TCP_CLIENT *c;
    char name[64];
    SOCKET s = accept(g_listenSock, (struct sockaddr *)&peer, &peerLen);
    if (s == INVALID_SOCKET)
        return;
    format_peer(&peer, name, sizeof name);
    for (i = 0; i < TCP_MAX_CLIENTS && slot < 0; i++)
        if (!g_clients[i])
            slot = i;
    c = slot >= 0 ? (TCP_CLIENT *)calloc(1, sizeof *c) : NULL;
    if (!c) {
        static const char full[] = "# too many clients\r\n";
        send(s, full, (int)sizeof full - 1, 0);
        closesocket(s);
        log_msg("TCP connection from %s refused - too many clients", name);
        return;
    }
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout, sizeof timeout);
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char *)&on, sizeof on);
    c->s = s;
    c->uid = ++g_nextClientUid ? g_nextClientUid : ++g_nextClientUid;   /* never 0 */
    strcpy(c->peer, name);
    EnterCriticalSection(&g_clientLock);
    g_clients[slot] = c;
    LeaveCriticalSection(&g_clientLock);
    link_write_line("# %s connected", c->peer);
}

/* Network thread only.  The slot is cleared under g_clientLock, so the
 * writer thread is never inside a send on the socket being closed. */
static void tcp_drop_client(int slot)
{
    TCP_CLIENT *c;
    EnterCriticalSection(&g_clientLock);
    c = g_clients[slot];
    g_clients[slot] = NULL;
    LeaveCriticalSection(&g_clientLock);
    if (!c)
        return;
    closesocket(c->s);
    link_write_line("# %s disconnected", c->peer);
    free(c);
}

/* TCP network thread: accepts clients and reads their lines.  It is the only
 * thread that adds, removes or closes clients. */
static DWORD WINAPI tcp_thread(LPVOID arg)
{
    HMODULE self = (HMODULE)arg;
    int i;
    while (WaitForSingleObject(g_stopEvt, 0) != WAIT_OBJECT_0) {
        fd_set rd;
        struct timeval tv;
        FD_ZERO(&rd);
        FD_SET(g_listenSock, &rd);
        for (i = 0; i < TCP_MAX_CLIENTS; i++) {
            if (g_clients[i] && g_clients[i]->dead)
                tcp_drop_client(i);
            if (g_clients[i])
                FD_SET(g_clients[i]->s, &rd);
        }
        tv.tv_sec = 0;
        tv.tv_usec = 200000;
        i = select(0, &rd, NULL, NULL, &tv);
        if (i <= 0) {
            if (i < 0)
                Sleep(50);
            continue;
        }
        if (FD_ISSET(g_listenSock, &rd))
            tcp_accept_client();
        for (i = 0; i < TCP_MAX_CLIENTS; i++) {
            TCP_CLIENT *c = g_clients[i];
            char buf[1024];
            int r;
            if (!c || !FD_ISSET(c->s, &rd))
                continue;
            r = recv(c->s, buf, sizeof buf, 0);
            if (r <= 0)
                tcp_drop_client(i);
            else
                split_lines(c->buf, &c->len, sizeof c->buf, buf, (size_t)r, tcp_client_line, c);
        }
    }
    for (i = 0; i < TCP_MAX_CLIENTS; i++)
        tcp_drop_client(i);
    FreeLibraryAndExitThread(self, 0);
}

/* Creates g_listenSock.  On success Winsock stays initialised until
 * link_close(); on failure everything is cleaned up here. */
static BOOL tcp_listen(void)
{
    WSADATA wsa;
    struct sockaddr_storage a;
    int aLen;
    BOOL on = TRUE, off = FALSE;
    SOCKET s;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        log_msg("WSAStartup failed");
        return FALSE;
    }
    /* [Tcp] Bind is an IPv4 or an IPv6 address. */
    memset(&a, 0, sizeof a);
    if (inet_pton(AF_INET, g_tcpBind, &((struct sockaddr_in *)&a)->sin_addr) == 1) {
        a.ss_family = AF_INET;
        ((struct sockaddr_in *)&a)->sin_port = htons((u_short)g_tcpPort);
        aLen = sizeof(struct sockaddr_in);
    } else if (inet_pton(AF_INET6, g_tcpBind, &((struct sockaddr_in6 *)&a)->sin6_addr) == 1) {
        a.ss_family = AF_INET6;
        ((struct sockaddr_in6 *)&a)->sin6_port = htons((u_short)g_tcpPort);
        aLen = sizeof(struct sockaddr_in6);
    } else {
        log_msg("invalid [Tcp] Bind address '%s'", g_tcpBind);
        WSACleanup();
        return FALSE;
    }
    s = socket(a.ss_family, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET) {
        log_msg("cannot create socket (%d)", WSAGetLastError());
        WSACleanup();
        return FALSE;
    }
    setsockopt(s, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, (const char *)&on, sizeof on);
    /* An IPv6 socket bound to "::" also accepts IPv4 clients (dual stack). */
    if (a.ss_family == AF_INET6)
        setsockopt(s, IPPROTO_IPV6, IPV6_V6ONLY, (const char *)&off, sizeof off);
    if (bind(s, (struct sockaddr *)&a, aLen) != 0 || listen(s, SOMAXCONN) != 0) {
        log_msg("cannot listen on %s (error %d)", g_tcpDesc, WSAGetLastError());
        closesocket(s);
        WSACleanup();
        return FALSE;
    }
    g_listenSock = s;
    return TRUE;
}

/* Opens g_port for overlapped I/O, 8N1 with no flow control.  Errors from the
 * comm-state calls are ignored: virtual ports such as com0com accept anything. */
static BOOL serial_setup(void)
{
    char path[80];
    DCB dcb;
    COMMTIMEOUTS to;
    _snprintf(path, sizeof path - 1, "\\\\.\\%s", g_portName);
    path[sizeof path - 1] = 0;
    g_port = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
                         FILE_FLAG_OVERLAPPED, NULL);
    if (g_port == INVALID_HANDLE_VALUE) {
        log_msg("cannot open %s (error %lu)", g_portName, GetLastError());
        return FALSE;
    }
    memset(&dcb, 0, sizeof dcb);
    dcb.DCBlength = sizeof dcb;
    GetCommState(g_port, &dcb);
    dcb.BaudRate = g_baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    SetCommState(g_port, &dcb);
    memset(&to, 0, sizeof to);
    to.ReadIntervalTimeout = MAXDWORD;          /* return as soon as data arrives */
    to.ReadTotalTimeoutMultiplier = MAXDWORD;
    to.ReadTotalTimeoutConstant = 1000;
    to.WriteTotalTimeoutConstant = 1000;
    SetCommTimeouts(g_port, &to);
    PurgeComm(g_port, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return TRUE;
}

static void link_close(void);

static HANDLE start_worker(LPTHREAD_START_ROUTINE fn)
{
    HMODULE self = NULL;
    HANDLE t;
    /* Each worker thread holds a module reference so the DLL cannot be
     * unloaded underneath it; the reference is dropped when it exits. */
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, (LPCSTR)fn, &self);
    t = CreateThread(NULL, 0, fn, self, 0, NULL);
    if (!t && self)
        FreeLibrary(self);
    return t;
}

/* Opens every configured link (serial port and/or TCP server).  Succeeds when
 * at least one of them could be opened. */
static BOOL link_open(void)
{
    BOOL serialOk, tcpOk;
    if (g_linkOpen)
        return TRUE;
    if (!use_serial() && !g_tcpPort) {
        log_msg("no valid port specified - not opening a port");
        return FALSE;
    }
    serialOk = use_serial() && serial_setup();
    tcpOk = g_tcpPort && tcp_listen();
    if (!serialOk && !tcpOk)
        return FALSE;
    ResetEvent(g_stopEvt);
    EnterCriticalSection(&g_txLock);
    g_serialQ.active = serialOk;
    g_tcpQ.active = tcpOk;
    g_linkOpen = TRUE;
    LeaveCriticalSection(&g_txLock);
    g_rxThread = serialOk ? start_worker(serial_rx_thread) : NULL;
    g_serialQ.thread = serialOk ? start_worker(serial_tx_thread) : NULL;
    g_netThread = tcpOk ? start_worker(tcp_thread) : NULL;
    g_tcpQ.thread = tcpOk ? start_worker(tcp_tx_thread) : NULL;
    if ((serialOk && (!g_rxThread || !g_serialQ.thread)) || (tcpOk && (!g_netThread || !g_tcpQ.thread))) {
        log_msg("cannot start worker threads");
        link_close();
        return FALSE;
    }
    if (serialOk)
        log_msg("opened %s at %lu baud", g_portName, g_baud);
    if (tcpOk)
        log_msg("listening on %s", g_tcpDesc);
    return TRUE;
}

/* Waits (bounded) for a worker to exit after g_stopEvt is set.  A worker that
 * calls this on itself (the host disconnecting from inside a callback fired by
 * a reader thread) is not waited for: it exits once it returns to its loop. */
static void stop_thread(HANDLE *t)
{
    if (!*t)
        return;
    if (GetCurrentThreadId() != GetThreadId(*t))
        WaitForSingleObject(*t, 3000);
    CloseHandle(*t);
    *t = NULL;
}

/* Stops all workers and closes every link.  Must not be called with g_lock
 * held: a reader thread may be waiting for it. */
static void link_close(void)
{
    DWORD t0 = GetTickCount();
    if (!g_linkOpen)
        return;
    /* Give the writers a moment to flush queued lines (e.g. "BYE"). */
    while (((g_serialQ.thread && g_serialQ.count) || (g_tcpQ.thread && g_tcpQ.count)) &&
           GetTickCount() - t0 < 2 * TX_WRITE_TIMEOUT)
        Sleep(5);
    SetEvent(g_stopEvt);
    stop_thread(&g_rxThread);
    stop_thread(&g_netThread);  /* the TCP thread disconnects all clients */
    stop_thread(&g_serialQ.thread);
    stop_thread(&g_tcpQ.thread);
    EnterCriticalSection(&g_txLock);
    g_linkOpen = FALSE;
    LeaveCriticalSection(&g_txLock);
    tx_queue_clear(&g_serialQ);
    tx_queue_clear(&g_tcpQ);
    if (g_listenSock != INVALID_SOCKET) {
        closesocket(g_listenSock);
        g_listenSock = INVALID_SOCKET;
        WSACleanup();
        log_msg("closed %s", g_tcpDesc);
    }
    if (g_port != INVALID_HANDLE_VALUE) {
        CloseHandle(g_port);
        g_port = INVALID_HANDLE_VALUE;
        log_msg("closed %s", g_portName);
    }
}

/* ------------------------------------------------------------------------ */
/* Receive path (link lines -> events)                                       */
/* ------------------------------------------------------------------------ */

/* Caller holds g_lock.  A frame matching any BLOCK filter is dropped; if any
 * PASS filter exists for the bus, the frame must match one of them. */
static BOOL sw_filter_pass(BUS bus, const UNUM8 *frame, UNUM32 len)
{
    UNUM32 i, n = bus == BUS_ALDL ? 1 : 3;
    BOOL anyPass = FALSE, passed = FALSE;
    if (!g_swFilters)
        return TRUE;
    for (i = 0; i < g_numFilters; i++) {
        const SW_FILTER *f = &g_filters[i];
        UNUM32 k;
        BOOL match = TRUE;
        if (f->bus != bus)
            continue;
        for (k = 0; k < n; k++) {
            UNUM8 b = k < len ? frame[k] : 0;
            if ((b & f->mask[k]) != (f->pattern[k] & f->mask[k])) { match = FALSE; break; }
        }
        if (f->type == PDU_FLT_PASS || f->type == PDU_FLT_PASS_UUDT) {
            anyPass = TRUE;
            if (match) passed = TRUE;
        } else if (match) {
            return FALSE;
        }
    }
    return !anyPass || passed;
}

/* Caller holds g_lock.  NULL when the CLL can take a result event; otherwise
 * why not, worded to precede "logical link <hCLL>".  Results are only queued
 * while a receive ComPrimitive is running and the host has registered an
 * event callback for the CLL, so a host that is not listening cannot make
 * the event queue grow without bound. */
static const char *rx_unavailable(const CLL *c)
{
    if (c->rxCopStatus != PDU_COPST_EXECUTING)
        return "no receive ComPrimitive running on";
    if (!c->cb.fn)
        return "no event callback registered for";
    return NULL;
}

/* VPW / ALDL frame from the simulator.  The event goes to the receive
 * ComPrimitive of the first CLL on that bus, with UniqueRespIdentifier =
 * ID_UNDEF and AcceptanceId = 1. */
static void deliver_uart(BUS bus, const UNUM8 *frame, UNUM32 len)
{
    static const UNUM8 flags[4] = {0, 0, 0, 0};
    CB_CALL cb;
    CLL *c = NULL;
    const char *why;
    UNUM32 i;

    EnterCriticalSection(&g_lock);
    for (i = 0; i < g_numClls; i++)
        if (g_clls[i]->bus == bus) { c = g_clls[i]; break; }
    if (!c || g_curBus != bus) {
        BUS cur = g_curBus;
        LeaveCriticalSection(&g_lock);
        log_msg("RX %s dropped: no logical link on the active bus (%s)", bus_name(bus), bus_name(cur));
        return;
    }
    if ((why = rx_unavailable(c)) != NULL) {
        UNUM32 h = c->hCLL;
        LeaveCriticalSection(&g_lock);
        log_msg("RX %s dropped: %s logical link %08X", bus_name(bus), why, h);
        return;
    }
    if (!sw_filter_pass(bus, frame, len)) {
        LeaveCriticalSection(&g_lock);
        log_msg("RX %s dropped by message filter", bus_name(bus));
        return;
    }
    enqueue_event(make_result_event(c->hRxCop, c->rxCopTag, flags, PDU_ID_UNDEF, 1, frame, len));
    cb = cll_callback(c);
    LeaveCriticalSection(&g_lock);
    fire(&cb);
}

/* CAN message from the simulator: matched against the unique response id
 * table (USDT id for format 5, UUDT id for format 0) and then against the
 * expected responses of the receive ComPrimitive.  The first matching CLL is
 * the target; if it cannot take the result (rx_unavailable) the message is
 * dropped, not offered to another CLL. */
static void deliver_can(BUS bus, UNUM32 canId, const UNUM8 *payload, UNUM32 len)
{
    static const UNUM8 usdtFlags[4] = {0x00, 0x00, 0x00, 0x40};
    static const UNUM8 uudtFlags[4] = {0x80, 0x00, 0x00, 0x00};
    UNUM8 data[MAX_FRAME];
    CB_CALL cb;
    CLL *hit = NULL;
    const char *why;
    UNUM32 i, k, e, urid = 0, acc = 0, fmt = 0;

    if (len > MAX_FRAME - 4)
        len = MAX_FRAME - 4;
    EnterCriticalSection(&g_lock);
    if (g_curBus != bus) {
        BUS cur = g_curBus;
        LeaveCriticalSection(&g_lock);
        log_msg("RX %s %X dropped: active bus is %s", bus_name(bus), canId, bus_name(cur));
        return;
    }
    for (i = 0; i < g_numClls && !hit; i++) {
        CLL *c = g_clls[i];
        if (c->bus != bus)
            continue;
        for (k = 0; k < c->numUrid && !hit; k++) {
            const URID_ENTRY *u = &c->urid[k];
            BOOL idMatch;
            if (u->respFormat == 5)
                idMatch = u->respUsdtId == canId;
            else if (u->respFormat == 0)
                idMatch = u->respUudtId == canId;
            else
                continue; /* unsupported response format */
            if (!idMatch)
                continue;
            for (e = 0; e < c->numExpResp; e++) {
                const EXP_RESP *x = &c->expResp[e];
                if ((x->pattern & x->mask) == (canId & x->mask)) {
                    hit = c;
                    urid = u->uniqueRespId;
                    fmt = u->respFormat;
                    acc = x->acceptanceId;
                    break;
                }
            }
        }
    }
    if (!hit) {
        LeaveCriticalSection(&g_lock);
        log_msg("RX %s %X: match not found (no unique response id / expected response)", bus_name(bus), canId);
        return;
    }
    if ((why = rx_unavailable(hit)) != NULL) {
        UNUM32 h = hit->hCLL;
        LeaveCriticalSection(&g_lock);
        log_msg("RX %s %X dropped: %s logical link %08X", bus_name(bus), canId, why, h);
        return;
    }
    put_be32(data, canId);
    memcpy(data + 4, payload, len);
    enqueue_event(make_result_event(hit->hRxCop, hit->rxCopTag, fmt ? usdtFlags : uudtFlags,
                                    urid, acc, data, len + 4));
    cb = cll_callback(hit);
    LeaveCriticalSection(&g_lock);
    fire(&cb);
}

static BUS parse_bus(const char *s)
{
    if (!_stricmp(s, "HSCAN") || !_stricmp(s, "CAN")) return BUS_HSCAN;
    if (!_stricmp(s, "SWCAN") || !_stricmp(s, "GMLAN")) return BUS_SWCAN;
    if (!_stricmp(s, "VPW") || !_stricmp(s, "J1850")) return BUS_VPW;
    if (!_stricmp(s, "ALDL") || !_stricmp(s, "UART")) return BUS_ALDL;
    return BUS_NONE;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)toupper((unsigned char)c);
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Parses hex byte tokens ("01 02 0A", "01020A" or "0x0A"); returns the byte
 * count, or -1 on bad input or more than max bytes.  Modifies s.  strtok_s
 * because the serial and TCP reader threads parse concurrently. */
static int parse_hex_bytes(char *s, UNUM8 *out, int max)
{
    int n = 0;
    char *ctx = NULL;
    char *tok = strtok_s(s, " \t,", &ctx);
    while (tok) {
        size_t len = strlen(tok), j;
        if (len > 2 && tok[0] == '0' && (tok[1] == 'x' || tok[1] == 'X')) { tok += 2; len -= 2; }
        if (len == 0) return -1;
        if (len <= 2) {
            int v = 0;
            for (j = 0; j < len; j++) { int h = hexval(tok[j]); if (h < 0) return -1; v = v * 16 + h; }
            if (n >= max) return -1;
            out[n++] = (UNUM8)v;
        } else {
            if (len & 1) return -1;
            for (j = 0; j < len; j += 2) {
                int hi = hexval(tok[j]), lo = hexval(tok[j + 1]);
                if (hi < 0 || lo < 0 || n >= max) return -1;
                out[n++] = (UNUM8)(hi * 16 + lo);
            }
        }
        tok = strtok_s(NULL, " \t,", &ctx);
    }
    return n;
}

/* Handles one line from a client (reader threads only): RX frames become
 * result events, VBATT sets the battery voltage, '#' comments are ignored. */
static void process_line(char *line)
{
    char *p = line, *cmd, *rest;
    UNUM8 bytes[MAX_FRAME];
    int n;

    while (*p == ' ' || *p == '\t') p++;
    if (!*p || *p == '#')
        return;
    log_msg("<< %s", p);
    cmd = p;
    while (*p && *p != ' ' && *p != '\t') p++;
    if (*p) *p++ = 0;
    rest = p;

    if (!_stricmp(cmd, "RX")) {
        char *busTok, *idTok;
        BUS bus;
        while (*rest == ' ' || *rest == '\t') rest++;
        busTok = rest;
        while (*rest && *rest != ' ' && *rest != '\t') rest++;
        if (*rest) *rest++ = 0;
        bus = parse_bus(busTok);
        if (bus == BUS_NONE) {
            log_msg("RX: unknown bus '%s'", busTok);
            return;
        }
        if (bus_is_can(bus)) {
            UNUM32 canId;
            char *end;
            while (*rest == ' ' || *rest == '\t') rest++;
            idTok = rest;
            while (*rest && *rest != ' ' && *rest != '\t') rest++;
            if (*rest) *rest++ = 0;
            canId = strtoul(idTok, &end, 16);
            if (!*idTok || *end) {
                log_msg("RX: bad CAN id '%s'", idTok);
                return;
            }
            n = parse_hex_bytes(rest, bytes, MAX_FRAME - 4);
            if (n < 0) { log_msg("RX: bad data bytes"); return; }
            deliver_can(bus, canId, bytes, (UNUM32)n);
        } else {
            n = parse_hex_bytes(rest, bytes, MAX_FRAME);
            if (n <= 0) { log_msg("RX: bad data bytes"); return; }
            deliver_uart(bus, bytes, (UNUM32)n);
        }
    } else if (!_stricmp(cmd, "VBATT")) {
        InterlockedExchange(&g_vbattMv, (LONG)strtoul(rest, NULL, 10));
    } else {
        log_msg("unknown command '%s' ignored", cmd);
    }
}

/* ------------------------------------------------------------------------ */
/* Reset                                                                     */
/* ------------------------------------------------------------------------ */

/* Caller holds g_lock.  Items already handed to the host stay registered so
 * a later PDUDestroyItem still frees them. */
static void reset_state(void)
{
    UNUM32 i;
    for (i = 0; i < g_numClls; i++)
        cll_free(g_clls[i]);
    free(g_clls);
    g_clls = NULL;
    g_numClls = 0;
    clear_events();
    free(g_filters);
    g_filters = NULL;
    g_numFilters = 0;
    g_moduleStatus = PDU_MODST_AVAIL;
    g_curBus = BUS_NONE;
    memset(&g_sysCb, 0, sizeof g_sysCb);
    memset(&g_modCb, 0, sizeof g_modCb);
    g_moduleListed = FALSE;
}

/* ------------------------------------------------------------------------ */
/* Exported API                                                              */
/* ------------------------------------------------------------------------ */

/* Shown (outside the state lock, the box is modal) when the ini file names
 * neither a usable serial port nor a usable TCP port. */
static void report_port_problem(void)
{
    char msg[MAX_PATH + 768], notes[256] = "";
    if (g_serialBad)
        _snprintf(notes, sizeof notes - 1, "[Serial] Port=%s is not a COM port name.\n", g_portName);
    if (g_tcpBad)
        _snprintf(notes + strlen(notes), sizeof notes - 1 - strlen(notes),
                  "[Tcp] Port=%s is not a valid TCP port number (1-65535).\n", g_tcpPortStr);
    _snprintf(msg, sizeof msg - 1,
              "No COM port or TCP port is specified in the ini file:\n\n%s\n\n%s%s"
              "Add a COM port, a TCP port number to listen on, or both, for example:\n\n"
              "[Serial]\nPort=COM8\n\n[Tcp]\nPort=5000\n\n"
              "No port will be opened.", g_iniPath, notes, notes[0] ? "\n" : "");
    msg[sizeof msg - 1] = 0;
    MessageBoxA(NULL, msg, DRIVER_NAME, MB_OK | MB_ICONERROR | MB_TOPMOST | MB_SETFOREGROUND);
}

T_PDU_ERROR PDU_API_CALL PDUConstruct(CHAR8 *OptionStr, void *pAPITag)
{
    T_PDU_ERROR ret;
    BOOL badPort = FALSE;
    EnterCriticalSection(&g_lock);
    log_msg("PDUConstruct Called: pszOption=%s pAPITag=%p", OptionStr ? OptionStr : "(null)", pAPITag);
    g_apiTag = pAPITag;
    if (g_constructed) {
        ret = PDU_ERR_SHARING_VIOLATION;
    } else {
        reset_state();
        if (g_serialBad)
            log_msg("[Serial] Port=%s is not a COM port name - serial link disabled", g_portName);
        if (g_tcpBad)
            log_msg("[Tcp] Port=%s is not a valid TCP port number - TCP server disabled", g_tcpPortStr);
        /* The module is listed when the TCP server is configured, or when the
         * configured COM port exists. */
        if (!use_serial() && !g_tcpPort) {
            badPort = TRUE;
            log_msg("no valid port specified in %s - no module listed, no port will be opened", g_iniPath);
        } else {
            BOOL serialPresent = use_serial() && port_exists(g_portName);
            if (use_serial() && !serialPresent)
                log_msg("serial port %s not found", g_portName);
            g_moduleListed = serialPresent || g_tcpPort;
            if (!g_moduleListed)
                log_msg("no module listed");
        }
        g_constructed = TRUE;
        ret = PDU_STATUS_NOERROR;
    }
    log_msg("PDUConstruct -> %u", ret);
    LeaveCriticalSection(&g_lock);
    if (badPort)
        report_port_problem();
    return ret;
}

T_PDU_ERROR PDU_API_CALL PDUDestruct(void)
{
    BOOL was;
    EnterCriticalSection(&g_lock);
    log_msg("PDUDestruct Called");
    was = g_constructed;
    g_constructed = FALSE;
    LeaveCriticalSection(&g_lock);
    if (!was)
        return PDU_ERR_PDUAPI_NOT_CONSTRUCTED;
    link_close();
    EnterCriticalSection(&g_lock);
    reset_state();
    g_connectedMod = 0;
    LeaveCriticalSection(&g_lock);
    return PDU_STATUS_NOERROR;
}

/* Opens the links (outside g_lock, see link_close) and announces the driver
 * with a HELLO line. */
T_PDU_ERROR PDU_API_CALL PDUModuleConnect(UNUM32 hMod)
{
    BOOL ok;
    EnterCriticalSection(&g_lock);
    log_msg("PDUModuleConnect Called: hMod=%08X", hMod);
    g_connectedMod = hMod;
    if (!g_constructed) {
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_PDUAPI_NOT_CONSTRUCTED;
    }
    if (hMod != PDU_HANDLE_UNDEF && (!g_moduleListed || hMod != MODULE_HANDLE)) {
        LeaveCriticalSection(&g_lock);
        log_msg("ReturnValue: PDU_ERR_INVALID_HANDLE");
        return PDU_ERR_INVALID_HANDLE;
    }
    LeaveCriticalSection(&g_lock);

    ok = link_open();
    if (!ok) {
        log_msg("Failed connect comport - ReturnValue: PDU_ERR_FCT_FAILED");
        return PDU_ERR_FCT_FAILED;
    }
    link_write_line("HELLO " DRIVER_NAME " 1.0");
    EnterCriticalSection(&g_lock);
    g_moduleStatus = PDU_MODST_READY;
    LeaveCriticalSection(&g_lock);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUModuleDisconnect(UNUM32 hMod)
{
    log_msg("PDUModuleDisconnect Called: hMod=%08X", hMod);
    if (hMod != MODULE_HANDLE) {
        log_msg("Invalid hMod supplied");
        return PDU_ERR_INVALID_HANDLE;
    }
    link_write_line("BYE");
    link_close();
    EnterCriticalSection(&g_lock);
    g_moduleStatus = PDU_MODST_AVAIL;
    g_connectedMod = 0;
    LeaveCriticalSection(&g_lock);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDURegisterEventCallback(UNUM32 hMod, UNUM32 hCLL, CALLBACKFNC EventCallbackFunction)
{
    CB_CALL cb;
    memset(&cb, 0, sizeof cb);
    EnterCriticalSection(&g_lock);
    log_msg("PDURegisterEventCallback Called: hMod=%08X hCLL=%08X fn=%p", hMod, hCLL, EventCallbackFunction);
    if (!g_constructed) {
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_PDUAPI_NOT_CONSTRUCTED;
    }
    if (hMod == PDU_HANDLE_UNDEF) {
        if (hCLL == PDU_HANDLE_UNDEF) {
            /* Default/system callback: stored, never called. */
            g_sysCb.hMod = hMod;
            g_sysCb.hCLL = hCLL;
            g_sysCb.fn = EventCallbackFunction;
        }
        LeaveCriticalSection(&g_lock);
        return PDU_STATUS_NOERROR;
    }
    if (hMod != g_connectedMod) {
        LeaveCriticalSection(&g_lock);
        log_msg("ReturnValue: PDU_ERR_INVALID_HANDLE");
        return PDU_ERR_INVALID_HANDLE;
    }
    if (hCLL == PDU_HANDLE_UNDEF) {
        g_modCb.hMod = hMod;
        g_modCb.hCLL = hCLL;
        g_modCb.fn = EventCallbackFunction;
        if (EventCallbackFunction) {
            /* Report the module as ready straight away. */
            enqueue_event(make_status_event(PDU_HANDLE_UNDEF, NULL, 0, PDU_MODST_READY));
            cb.fn = EventCallbackFunction;
            cb.evt = PDU_EVT_DATA_AVAILABLE;
            cb.hMod = hMod;
            cb.hCLL = hCLL;
            cb.cllTag = NULL;
            cb.apiTag = NULL;
        }
    } else {
        CLL *c = find_cll(hCLL);
        if (!c) {
            LeaveCriticalSection(&g_lock);
            log_msg("ReturnValue: PDU_ERR_INVALID_HANDLE");
            return PDU_ERR_INVALID_HANDLE;
        }
        c->cb.hMod = hMod;
        c->cb.hCLL = hCLL;
        c->cb.fn = EventCallbackFunction;
    }
    LeaveCriticalSection(&g_lock);
    fire(&cb);
    return PDU_STATUS_NOERROR;
}

/* READ_VBATT, START_MSG_FILTER and CLEAR_MSG_FILTER.  Filters are global and
 * belong to the bus that is active when they are started; they are reported
 * as FILTER / FILTERCLR lines. */
T_PDU_ERROR PDU_API_CALL PDUIoCtl(UNUM32 hMod, UNUM32 hCLL, UNUM32 IoCtlCommandId,
                                  PDU_DATA_ITEM *pInputData, PDU_DATA_ITEM **pOutputData)
{
    T_PDU_ERROR ret = PDU_STATUS_NOERROR;
    EnterCriticalSection(&g_lock);
    log_msg("PDUIoCtl Called: hMod=%08X hCLL=%08X id=%u", hMod, hCLL, IoCtlCommandId);
    if (IoCtlCommandId == IOCTL_READ_VBATT) {
        PDU_DATA_ITEM *it;
        if (!pOutputData) {
            ret = PDU_ERR_INVALID_PARAMETERS;
        } else if ((it = (PDU_DATA_ITEM *)item_alloc(sizeof(PDU_DATA_ITEM) + sizeof(UNUM32))) == NULL) {
            ret = PDU_ERR_FCT_FAILED;
        } else {
            it->ItemType = PDU_IT_IO_UNUM32;
            it->pData = it + 1;
            *(UNUM32 *)it->pData = (UNUM32)g_vbattMv;
            *pOutputData = it;
        }
        LeaveCriticalSection(&g_lock);
        return ret;
    }
    if (IoCtlCommandId == IOCTL_CLEAR_MSG_FILTER) {
        free(g_filters);
        g_filters = NULL;
        g_numFilters = 0;
        LeaveCriticalSection(&g_lock);
        link_write_line("FILTERCLR %08X", hCLL);
        return PDU_STATUS_NOERROR;
    }
    if (IoCtlCommandId == IOCTL_START_MSG_FILTER) {
        PDU_IO_FILTER_LIST *list;
        UNUM32 i;
        BUS bus = g_curBus;
        char lines[32][160];    /* FILTER lines, sent after g_lock is released */
        UNUM32 nLines = 0;
        if (!pInputData || !pInputData->pData) {
            LeaveCriticalSection(&g_lock);
            return PDU_ERR_INVALID_PARAMETERS;
        }
        list = (PDU_IO_FILTER_LIST *)pInputData->pData;
        for (i = 0; i < list->NumFilterEntries && list->pFilterData; i++) {
            const PDU_IO_FILTER_DATA *fd = &list->pFilterData[i];
            SW_FILTER *nf = (SW_FILTER *)realloc(g_filters, (g_numFilters + 1) * sizeof *nf);
            char m[40], pt[40];
            if (!nf)
                break;
            g_filters = nf;
            nf = &g_filters[g_numFilters++];
            nf->bus = bus;
            nf->type = fd->FilterType;
            memcpy(nf->mask, fd->FilterMaskMessage, 4);
            memcpy(nf->pattern, fd->FilterPatternMessage, 4);
            hex_bytes(m, sizeof m, fd->FilterMaskMessage, fd->FilterCompareSize > 12 ? 12 : fd->FilterCompareSize);
            hex_bytes(pt, sizeof pt, fd->FilterPatternMessage, fd->FilterCompareSize > 12 ? 12 : fd->FilterCompareSize);
            if (nLines < 32)
                _snprintf(lines[nLines++], 159, "FILTER %08X %s %s #%u MASK=%s PATTERN=%s", hCLL, bus_name(bus),
                          (fd->FilterType == PDU_FLT_BLOCK || fd->FilterType == PDU_FLT_BLOCK_UUDT) ? "BLOCK" : "PASS",
                          fd->FilterNumber, m, pt);
        }
        LeaveCriticalSection(&g_lock);
        for (i = 0; i < nLines; i++) {
            lines[i][159] = 0;
            link_write_line("%s", lines[i]);
        }
        return PDU_STATUS_NOERROR;
    }
    LeaveCriticalSection(&g_lock);
    log_msg("ReturnValue: PDU_ERR_ID_NOT_SUPPORTED");
    return PDU_ERR_ID_NOT_SUPPORTED;
}

T_PDU_ERROR PDU_API_CALL PDUGetVersion(UNUM32 hMod, PDU_VERSION_DATA *pVersionData)
{
    PDU_VERSION_DATA v;
    /* Fixed identification values; only the names come from the ini. */
    log_msg("PDUGetVersion Called: hMod=%08X", hMod);
    if (!pVersionData)
        return PDU_ERR_INVALID_PARAMETERS;
    memset(&v, 0, sizeof v);
    v.MVCI_Part1StandardVersion = 0x02010100;
    v.MVCI_Part2StandardVersion = 0x02020000;
    v.HwSerialNumber = 0x01519226;
    strncpy(v.HwName, g_moduleName, sizeof v.HwName - 1);
    v.HwVersion = 0x00010000;
    v.HwDate = 0x26011000;
    v.HwInterface = 1;
    strncpy(v.FwName, g_moduleName, sizeof v.FwName - 1);
    v.FwVersion = 0x0805214E;
    v.FwDate = 0x33011D00;
    strncpy(v.VendorName, "dpdu-bridge", sizeof v.VendorName - 1);
    strncpy(v.PDUApiSwName, DRIVER_NAME " Serial PDU-API", sizeof v.PDUApiSwName - 1);
    v.PDUApiSwVersion = 0x0805214E;
    v.PDUApiSwDate = 0x33011D00;
    *pVersionData = v;
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUGetStatus(UNUM32 hMod, UNUM32 hCLL, UNUM32 hCoP, T_PDU_STATUS *pStatusCode,
                                      UNUM32 *pTimestamp, UNUM32 *pExtraInfo)
{
    CLL *c;
    UNUM32 ts = now_us();
    EnterCriticalSection(&g_lock);
    log_msg("PDUGetStatus Called: hMod=%08X hCLL=%08X hCoP=%08X", hMod, hCLL, hCoP);
    if (hCoP == 0 || (c = find_cll(hCLL)) == NULL) {
        LeaveCriticalSection(&g_lock);
        log_msg("ReturnValue: PDU_ERR_INVALID_HANDLE");
        return PDU_ERR_INVALID_HANDLE;
    }
    if (hCoP == c->hStartComm || hCoP == c->hTxCop) {
        if (pStatusCode) *pStatusCode = PDU_COPST_FINISHED;
        if (pTimestamp) *pTimestamp = ts;
    } else if (hCoP == c->hRxCop) {
        if (pStatusCode) *pStatusCode = c->rxCopStatus;
        if (pTimestamp) *pTimestamp = ts;
    }
    /* Any other handle leaves status and timestamp untouched. */
    if (pExtraInfo) *pExtraInfo = 0;
    LeaveCriticalSection(&g_lock);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUGetLastError(UNUM32 hMod, UNUM32 hCLL, UNUM32 *pErrorCode, UNUM32 *phCoP,
                                         UNUM32 *pTimestamp, UNUM32 *pExtraErrorInfo)
{
    (void)pErrorCode; (void)phCoP; (void)pTimestamp; (void)pExtraErrorInfo;
    log_msg("PDUGetLastError Called: hMod=%08X hCLL=%08X", hMod, hCLL);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUGetTimestamp(UNUM32 hMod, UNUM32 *pTimestamp)
{
    log_msg("PDUGetTimestamp Called: hMod=%08X", hMod);
    if (pTimestamp)
        *pTimestamp = now_us();
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUGetObjectId(T_PDU_OBJT pduObjectType, CHAR8 *pShortname, UNUM32 *pPduObjectId)
{
    size_t i;
    if (!pShortname || !pPduObjectId) {
        log_msg("PDUGetObjectId Called: NULL pointer");
        return PDU_ERR_INVALID_PARAMETERS;
    }
    for (i = 0; i < sizeof g_objects / sizeof g_objects[0]; i++) {
        if (g_objects[i].type == pduObjectType && !strcmp(g_objects[i].name, pShortname)) {
            *pPduObjectId = g_objects[i].id;
            log_msg("PDUGetObjectId Called: type=%X %s -> %u", pduObjectType, pShortname, g_objects[i].id);
            return PDU_STATUS_NOERROR;
        }
    }
    log_msg("PDUGetObjectId Called: type=%X %s -> PDU_ERR_INVALID_PARAMETERS", pduObjectType, pShortname);
    return PDU_ERR_INVALID_PARAMETERS;
}

/* The resource functions are stubs that return PDU_STATUS_NOERROR without
 * touching their output parameters. */
T_PDU_ERROR PDU_API_CALL PDUGetResourceIds(UNUM32 hMod, PDU_RSC_DATA *pResourceIdData, PDU_RSC_ID_ITEM **pResourceIdList)
{
    (void)pResourceIdData; (void)pResourceIdList;
    log_msg("PDUGetResourceIds Called: hMod=%08X", hMod);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUGetResourceStatus(PDU_RSC_STATUS_ITEM *pResourceStatus)
{
    (void)pResourceStatus;
    log_msg("PDUGetResourceStatus Called");
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUGetConflictingResources(UNUM32 resourceId, PDU_MODULE_ITEM *pInputModuleList,
                                                    PDU_RSC_CONFLICT_ITEM **pOutputConflictList)
{
    (void)pInputModuleList; (void)pOutputConflictList;
    log_msg("PDUGetConflictingResources Called: resourceId=%u", resourceId);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUGetModuleIds(PDU_MODULE_ITEM **pModuleIdList)
{
    PDU_MODULE_ITEM *it;
    PDU_MODULE_DATA *md;
    char *name, *info;
    size_t nameLen, infoLen;
    EnterCriticalSection(&g_lock);
    log_msg("PDUGetModuleIds Called");
    if (!pModuleIdList) {
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_INVALID_PARAMETERS;
    }
    if (!g_constructed) {
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_PDUAPI_NOT_CONSTRUCTED;
    }
    nameLen = strlen(g_moduleName) + 1;
    infoLen = strlen(g_linkDesc) + 1;
    it = (PDU_MODULE_ITEM *)item_alloc(sizeof *it + sizeof *md + nameLen + infoLen);
    if (!it) {
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_FCT_FAILED;
    }
    md = (PDU_MODULE_DATA *)(it + 1);
    name = (char *)(md + 1);
    info = name + nameLen;
    memcpy(name, g_moduleName, nameLen);
    memcpy(info, g_linkDesc, infoLen);
    it->ItemType = PDU_IT_MODULE_ID;
    it->NumEntries = g_moduleListed ? 1 : 0;
    it->pModuleData = g_moduleListed ? md : NULL;
    md->ModuleTypeId = MODULE_TYPE_ID;
    md->hMod = MODULE_HANDLE;
    md->pVendorModuleName = name;
    md->pVendorAdditionalInfo = info;
    md->ModuleStatus = g_moduleStatus == PDU_MODST_READY ? PDU_MODST_READY : PDU_MODST_AVAIL;
    *pModuleIdList = it;
    log_msg("  NumEntries=%u", it->NumEntries);
    LeaveCriticalSection(&g_lock);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDULockResource(UNUM32 hMod, UNUM32 hCLL, UNUM32 LockMask)
{
    log_msg("PDULockResource Called: hMod=%08X hCLL=%08X mask=%X", hMod, hCLL, LockMask);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUUnlockResource(UNUM32 hMod, UNUM32 hCLL, UNUM32 LockMask)
{
    log_msg("PDUUnLockResource Called: hMod=%08X hCLL=%08X mask=%X", hMod, hCLL, LockMask);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUCreateComLogicalLink(UNUM32 hMod, PDU_RSC_DATA *pRscData, UNUM32 resourceId,
                                                 void *pCllTag, UNUM32 *phCLL, PDU_FLAG_DATA *pCllCreateFlag)
{
    CLL *c, **nl;
    BUS bus;
    UNUM8 flag0 = 0;
    UNUM32 i, hCLL;
    PDU_PIN_DATA *pins;

    EnterCriticalSection(&g_lock);
    log_msg("PDUCreateComLogicalLink Called: hMod=%08X resourceId=%08X pCllTag=%p", hMod, resourceId, pCllTag);
    if (!g_constructed) {
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_PDUAPI_NOT_CONSTRUCTED;
    }
    if (hMod != g_connectedMod) {
        LeaveCriticalSection(&g_lock);
        log_msg("ReturnValue: PDU_ERR_INVALID_HANDLE");
        return PDU_ERR_INVALID_HANDLE;
    }
    if (g_moduleStatus != PDU_MODST_READY) {
        LeaveCriticalSection(&g_lock);
        log_msg("ReturnValue: PDU_ERR_MODULE_NOT_CONNECTED");
        return PDU_ERR_MODULE_NOT_CONNECTED;
    }
    if (!pRscData || !phCLL) {
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_INVALID_PARAMETERS;
    }
    if (pCllCreateFlag && pCllCreateFlag->NumFlagBytes && pCllCreateFlag->pFlagData)
        flag0 = pCllCreateFlag->pFlagData[0];

    pins = pRscData->pDLCPinData;
    log_msg("  BusTypeId=%u ProtocolId=%u NumPinData=%u flags=%02X", pRscData->BusTypeId,
            pRscData->ProtocolId, pRscData->NumPinData, flag0);
    for (i = 0; pins && i < pRscData->NumPinData; i++)
        log_msg("  pin %u type %u", pins[i].DLCPinNumber, pins[i].DLCPinTypeId);

    switch (pRscData->BusTypeId) {
    case BUSTYPE_SAE_J1850_VPW:
        bus = BUS_VPW;
        break;
    case BUSTYPE_SAE_J2740_UART:
        bus = BUS_ALDL;
        break;
    case BUSTYPE_ISO_11898_2_DWCAN:
        bus = BUS_HSCAN;
        for (i = 0; i < g_numClls; i++)
            if (g_clls[i]->bus == BUS_HSCAN)
                log_msg("Requested HS CAN twice, ensuring HSCAN protocol is set...");
        if (pRscData->NumPinData != 2 || !pins || pins[0].DLCPinNumber != 6 || pins[1].DLCPinNumber != 14) {
            LeaveCriticalSection(&g_lock);
            log_msg("ReturnValue: PDU_ERR_INVALID_PARAMETERS (pins)");
            return PDU_ERR_INVALID_PARAMETERS;
        }
        break;
    case BUSTYPE_SAE_J2411_SWCAN:
        bus = BUS_SWCAN;
        if (pRscData->NumPinData != 1 || !pins || pins[0].DLCPinNumber != 1) {
            LeaveCriticalSection(&g_lock);
            log_msg("ReturnValue: PDU_ERR_INVALID_PARAMETERS (pins)");
            return PDU_ERR_INVALID_PARAMETERS;
        }
        break;
    default:
        LeaveCriticalSection(&g_lock);
        log_msg("ReturnValue: PDU_ERR_INVALID_PARAMETERS (invalid protocol)");
        return PDU_ERR_INVALID_PARAMETERS;
    }

    c = (CLL *)calloc(1, sizeof *c);
    nl = (CLL **)realloc(g_clls, (g_numClls + 1) * sizeof *nl);
    if (!c || !nl) {
        free(c);
        if (nl) g_clls = nl;
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_FCT_FAILED;
    }
    g_clls = nl;
    c->hCLL = hCLL = g_nextCll++;
    c->bus = bus;
    c->pCllTag = pCllTag;
    c->cllStatus = PDU_CLLST_OFFLINE;
    c->rxCopStatus = PDU_COPST_IDLE;
    g_clls[g_numClls++] = c;
    g_curBus = bus;
    *phCLL = hCLL;
    log_msg("  hCLL=%08X bus=%s (total logicals %u)", hCLL, bus_name(bus), g_numClls);
    LeaveCriticalSection(&g_lock);
    link_write_line("OPEN %08X %s PROTOCOL=%u RAW=%d CHECKSUM=%d", hCLL, bus_name(bus),
                      pRscData->ProtocolId, (flag0 & 0x80) ? 1 : 0, (flag0 & 0x40) ? 1 : 0);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUDestroyComLogicalLink(UNUM32 hMod, UNUM32 hCLL)
{
    UNUM32 i;
    EnterCriticalSection(&g_lock);
    log_msg("PDUDestroyComLogicalLink Called: hMod=%08X hCLL=%08X", hMod, hCLL);
    for (i = 0; i < g_numClls; i++) {
        if (g_clls[i]->hCLL == hCLL) {
            cll_free(g_clls[i]);
            memmove(&g_clls[i], &g_clls[i + 1], (g_numClls - i - 1) * sizeof *g_clls);
            g_numClls--;
            LeaveCriticalSection(&g_lock);
            link_write_line("CLOSE %08X", hCLL);
            return PDU_STATUS_NOERROR;
        }
    }
    LeaveCriticalSection(&g_lock);
    log_msg("ReturnValue: PDU_ERR_INVALID_HANDLE");
    return PDU_ERR_INVALID_HANDLE;
}

static T_PDU_ERROR set_link_state(UNUM32 hMod, UNUM32 hCLL, BOOL online)
{
    CB_CALL cb;
    CLL *c;
    EnterCriticalSection(&g_lock);
    log_msg("%s Called: hMod=%08X hCLL=%08X", online ? "PDUConnect" : "PDUDisconnect", hMod, hCLL);
    if (hMod != g_connectedMod || (c = find_cll(hCLL)) == NULL) {
        LeaveCriticalSection(&g_lock);
        log_msg("ReturnValue: PDU_ERR_INVALID_HANDLE");
        return PDU_ERR_INVALID_HANDLE;
    }
    if (online && c->cllStatus == PDU_CLLST_ONLINE) {
        LeaveCriticalSection(&g_lock);
        log_msg("ReturnValue: PDU_ERR_CLL_CONNECTED (Already Connected)");
        return PDU_ERR_CLL_CONNECTED;
    }
    c->cllStatus = online ? PDU_CLLST_ONLINE : PDU_CLLST_OFFLINE;
    enqueue_event(make_status_event(PDU_HANDLE_UNDEF, NULL, now_us(), c->cllStatus));
    cb = cll_callback(c);
    LeaveCriticalSection(&g_lock);
    link_write_line("%s %08X", online ? "CONNECT" : "DISCONNECT", hCLL);
    fire(&cb);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUConnect(UNUM32 hMod, UNUM32 hCLL)
{
    return set_link_state(hMod, hCLL, TRUE);
}

T_PDU_ERROR PDU_API_CALL PDUDisconnect(UNUM32 hMod, UNUM32 hCLL)
{
    return set_link_state(hMod, hCLL, FALSE);
}

/* Stub: returns PDU_STATUS_NOERROR without filling pParamItem. */
T_PDU_ERROR PDU_API_CALL PDUGetComParam(UNUM32 hMod, UNUM32 hCLL, UNUM32 ParamId, PDU_PARAM_ITEM **pParamItem)
{
    (void)pParamItem;
    log_msg("PDUGetComParam Called: hMod=%08X hCLL=%08X ParamId=%u", hMod, hCLL, ParamId);
    return PDU_STATUS_NOERROR;
}

/* Scalar values are applied (CP_TransmitIndEnable) and reported as PARAM
 * lines; non-scalar ComParams are logged and otherwise ignored. */
T_PDU_ERROR PDU_API_CALL PDUSetComParam(UNUM32 hMod, UNUM32 hCLL, PDU_PARAM_ITEM *pParamItem)
{
    CLL *c;
    UNUM32 value;
    BOOL scalar;
    const char *name;
    EnterCriticalSection(&g_lock);
    if (!pParamItem) {
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_INVALID_PARAMETERS;
    }
    scalar = comparam_scalar(pParamItem, &value);
    name = comparam_name(pParamItem->ComParamId);
    if (scalar)
        log_msg("PDUSetComParam Called: hMod=%08X hCLL=%08X %s(%u) type=%X class=%u value=%u", hMod, hCLL,
                name ? name : "?", pParamItem->ComParamId, pParamItem->ComParamDataType,
                pParamItem->ComParamClass, value);
    else
        log_msg("PDUSetComParam Called: hMod=%08X hCLL=%08X %s(%u) type=%X class=%u value not a scalar - ignored",
                hMod, hCLL, name ? name : "?", pParamItem->ComParamId, pParamItem->ComParamDataType,
                pParamItem->ComParamClass);
    c = find_cll(hCLL);
    if (!c) {
        LeaveCriticalSection(&g_lock);
        log_msg("ReturnValue: PDU_ERR_INVALID_HANDLE");
        return PDU_ERR_INVALID_HANDLE;
    }
    if (!scalar) {
        LeaveCriticalSection(&g_lock);
        return PDU_STATUS_NOERROR;
    }
    if (pParamItem->ComParamId == CP_TRANSMITINDENABLE)
        c->txIndEnable = value;
    LeaveCriticalSection(&g_lock);
    if (name)
        link_write_line("PARAM %08X %s %u", hCLL, name, value);
    else
        link_write_line("PARAM %08X %u %u", hCLL, pParamItem->ComParamId, value);
    return PDU_STATUS_NOERROR;
}

/* Stub: returns PDU_STATUS_NOERROR without filling the table. */
T_PDU_ERROR PDU_API_CALL PDUGetUniqueRespIdTable(UNUM32 hMod, UNUM32 hCLL, PDU_UNIQUE_RESP_ID_TABLE_ITEM **pUniqueRespIdTable)
{
    (void)pUniqueRespIdTable;
    log_msg("PDUGetUniqueRespIdTable Called: hMod=%08X hCLL=%08X", hMod, hCLL);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUSetUniqueRespIdTable(UNUM32 hMod, UNUM32 hCLL, PDU_UNIQUE_RESP_ID_TABLE_ITEM *pUniqueRespIdTable)
{
    CLL *c;
    URID_ENTRY *tab = NULL;
    UNUM32 i, k, n = 0;
    char (*lines)[128] = NULL;
    BUS bus;

    EnterCriticalSection(&g_lock);
    log_msg("PDUSetUniqueRespIdTable Called: hMod=%08X hCLL=%08X", hMod, hCLL);
    if (hMod != g_connectedMod || (c = find_cll(hCLL)) == NULL) {
        LeaveCriticalSection(&g_lock);
        log_msg("ReturnValue: PDU_ERR_INVALID_HANDLE");
        return PDU_ERR_INVALID_HANDLE;
    }
    bus = c->bus;
    if (!bus_is_can(bus)) {
        /* The table is only kept for CAN links. */
        LeaveCriticalSection(&g_lock);
        return PDU_STATUS_NOERROR;
    }
    free(c->urid);
    c->urid = NULL;
    c->numUrid = 0;
    if (!pUniqueRespIdTable || (pUniqueRespIdTable->NumEntries && !pUniqueRespIdTable->pUniqueData)) {
        LeaveCriticalSection(&g_lock);
        log_msg("PDU_ECU_UNIQUE_RESP_DATA is null, invalid data!");
        return PDU_ERR_INVALID_PARAMETERS;
    }
    n = pUniqueRespIdTable->NumEntries;
    if (n) {
        tab = (URID_ENTRY *)calloc(n, sizeof *tab);
        lines = (char (*)[128])calloc(n, 128);
        if (!tab || !lines) {
            free(tab);
            free(lines);
            LeaveCriticalSection(&g_lock);
            return PDU_ERR_FCT_FAILED;
        }
    }
    for (i = 0; i < n; i++) {
        const PDU_ECU_UNIQUE_RESP_DATA *d = &pUniqueRespIdTable->pUniqueData[i];
        tab[i].uniqueRespId = d->UniqueRespIdentifier;
        for (k = 0; k < d->NumParamItems && d->pParams; k++) {
            const PDU_PARAM_ITEM *pi = &d->pParams[k];
            UNUM32 v;
            if (!comparam_scalar(pi, &v)) {
                /* The field keeps its value (0 unless set by an earlier item). */
                const char *pname = comparam_name(pi->ComParamId);
                log_msg("  URID %08X: %s(%u) type=%X value not a scalar - ignored", d->UniqueRespIdentifier,
                        pname ? pname : "?", pi->ComParamId, pi->ComParamDataType);
                continue;
            }
            switch (pi->ComParamId) {
            case CP_CANPHYSREQID:      tab[i].physReqId = v; break;
            case CP_CANRESPUSDTID:     tab[i].respUsdtId = v; break;
            case CP_CANRESPUSDTFORMAT: tab[i].respFormat = v; break;
            case CP_CANRESPUUDTID:     tab[i].respUudtId = v; break;
            default: break;
            }
        }
        _snprintf(lines[i], 127, "ECU %08X %s URID=%08X REQ=%X RESP=%X FORMAT=%u UUDT=%X", hCLL, bus_name(bus),
                  tab[i].uniqueRespId, tab[i].physReqId, tab[i].respUsdtId, tab[i].respFormat, tab[i].respUudtId);
        log_msg("  %s", lines[i]);
    }
    c->urid = tab;
    c->numUrid = n;
    LeaveCriticalSection(&g_lock);
    for (i = 0; i < n; i++)
        link_write_line("%s", lines[i]);
    free(lines);
    return PDU_STATUS_NOERROR;
}

/* Sends one message from the host as a TX line (or WAKEUP) on every link.
 * CAN messages start with the 4-byte big-endian CAN id. */
static void transmit(BUS bus, const UNUM8 *data, UNUM32 len)
{
    char hex[MAX_FRAME * 3 + 8];
    if (bus_is_can(bus)) {
        UNUM32 id;
        if (len < 4) {
            log_msg("Failed transmitting msg: CAN message shorter than its 4 byte id");
            return;
        }
        id = be32(data);
        if (id == GMLAN_HV_WAKEUP_ID) {
            /* Id 0x100 requests a high-voltage wake-up on SW-CAN (sent as a
             * WAKEUP line); on either bus it is never sent as a TX line. */
            if (bus == BUS_SWCAN) {
                log_msg("Sending HVWakup");
                link_write_line("WAKEUP %s", bus_name(bus));
            }
            return;
        }
        hex_bytes(hex, sizeof hex, data + 4, len - 4);
        link_write_line("TX %s %0*X%s%s", bus_name(bus), id > 0x7FF ? 8 : 3, id, len > 4 ? " " : "", hex);
    } else {
        hex_bytes(hex, sizeof hex, data, len);
        link_write_line("TX %s %s", bus_name(bus), hex);
    }
}

T_PDU_ERROR PDU_API_CALL PDUStartComPrimitive(UNUM32 hMod, UNUM32 hCLL, UNUM32 CoPType, UNUM32 CoPDataSize,
                                              UNUM8 *pCoPData, PDU_COP_CTRL_DATA *pCopCtrlData,
                                              void *pCoPTag, UNUM32 *phCoP)
{
    CLL *c;
    CB_CALL cb;
    UNUM8 *data;
    UNUM32 h;

    EnterCriticalSection(&g_lock);
    log_msg("PDUStartComPrimitive Called: hMod=%08X hCLL=%08X CoPType=%X size=%u tag=%p",
            hMod, hCLL, CoPType, CoPDataSize, pCoPTag);
    if (hMod != g_connectedMod || (c = find_cll(hCLL)) == NULL) {
        LeaveCriticalSection(&g_lock);
        log_msg("ReturnValue: PDU_ERR_INVALID_HANDLE");
        return PDU_ERR_INVALID_HANDLE;
    }

    if (CoPType == PDU_COPT_UPDATEPARAM) {
        void *lastTxTag = c->txCopTag;
        enqueue_event(make_status_event(HCOP_UPDATEPARAM, NULL, now_us(), PDU_COPST_EXECUTING));
        cb = cll_callback(c);
        LeaveCriticalSection(&g_lock);
        fire(&cb);
        EnterCriticalSection(&g_lock);
        enqueue_event(make_status_event(HCOP_UPDATEPARAM, lastTxTag, now_us(), PDU_COPST_FINISHED));
        LeaveCriticalSection(&g_lock);
        return PDU_STATUS_NOERROR;
    }
    if (CoPType == PDU_COPT_RESTORE_PARAM || !pCopCtrlData) {
        /* Nothing to do here (*phCoP is not set). */
        LeaveCriticalSection(&g_lock);
        return PDU_STATUS_NOERROR;
    }

    if (CoPType == PDU_COPT_STARTCOMM) {
        h = c->hStartComm = g_nextStart++;
        if (phCoP) *phCoP = h;
        enqueue_event(make_status_event(h, NULL, now_us(), PDU_COPST_EXECUTING));
        cb = cll_callback(c);
        LeaveCriticalSection(&g_lock);
        fire(&cb);
        EnterCriticalSection(&g_lock);
        enqueue_event(make_status_event(PDU_HANDLE_UNDEF, NULL, now_us(), PDU_CLLST_COMM_STARTED));
        enqueue_event(make_status_event(h, NULL, now_us(), PDU_COPST_FINISHED));
        LeaveCriticalSection(&g_lock);
        link_write_line("STARTCOMM %08X", hCLL);
        return PDU_STATUS_NOERROR;
    }

    if (CoPType == PDU_COPT_STOPCOMM) {
        h = c->hStartComm;
        enqueue_event(make_status_event(h, NULL, now_us(), PDU_COPST_EXECUTING));
        cb = cll_callback(c);
        LeaveCriticalSection(&g_lock);
        fire(&cb);
        EnterCriticalSection(&g_lock);
        enqueue_event(make_status_event(h, NULL, now_us(), PDU_COPST_FINISHED));
        LeaveCriticalSection(&g_lock);
        link_write_line("STOPCOMM %08X", hCLL);
        return PDU_STATUS_NOERROR;
    }

    if (CoPType != PDU_COPT_SENDRECV) {
        /* PDU_COPT_DELAY: accepted, nothing happens. */
        LeaveCriticalSection(&g_lock);
        return PDU_STATUS_NOERROR;
    }

    if (CoPDataSize == 0) {
        /* Receive-only ComPrimitive: it owns all future responses and its
         * expected responses filter them. */
        EXP_RESP *list = NULL;
        UNUM32 i, n = pCopCtrlData->pExpectedResponseArray ? pCopCtrlData->NumPossibleExpectedResponses : 0;
        if (n) {
            list = (EXP_RESP *)calloc(n, sizeof *list);
            if (!list) {
                LeaveCriticalSection(&g_lock);
                return PDU_ERR_FCT_FAILED;
            }
        }
        for (i = 0; i < n; i++) {
            const PDU_EXP_RESP_DATA *x = &pCopCtrlData->pExpectedResponseArray[i];
            list[i].respType = x->ResponseType;
            list[i].acceptanceId = x->AcceptanceId;
            if (x->NumMaskPatternBytes == 4 && x->pMaskData && x->pPatternData) {
                list[i].mask = be32(x->pMaskData);
                list[i].pattern = be32(x->pPatternData);
            }
            log_msg("  expected response %u: type=%u acceptanceId=%u mask=%08X pattern=%08X",
                    i, list[i].respType, list[i].acceptanceId, list[i].mask, list[i].pattern);
        }
        free(c->expResp);
        c->expResp = list;
        c->numExpResp = n;
        h = c->hRxCop = g_nextRxCop++;
        c->rxCopTag = pCoPTag;
        if (phCoP) *phCoP = h;
        c->rxCopStatus = PDU_COPST_EXECUTING;
        enqueue_event(make_status_event(h, pCoPTag, now_us(), PDU_COPST_EXECUTING));
        cb = cll_callback(c);
        LeaveCriticalSection(&g_lock);
        log_msg("  RXD ComPrimitive hCoP=%08X", h);
        fire(&cb);
        return PDU_STATUS_NOERROR;
    }

    /* Transmit ComPrimitive. */
    if (!pCoPData) {
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_INVALID_PARAMETERS;
    }
    data = (UNUM8 *)malloc(CoPDataSize);
    if (!data) {
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_FCT_FAILED;
    }
    memcpy(data, pCoPData, CoPDataSize);
    h = c->hTxCop = g_nextTxCop++;
    c->txCopTag = pCoPTag;
    if (phCoP) *phCoP = h;
    enqueue_event(make_status_event(h, pCoPTag, now_us(), PDU_COPST_EXECUTING));
    cb = cll_callback(c);
    LeaveCriticalSection(&g_lock);
    log_msg("  TXD ComPrimitive hCoP=%08X", h);
    fire(&cb);

    /* The callback ran without the lock, so the CLL may have been destroyed. */
    EnterCriticalSection(&g_lock);
    c = find_cll(hCLL);
    if (c) {
        BOOL busChanged = FALSE;
        BUS bus = c->bus;
        if (c->txIndEnable == 1) {
            /* The indication is a result of the receive ComPrimitive, so it
             * follows the same rule as received messages. */
            static const UNUM8 txInd[4] = {0x00, 0x00, 0x00, 0x88};
            const char *why = rx_unavailable(c);
            if (!why)
                enqueue_event(make_result_event(c->hRxCop, c->rxCopTag, txInd, 0, 0, NULL, 0));
            else
                log_msg("TX indication dropped: %s logical link %08X", why, hCLL);
        }
        if (g_curBus != bus) {
            g_curBus = bus;
            busChanged = TRUE;
        }
        LeaveCriticalSection(&g_lock);
        if (busChanged)
            link_write_line("PROTOCOL %s", bus_name(bus));
        transmit(bus, data, CoPDataSize);
        EnterCriticalSection(&g_lock);
        if (find_cll(hCLL))
            enqueue_event(make_status_event(h, pCoPTag, now_us(), PDU_COPST_FINISHED));
    }
    LeaveCriticalSection(&g_lock);
    free(data);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUCancelComPrimitive(UNUM32 hMod, UNUM32 hCLL, UNUM32 hCoP)
{
    CLL *c;
    CB_CALL cb;
    EnterCriticalSection(&g_lock);
    log_msg("PDUCancelComPrimitive Called: hMod=%08X hCLL=%08X hCoP=%08X", hMod, hCLL, hCoP);
    if ((c = find_cll(hCLL)) == NULL) {
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_INVALID_HANDLE;
    }
    if (hCoP != c->hRxCop) {
        /* TX / STARTCOMM primitives are already finished. */
        LeaveCriticalSection(&g_lock);
        return PDU_STATUS_NOERROR;
    }
    c->rxCopStatus = PDU_COPST_CANCELLED;
    enqueue_event(make_status_event(c->hRxCop, c->rxCopTag, now_us(), PDU_COPST_CANCELLED));
    cb = cll_callback(c);
    cb.hCLL = c->hCLL;
    LeaveCriticalSection(&g_lock);
    fire(&cb);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUGetEventItem(UNUM32 hMod, UNUM32 hCLL, PDU_EVENT_ITEM **pEventItem)
{
    EVT_NODE *n;
    /* One queue shared by the module and all CLLs. */
    EnterCriticalSection(&g_lock);
    if (!pEventItem) {
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_INVALID_PARAMETERS;
    }
    n = g_evHead;
    if (!n) {
        LeaveCriticalSection(&g_lock);
        return PDU_ERR_EVENT_QUEUE_EMPTY;
    }
    g_evHead = n->next;
    if (!g_evHead)
        g_evTail = NULL;
    *pEventItem = n->item;
    if (n->item->ItemType == PDU_IT_STATUS)
        log_msg("PDUGetEventItem(hMod=%08X hCLL=%08X): STATUS hCoP=%08X status=%X", hMod, hCLL,
                n->item->hCop, *(UNUM32 *)n->item->pData);
    else
        log_msg("PDUGetEventItem(hMod=%08X hCLL=%08X): RESULT hCoP=%08X bytes=%u", hMod, hCLL,
                n->item->hCop, ((PDU_RESULT_DATA *)n->item->pData)->NumDataBytes);
    free(n);
    LeaveCriticalSection(&g_lock);
    return PDU_STATUS_NOERROR;
}

T_PDU_ERROR PDU_API_CALL PDUDestroyItem(PDU_ITEM *pItem)
{
    EnterCriticalSection(&g_lock);
    if (!item_free(pItem))
        log_msg("PDUDestroyItem Called: %p is not an item of this driver - ignored", pItem);
    LeaveCriticalSection(&g_lock);
    return PDU_STATUS_NOERROR;
}

/* ------------------------------------------------------------------------ */

/* Initialises the locks and events and reads the ini.  Nothing is torn down at
 * unload: worker threads hold a module reference, so the DLL cannot be
 * unloaded while a link is open. */
BOOL WINAPI DllMain(HINSTANCE hinst, DWORD reason, LPVOID reserved)
{
    (void)reserved;
    if (reason == DLL_PROCESS_ATTACH) {
        g_hInst = hinst;
        DisableThreadLibraryCalls(hinst);
        InitializeCriticalSection(&g_lock);
        InitializeCriticalSection(&g_logLock);
        InitializeCriticalSection(&g_txLock);
        InitializeCriticalSection(&g_clientLock);
        g_stopEvt = CreateEventA(NULL, TRUE, FALSE, NULL);
        g_serialQ.evt = CreateEventA(NULL, FALSE, FALSE, NULL);
        g_tcpQ.evt = CreateEventA(NULL, FALSE, FALSE, NULL);
        QueryPerformanceFrequency(&g_qpf);
        load_config();
        log_msg(DRIVER_NAME " loaded (port %s)", g_linkDesc[0] ? g_linkDesc : "not specified");
    }
    return TRUE;
}
