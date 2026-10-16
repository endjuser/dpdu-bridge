/*
 * j2534.c - SAE J2534-1 (04.04) PassThru front end of dpdu-bridge-c.dll: the
 * same bridge as the D-PDU API, for hosts that talk J2534 (README.md, "J2534").
 * src/cs/J2534.cs is the C# implementation with identical behaviour.
 *
 * Included by dpdu_bridge.c (one translation unit, so it shares that file's
 * static helpers and links); not compiled on its own.
 *
 * PassThru calls become the same text lines the D-PDU side writes (HELLO /
 * OPEN / PARAM / FILTER / ECU / TX / CLOSE / BYE).  While the device is open,
 * process_line hands received RX lines to j2534_deliver, which filters them
 * into per-channel receive queues that PassThruReadMsgs drains.
 *
 * One device (id 1) and any number of channels, one per protocol id.  The
 * D-PDU API and J2534 share the links, so only one of them can have the
 * device open at a time.
 *
 * Threads: all state below is guarded by g_jLock; PassThruReadMsgs waits on
 * g_jCond for received messages.  Periodic messages are sent from timer-queue
 * threads.  Link writes and timer deletions (which wait for running timer
 * callbacks, and those take g_jLock) happen with no lock held.
 */

/* Return codes */
#define J_STATUS_NOERROR              0x00
#define J_ERR_NOT_SUPPORTED           0x01
#define J_ERR_INVALID_CHANNEL_ID      0x02
#define J_ERR_INVALID_PROTOCOL_ID     0x03
#define J_ERR_NULL_PARAMETER          0x04
#define J_ERR_FAILED                  0x07
#define J_ERR_DEVICE_NOT_CONNECTED    0x08
#define J_ERR_TIMEOUT                 0x09
#define J_ERR_INVALID_MSG             0x0A
#define J_ERR_INVALID_TIME_INTERVAL   0x0B
#define J_ERR_EXCEEDED_LIMIT          0x0C
#define J_ERR_INVALID_MSG_ID          0x0D
#define J_ERR_DEVICE_IN_USE           0x0E
#define J_ERR_INVALID_IOCTL_ID        0x0F
#define J_ERR_BUFFER_EMPTY            0x10
#define J_ERR_CHANNEL_IN_USE          0x14
#define J_ERR_MSG_PROTOCOL_ID         0x15
#define J_ERR_INVALID_FILTER_ID       0x16
#define J_ERR_INVALID_BAUDRATE        0x19
#define J_ERR_INVALID_DEVICE_ID       0x1A

/* Protocol ids (J2534-1, the J2534-2 pin-selected variants, GM's vendor GM UART) */
#define J_J1850VPW        1u
#define J_CAN             5u
#define J_ISO15765        6u
#define J_CAN_PS          0x8004u
#define J_ISO15765_PS     0x8005u
#define J_SW_ISO15765_PS  0x8007u
#define J_SW_CAN_PS       0x8008u
#define J_GM_UART_PS      0x8009u
#define J_GM_UART_VENDOR  0x10003u

/* Ioctl ids */
#define J_GET_CONFIG                          0x01u
#define J_SET_CONFIG                          0x02u
#define J_READ_VBATT                          0x03u
#define J_FIVE_BAUD_INIT                      0x04u
#define J_FAST_INIT                           0x05u
#define J_CLEAR_TX_BUFFER                     0x07u
#define J_CLEAR_RX_BUFFER                     0x08u
#define J_CLEAR_PERIODIC_MSGS                 0x09u
#define J_CLEAR_MSG_FILTERS                   0x0Au
#define J_CLEAR_FUNCT_MSG_LOOKUP_TABLE        0x0Bu
#define J_ADD_TO_FUNCT_MSG_LOOKUP_TABLE       0x0Cu
#define J_DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE  0x0Du
#define J_READ_PROG_VOLTAGE                   0x0Eu
#define J_SW_CAN_HS                           0x8000u
#define J_SW_CAN_NS                           0x8001u

/* Configuration parameters */
#define J_DATA_RATE         0x01u
#define J_LOOPBACK          0x03u
#define J_CAN_MIXED_FORMAT  0x8000u

/* Filter types, TxFlags, RxStatus */
#define J_PASS_FILTER          1u
#define J_BLOCK_FILTER         2u
#define J_FLOW_CONTROL_FILTER  3u
#define J_CAN_29BIT_ID         0x100u
#define J_SW_CAN_HV_TX         0x400u
#define J_TX_MSG_TYPE          0x01u
#define J_TX_DONE              0x08u

#define J_DEVICE_ID     1u
#define J_MAX_DATA      4128   /* PASSTHRU_MSG.Data */
/* Per channel.  J2534-1 only requires 10, but Tech2Win sets one filter per
 * CANdi table entry: 64 for the default GMLAN table. */
#define J_MAX_FILTERS   128
#define J_MAX_PERIODIC  10     /* per channel (J2534-1 minimum) */
#define J_MAX_CHANNELS  16     /* one per supported protocol id is enough */
#define J_MAX_CONFIG    64     /* stored SET_CONFIG parameters per channel */
#define J_RX_QUEUE_MAX  2000   /* per channel; further messages are dropped */

typedef struct {
    unsigned long ProtocolID, RxStatus, TxFlags, Timestamp, DataSize, ExtraDataIndex;
    unsigned char Data[J_MAX_DATA];
} PASSTHRU_MSG;

typedef struct { unsigned long Parameter, Value; } SCONFIG;
typedef struct { unsigned long NumOfParams; SCONFIG *ConfigPtr; } SCONFIG_LIST;

/* A received (or looped back) message waiting for PassThruReadMsgs. */
typedef struct J_RX {
    struct J_RX *next;
    UNUM32 protocol, rxStatus, timestamp, len;
    UNUM8 data[1];
} J_RX;

typedef struct { UNUM32 id, type, len; UNUM8 mask[12], pattern[12], flow[12]; } J_FILTER;

typedef struct {
    UNUM32 id;
    HANDLE timer;               /* timer-queue timer */
    UNUM32 channel, protocol, txFlags, len;
    UNUM8 *data;
} J_PERIODIC;

typedef struct { UNUM32 param, value; } J_CONFIG;

typedef struct {
    UNUM32 id, protocol, flags;
    BUS bus;
    BOOL iso;                   /* ISO 15765: complete messages, flow control filters */
    J_CONFIG config[J_MAX_CONFIG];
    UNUM32 numConfig;
    J_RX *rxHead, *rxTail;
    UNUM32 rxCount;
    BOOL rxFullLogged;
    J_FILTER filters[J_MAX_FILTERS];
    UNUM32 numFilters, nextFilter;
    J_PERIODIC *periodics[J_MAX_PERIODIC];
    UNUM32 numPeriodics, nextPeriodic;
} J_CHANNEL;

static CRITICAL_SECTION   g_jLock;
static CONDITION_VARIABLE g_jCond;       /* received messages / channel gone */
static BOOL       g_jOpen;
static BUS        g_jCurBus = BUS_NONE;
static J_CHANNEL *g_jChannels[J_MAX_CHANNELS];
static UNUM32     g_jNumChannels;
static UNUM32     g_jNextChannel = 1;    /* never reset, like the D-PDU handles */
static char       g_jLastError[80];

/* Called from DllMain. */
static void j2534_init(void)
{
    InitializeCriticalSection(&g_jLock);
    InitializeConditionVariable(&g_jCond);
}

/* ------------------------------------------------------------------------ */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------ */

/* Records the error text for PassThruGetLastError, logs it and returns code. */
static long j_fail(long code, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    _vsnprintf(g_jLastError, sizeof g_jLastError - 1, fmt, ap);
    va_end(ap);
    g_jLastError[sizeof g_jLastError - 1] = 0;
    log_msg("  ReturnValue: 0x%02lX (%s)", code, g_jLastError);
    return code;
}

/* TRUE while a J2534 host has the device open (the links belong to J2534). */
static BOOL j2534_is_open(void)
{
    BOOL open;
    EnterCriticalSection(&g_jLock);
    open = g_jOpen;
    LeaveCriticalSection(&g_jLock);
    return open;
}

/* Caller holds g_jLock. */
static J_CHANNEL *j_find_channel(UNUM32 id)
{
    UNUM32 i;
    for (i = 0; i < g_jNumChannels; i++)
        if (g_jChannels[i]->id == id)
            return g_jChannels[i];
    return NULL;
}

/* Bus and ISO 15765 flag of a protocol id; BUS_NONE when it is not supported. */
static BUS j_protocol_bus(UNUM32 protocol, BOOL *iso)
{
    *iso = protocol == J_ISO15765 || protocol == J_ISO15765_PS || protocol == J_SW_ISO15765_PS;
    switch (protocol) {
    case J_J1850VPW: return BUS_VPW;
    case J_CAN: case J_ISO15765: case J_CAN_PS: case J_ISO15765_PS: return BUS_HSCAN;
    case J_SW_CAN_PS: case J_SW_ISO15765_PS: return BUS_SWCAN;
    case J_GM_UART_PS: case J_GM_UART_VENDOR: return BUS_ALDL;
    default: return BUS_NONE;
    }
}

/* The raw CAN protocol id that goes with an ISO 15765 protocol id (mixed format). */
static UNUM32 j_raw_can_of(UNUM32 protocol)
{
    switch (protocol) {
    case J_ISO15765: return J_CAN;
    case J_ISO15765_PS: return J_CAN_PS;
    case J_SW_ISO15765_PS: return J_SW_CAN_PS;
    default: return protocol;
    }
}

/* Name of a configuration parameter; unknown ones as 0x..., in buf. */
static const char *j_param_name(UNUM32 id, char *buf, size_t size)
{
    static const struct { UNUM32 id; const char *name; } names[] = {
        {0x01, "DATA_RATE"}, {0x03, "LOOPBACK"}, {0x04, "NODE_ADDRESS"}, {0x05, "NETWORK_LINE"},
        {0x06, "P1_MIN"}, {0x07, "P1_MAX"}, {0x08, "P2_MIN"}, {0x09, "P2_MAX"}, {0x0A, "P3_MIN"},
        {0x0B, "P3_MAX"}, {0x0C, "P4_MIN"}, {0x0D, "P4_MAX"}, {0x0E, "W1"}, {0x0F, "W2"}, {0x10, "W3"},
        {0x11, "W4"}, {0x12, "W5"}, {0x13, "TIDLE"}, {0x14, "TINIL"}, {0x15, "TWUP"}, {0x16, "PARITY"},
        {0x17, "BIT_SAMPLE_POINT"}, {0x18, "SYNC_JUMP_WIDTH"}, {0x19, "W0"}, {0x1A, "T1_MAX"},
        {0x1B, "T2_MAX"}, {0x1C, "T4_MAX"}, {0x1D, "T5_MAX"}, {0x1E, "ISO15765_BS"}, {0x1F, "ISO15765_STMIN"},
        {0x20, "DATA_BITS"}, {0x21, "FIVE_BAUD_MOD"}, {0x22, "BS_TX"}, {0x23, "STMIN_TX"}, {0x24, "T3_MAX"},
        {0x25, "ISO15765_WFT_MAX"}, {0x8000, "CAN_MIXED_FORMAT"}, {0x8001, "J1962_PINS"},
    };
    size_t i;
    for (i = 0; i < sizeof names / sizeof names[0]; i++)
        if (names[i].id == id)
            return names[i].name;
    _snprintf(buf, size - 1, "0x%X", id);
    buf[size - 1] = 0;
    return buf;
}

/* Caller holds g_jLock.  FALSE (value 0) for a parameter never set. */
static BOOL j_get_config(const J_CHANNEL *ch, UNUM32 param, UNUM32 *value)
{
    UNUM32 i;
    *value = 0;
    for (i = 0; i < ch->numConfig; i++) {
        if (ch->config[i].param == param) {
            *value = ch->config[i].value;
            return TRUE;
        }
    }
    return FALSE;
}

/* Caller holds g_jLock. */
static void j_set_config(J_CHANNEL *ch, UNUM32 param, UNUM32 value)
{
    UNUM32 i;
    for (i = 0; i < ch->numConfig; i++) {
        if (ch->config[i].param == param) {
            ch->config[i].value = value;
            return;
        }
    }
    if (ch->numConfig < J_MAX_CONFIG) {
        ch->config[ch->numConfig].param = param;
        ch->config[ch->numConfig].value = value;
        ch->numConfig++;
    } else {
        log_msg("channel %u: more than %d configuration parameters - 0x%X not stored", ch->id, J_MAX_CONFIG, param);
    }
}

static void j_copy_str(char *dst, const char *s)
{
    strncpy(dst, s, 79);
    dst[79] = 0;
}

/* ------------------------------------------------------------------------ */
/* Receive path (client lines -> channel queues)                             */
/* ------------------------------------------------------------------------ */

/* Mask / pattern check over the filter's length; shorter messages never match. */
static BOOL j_match(const UNUM8 *mask, const UNUM8 *pattern, UNUM32 n, const UNUM8 *data, UNUM32 len)
{
    UNUM32 k;
    if (len < n)
        return FALSE;
    for (k = 0; k < n; k++)
        if ((data[k] & mask[k]) != (pattern[k] & mask[k]))
            return FALSE;
    return TRUE;
}

/* J2534 pass / block rule: no block filter matches and at least one pass filter does. */
static BOOL j_pass_block(const J_CHANNEL *ch, const UNUM8 *data, UNUM32 len)
{
    UNUM32 i;
    BOOL passed = FALSE;
    for (i = 0; i < ch->numFilters; i++) {
        const J_FILTER *f = &ch->filters[i];
        if (f->type == J_BLOCK_FILTER && j_match(f->mask, f->pattern, f->len, data, len))
            return FALSE;
        if (f->type == J_PASS_FILTER && j_match(f->mask, f->pattern, f->len, data, len))
            passed = TRUE;
    }
    return passed;
}

/* Caller holds g_jLock. */
static void j_enqueue(J_CHANNEL *ch, UNUM32 protocol, UNUM32 rxStatus, const UNUM8 *data, UNUM32 len)
{
    J_RX *m;
    if (ch->rxCount >= J_RX_QUEUE_MAX) {
        if (!ch->rxFullLogged)
            log_msg("channel %08X: receive queue full - messages dropped until the host reads", ch->id);
        ch->rxFullLogged = TRUE;
        return;
    }
    m = (J_RX *)malloc(sizeof *m + len);
    if (!m)
        return;
    ch->rxFullLogged = FALSE;
    m->next = NULL;
    m->protocol = protocol;
    m->rxStatus = rxStatus;
    m->timestamp = now_us();
    m->len = len;
    memcpy(m->data, data, len);
    if (ch->rxTail) ch->rxTail->next = m; else ch->rxHead = m;
    ch->rxTail = m;
    ch->rxCount++;
}

/* Caller holds g_jLock. */
static void j_rx_clear(J_CHANNEL *ch)
{
    while (ch->rxHead) {
        J_RX *m = ch->rxHead;
        ch->rxHead = m->next;
        free(m);
    }
    ch->rxTail = NULL;
    ch->rxCount = 0;
}

/* A received message (CAN: 4-byte id + complete payload) goes to every
 * channel on its bus that accepts it (reader threads).  On an ISO 15765
 * channel, ids that match a flow control filter arrive as ISO 15765 messages
 * of any length; with CAN_MIXED_FORMAT on, other single frames (up to 8
 * bytes) arrive as raw CAN through the pass / block filters.  Every other
 * channel uses only the pass / block filters. */
static void j2534_deliver(BUS bus, const UNUM8 *data, UNUM32 len, UNUM32 canId)
{
    UNUM32 i, k, status = bus_is_can(bus) && canId > 0x7FF ? J_CAN_29BIT_ID : 0;
    BOOL any = FALSE;
    if (len > J_MAX_DATA) {
        log_msg("RX %s dropped: %u bytes is more than a J2534 message holds", bus_name(bus), len);
        return;
    }
    EnterCriticalSection(&g_jLock);
    for (i = 0; i < g_jNumChannels; i++) {
        J_CHANNEL *ch = g_jChannels[i];
        if (ch->bus != bus)
            continue;
        if (ch->iso) {
            BOOL fc = FALSE;
            UNUM32 mixed;
            for (k = 0; k < ch->numFilters && !fc; k++)
                fc = ch->filters[k].type == J_FLOW_CONTROL_FILTER &&
                     j_match(ch->filters[k].mask, ch->filters[k].pattern, ch->filters[k].len, data, len);
            if (fc) {
                j_enqueue(ch, ch->protocol, status, data, len);
                any = TRUE;
                continue;
            }
            if (!j_get_config(ch, J_CAN_MIXED_FORMAT, &mixed) || !mixed || len > 12)
                continue;
            if (j_pass_block(ch, data, len)) {
                j_enqueue(ch, j_raw_can_of(ch->protocol), status, data, len);
                any = TRUE;
            }
        } else if (j_pass_block(ch, data, len)) {
            j_enqueue(ch, ch->protocol, status, data, len);
            any = TRUE;
        }
    }
    if (any)
        WakeAllConditionVariable(&g_jCond);
    else if (bus_is_can(bus))
        log_msg("RX %s %X dropped: no channel / filter accepts it", bus_name(bus), canId);
    else
        log_msg("RX %s dropped: no channel / filter accepts it", bus_name(bus));
    LeaveCriticalSection(&g_jLock);
}

/* ------------------------------------------------------------------------ */
/* Transmit path                                                             */
/* ------------------------------------------------------------------------ */

/* Checks a message to send on ch (caller holds g_jLock): its protocol id is
 * the channel's, or on an ISO 15765 channel also the raw CAN one (mixed
 * format); CAN messages start with the 4-byte id. */
static long j_check_tx(const J_CHANNEL *ch, const PASSTHRU_MSG *m, char *why, size_t size)
{
    BOOL iso;
    UNUM32 min, max;
    if (m->ProtocolID != ch->protocol && !(ch->iso && m->ProtocolID == j_raw_can_of(ch->protocol))) {
        _snprintf(why, size - 1, "message protocol 0x%lX on a 0x%X channel", m->ProtocolID, ch->protocol);
        why[size - 1] = 0;
        return J_ERR_MSG_PROTOCOL_ID;
    }
    iso = m->ProtocolID == ch->protocol && ch->iso;
    max = !bus_is_can(ch->bus) ? J_MAX_DATA : iso ? 4 + 1 + 4095 : 12;
    min = bus_is_can(ch->bus) ? 4 : 1;
    if (m->DataSize < min || m->DataSize > max) {
        _snprintf(why, size - 1, "DataSize %lu not %u..%u", m->DataSize, min, max);
        why[size - 1] = 0;
        return J_ERR_INVALID_MSG;
    }
    return J_STATUS_NOERROR;
}

/* Sends one checked message as a TX line (PROTOCOL first when the active bus
 * changes); no lock held.  Unless periodic, an ISO 15765 message gets a
 * TX_DONE indication and, with LOOPBACK on, the message is echoed with
 * TX_MSG_TYPE. */
static void j_send(UNUM32 channelId, UNUM32 protocol, UNUM32 txFlags, const UNUM8 *data, UNUM32 len, BOOL periodic)
{
    J_CHANNEL *ch;
    BUS bus;
    BOOL busChanged = FALSE;
    EnterCriticalSection(&g_jLock);
    ch = j_find_channel(channelId);
    if (!ch) {
        LeaveCriticalSection(&g_jLock);
        return;
    }
    bus = ch->bus;
    if (g_jCurBus != bus) {
        g_jCurBus = bus;
        busChanged = TRUE;
    }
    if (!periodic) {
        UNUM32 status = bus_is_can(bus) && (txFlags & J_CAN_29BIT_ID) ? J_CAN_29BIT_ID : 0, lb;
        if (ch->iso && protocol == ch->protocol)
            j_enqueue(ch, protocol, J_TX_DONE | status, data, 4);
        if (j_get_config(ch, J_LOOPBACK, &lb) && lb)
            j_enqueue(ch, protocol, J_TX_MSG_TYPE | status, data, len);
        WakeAllConditionVariable(&g_jCond);
    }
    LeaveCriticalSection(&g_jLock);
    if (busChanged)
        link_write_line("PROTOCOL %s", bus_name(bus));
    /* Id 0x100 is sent as a WAKEUP line by transmit() already. */
    if (bus == BUS_SWCAN && (txFlags & J_SW_CAN_HV_TX) && !(len >= 4 && be32(data) == GMLAN_HV_WAKEUP_ID))
        link_write_line("WAKEUP SWCAN");
    transmit(bus, data, len);
}

static VOID CALLBACK j_periodic_cb(PVOID param, BOOLEAN fired)
{
    J_PERIODIC *p = (J_PERIODIC *)param;
    (void)fired;
    j_send(p->channel, p->protocol, p->txFlags, p->data, p->len, TRUE);
}

/* Stops a periodic message (waits for a running callback) and frees it; no lock held. */
static void j_periodic_free(J_PERIODIC *p)
{
    DeleteTimerQueueTimer(NULL, p->timer, INVALID_HANDLE_VALUE);
    free(p->data);
    free(p);
}

/* ------------------------------------------------------------------------ */
/* Exported PassThru API (dpdu_bridge.def)                                   */
/* ------------------------------------------------------------------------ */

long __stdcall PassThruOpen(void *pName, unsigned long *pDeviceID)
{
    BOOL inUse, dpdu;
    const char *ini;
    log_msg("PassThruOpen Called: pName=%s", pName ? (const char *)pName : "(null)");
    if (!pDeviceID)
        return j_fail(J_ERR_NULL_PARAMETER, "pDeviceID is NULL");
    inUse = j2534_is_open();
    if (inUse)
        return j_fail(J_ERR_DEVICE_IN_USE, "the device is already open");
    EnterCriticalSection(&g_lock);
    dpdu = g_moduleStatus == PDU_MODST_READY;
    LeaveCriticalSection(&g_lock);
    if (dpdu)
        return j_fail(J_ERR_DEVICE_IN_USE, "the D-PDU API of this DLL has the device connected");
    if (!use_serial() && !g_tcpPort) {
        log_msg("no valid port specified in %s - no port will be opened", g_iniPath);
        report_port_problem();
        ini = strrchr(g_iniPath, '\\');
        return j_fail(J_ERR_DEVICE_NOT_CONNECTED, "no COM port or TCP port in %s", ini ? ini + 1 : g_iniPath);
    }
    if (!link_open())
        return j_fail(J_ERR_DEVICE_NOT_CONNECTED, "cannot open %s", g_linkDesc);
    EnterCriticalSection(&g_jLock);
    g_jOpen = TRUE;
    g_jCurBus = BUS_NONE;
    LeaveCriticalSection(&g_jLock);
    link_write_line("HELLO " DRIVER_NAME " " DRIVER_VERSION " J2534");
    *pDeviceID = J_DEVICE_ID;
    return J_STATUS_NOERROR;
}

long __stdcall PassThruClose(unsigned long DeviceID)
{
    J_PERIODIC *periodics[J_MAX_CHANNELS * J_MAX_PERIODIC];
    UNUM32 i, k, numPeriodics = 0;
    EnterCriticalSection(&g_jLock);
    log_msg("PassThruClose Called: DeviceID=%lu", DeviceID);
    if (!g_jOpen || DeviceID != J_DEVICE_ID) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_DEVICE_ID, "invalid device id %lu", DeviceID);
    }
    for (i = 0; i < g_jNumChannels; i++) {
        J_CHANNEL *ch = g_jChannels[i];
        for (k = 0; k < ch->numPeriodics; k++)
            periodics[numPeriodics++] = ch->periodics[k];
        j_rx_clear(ch);
        free(ch);
        g_jChannels[i] = NULL;
    }
    g_jNumChannels = 0;
    g_jOpen = FALSE;
    WakeAllConditionVariable(&g_jCond);
    LeaveCriticalSection(&g_jLock);
    for (i = 0; i < numPeriodics; i++)
        j_periodic_free(periodics[i]);
    link_write_line("BYE");
    link_close();
    return J_STATUS_NOERROR;
}

long __stdcall PassThruConnect(unsigned long DeviceID, unsigned long ProtocolID, unsigned long Flags,
                               unsigned long BaudRate, unsigned long *pChannelID)
{
    J_CHANNEL *ch;
    BUS bus;
    BOOL iso;
    UNUM32 i, id;
    EnterCriticalSection(&g_jLock);
    log_msg("PassThruConnect Called: DeviceID=%lu ProtocolID=0x%lX Flags=0x%lX BaudRate=%lu",
            DeviceID, ProtocolID, Flags, BaudRate);
    if (!g_jOpen || DeviceID != J_DEVICE_ID) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_DEVICE_ID, "invalid device id %lu", DeviceID);
    }
    if (!pChannelID) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_NULL_PARAMETER, "pChannelID is NULL");
    }
    bus = j_protocol_bus(ProtocolID, &iso);
    if (bus == BUS_NONE) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_PROTOCOL_ID, "protocol 0x%lX is not supported", ProtocolID);
    }
    for (i = 0; i < g_jNumChannels; i++) {
        if (g_jChannels[i]->protocol == ProtocolID) {
            LeaveCriticalSection(&g_jLock);
            return j_fail(J_ERR_CHANNEL_IN_USE, "protocol 0x%lX is already connected", ProtocolID);
        }
    }
    if (!BaudRate) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_BAUDRATE, "baud rate 0");
    }
    if (g_jNumChannels >= J_MAX_CHANNELS || (ch = (J_CHANNEL *)calloc(1, sizeof *ch)) == NULL) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_FAILED, "out of channels");
    }
    ch->id = id = g_jNextChannel++;
    ch->protocol = ProtocolID;
    ch->flags = Flags;
    ch->bus = bus;
    ch->iso = iso;
    ch->nextFilter = ch->nextPeriodic = 1;
    j_set_config(ch, J_DATA_RATE, BaudRate);
    j_set_config(ch, J_LOOPBACK, 0);
    g_jChannels[g_jNumChannels++] = ch;
    g_jCurBus = bus;
    *pChannelID = id;
    log_msg("  ChannelID=%u bus=%s (total channels %u)", id, bus_name(bus), g_jNumChannels);
    LeaveCriticalSection(&g_jLock);
    link_write_line("OPEN %08X %s PROTOCOL=%lu RAW=%d CHECKSUM=0", id, bus_name(bus), ProtocolID, iso ? 0 : 1);
    link_write_line("PARAM %08X DATA_RATE %lu", id, BaudRate);
    return J_STATUS_NOERROR;
}

long __stdcall PassThruDisconnect(unsigned long ChannelID)
{
    J_PERIODIC *periodics[J_MAX_PERIODIC];
    UNUM32 i, numPeriodics = 0;
    J_CHANNEL *ch = NULL;
    EnterCriticalSection(&g_jLock);
    log_msg("PassThruDisconnect Called: ChannelID=%lu", ChannelID);
    for (i = 0; i < g_jNumChannels; i++) {
        if (g_jChannels[i]->id == ChannelID) {
            ch = g_jChannels[i];
            g_jChannels[i] = g_jChannels[--g_jNumChannels];
            g_jChannels[g_jNumChannels] = NULL;
            break;
        }
    }
    if (!ch) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_CHANNEL_ID, "invalid channel id %lu", ChannelID);
    }
    for (i = 0; i < ch->numPeriodics; i++)
        periodics[numPeriodics++] = ch->periodics[i];
    j_rx_clear(ch);
    free(ch);
    WakeAllConditionVariable(&g_jCond);
    LeaveCriticalSection(&g_jLock);
    for (i = 0; i < numPeriodics; i++)
        j_periodic_free(periodics[i]);
    link_write_line("CLOSE %08X", ChannelID);
    return J_STATUS_NOERROR;
}

long __stdcall PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs,
                                unsigned long Timeout)
{
    static char hex[J_MAX_DATA * 3 + 8];   /* used under g_jLock only */
    J_CHANNEL *ch;
    unsigned long want, n = 0;
    ULONGLONG deadline;
    if (!pMsg || !pNumMsgs)
        return j_fail(J_ERR_NULL_PARAMETER, "pMsg or pNumMsgs is NULL");
    want = *pNumMsgs;
    *pNumMsgs = 0;
    EnterCriticalSection(&g_jLock);
    ch = j_find_channel(ChannelID);
    if (!ch) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_CHANNEL_ID, "invalid channel id %lu", ChannelID);
    }
    deadline = GetTickCount64() + Timeout;
    for (;;) {
        while (n < want && ch->rxHead) {
            J_RX *r = ch->rxHead;
            PASSTHRU_MSG *m = &pMsg[n++];
            ch->rxHead = r->next;
            if (!ch->rxHead) ch->rxTail = NULL;
            ch->rxCount--;
            m->ProtocolID = r->protocol;
            m->RxStatus = r->rxStatus;
            m->TxFlags = 0;
            m->Timestamp = r->timestamp;
            m->DataSize = r->len;
            m->ExtraDataIndex = r->len;
            memcpy(m->Data, r->data, r->len);
            hex_bytes(hex, sizeof hex, r->data, r->len);
            log_msg("PassThruReadMsgs ChannelID=%lu: Protocol=0x%X RxStatus=0x%X %s", ChannelID, r->protocol, r->rxStatus, hex);
            free(r);
        }
        if (n >= want || Timeout == 0)
            break;
        {
            ULONGLONG now = GetTickCount64();
            if (now >= deadline)
                break;
            SleepConditionVariableCS(&g_jCond, &g_jLock, (DWORD)(deadline - now));
        }
        /* The channel may have been disconnected (and freed) meanwhile. */
        ch = j_find_channel(ChannelID);
        if (!ch) {
            LeaveCriticalSection(&g_jLock);
            *pNumMsgs = n;
            return j_fail(J_ERR_INVALID_CHANNEL_ID, "channel %lu was disconnected", ChannelID);
        }
    }
    LeaveCriticalSection(&g_jLock);
    *pNumMsgs = n;
    if (n == 0) {
        j_copy_str(g_jLastError, Timeout == 0 ? "no messages" : "no messages within the timeout");
        return Timeout == 0 ? J_ERR_BUFFER_EMPTY : J_ERR_TIMEOUT;
    }
    if (n < want && Timeout > 0) {
        _snprintf(g_jLastError, sizeof g_jLastError - 1, "only %lu of %lu messages within the timeout", n, want);
        return J_ERR_TIMEOUT;
    }
    return J_STATUS_NOERROR;
}

long __stdcall PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pNumMsgs,
                                 unsigned long Timeout)
{
    J_CHANNEL *ch;
    unsigned long want, i, ok = 0;
    long err = J_STATUS_NOERROR;
    char why[80];
    if (!pMsg || !pNumMsgs)
        return j_fail(J_ERR_NULL_PARAMETER, "pMsg or pNumMsgs is NULL");
    want = *pNumMsgs;
    EnterCriticalSection(&g_jLock);
    log_msg("PassThruWriteMsgs Called: ChannelID=%lu NumMsgs=%lu Timeout=%lu", ChannelID, want, Timeout);
    ch = j_find_channel(ChannelID);
    if (!ch) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_CHANNEL_ID, "invalid channel id %lu", ChannelID);
    }
    for (i = 0; i < want; i++) {
        err = j_check_tx(ch, &pMsg[i], why, sizeof why);
        if (err != J_STATUS_NOERROR) {
            j_fail(err, "message %lu: %s", i, why);
            break;
        }
        ok++;
    }
    LeaveCriticalSection(&g_jLock);
    /* The host's messages stay valid for the whole call. */
    for (i = 0; i < ok; i++)
        j_send(ChannelID, pMsg[i].ProtocolID, pMsg[i].TxFlags, pMsg[i].Data, pMsg[i].DataSize, FALSE);
    *pNumMsgs = ok;
    return err;
}

long __stdcall PassThruStartPeriodicMsg(unsigned long ChannelID, PASSTHRU_MSG *pMsg, unsigned long *pMsgID,
                                        unsigned long TimeInterval)
{
    J_CHANNEL *ch;
    J_PERIODIC *p;
    long err;
    char why[80], hex[J_MAX_DATA * 3 + 8];
    EnterCriticalSection(&g_jLock);
    log_msg("PassThruStartPeriodicMsg Called: ChannelID=%lu TimeInterval=%lu", ChannelID, TimeInterval);
    if (!pMsg || !pMsgID) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_NULL_PARAMETER, "pMsg or pMsgID is NULL");
    }
    ch = j_find_channel(ChannelID);
    if (!ch) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_CHANNEL_ID, "invalid channel id %lu", ChannelID);
    }
    err = j_check_tx(ch, pMsg, why, sizeof why);
    if (err != J_STATUS_NOERROR) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(err, "%s", why);
    }
    if (TimeInterval < 5 || TimeInterval > 65535) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_TIME_INTERVAL, "interval %lu ms not 5..65535", TimeInterval);
    }
    if (ch->numPeriodics >= J_MAX_PERIODIC) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_EXCEEDED_LIMIT, "more than %d periodic messages", J_MAX_PERIODIC);
    }
    p = (J_PERIODIC *)calloc(1, sizeof *p);
    if (p)
        p->data = (UNUM8 *)malloc(pMsg->DataSize);
    if (!p || !p->data) {
        free(p);
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_FAILED, "out of memory");
    }
    p->id = ch->nextPeriodic++;
    p->channel = ch->id;
    p->protocol = pMsg->ProtocolID;
    p->txFlags = pMsg->TxFlags;
    p->len = pMsg->DataSize;
    memcpy(p->data, pMsg->Data, p->len);
    if (!CreateTimerQueueTimer(&p->timer, NULL, j_periodic_cb, p, TimeInterval, TimeInterval, WT_EXECUTEDEFAULT)) {
        free(p->data);
        free(p);
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_FAILED, "cannot create a timer (error %lu)", GetLastError());
    }
    ch->periodics[ch->numPeriodics++] = p;
    *pMsgID = p->id;
    hex_bytes(hex, sizeof hex, p->data, p->len);
    log_msg("  MsgID=%u: %s", p->id, hex);
    LeaveCriticalSection(&g_jLock);
    return J_STATUS_NOERROR;
}

long __stdcall PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long MsgID)
{
    J_CHANNEL *ch;
    J_PERIODIC *p = NULL;
    UNUM32 i;
    EnterCriticalSection(&g_jLock);
    log_msg("PassThruStopPeriodicMsg Called: ChannelID=%lu MsgID=%lu", ChannelID, MsgID);
    ch = j_find_channel(ChannelID);
    if (!ch) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_CHANNEL_ID, "invalid channel id %lu", ChannelID);
    }
    for (i = 0; i < ch->numPeriodics; i++) {
        if (ch->periodics[i]->id == MsgID) {
            p = ch->periodics[i];
            ch->periodics[i] = ch->periodics[--ch->numPeriodics];
            break;
        }
    }
    LeaveCriticalSection(&g_jLock);
    if (!p)
        return j_fail(J_ERR_INVALID_MSG_ID, "invalid periodic message id %lu", MsgID);
    j_periodic_free(p);
    return J_STATUS_NOERROR;
}

/* PASS / BLOCK filters (1-12 bytes, on CAN including the 4-byte id) and, on
 * ISO 15765 channels, FLOW_CONTROL filters.  A flow control filter is also
 * reported as an ECU line, so the simulator learns the request / response id
 * pair. */
long __stdcall PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType, PASSTHRU_MSG *pMaskMsg,
                                      PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg,
                                      unsigned long *pFilterID)
{
    J_CHANNEL *ch;
    J_FILTER *f;
    UNUM32 chId, len;
    BUS bus;
    char mask[40], pattern[40], flow[40];
    J_FILTER copy;
    EnterCriticalSection(&g_jLock);
    log_msg("PassThruStartMsgFilter Called: ChannelID=%lu FilterType=%lu", ChannelID, FilterType);
    ch = j_find_channel(ChannelID);
    if (!ch) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_CHANNEL_ID, "invalid channel id %lu", ChannelID);
    }
    if (!pMaskMsg || !pPatternMsg || !pFilterID || (FilterType == J_FLOW_CONTROL_FILTER && !pFlowControlMsg)) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_NULL_PARAMETER, "pMaskMsg, pPatternMsg, pFlowControlMsg or pFilterID is NULL");
    }
    if (FilterType != J_PASS_FILTER && FilterType != J_BLOCK_FILTER && FilterType != J_FLOW_CONTROL_FILTER) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_FAILED, "invalid filter type %lu", FilterType);
    }
    if (FilterType == J_FLOW_CONTROL_FILTER && !ch->iso) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_MSG_PROTOCOL_ID, "flow control filters need an ISO 15765 channel");
    }
    len = pMaskMsg->DataSize;
    if (len != pPatternMsg->DataSize || len == 0 || len > 12 ||
        (FilterType == J_FLOW_CONTROL_FILTER && (len < 4 || len > 5 || pFlowControlMsg->DataSize != len))) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_MSG, "mask / pattern / flow control sizes %lu / %lu / %lu", pMaskMsg->DataSize,
                      pPatternMsg->DataSize, pFlowControlMsg ? pFlowControlMsg->DataSize : 0);
    }
    if (ch->numFilters >= J_MAX_FILTERS) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_EXCEEDED_LIMIT, "more than %d filters", J_MAX_FILTERS);
    }
    f = &ch->filters[ch->numFilters++];
    memset(f, 0, sizeof *f);
    f->id = ch->nextFilter++;
    f->type = FilterType;
    f->len = len;
    memcpy(f->mask, pMaskMsg->Data, len);
    memcpy(f->pattern, pPatternMsg->Data, len);
    if (FilterType == J_FLOW_CONTROL_FILTER)
        memcpy(f->flow, pFlowControlMsg->Data, len);
    if (bus_is_can(ch->bus)) {
        /* Tech2Win compatibility: the top 3 bits of a CAN id are cleared
         * (Tech2Win marks 29-bit ids with bit 29), and a 5-byte filter
         * (extended address) compares only the 4-byte id, because Tech2Win
         * leaves the 5th byte uninitialised. */
        if (len >= 4) {
            f->mask[0] &= 0x1F;
            f->pattern[0] &= 0x1F;
            f->flow[0] &= 0x1F;
        }
        if (len == 5)
            f->len = 4;
    }
    *pFilterID = f->id;
    log_msg("  FilterID=%u", f->id);
    copy = *f;
    chId = ch->id;
    bus = ch->bus;
    LeaveCriticalSection(&g_jLock);

    hex_bytes(mask, sizeof mask, copy.mask, copy.len);
    hex_bytes(pattern, sizeof pattern, copy.pattern, copy.len);
    if (copy.type == J_FLOW_CONTROL_FILTER) {
        hex_bytes(flow, sizeof flow, copy.flow, copy.len);
        link_write_line("FILTER %08X %s FLOW #%u MASK=%s PATTERN=%s FLOW=%s", chId, bus_name(bus), copy.id,
                        mask, pattern, flow);
        link_write_line("ECU %08X %s URID=%08X REQ=%X RESP=%X FORMAT=5 UUDT=0", chId, bus_name(bus), copy.id,
                        be32(copy.flow), be32(copy.pattern));
    } else {
        link_write_line("FILTER %08X %s %s #%u MASK=%s PATTERN=%s", chId, bus_name(bus),
                        copy.type == J_PASS_FILTER ? "PASS" : "BLOCK", copy.id, mask, pattern);
    }
    return J_STATUS_NOERROR;
}

long __stdcall PassThruStopMsgFilter(unsigned long ChannelID, unsigned long FilterID)
{
    J_CHANNEL *ch;
    UNUM32 i;
    EnterCriticalSection(&g_jLock);
    log_msg("PassThruStopMsgFilter Called: ChannelID=%lu FilterID=%lu", ChannelID, FilterID);
    ch = j_find_channel(ChannelID);
    if (!ch) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_CHANNEL_ID, "invalid channel id %lu", ChannelID);
    }
    for (i = 0; i < ch->numFilters; i++) {
        if (ch->filters[i].id == FilterID) {
            memmove(&ch->filters[i], &ch->filters[i + 1], (ch->numFilters - i - 1) * sizeof ch->filters[0]);
            ch->numFilters--;
            LeaveCriticalSection(&g_jLock);
            return J_STATUS_NOERROR;
        }
    }
    LeaveCriticalSection(&g_jLock);
    return j_fail(J_ERR_INVALID_FILTER_ID, "invalid filter id %lu", FilterID);
}

/* No programmable pins: accepted and logged. */
long __stdcall PassThruSetProgrammingVoltage(unsigned long DeviceID, unsigned long PinNumber, unsigned long Voltage)
{
    log_msg("PassThruSetProgrammingVoltage Called: DeviceID=%lu Pin=%lu Voltage=%lu - ignored", DeviceID, PinNumber, Voltage);
    if (!j2534_is_open() || DeviceID != J_DEVICE_ID)
        return j_fail(J_ERR_INVALID_DEVICE_ID, "invalid device id %lu", DeviceID);
    return J_STATUS_NOERROR;
}

long __stdcall PassThruReadVersion(unsigned long DeviceID, char *pFirmwareVersion, char *pDllVersion, char *pApiVersion)
{
    char fw[80];
    log_msg("PassThruReadVersion Called: DeviceID=%lu", DeviceID);
    if (!pFirmwareVersion || !pDllVersion || !pApiVersion)
        return j_fail(J_ERR_NULL_PARAMETER, "a version buffer is NULL");
    if (!j2534_is_open() || DeviceID != J_DEVICE_ID)
        return j_fail(J_ERR_INVALID_DEVICE_ID, "invalid device id %lu", DeviceID);
    _snprintf(fw, sizeof fw - 1, "%s " DRIVER_VERSION, g_moduleName);
    fw[sizeof fw - 1] = 0;
    j_copy_str(pFirmwareVersion, fw);
    j_copy_str(pDllVersion, DRIVER_NAME " " DRIVER_VERSION);
    j_copy_str(pApiVersion, "04.04");
    return J_STATUS_NOERROR;
}

long __stdcall PassThruGetLastError(char *pErrorDescription)
{
    if (!pErrorDescription)
        return J_ERR_NULL_PARAMETER;
    j_copy_str(pErrorDescription, g_jLastError);
    return J_STATUS_NOERROR;
}

long __stdcall PassThruIoctl(unsigned long ChannelID, unsigned long IoctlID, void *pInput, void *pOutput)
{
    J_CHANNEL *ch;
    J_PERIODIC *periodics[J_MAX_PERIODIC];
    UNUM32 i, numPeriodics = 0, chId;
    SCONFIG_LIST *list = (SCONFIG_LIST *)pInput;
    char name[16];
    BOOL clearFilters = FALSE;

    EnterCriticalSection(&g_jLock);
    log_msg("PassThruIoctl Called: ChannelID=%lu IoctlID=0x%lX", ChannelID, IoctlID);
    if (IoctlID == J_READ_VBATT || IoctlID == J_READ_PROG_VOLTAGE) {
        /* The device id or a channel id. */
        if (!g_jOpen || (ChannelID != J_DEVICE_ID && !j_find_channel(ChannelID))) {
            LeaveCriticalSection(&g_jLock);
            return j_fail(J_ERR_INVALID_DEVICE_ID, "invalid device id %lu", ChannelID);
        }
        LeaveCriticalSection(&g_jLock);
        if (!pOutput)
            return j_fail(J_ERR_NULL_PARAMETER, "pOutput is NULL");
        *(unsigned long *)pOutput = IoctlID == J_READ_VBATT ? (unsigned long)g_vbattMv : 0;
        return J_STATUS_NOERROR;
    }
    ch = j_find_channel(ChannelID);
    if (!ch) {
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_CHANNEL_ID, "invalid channel id %lu", ChannelID);
    }
    chId = ch->id;
    switch (IoctlID) {
    case J_GET_CONFIG:
    case J_SET_CONFIG:
        if (!list || (list->NumOfParams > 0 && !list->ConfigPtr)) {
            LeaveCriticalSection(&g_jLock);
            return j_fail(J_ERR_NULL_PARAMETER, "SCONFIG_LIST is NULL");
        }
        for (i = 0; i < list->NumOfParams; i++) {
            SCONFIG *p = &list->ConfigPtr[i];
            if (IoctlID == J_GET_CONFIG) {
                UNUM32 v;
                j_get_config(ch, p->Parameter, &v);
                p->Value = v;
                log_msg("  GET %s = %lu", j_param_name(p->Parameter, name, sizeof name), p->Value);
            } else {
                j_set_config(ch, p->Parameter, p->Value);
                log_msg("  SET %s = %lu", j_param_name(p->Parameter, name, sizeof name), p->Value);
            }
        }
        break;
    case J_CLEAR_TX_BUFFER:
        break;   /* messages are sent straight away */
    case J_CLEAR_RX_BUFFER:
        j_rx_clear(ch);
        break;
    case J_CLEAR_PERIODIC_MSGS:
        for (i = 0; i < ch->numPeriodics; i++)
            periodics[numPeriodics++] = ch->periodics[i];
        ch->numPeriodics = 0;
        break;
    case J_CLEAR_MSG_FILTERS:
        ch->numFilters = 0;
        clearFilters = TRUE;
        break;
    case J_SW_CAN_HS:
    case J_SW_CAN_NS:
        if (ch->bus != BUS_SWCAN) {
            LeaveCriticalSection(&g_jLock);
            return j_fail(J_ERR_INVALID_IOCTL_ID, "SW_CAN_HS / SW_CAN_NS need a single wire CAN channel");
        }
        break;   /* speed changes are not simulated */
    case J_FIVE_BAUD_INIT:
    case J_FAST_INIT:
    case J_CLEAR_FUNCT_MSG_LOOKUP_TABLE:
    case J_ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
    case J_DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE:
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_NOT_SUPPORTED, "ioctl 0x%lX is not supported", IoctlID);
    default:
        LeaveCriticalSection(&g_jLock);
        return j_fail(J_ERR_INVALID_IOCTL_ID, "invalid ioctl id 0x%lX", IoctlID);
    }
    LeaveCriticalSection(&g_jLock);

    for (i = 0; i < numPeriodics; i++)
        j_periodic_free(periodics[i]);
    /* PARAM lines are written from the host's list, which stays valid for the call. */
    if (IoctlID == J_SET_CONFIG)
        for (i = 0; i < list->NumOfParams; i++)
            link_write_line("PARAM %08X %s %lu", chId, j_param_name(list->ConfigPtr[i].Parameter, name, sizeof name),
                            list->ConfigPtr[i].Value);
    if (clearFilters)
        link_write_line("FILTERCLR %08X", chId);
    return J_STATUS_NOERROR;
}
