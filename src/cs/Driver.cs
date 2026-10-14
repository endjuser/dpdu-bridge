// D-PDU API driver logic, a port of the C reference implementation
// (src/c/dpdu_bridge.c) with the same behaviour, log messages and wire
// protocol (README.md, "Driver behaviour" and "Line protocol").
//
// Exports forwards each API call here. API calls translate into text lines
// written through Link; lines received by Link come back through ProcessLine
// (on a link thread) and become result events for the host.
//
// Threading: all driver state is guarded by Sync. Link writes and host
// callbacks happen outside the lock, because a callback may call back into
// the API and the link threads take Sync to deliver received frames.
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;

namespace DpduBridge;

/// <summary>Bus of a logical link; the names are the &lt;BUS&gt; words of the line protocol.</summary>
internal enum Bus { None, HSCAN, SWCAN, VPW, ALDL }

/// <summary>Expected response of the receive ComPrimitive; Mask/Pattern apply to the 4-byte CAN id.</summary>
internal struct ExpResp { public uint RespType, Mask, Pattern, AcceptanceId; }

/// <summary>One entry of the unique response id table (PDUSetUniqueRespIdTable).</summary>
internal struct UridEntry { public uint UniqueRespId, PhysReqId, RespUsdtId, RespFormat, RespUudtId; }

/// <summary>A START_MSG_FILTER entry, applied to received VPW / ALDL frames on <see cref="Bus"/>.</summary>
internal struct SwFilter { public Bus Bus; public uint Type; public byte[] Mask, Pattern; }

/// <summary>A ComLogicalLink and the state of its ComPrimitives.</summary>
internal sealed unsafe class Cll
{
    public uint Handle;
    public Bus Bus;
    public void* CllTag;
    public uint TxIndEnable;            // CP_TransmitIndEnable
    public uint HStartComm, HRxCop, HTxCop;
    public void* RxCopTag, TxCopTag;
    public uint RxCopStatus = Pdu.COPST_IDLE, CllStatus = Pdu.CLLST_OFFLINE;
    public ExpResp[] ExpResp = Array.Empty<ExpResp>();
    public UridEntry[] Urid = Array.Empty<UridEntry>();
    // Event callback registered for this link (PDURegisterEventCallback).
    public uint CbHMod, CbHCll;
    public nint CbFn;                   // delegate* unmanaged[Stdcall]<uint,uint,uint,void*,void*,void>
}

/// <summary>
/// A host event callback captured under the state lock and invoked after it
/// has been released (the host may call back into the API from it).
/// </summary>
internal unsafe struct CbCall
{
    public nint Fn;
    public uint HMod, HCll;
    public void* CllTag, ApiTag;

    public void Fire()
    {
        if (Fn != 0)
            ((delegate* unmanaged[Stdcall]<uint, uint, uint, void*, void*, void>)Fn)(Pdu.EVT_DATA_AVAILABLE, HMod, HCll, CllTag, ApiTag);
    }
}

/// <summary>Implementation of the D-PDU API functions (one module, any number of logical links).</summary>
internal static unsafe class Driver
{
    // ---- constants ----------------------------------------------------------------
    private const uint ModuleHandle = 1, ModuleTypeId = 1;
    private const uint HcopUpdateParam = 0x02100007;
    private const uint BusTypeDwCan = 3, BusTypeVpw = 12, BusTypeSwCan = 14, BusTypeJ2740 = 16;
    private const uint CpCanPhysReqId = 31, CpCanRespUsdtFormat = 33, CpCanRespUsdtId = 34,
                       CpCanRespUudtId = 37, CpTransmitIndEnable = 190;
    private const uint IoctlReadVbatt = 6, IoctlStartMsgFilter = 12, IoctlClearMsgFilter = 14;
    private const uint GmlanHvWakeupId = 0x100;
    private const int MaxFrame = 4200;   // ISO-TP max 4095 + 4-byte CAN id, rounded up
    // Sizes of the C version's text buffers; output that would not fit there
    // is cut at the same place here.
    private const int TxHexSize = MaxFrame * 3 + 8;   // hex of a TX message
    private const int FilterHexSize = 40;             // hex of a filter mask / pattern
    private const int FilterLineMax = 159;            // FILTER line
    private const int MaxFilterLines = 32;            // FILTER lines per PDUIoCtl call
    private const int EcuLineMax = 127;               // ECU line

    // PDUGetObjectId table
    private static readonly (uint Type, string Name, uint Id)[] Objects =
    {
        (Pdu.OBJT_PROTOCOL, "ISO_11898_RAW", 6), (Pdu.OBJT_PROTOCOL, "ISO_14230_3_on_ISO_14230_2", 8),
        (Pdu.OBJT_PROTOCOL, "ISO_15031_5_on_ISO_15765_4", 11), (Pdu.OBJT_PROTOCOL, "ISO_15765_3_on_ISO_15765_2", 16),
        (Pdu.OBJT_PROTOCOL, "KW309_on_KW_UART", 22), (Pdu.OBJT_PROTOCOL, "KW81_on_KW_UART", 24),
        (Pdu.OBJT_PROTOCOL, "KW82_on_KW_UART", 25), (Pdu.OBJT_PROTOCOL, "SAE_J1939_73_on_SAE_J1939_21", 34),
        (Pdu.OBJT_PROTOCOL, "SAE_J2190_on_ISO_9141_2", 38), (Pdu.OBJT_PROTOCOL, "SAE_J2190_on_SAE_J1850_VPW", 40),
        (Pdu.OBJT_PROTOCOL, "XDE_5024_on_XDE_5024_UART", 46),
        (Pdu.OBJT_BUSTYPE, "ISO_11898_2_DWCAN", 3), (Pdu.OBJT_BUSTYPE, "ISO_11898_3_DWFTCAN", 4),
        (Pdu.OBJT_BUSTYPE, "ISO_14230_1_UART", 6), (Pdu.OBJT_BUSTYPE, "ISO_9141_2_UART", 7),
        (Pdu.OBJT_BUSTYPE, "KW_UART", 8), (Pdu.OBJT_BUSTYPE, "SAE_J1850_VPW", 12),
        (Pdu.OBJT_BUSTYPE, "SAE_J1939_11_DWCAN", 13), (Pdu.OBJT_BUSTYPE, "SAE_J2411_SWCAN", 14),
        (Pdu.OBJT_BUSTYPE, "SAE_J2740_UART", 16),
        (Pdu.OBJT_IO_CTRL, "PDU_IOCTL_READ_VBATT", 6), (Pdu.OBJT_IO_CTRL, "PDU_IOCTL_START_MSG_FILTER", 12),
        (Pdu.OBJT_IO_CTRL, "PDU_IOCTL_CLEAR_MSG_FILTER", 14),
        (Pdu.OBJT_PINTYPE, "HI", 1), (Pdu.OBJT_PINTYPE, "LOW", 2), (Pdu.OBJT_PINTYPE, "K", 3),
        (Pdu.OBJT_PINTYPE, "L", 4), (Pdu.OBJT_PINTYPE, "PLUS", 7), (Pdu.OBJT_PINTYPE, "SINGLE", 9),
        (Pdu.OBJT_COMPARAM, "CP_5BaudAddressPhys", 2), (Pdu.OBJT_COMPARAM, "CP_5BaudCommBaudrateOverride", 3),
        (Pdu.OBJT_COMPARAM, "CP_5BaudInitBaudrate", 4), (Pdu.OBJT_COMPARAM, "CP_5BaudMode", 5),
        (Pdu.OBJT_COMPARAM, "CP_As", 8), (Pdu.OBJT_COMPARAM, "CP_Baudrate", 9),
        (Pdu.OBJT_COMPARAM, "CP_BecomeMaster", 10), (Pdu.OBJT_COMPARAM, "CP_BecomeSlave", 11),
        (Pdu.OBJT_COMPARAM, "CP_BitSamplePoint", 12), (Pdu.OBJT_COMPARAM, "CP_Bs", 17),
        (Pdu.OBJT_COMPARAM, "CP_ByteCountOffset", 18), (Pdu.OBJT_COMPARAM, "CP_CanFillerByte", 21),
        (Pdu.OBJT_COMPARAM, "CP_CanFillerByteHandling", 22), (Pdu.OBJT_COMPARAM, "CP_CanPhysReqExtAddr", 29),
        (Pdu.OBJT_COMPARAM, "CP_CanPhysReqFormat", 30), (Pdu.OBJT_COMPARAM, "CP_CanPhysReqId", 31),
        (Pdu.OBJT_COMPARAM, "CP_CanRespUSDTExtAddr", 32), (Pdu.OBJT_COMPARAM, "CP_CanRespUSDTFormat", 33),
        (Pdu.OBJT_COMPARAM, "CP_CanRespUSDTId", 34), (Pdu.OBJT_COMPARAM, "CP_CanRespUUDTExtAddr", 35),
        (Pdu.OBJT_COMPARAM, "CP_CanRespUUDTFormat", 36), (Pdu.OBJT_COMPARAM, "CP_CanRespUUDTId", 37),
        (Pdu.OBJT_COMPARAM, "CP_Cr", 51), (Pdu.OBJT_COMPARAM, "CP_Cs", 52),
        (Pdu.OBJT_COMPARAM, "CP_HeaderFormatKW", 79), (Pdu.OBJT_COMPARAM, "CP_InitializationSettings", 86),
        (Pdu.OBJT_COMPARAM, "CP_K_L_LineInit", 104), (Pdu.OBJT_COMPARAM, "CP_Loopback", 112),
        (Pdu.OBJT_COMPARAM, "CP_MessageScheduler", 120), (Pdu.OBJT_COMPARAM, "CP_P2Max", 126),
        (Pdu.OBJT_COMPARAM, "CP_P2Min", 127), (Pdu.OBJT_COMPARAM, "CP_P3Max", 130),
        (Pdu.OBJT_COMPARAM, "CP_P3Min", 131), (Pdu.OBJT_COMPARAM, "CP_P3Min_J2740", 132),
        (Pdu.OBJT_COMPARAM, "CP_P3Phys", 133), (Pdu.OBJT_COMPARAM, "CP_P4Max", 134),
        (Pdu.OBJT_COMPARAM, "CP_P4Min", 135), (Pdu.OBJT_COMPARAM, "CP_RepeatReqCountApp", 150),
        (Pdu.OBJT_COMPARAM, "CP_RequestAddrMode", 152), (Pdu.OBJT_COMPARAM, "CP_SamplesPerBit", 153),
        (Pdu.OBJT_COMPARAM, "CP_SetPollResponse", 160), (Pdu.OBJT_COMPARAM, "CP_SetRelinquishMastership", 161),
        (Pdu.OBJT_COMPARAM, "CP_StartMsgIndEnable", 164), (Pdu.OBJT_COMPARAM, "CP_SwCan_HighVoltage", 166),
        (Pdu.OBJT_COMPARAM, "CP_SyncJumpWidth", 167), (Pdu.OBJT_COMPARAM, "CP_TIdle", 169),
        (Pdu.OBJT_COMPARAM, "CP_TInil", 170), (Pdu.OBJT_COMPARAM, "CP_TWup", 171),
        (Pdu.OBJT_COMPARAM, "CP_TerminationType", 177), (Pdu.OBJT_COMPARAM, "CP_TesterPresentHandling", 183),
        (Pdu.OBJT_COMPARAM, "CP_TransmitIndEnable", 190), (Pdu.OBJT_COMPARAM, "CP_W1Max", 194),
        (Pdu.OBJT_COMPARAM, "CP_W1Min", 195), (Pdu.OBJT_COMPARAM, "CP_W2Max", 196),
        (Pdu.OBJT_COMPARAM, "CP_W2Min", 197), (Pdu.OBJT_COMPARAM, "CP_W3Max", 198),
        (Pdu.OBJT_COMPARAM, "CP_W3Min", 199), (Pdu.OBJT_COMPARAM, "CP_W4Max", 200),
        (Pdu.OBJT_COMPARAM, "CP_W4Min", 201),
    };

    // ---- state ----------------------------------------------------------------
    private static readonly object Sync = new();
    private static bool _constructed;
    private static void* _apiTag;
    private static uint _moduleStatus = Pdu.MODST_AVAIL;
    private static uint _connectedMod;
    private static bool _moduleListed;
    private static Bus _curBus = Bus.None;
    private static nint _sysCbFn, _modCbFn;   // stored as registered; never called
    private static readonly List<Cll> Clls = new();
    private static readonly Queue<nint> Events = new();          // PDU_EVENT_ITEM*, one queue for module and all CLLs
    private static readonly List<SwFilter> Filters = new();
    // Every native item handed to the host, so PDUDestroyItem only frees
    // memory this DLL allocated.
    private static readonly HashSet<nint> Items = new();

    // Handle counters: never reset by PDUDestruct, only at DLL load.
    private static uint _nextCll = 0x02000002, _nextRxCop = 0x02100002, _nextTxCop = 0x02200004, _nextStart = 0x02500001;
    // Written by the link threads (VBATT lines) without the lock.
    private static uint _vbattMv;

    // Runs on the first API call (the counterpart of the C version's DllMain).
    static Driver()
    {
        Config.Load();
        _vbattMv = Config.VBattMilliVolts;
        Link.LineReceived = ProcessLine;
        Log.Write($"{Config.DriverName} loaded (port {(Config.LinkDesc.Length > 0 ? Config.LinkDesc : "not specified")})");
    }

    // ---- helpers ----------------------------------------------------------------
    private static readonly double UsPerTick = 1_000_000.0 / System.Diagnostics.Stopwatch.Frequency;

    /// <summary>Microseconds, truncated to 32 bits.</summary>
    private static uint NowUs() => (uint)(ulong)(System.Diagnostics.Stopwatch.GetTimestamp() * UsPerTick);

    private static string BusName(Bus b) => b == Bus.None ? "NONE" : b.ToString();
    private static bool IsCan(Bus b) => b == Bus.HSCAN || b == Bus.SWCAN;
    private static uint Be32(byte* p) => (uint)(p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]);

    /// <summary>
    /// "01 02 0A" (the C version's hex_bytes): output that would not fit in a
    /// C buffer of <paramref name="outSize"/> bytes is left out.
    /// </summary>
    private static string HexBytes(ReadOnlySpan<byte> p, int outSize)
    {
        var sb = new StringBuilder(p.Length * 3);
        int pos = 0;
        for (int i = 0; i < p.Length && pos + 4 < outSize; i++)
        {
            if (i > 0)
            {
                sb.Append(' ');
                pos++;
            }
            sb.Append(p[i].ToString("X2"));
            pos += 2;
        }
        return sb.ToString();
    }

    private static string Cut(string s, int max) => s.Length > max ? s.Substring(0, max) : s;

    /// <summary>A NUL-terminated string from the host, one char per byte (as the C version logs it).</summary>
    private static string CStr(byte* p)
    {
        int n = 0;
        while (p[n] != 0)
            n++;
        return Link.Wire.GetString(p, n);
    }

    private static string ComParamName(uint id)
    {
        foreach (var o in Objects)
            if (o.Type == Pdu.OBJT_COMPARAM && o.Id == id)
                return o.Name;
        return null;
    }

    /// <summary>
    /// Reads a scalar ComParam value with the width its ComParamDataType gives
    /// (signed types sign-extended to 32 bits). A scalar with no data reads as
    /// 0. Returns false, without touching pComParamData, for any other data
    /// type (byte / struct / long fields): those are not supported as values.
    /// </summary>
    private static bool ComParamScalar(PDU_PARAM_ITEM* p, out uint value)
    {
        void* d = p->pComParamData;
        value = 0;
        switch (p->ComParamDataType)
        {
            case Pdu.PT_UNUM8: if (d != null) value = *(byte*)d; return true;
            case Pdu.PT_SNUM8: if (d != null) value = (uint)(int)*(sbyte*)d; return true;
            case Pdu.PT_UNUM16: if (d != null) value = *(ushort*)d; return true;
            case Pdu.PT_SNUM16: if (d != null) value = (uint)(int)*(short*)d; return true;
            case Pdu.PT_UNUM32: if (d != null) value = *(uint*)d; return true;
            case Pdu.PT_SNUM32: if (d != null) value = (uint)*(int*)d; return true;
            default: return false;
        }
    }

    private static Cll FindCll(uint h) => Clls.FirstOrDefault(c => c.Handle == h);

    private static void* ItemAlloc(int size)
    {
        void* p = System.Runtime.InteropServices.NativeMemory.AllocZeroed((nuint)size);
        Items.Add((nint)p);
        return p;
    }

    private static bool ItemFree(void* p)
    {
        if (!Items.Remove((nint)p))
            return false;
        System.Runtime.InteropServices.NativeMemory.Free(p);
        return true;
    }

    private static void Enqueue(PDU_EVENT_ITEM* ev)
    {
        if (ev != null)
            Events.Enqueue((nint)ev);
    }

    /// <summary>PDU_IT_STATUS event; pData points at a single status code.</summary>
    private static PDU_EVENT_ITEM* StatusEvent(uint hCop, void* tag, uint ts, uint status)
    {
        var ev = (PDU_EVENT_ITEM*)ItemAlloc(sizeof(PDU_EVENT_ITEM) + 4);
        ev->ItemType = Pdu.IT_STATUS;
        ev->hCop = hCop;
        ev->pCoPTag = tag;
        ev->Timestamp = ts;
        ev->pData = ev + 1;
        *(uint*)ev->pData = status;
        return ev;
    }

    /// <summary>PDU_IT_RESULT event; the whole tree lives in one allocation.</summary>
    private static PDU_EVENT_ITEM* ResultEvent(uint hCop, void* tag, ReadOnlySpan<byte> rxFlag, uint urid,
                                               uint acceptanceId, ReadOnlySpan<byte> data)
    {
        int size = sizeof(PDU_EVENT_ITEM) + sizeof(PDU_RESULT_DATA) + sizeof(PDU_EXTRA_INFO) + 8 + data.Length;
        var ev = (PDU_EVENT_ITEM*)ItemAlloc(size);
        var res = (PDU_RESULT_DATA*)(ev + 1);
        var extra = (PDU_EXTRA_INFO*)(res + 1);
        var flags = (byte*)(extra + 1);
        rxFlag.CopyTo(new Span<byte>(flags, 4));     // RxFlag; TimestampFlags (flags+4) stay zero
        res->RxFlag.NumFlagBytes = 4;
        res->RxFlag.pFlagData = flags;
        res->UniqueRespIdentifier = urid;
        res->AcceptanceId = acceptanceId;
        res->TimestampFlags.NumFlagBytes = 4;
        res->TimestampFlags.pFlagData = flags + 4;
        res->pExtraInfo = extra;
        res->NumDataBytes = (uint)data.Length;
        res->pDataBytes = data.Length > 0 ? flags + 8 : null;
        data.CopyTo(new Span<byte>(flags + 8, data.Length));
        ev->ItemType = Pdu.IT_RESULT;
        ev->hCop = hCop;
        ev->pCoPTag = tag;
        ev->Timestamp = NowUs();
        ev->pData = res;
        return ev;
    }

    /// <summary>CLL callback: registered hMod/hCLL, CLL tag, API tag.</summary>
    private static CbCall CllCallback(Cll c) =>
        new() { Fn = c.CbFn, HMod = c.CbHMod, HCll = c.CbHCll, CllTag = c.CllTag, ApiTag = _apiTag };

    /// <summary>
    /// Caller holds Sync. Null when the CLL can take a result event; otherwise
    /// why not, worded to precede "logical link &lt;hCLL&gt;". Results are only
    /// queued while a receive ComPrimitive is running and the host has
    /// registered an event callback for the CLL (a module-level callback does
    /// not count), so a host that is not listening cannot make the event queue
    /// grow without bound.
    /// </summary>
    private static string RxUnavailable(Cll c)
    {
        if (c.RxCopStatus != Pdu.COPST_EXECUTING)
            return "no receive ComPrimitive running on";
        if (c.CbFn == 0)
            return "no event callback registered for";
        return null;
    }

    /// <summary>Back to the state after load (handle counters keep counting). Caller holds Sync.</summary>
    private static void ResetState()
    {
        Clls.Clear();
        while (Events.Count > 0)
            ItemFree((void*)Events.Dequeue());
        Filters.Clear();
        _moduleStatus = Pdu.MODST_AVAIL;
        _curBus = Bus.None;
        _sysCbFn = _modCbFn = 0;
        _moduleListed = false;
    }

    // ---- receive path (client lines -> events) ----------------------------------
    //
    // The parsing follows the C version character for character: words are
    // separated by spaces / tabs, data bytes also by commas, and numbers are
    // read with strtoul() semantics.

    private static bool IsBlank(char ch) => ch == ' ' || ch == '\t';

    private static void SkipBlanks(string s, ref int pos)
    {
        while (pos < s.Length && IsBlank(s[pos]))
            pos++;
    }

    /// <summary>The word at pos (up to a space / tab); pos moves past it and one separator.</summary>
    private static string Word(string s, ref int pos)
    {
        int start = pos;
        while (pos < s.Length && !IsBlank(s[pos]))
            pos++;
        string w = s.Substring(start, pos - start);
        if (pos < s.Length)
            pos++;
        return w;
    }

    /// <summary>Handles one line from a client (link thread): RX frames, VBATT; '#' lines are comments.</summary>
    private static void ProcessLine(string line)
    {
        int p = 0;
        SkipBlanks(line, ref p);
        if (p == line.Length || line[p] == '#')
            return;
        line = line.Substring(p);
        Log.Write("<< " + line);
        int pos = 0;
        string cmd = Word(line, ref pos);

        if (cmd.Equals("RX", StringComparison.OrdinalIgnoreCase))
        {
            SkipBlanks(line, ref pos);
            string busTok = Word(line, ref pos);
            Bus bus = ParseBus(busTok);
            if (bus == Bus.None)
            {
                Log.Write($"RX: unknown bus '{busTok}'");
                return;
            }
            if (IsCan(bus))
            {
                SkipBlanks(line, ref pos);
                string idTok = Word(line, ref pos);
                uint canId = StrToUL(idTok, 16, out int end);
                if (idTok.Length == 0 || end != idTok.Length)
                {
                    Log.Write($"RX: bad CAN id '{idTok}'");
                    return;
                }
                var payload = ParseHexBytes(line.Substring(pos), MaxFrame - 4);
                if (payload == null) { Log.Write("RX: bad data bytes"); return; }
                DeliverCan(bus, canId, payload);
            }
            else
            {
                var frame = ParseHexBytes(line.Substring(pos), MaxFrame);
                if (frame == null || frame.Length == 0) { Log.Write("RX: bad data bytes"); return; }
                DeliverUart(bus, frame);
            }
        }
        else if (cmd.Equals("VBATT", StringComparison.OrdinalIgnoreCase))
        {
            Volatile.Write(ref _vbattMv, StrToUL(line.Substring(pos), 10, out _));
        }
        else
        {
            Log.Write($"unknown command '{cmd}' ignored");
        }
    }

    private static Bus ParseBus(string s)
    {
        if (s.Equals("HSCAN", StringComparison.OrdinalIgnoreCase) || s.Equals("CAN", StringComparison.OrdinalIgnoreCase)) return Bus.HSCAN;
        if (s.Equals("SWCAN", StringComparison.OrdinalIgnoreCase) || s.Equals("GMLAN", StringComparison.OrdinalIgnoreCase)) return Bus.SWCAN;
        if (s.Equals("VPW", StringComparison.OrdinalIgnoreCase) || s.Equals("J1850", StringComparison.OrdinalIgnoreCase)) return Bus.VPW;
        if (s.Equals("ALDL", StringComparison.OrdinalIgnoreCase) || s.Equals("UART", StringComparison.OrdinalIgnoreCase)) return Bus.ALDL;
        return Bus.None;
    }

    private static int HexVal(char c) =>
        c >= '0' && c <= '9' ? c - '0' :
        c >= 'A' && c <= 'F' ? c - 'A' + 10 :
        c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;

    /// <summary>
    /// Hex byte tokens ("01 02 0A", "01020A" or "0x0A", separated by spaces,
    /// tabs or commas); null on bad input or more than max bytes.
    /// </summary>
    private static byte[] ParseHexBytes(string s, int max)
    {
        var bytes = new List<byte>();
        foreach (string token in s.Split(new[] { ' ', '\t', ',' }, StringSplitOptions.RemoveEmptyEntries))
        {
            string t = token;
            if (t.Length > 2 && t[0] == '0' && (t[1] == 'x' || t[1] == 'X'))
                t = t.Substring(2);
            if (t.Length <= 2)
            {
                int v = 0;
                foreach (char ch in t)
                {
                    int h = HexVal(ch);
                    if (h < 0)
                        return null;
                    v = v * 16 + h;
                }
                if (bytes.Count >= max)
                    return null;
                bytes.Add((byte)v);
            }
            else
            {
                if ((t.Length & 1) != 0)
                    return null;
                for (int j = 0; j < t.Length; j += 2)
                {
                    int hi = HexVal(t[j]), lo = HexVal(t[j + 1]);
                    if (hi < 0 || lo < 0 || bytes.Count >= max)
                        return null;
                    bytes.Add((byte)(hi * 16 + lo));
                }
            }
        }
        return bytes.ToArray();
    }

    /// <summary>
    /// strtoul() as in the Microsoft C runtime: leading white space, optional
    /// sign (negative values wrap), optional 0x for base 16, overflow gives
    /// 0xFFFFFFFF. end is the index after the number, or 0 when there are no
    /// digits.
    /// </summary>
    private static uint StrToUL(string s, int radix, out int end)
    {
        int i = 0;
        while (i < s.Length && (s[i] == ' ' || (s[i] >= '\t' && s[i] <= '\r')))
            i++;
        bool negative = false;
        if (i < s.Length && (s[i] == '+' || s[i] == '-'))
        {
            negative = s[i] == '-';
            i++;
        }
        if (radix == 16 && i + 1 < s.Length && s[i] == '0' && (s[i + 1] == 'x' || s[i + 1] == 'X'))
            i += 2;
        int start = i;
        ulong v = 0;
        bool overflow = false;
        for (; i < s.Length; i++)
        {
            int d = HexVal(s[i]);
            if (d < 0 || d >= radix)
                break;
            v = v * (ulong)radix + (ulong)d;
            if (v > uint.MaxValue)
            {
                overflow = true;
                v = uint.MaxValue;
            }
        }
        if (i == start)
        {
            end = 0;
            return 0;
        }
        end = i;
        if (overflow)
            return uint.MaxValue;
        return negative ? unchecked((uint)-(long)v) : (uint)v;
    }

    /// <summary>
    /// START_MSG_FILTER check on the header bytes (3 for VPW, 1 for ALDL): a
    /// matching block filter drops the frame; when pass filters exist, one of
    /// them must match.
    /// </summary>
    private static bool SwFilterPass(Bus bus, byte[] frame)
    {
        if (!Config.SoftwareFilters)
            return true;
        int n = bus == Bus.ALDL ? 1 : 3;
        bool anyPass = false, passed = false;
        foreach (var f in Filters)
        {
            if (f.Bus != bus)
                continue;
            bool match = true;
            for (int k = 0; k < n; k++)
            {
                byte b = k < frame.Length ? frame[k] : (byte)0;
                if ((b & f.Mask[k]) != (f.Pattern[k] & f.Mask[k])) { match = false; break; }
            }
            if (f.Type == Pdu.FLT_PASS || f.Type == Pdu.FLT_PASS_UUDT)
            {
                anyPass = true;
                passed |= match;
            }
            else if (match)
            {
                return false;
            }
        }
        return !anyPass || passed;
    }

    /// <summary>
    /// VPW / ALDL frame: goes to the receive ComPrimitive of the first CLL on
    /// the bus, with UniqueRespIdentifier = ID_UNDEF and AcceptanceId = 1.
    /// </summary>
    private static void DeliverUart(Bus bus, byte[] frame)
    {
        CbCall cb;
        lock (Sync)
        {
            var c = Clls.FirstOrDefault(x => x.Bus == bus);
            if (c == null || _curBus != bus)
            {
                Log.Write($"RX {BusName(bus)} dropped: no logical link on the active bus ({BusName(_curBus)})");
                return;
            }
            string why = RxUnavailable(c);
            if (why != null)
            {
                Log.Write($"RX {BusName(bus)} dropped: {why} logical link {c.Handle:X8}");
                return;
            }
            if (!SwFilterPass(bus, frame))
            {
                Log.Write($"RX {BusName(bus)} dropped by message filter");
                return;
            }
            Enqueue(ResultEvent(c.HRxCop, c.RxCopTag, stackalloc byte[4], Pdu.ID_UNDEF, 1, frame));
            cb = CllCallback(c);
        }
        cb.Fire();
    }

    /// <summary>
    /// CAN message: matched against the unique response id table (USDT id for
    /// format 5, UUDT id for format 0) and the receive ComPrimitive's expected
    /// responses. The first matching CLL is the target; if it cannot take the
    /// result the message is dropped, not offered to another CLL.
    /// </summary>
    private static void DeliverCan(Bus bus, uint canId, byte[] payload)
    {
        CbCall cb;
        lock (Sync)
        {
            if (_curBus != bus)
            {
                Log.Write($"RX {BusName(bus)} {canId:X} dropped: active bus is {BusName(_curBus)}");
                return;
            }
            Cll hit = null;
            uint urid = 0, acc = 0, fmt = 0;
            foreach (var c in Clls)
            {
                if (c.Bus != bus || hit != null)
                    continue;
                foreach (var u in c.Urid)
                {
                    bool idMatch;
                    if (u.RespFormat == 5) idMatch = u.RespUsdtId == canId;
                    else if (u.RespFormat == 0) idMatch = u.RespUudtId == canId;
                    else continue; // unsupported response format
                    if (!idMatch)
                        continue;
                    foreach (var x in c.ExpResp)
                    {
                        if ((x.Pattern & x.Mask) == (canId & x.Mask))
                        {
                            hit = c;
                            urid = u.UniqueRespId;
                            fmt = u.RespFormat;
                            acc = x.AcceptanceId;
                            break;
                        }
                    }
                    if (hit != null)
                        break;
                }
            }
            if (hit == null)
            {
                Log.Write($"RX {BusName(bus)} {canId:X}: match not found (no unique response id / expected response)");
                return;
            }
            string why = RxUnavailable(hit);
            if (why != null)
            {
                Log.Write($"RX {BusName(bus)} {canId:X} dropped: {why} logical link {hit.Handle:X8}");
                return;
            }
            var data = new byte[payload.Length + 4];
            data[0] = (byte)(canId >> 24); data[1] = (byte)(canId >> 16); data[2] = (byte)(canId >> 8); data[3] = (byte)canId;
            payload.CopyTo(data, 4);
            ReadOnlySpan<byte> flags = fmt != 0 ? stackalloc byte[] { 0, 0, 0, 0x40 } : stackalloc byte[] { 0x80, 0, 0, 0 };
            Enqueue(ResultEvent(hit.HRxCop, hit.RxCopTag, flags, urid, acc, data));
            cb = CllCallback(hit);
        }
        cb.Fire();
    }

    // ---- API (called through Exports) ------------------------------------------------
    //
    // Log lines, including the "ReturnValue: ..." lines on some error paths,
    // match the C version's.

    public static uint Construct(byte* option, void* apiTag)
    {
        uint ret;
        bool badPort = false;
        lock (Sync)
        {
            Log.Write($"PDUConstruct Called: pszOption={(option == null ? "(null)" : CStr(option))} pAPITag={(nint)apiTag:X8}");
            _apiTag = apiTag;
            if (_constructed)
            {
                ret = Pdu.ERR_SHARING_VIOLATION;
            }
            else
            {
                ResetState();
                if (Config.SerialBad)
                    Log.Write($"[Serial] Port={Config.PortName} is not a COM port name - serial link disabled");
                if (Config.TcpBad)
                    Log.Write($"[Tcp] Port={Config.TcpPortStr} is not a valid TCP port number - TCP server disabled");
                // The module is listed when the TCP server is configured, or when the
                // configured COM port exists.
                if (!Config.UseSerial && Config.TcpPort == 0)
                {
                    badPort = true;
                    Log.Write($"no valid port specified in {Config.IniPath} - no module listed, no port will be opened");
                }
                else
                {
                    bool serialPresent = Config.UseSerial && Link.PortExists(Config.PortName);
                    if (Config.UseSerial && !serialPresent)
                        Log.Write($"serial port {Config.PortName} not found");
                    _moduleListed = serialPresent || Config.TcpPort > 0;
                    if (!_moduleListed)
                        Log.Write("no module listed");
                }
                _constructed = true;
                ret = Pdu.STATUS_NOERROR;
            }
            Log.Write($"PDUConstruct -> {ret}");
        }
        if (badPort)
            ReportPortProblem();
        return ret;
    }

    /// <summary>Shown (outside the state lock, the box is modal) when neither a serial nor a TCP port is usable.</summary>
    private static void ReportPortProblem()
    {
        string notes = "";
        if (Config.SerialBad)
            notes += $"[Serial] Port={Config.PortName} is not a COM port name.\n";
        if (Config.TcpBad)
            notes += $"[Tcp] Port={Config.TcpPortStr} is not a valid TCP port number (1-65535).\n";
        string msg = $"No COM port or TCP port is specified in the ini file:\n\n{Config.IniPath}\n\n" +
                     notes + (notes.Length > 0 ? "\n" : "") +
                     "Add a COM port, a TCP port number to listen on, or both, for example:\n\n" +
                     "[Serial]\nPort=COM8\n\n[Tcp]\nPort=5000\n\n" +
                     "No port will be opened.";
        fixed (char* m = msg, t = Config.DriverName)
            Native.MessageBoxW(0, m, t, Native.MB_OK | Native.MB_ICONERROR | Native.MB_TOPMOST | Native.MB_SETFOREGROUND);
    }

    public static uint Destruct()
    {
        lock (Sync)
        {
            Log.Write("PDUDestruct Called");
            if (!_constructed)
                return Pdu.ERR_PDUAPI_NOT_CONSTRUCTED;
            _constructed = false;
        }
        Link.Close();
        lock (Sync)
        {
            ResetState();
            _connectedMod = 0;
        }
        return Pdu.STATUS_NOERROR;
    }

    /// <summary>Opens the links (outside the lock, see Link.Close) and announces the driver with a HELLO line.</summary>
    public static uint ModuleConnect(uint hMod)
    {
        lock (Sync)
        {
            Log.Write($"PDUModuleConnect Called: hMod={hMod:X8}");
            _connectedMod = hMod;
            if (!_constructed)
                return Pdu.ERR_PDUAPI_NOT_CONSTRUCTED;
            if (hMod != Pdu.HANDLE_UNDEF && (!_moduleListed || hMod != ModuleHandle))
            {
                Log.Write("ReturnValue: PDU_ERR_INVALID_HANDLE");
                return Pdu.ERR_INVALID_HANDLE;
            }
        }
        if (!Link.Open())
        {
            Log.Write("Failed connect comport - ReturnValue: PDU_ERR_FCT_FAILED");
            return Pdu.ERR_FCT_FAILED;
        }
        Link.WriteLine($"HELLO {Config.DriverName} 1.0");
        lock (Sync)
            _moduleStatus = Pdu.MODST_READY;
        return Pdu.STATUS_NOERROR;
    }

    public static uint ModuleDisconnect(uint hMod)
    {
        Log.Write($"PDUModuleDisconnect Called: hMod={hMod:X8}");
        if (hMod != ModuleHandle)
        {
            Log.Write("Invalid hMod supplied");
            return Pdu.ERR_INVALID_HANDLE;
        }
        Link.WriteLine("BYE");
        Link.Close();
        lock (Sync)
        {
            _moduleStatus = Pdu.MODST_AVAIL;
            _connectedMod = 0;
        }
        return Pdu.STATUS_NOERROR;
    }

    public static uint RegisterEventCallback(uint hMod, uint hCll, nint fn)
    {
        CbCall cb = default;
        lock (Sync)
        {
            Log.Write($"PDURegisterEventCallback Called: hMod={hMod:X8} hCLL={hCll:X8} fn={fn:X8}");
            if (!_constructed)
                return Pdu.ERR_PDUAPI_NOT_CONSTRUCTED;
            if (hMod == Pdu.HANDLE_UNDEF)
            {
                if (hCll == Pdu.HANDLE_UNDEF)
                    _sysCbFn = fn;   // default/system callback: stored, never called
                return Pdu.STATUS_NOERROR;
            }
            if (hMod != _connectedMod)
            {
                Log.Write("ReturnValue: PDU_ERR_INVALID_HANDLE");
                return Pdu.ERR_INVALID_HANDLE;
            }
            if (hCll == Pdu.HANDLE_UNDEF)
            {
                _modCbFn = fn;
                if (fn != 0)
                {
                    // Report the module as ready straight away.
                    Enqueue(StatusEvent(Pdu.HANDLE_UNDEF, null, 0, Pdu.MODST_READY));
                    cb = new CbCall { Fn = fn, HMod = hMod, HCll = hCll };
                }
            }
            else
            {
                var c = FindCll(hCll);
                if (c == null)
                {
                    Log.Write("ReturnValue: PDU_ERR_INVALID_HANDLE");
                    return Pdu.ERR_INVALID_HANDLE;
                }
                c.CbHMod = hMod;
                c.CbHCll = hCll;
                c.CbFn = fn;
            }
        }
        cb.Fire();
        return Pdu.STATUS_NOERROR;
    }

    /// <summary>
    /// READ_VBATT, START_MSG_FILTER and CLEAR_MSG_FILTER. Filters are global
    /// and belong to the bus that is active when they are started; they are
    /// reported as FILTER / FILTERCLR lines.
    /// </summary>
    public static uint IoCtl(uint hMod, uint hCll, uint id, PDU_DATA_ITEM* input, PDU_DATA_ITEM** output)
    {
        var lines = new List<string>();
        lock (Sync)
        {
            Log.Write($"PDUIoCtl Called: hMod={hMod:X8} hCLL={hCll:X8} id={id}");
            if (id == IoctlReadVbatt)
            {
                if (output == null)
                    return Pdu.ERR_INVALID_PARAMETERS;
                var it = (PDU_DATA_ITEM*)ItemAlloc(sizeof(PDU_DATA_ITEM) + 4);
                it->ItemType = Pdu.IT_IO_UNUM32;
                it->pData = it + 1;
                *(uint*)it->pData = Volatile.Read(ref _vbattMv);
                *output = it;
                return Pdu.STATUS_NOERROR;
            }
            if (id == IoctlClearMsgFilter)
            {
                Filters.Clear();
                lines.Add($"FILTERCLR {hCll:X8}");
            }
            else if (id == IoctlStartMsgFilter)
            {
                if (input == null || input->pData == null)
                    return Pdu.ERR_INVALID_PARAMETERS;
                var list = (PDU_IO_FILTER_LIST*)input->pData;
                for (uint i = 0; i < list->NumFilterEntries && list->pFilterData != null; i++)
                {
                    var fd = &list->pFilterData[i];
                    int n = (int)Math.Min(fd->FilterCompareSize, 12u);
                    var mask = new ReadOnlySpan<byte>(fd->FilterMaskMessage, 12);
                    var pattern = new ReadOnlySpan<byte>(fd->FilterPatternMessage, 12);
                    Filters.Add(new SwFilter { Bus = _curBus, Type = fd->FilterType,
                                               Mask = mask.Slice(0, 4).ToArray(), Pattern = pattern.Slice(0, 4).ToArray() });
                    bool block = fd->FilterType == Pdu.FLT_BLOCK || fd->FilterType == Pdu.FLT_BLOCK_UUDT;
                    if (lines.Count < MaxFilterLines)
                        lines.Add(Cut($"FILTER {hCll:X8} {BusName(_curBus)} {(block ? "BLOCK" : "PASS")} #{fd->FilterNumber} " +
                                      $"MASK={HexBytes(mask.Slice(0, n), FilterHexSize)} PATTERN={HexBytes(pattern.Slice(0, n), FilterHexSize)}",
                                      FilterLineMax));
                }
            }
            else
            {
                Log.Write("ReturnValue: PDU_ERR_ID_NOT_SUPPORTED");
                return Pdu.ERR_ID_NOT_SUPPORTED;
            }
        }
        foreach (var l in lines)
            Link.WriteLine(l);
        return Pdu.STATUS_NOERROR;
    }

    /// <summary>Fixed identification values; only the names come from the ini.</summary>
    public static uint GetVersion(uint hMod, PDU_VERSION_DATA* v)
    {
        Log.Write($"PDUGetVersion Called: hMod={hMod:X8}");
        if (v == null)
            return Pdu.ERR_INVALID_PARAMETERS;
        *v = default;
        v->MVCI_Part1StandardVersion = 0x02010100;
        v->MVCI_Part2StandardVersion = 0x02020000;
        v->HwSerialNumber = 0x01519226;
        CopyStr(v->HwName, Config.ModuleName);
        v->HwVersion = 0x00010000;
        v->HwDate = 0x26011000;
        v->HwInterface = 1;
        CopyStr(v->FwName, Config.ModuleName);
        v->FwVersion = 0x0805214E;
        v->FwDate = 0x33011D00;
        CopyStr(v->VendorName, "dpdu-bridge");
        CopyStr(v->PDUApiSwName, Config.DriverName + " Serial PDU-API");
        v->PDUApiSwVersion = 0x0805214E;
        v->PDUApiSwDate = 0x33011D00;
        return Pdu.STATUS_NOERROR;
    }

    /// <summary>Copies s into a 64-byte field (at most 63 characters and a NUL).</summary>
    private static void CopyStr(byte* dst, string s)
    {
        var b = Link.Wire.GetBytes(s);
        int n = Math.Min(b.Length, 63);
        for (int i = 0; i < n; i++)
            dst[i] = b[i];
        dst[n] = 0;
    }

    public static uint GetStatus(uint hMod, uint hCll, uint hCop, uint* status, uint* ts, uint* extra)
    {
        uint now = NowUs();
        lock (Sync)
        {
            Log.Write($"PDUGetStatus Called: hMod={hMod:X8} hCLL={hCll:X8} hCoP={hCop:X8}");
            Cll c;
            if (hCop == 0 || (c = FindCll(hCll)) == null)
            {
                Log.Write("ReturnValue: PDU_ERR_INVALID_HANDLE");
                return Pdu.ERR_INVALID_HANDLE;
            }
            if (hCop == c.HStartComm || hCop == c.HTxCop)
            {
                if (status != null) *status = Pdu.COPST_FINISHED;
                if (ts != null) *ts = now;
            }
            else if (hCop == c.HRxCop)
            {
                if (status != null) *status = c.RxCopStatus;
                if (ts != null) *ts = now;
            }
            // Any other handle leaves status and timestamp untouched.
            if (extra != null) *extra = 0;
            return Pdu.STATUS_NOERROR;
        }
    }

    public static uint GetTimestamp(uint hMod, uint* ts)
    {
        Log.Write($"PDUGetTimestamp Called: hMod={hMod:X8}");
        if (ts != null)
            *ts = NowUs();
        return Pdu.STATUS_NOERROR;
    }

    public static uint GetObjectId(uint type, byte* shortName, uint* id)
    {
        if (shortName == null || id == null)
        {
            Log.Write("PDUGetObjectId Called: NULL pointer");
            return Pdu.ERR_INVALID_PARAMETERS;
        }
        string name = CStr(shortName);
        foreach (var o in Objects)
        {
            if (o.Type == type && o.Name == name)
            {
                *id = o.Id;
                Log.Write($"PDUGetObjectId Called: type={type:X} {name} -> {o.Id}");
                return Pdu.STATUS_NOERROR;
            }
        }
        Log.Write($"PDUGetObjectId Called: type={type:X} {name} -> PDU_ERR_INVALID_PARAMETERS");
        return Pdu.ERR_INVALID_PARAMETERS;
    }

    public static uint GetModuleIds(PDU_MODULE_ITEM** list)
    {
        lock (Sync)
        {
            Log.Write("PDUGetModuleIds Called");
            if (list == null)
                return Pdu.ERR_INVALID_PARAMETERS;
            if (!_constructed)
                return Pdu.ERR_PDUAPI_NOT_CONSTRUCTED;
            var name = Link.Wire.GetBytes(Config.ModuleName + "\0");
            var info = Link.Wire.GetBytes(Config.LinkDesc + "\0");
            var it = (PDU_MODULE_ITEM*)ItemAlloc(sizeof(PDU_MODULE_ITEM) + sizeof(PDU_MODULE_DATA) + name.Length + info.Length);
            var md = (PDU_MODULE_DATA*)(it + 1);
            var pName = (byte*)(md + 1);
            var pInfo = pName + name.Length;
            name.CopyTo(new Span<byte>(pName, name.Length));
            info.CopyTo(new Span<byte>(pInfo, info.Length));
            it->ItemType = Pdu.IT_MODULE_ID;
            it->NumEntries = _moduleListed ? 1u : 0u;
            it->pModuleData = _moduleListed ? md : null;
            md->ModuleTypeId = ModuleTypeId;
            md->hMod = ModuleHandle;
            md->pVendorModuleName = pName;
            md->pVendorAdditionalInfo = pInfo;
            md->ModuleStatus = _moduleStatus == Pdu.MODST_READY ? Pdu.MODST_READY : Pdu.MODST_AVAIL;
            *list = it;
            Log.Write($"  NumEntries={it->NumEntries}");
            return Pdu.STATUS_NOERROR;
        }
    }

    public static uint CreateComLogicalLink(uint hMod, PDU_RSC_DATA* rsc, uint resourceId, void* cllTag,
                                            uint* phCll, PDU_FLAG_DATA* createFlag)
    {
        string openLine;
        lock (Sync)
        {
            Log.Write($"PDUCreateComLogicalLink Called: hMod={hMod:X8} resourceId={resourceId:X8} pCllTag={(nint)cllTag:X8}");
            if (!_constructed)
                return Pdu.ERR_PDUAPI_NOT_CONSTRUCTED;
            if (hMod != _connectedMod)
            {
                Log.Write("ReturnValue: PDU_ERR_INVALID_HANDLE");
                return Pdu.ERR_INVALID_HANDLE;
            }
            if (_moduleStatus != Pdu.MODST_READY)
            {
                Log.Write("ReturnValue: PDU_ERR_MODULE_NOT_CONNECTED");
                return Pdu.ERR_MODULE_NOT_CONNECTED;
            }
            if (rsc == null || phCll == null)
                return Pdu.ERR_INVALID_PARAMETERS;
            byte flag0 = createFlag != null && createFlag->NumFlagBytes > 0 && createFlag->pFlagData != null
                ? createFlag->pFlagData[0] : (byte)0;
            var pins = rsc->pDLCPinData;
            Log.Write($"  BusTypeId={rsc->BusTypeId} ProtocolId={rsc->ProtocolId} NumPinData={rsc->NumPinData} flags={flag0:X2}");
            for (uint i = 0; pins != null && i < rsc->NumPinData; i++)
                Log.Write($"  pin {pins[i].DLCPinNumber} type {pins[i].DLCPinTypeId}");

            Bus bus;
            switch (rsc->BusTypeId)
            {
                case BusTypeVpw:
                    bus = Bus.VPW;
                    break;
                case BusTypeJ2740:
                    bus = Bus.ALDL;
                    break;
                case BusTypeDwCan:
                    bus = Bus.HSCAN;
                    foreach (var existing in Clls)
                        if (existing.Bus == Bus.HSCAN)
                            Log.Write("Requested HS CAN twice, ensuring HSCAN protocol is set...");
                    if (rsc->NumPinData != 2 || pins == null || pins[0].DLCPinNumber != 6 || pins[1].DLCPinNumber != 14)
                    {
                        Log.Write("ReturnValue: PDU_ERR_INVALID_PARAMETERS (pins)");
                        return Pdu.ERR_INVALID_PARAMETERS;
                    }
                    break;
                case BusTypeSwCan:
                    bus = Bus.SWCAN;
                    if (rsc->NumPinData != 1 || pins == null || pins[0].DLCPinNumber != 1)
                    {
                        Log.Write("ReturnValue: PDU_ERR_INVALID_PARAMETERS (pins)");
                        return Pdu.ERR_INVALID_PARAMETERS;
                    }
                    break;
                default:
                    Log.Write("ReturnValue: PDU_ERR_INVALID_PARAMETERS (invalid protocol)");
                    return Pdu.ERR_INVALID_PARAMETERS;
            }
            var c = new Cll { Handle = _nextCll++, Bus = bus, CllTag = cllTag };
            Clls.Add(c);
            _curBus = bus;
            *phCll = c.Handle;
            Log.Write($"  hCLL={c.Handle:X8} bus={BusName(bus)} (total logicals {Clls.Count})");
            openLine = $"OPEN {c.Handle:X8} {BusName(bus)} PROTOCOL={rsc->ProtocolId} RAW={(flag0 & 0x80) >> 7} CHECKSUM={(flag0 & 0x40) >> 6}";
        }
        Link.WriteLine(openLine);
        return Pdu.STATUS_NOERROR;
    }

    public static uint DestroyComLogicalLink(uint hMod, uint hCll)
    {
        lock (Sync)
        {
            Log.Write($"PDUDestroyComLogicalLink Called: hMod={hMod:X8} hCLL={hCll:X8}");
            var c = FindCll(hCll);
            if (c == null)
            {
                Log.Write("ReturnValue: PDU_ERR_INVALID_HANDLE");
                return Pdu.ERR_INVALID_HANDLE;
            }
            Clls.Remove(c);
        }
        Link.WriteLine($"CLOSE {hCll:X8}");
        return Pdu.STATUS_NOERROR;
    }

    /// <summary>PDUConnect (online) / PDUDisconnect.</summary>
    public static uint SetLinkState(uint hMod, uint hCll, bool online)
    {
        CbCall cb;
        lock (Sync)
        {
            Log.Write($"{(online ? "PDUConnect" : "PDUDisconnect")} Called: hMod={hMod:X8} hCLL={hCll:X8}");
            Cll c;
            if (hMod != _connectedMod || (c = FindCll(hCll)) == null)
            {
                Log.Write("ReturnValue: PDU_ERR_INVALID_HANDLE");
                return Pdu.ERR_INVALID_HANDLE;
            }
            if (online && c.CllStatus == Pdu.CLLST_ONLINE)
            {
                Log.Write("ReturnValue: PDU_ERR_CLL_CONNECTED (Already Connected)");
                return Pdu.ERR_CLL_CONNECTED;
            }
            c.CllStatus = online ? Pdu.CLLST_ONLINE : Pdu.CLLST_OFFLINE;
            Enqueue(StatusEvent(Pdu.HANDLE_UNDEF, null, NowUs(), c.CllStatus));
            cb = CllCallback(c);
        }
        Link.WriteLine($"{(online ? "CONNECT" : "DISCONNECT")} {hCll:X8}");
        cb.Fire();
        return Pdu.STATUS_NOERROR;
    }

    /// <summary>
    /// Scalar values are applied (CP_TransmitIndEnable) and reported as PARAM
    /// lines; non-scalar ComParams are logged and otherwise ignored.
    /// </summary>
    public static uint SetComParam(uint hMod, uint hCll, PDU_PARAM_ITEM* p)
    {
        string line;
        lock (Sync)
        {
            if (p == null)
                return Pdu.ERR_INVALID_PARAMETERS;
            bool scalar = ComParamScalar(p, out uint value);
            string name = ComParamName(p->ComParamId);
            if (scalar)
                Log.Write($"PDUSetComParam Called: hMod={hMod:X8} hCLL={hCll:X8} {name ?? "?"}({p->ComParamId}) " +
                          $"type={p->ComParamDataType:X} class={p->ComParamClass} value={value}");
            else
                Log.Write($"PDUSetComParam Called: hMod={hMod:X8} hCLL={hCll:X8} {name ?? "?"}({p->ComParamId}) " +
                          $"type={p->ComParamDataType:X} class={p->ComParamClass} value not a scalar - ignored");
            var c = FindCll(hCll);
            if (c == null)
            {
                Log.Write("ReturnValue: PDU_ERR_INVALID_HANDLE");
                return Pdu.ERR_INVALID_HANDLE;
            }
            if (!scalar)
                return Pdu.STATUS_NOERROR;
            if (p->ComParamId == CpTransmitIndEnable)
                c.TxIndEnable = value;
            line = $"PARAM {hCll:X8} {name ?? p->ComParamId.ToString()} {value}";
        }
        Link.WriteLine(line);
        return Pdu.STATUS_NOERROR;
    }

    public static uint SetUniqueRespIdTable(uint hMod, uint hCll, PDU_UNIQUE_RESP_ID_TABLE_ITEM* table)
    {
        var lines = new List<string>();
        lock (Sync)
        {
            Log.Write($"PDUSetUniqueRespIdTable Called: hMod={hMod:X8} hCLL={hCll:X8}");
            Cll c;
            if (hMod != _connectedMod || (c = FindCll(hCll)) == null)
            {
                Log.Write("ReturnValue: PDU_ERR_INVALID_HANDLE");
                return Pdu.ERR_INVALID_HANDLE;
            }
            if (!IsCan(c.Bus))
                return Pdu.STATUS_NOERROR;   // the table is only kept for CAN links
            c.Urid = Array.Empty<UridEntry>();
            if (table == null || (table->NumEntries > 0 && table->pUniqueData == null))
            {
                Log.Write("PDU_ECU_UNIQUE_RESP_DATA is null, invalid data!");
                return Pdu.ERR_INVALID_PARAMETERS;
            }
            var tab = new UridEntry[table->NumEntries];
            for (int i = 0; i < tab.Length; i++)
            {
                var d = &table->pUniqueData[i];
                tab[i].UniqueRespId = d->UniqueRespIdentifier;
                for (uint k = 0; k < d->NumParamItems && d->pParams != null; k++)
                {
                    var pi = &d->pParams[k];
                    if (!ComParamScalar(pi, out uint v))
                    {
                        // The field keeps its value (0 unless set by an earlier item).
                        Log.Write($"  URID {d->UniqueRespIdentifier:X8}: {ComParamName(pi->ComParamId) ?? "?"}({pi->ComParamId}) " +
                                  $"type={pi->ComParamDataType:X} value not a scalar - ignored");
                        continue;
                    }
                    switch (pi->ComParamId)
                    {
                        case CpCanPhysReqId: tab[i].PhysReqId = v; break;
                        case CpCanRespUsdtId: tab[i].RespUsdtId = v; break;
                        case CpCanRespUsdtFormat: tab[i].RespFormat = v; break;
                        case CpCanRespUudtId: tab[i].RespUudtId = v; break;
                    }
                }
                var e = tab[i];
                string line = Cut($"ECU {hCll:X8} {BusName(c.Bus)} URID={e.UniqueRespId:X8} REQ={e.PhysReqId:X} " +
                                  $"RESP={e.RespUsdtId:X} FORMAT={e.RespFormat} UUDT={e.RespUudtId:X}", EcuLineMax);
                Log.Write("  " + line);
                lines.Add(line);
            }
            c.Urid = tab;
        }
        foreach (var l in lines)
            Link.WriteLine(l);
        return Pdu.STATUS_NOERROR;
    }

    /// <summary>
    /// Sends one message from the host as a TX line (or WAKEUP) on every link.
    /// CAN messages start with the 4-byte big-endian CAN id.
    /// </summary>
    private static void Transmit(Bus bus, byte[] data)
    {
        if (IsCan(bus))
        {
            if (data.Length < 4)
            {
                Log.Write("Failed transmitting msg: CAN message shorter than its 4 byte id");
                return;
            }
            uint id = (uint)(data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]);
            if (id == GmlanHvWakeupId)
            {
                // Id 0x100 requests a high-voltage wake-up on SW-CAN (sent as a
                // WAKEUP line); on either bus it is never sent as a TX line.
                if (bus == Bus.SWCAN)
                {
                    Log.Write("Sending HVWakup");
                    Link.WriteLine($"WAKEUP {BusName(bus)}");
                }
                return;
            }
            string idText = id > 0x7FF ? id.ToString("X8") : id.ToString("X3");
            string payload = data.Length > 4 ? " " + HexBytes(data.AsSpan(4), TxHexSize) : "";
            Link.WriteLine($"TX {BusName(bus)} {idText}{payload}");
        }
        else
        {
            Link.WriteLine($"TX {BusName(bus)} {HexBytes(data, TxHexSize)}");
        }
    }

    public static uint StartComPrimitive(uint hMod, uint hCll, uint type, uint size, byte* data,
                                         PDU_COP_CTRL_DATA* ctrl, void* tag, uint* phCop)
    {
        // Each type is handled in two halves: the EXECUTING status event and its
        // callback first, then (after the callback has returned) the FINISHED
        // events and the line to the simulator.
        CbCall cb;
        Cll c;
        uint h = 0;
        void* lastTxTag = null;
        byte[] msg = null;
        lock (Sync)
        {
            Log.Write($"PDUStartComPrimitive Called: hMod={hMod:X8} hCLL={hCll:X8} CoPType={type:X} size={size} tag={(nint)tag:X8}");
            if (hMod != _connectedMod || (c = FindCll(hCll)) == null)
            {
                Log.Write("ReturnValue: PDU_ERR_INVALID_HANDLE");
                return Pdu.ERR_INVALID_HANDLE;
            }

            if (type == Pdu.COPT_UPDATEPARAM)
            {
                lastTxTag = c.TxCopTag;   // the FINISHED event carries the last transmit's tag
                Enqueue(StatusEvent(HcopUpdateParam, null, NowUs(), Pdu.COPST_EXECUTING));
                cb = CllCallback(c);
            }
            else if (type == Pdu.COPT_RESTORE_PARAM || ctrl == null)
            {
                return Pdu.STATUS_NOERROR;   // nothing to do (*phCoP is not set)
            }
            else if (type == Pdu.COPT_STARTCOMM)
            {
                h = c.HStartComm = _nextStart++;
                if (phCop != null) *phCop = h;
                Enqueue(StatusEvent(h, null, NowUs(), Pdu.COPST_EXECUTING));
                cb = CllCallback(c);
            }
            else if (type == Pdu.COPT_STOPCOMM)
            {
                h = c.HStartComm;
                Enqueue(StatusEvent(h, null, NowUs(), Pdu.COPST_EXECUTING));
                cb = CllCallback(c);
            }
            else if (type != Pdu.COPT_SENDRECV)
            {
                return Pdu.STATUS_NOERROR;   // PDU_COPT_DELAY: accepted, nothing happens
            }
            else if (size == 0)
            {
                // Receive-only ComPrimitive: owns all future responses; its expected responses filter them.
                uint n = ctrl->pExpectedResponseArray != null ? ctrl->NumPossibleExpectedResponses : 0;
                var list = new ExpResp[n];
                for (int i = 0; i < n; i++)
                {
                    var x = &ctrl->pExpectedResponseArray[i];
                    list[i].RespType = x->ResponseType;
                    list[i].AcceptanceId = x->AcceptanceId;
                    if (x->NumMaskPatternBytes == 4 && x->pMaskData != null && x->pPatternData != null)
                    {
                        list[i].Mask = Be32(x->pMaskData);
                        list[i].Pattern = Be32(x->pPatternData);
                    }
                    Log.Write($"  expected response {i}: type={list[i].RespType} acceptanceId={list[i].AcceptanceId} " +
                              $"mask={list[i].Mask:X8} pattern={list[i].Pattern:X8}");
                }
                c.ExpResp = list;
                h = c.HRxCop = _nextRxCop++;
                c.RxCopTag = tag;
                if (phCop != null) *phCop = h;
                c.RxCopStatus = Pdu.COPST_EXECUTING;
                Enqueue(StatusEvent(h, tag, NowUs(), Pdu.COPST_EXECUTING));
                cb = CllCallback(c);
                Log.Write($"  RXD ComPrimitive hCoP={h:X8}");
            }
            else
            {
                // Transmit ComPrimitive.
                if (data == null)
                    return Pdu.ERR_INVALID_PARAMETERS;
                msg = new ReadOnlySpan<byte>(data, checked((int)size)).ToArray();
                h = c.HTxCop = _nextTxCop++;
                c.TxCopTag = tag;
                if (phCop != null) *phCop = h;
                Enqueue(StatusEvent(h, tag, NowUs(), Pdu.COPST_EXECUTING));
                cb = CllCallback(c);
                Log.Write($"  TXD ComPrimitive hCoP={h:X8}");
            }
        }
        cb.Fire();

        // Second half, after the EXECUTING callback.
        if (type == Pdu.COPT_UPDATEPARAM)
        {
            lock (Sync)
                Enqueue(StatusEvent(HcopUpdateParam, lastTxTag, NowUs(), Pdu.COPST_FINISHED));
        }
        else if (type == Pdu.COPT_STARTCOMM)
        {
            lock (Sync)
            {
                Enqueue(StatusEvent(Pdu.HANDLE_UNDEF, null, NowUs(), Pdu.CLLST_COMM_STARTED));
                Enqueue(StatusEvent(h, null, NowUs(), Pdu.COPST_FINISHED));
            }
            Link.WriteLine($"STARTCOMM {hCll:X8}");
        }
        else if (type == Pdu.COPT_STOPCOMM)
        {
            lock (Sync)
                Enqueue(StatusEvent(h, null, NowUs(), Pdu.COPST_FINISHED));
            Link.WriteLine($"STOPCOMM {hCll:X8}");
        }
        else if (msg != null)
        {
            // The callback ran without the lock, so the CLL may have been
            // destroyed; then nothing is sent.
            bool busChanged = false;
            Bus bus;
            lock (Sync)
            {
                c = FindCll(hCll);
                if (c == null)
                    return Pdu.STATUS_NOERROR;
                bus = c.Bus;
                if (c.TxIndEnable == 1)
                {
                    // The indication is a result of the receive ComPrimitive, so
                    // it follows the same rule as received messages.
                    string why = RxUnavailable(c);
                    if (why == null)
                        Enqueue(ResultEvent(c.HRxCop, c.RxCopTag, stackalloc byte[] { 0, 0, 0, 0x88 }, 0, 0, default));
                    else
                        Log.Write($"TX indication dropped: {why} logical link {hCll:X8}");
                }
                if (_curBus != bus)
                {
                    _curBus = bus;
                    busChanged = true;
                }
            }
            if (busChanged)
                Link.WriteLine($"PROTOCOL {BusName(bus)}");
            Transmit(bus, msg);
            lock (Sync)
            {
                if (FindCll(hCll) != null)
                    Enqueue(StatusEvent(h, tag, NowUs(), Pdu.COPST_FINISHED));
            }
        }
        return Pdu.STATUS_NOERROR;
    }

    public static uint CancelComPrimitive(uint hMod, uint hCll, uint hCop)
    {
        CbCall cb;
        lock (Sync)
        {
            Log.Write($"PDUCancelComPrimitive Called: hMod={hMod:X8} hCLL={hCll:X8} hCoP={hCop:X8}");
            var c = FindCll(hCll);
            if (c == null)
                return Pdu.ERR_INVALID_HANDLE;
            if (hCop != c.HRxCop)
                return Pdu.STATUS_NOERROR;   // TX / STARTCOMM primitives are already finished
            c.RxCopStatus = Pdu.COPST_CANCELLED;
            Enqueue(StatusEvent(c.HRxCop, c.RxCopTag, NowUs(), Pdu.COPST_CANCELLED));
            cb = CllCallback(c);
            cb.HCll = c.Handle;
        }
        cb.Fire();
        return Pdu.STATUS_NOERROR;
    }

    public static uint GetEventItem(uint hMod, uint hCll, PDU_EVENT_ITEM** item)
    {
        // One queue shared by the module and all CLLs.
        lock (Sync)
        {
            if (item == null)
                return Pdu.ERR_INVALID_PARAMETERS;
            if (Events.Count == 0)
                return Pdu.ERR_EVENT_QUEUE_EMPTY;
            var ev = (PDU_EVENT_ITEM*)Events.Dequeue();
            *item = ev;
            if (ev->ItemType == Pdu.IT_STATUS)
                Log.Write($"PDUGetEventItem(hMod={hMod:X8} hCLL={hCll:X8}): STATUS hCoP={ev->hCop:X8} status={*(uint*)ev->pData:X}");
            else
                Log.Write($"PDUGetEventItem(hMod={hMod:X8} hCLL={hCll:X8}): RESULT hCoP={ev->hCop:X8} bytes={((PDU_RESULT_DATA*)ev->pData)->NumDataBytes}");
            return Pdu.STATUS_NOERROR;
        }
    }

    public static uint DestroyItem(void* item)
    {
        lock (Sync)
        {
            if (!ItemFree(item))
                Log.Write($"PDUDestroyItem Called: {(nint)item:X8} is not an item of this driver - ignored");
        }
        return Pdu.STATUS_NOERROR;
    }
}
