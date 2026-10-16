// SAE J2534-1 (04.04) PassThru front end: the same bridge as Driver, for hosts
// that talk J2534 instead of the D-PDU API (README.md, "J2534").
//
// J2534Exports forwards each PassThru call here. Calls become the same text
// lines Driver writes (HELLO / OPEN / PARAM / FILTER / ECU / TX / CLOSE /
// BYE), and RX lines from the clients are filtered into per-channel receive
// queues that PassThruReadMsgs drains.
//
// One device (id 1) and any number of channels, one per protocol id. The
// D-PDU API and J2534 share the links, so only one of them can have the
// device open at a time.
//
// Threading: all state is guarded by Sync. PassThruReadMsgs waits on Sync
// (Monitor.Wait) for received messages; Link writes happen outside the lock,
// as in Driver. Periodic messages are sent from timer threads.
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;

namespace DpduBridge;

internal static unsafe class J2534
{
    private const uint DeviceId = 1;
    private const int RxQueueMax = 2000;   // per channel; further messages are dropped

    /// <summary>A received (or looped back) message waiting for PassThruReadMsgs.</summary>
    private struct RxMsg { public uint Protocol, RxStatus, Timestamp; public byte[] Data; }

    private sealed class Filter { public uint Id, Type; public byte[] Mask, Pattern, Flow; }

    private sealed class Periodic { public uint Id; public Timer Timer; }

    private sealed class Channel
    {
        public uint Id, Protocol, Flags;
        public Bus Bus;
        public bool Iso;                                    // ISO 15765: complete messages, flow control filters
        public readonly Dictionary<uint, uint> Config = new();
        public readonly Queue<RxMsg> Rx = new();
        public readonly List<Filter> Filters = new();
        public readonly List<Periodic> Periodics = new();
        public uint NextFilter = 1, NextPeriodic = 1;
        public bool RxFullLogged;
    }

    private static readonly object Sync = new();
    private static bool _open;
    private static Bus _curBus = Bus.None;
    private static readonly List<Channel> Channels = new();
    private static uint _nextChannel = 1;       // never reset, like Driver's handles
    private static string _lastError = "";
    private static uint _vbattMv;               // written by the link threads (VBATT lines)

    static J2534()
    {
        Config.EnsureLoaded();
        _vbattMv = Config.VBattMilliVolts;
    }

    /// <summary>True while a J2534 host has the device open (the links belong to J2534).</summary>
    public static bool IsOpen
    {
        get { lock (Sync) return _open; }
    }

    // ---- helpers ----------------------------------------------------------------

    private static readonly double UsPerTick = 1_000_000.0 / System.Diagnostics.Stopwatch.Frequency;
    private static uint NowUs() => (uint)(ulong)(System.Diagnostics.Stopwatch.GetTimestamp() * UsPerTick);

    private static string BusName(Bus b) => b == Bus.None ? "NONE" : b.ToString();
    private static bool IsCan(Bus b) => b == Bus.HSCAN || b == Bus.SWCAN;

    /// <summary>Records the error text for PassThruGetLastError, logs it and returns code.</summary>
    private static uint Fail(uint code, string text)
    {
        _lastError = text;
        Log.Write($"  ReturnValue: 0x{code:X2} ({text})");
        return code;
    }

    private static Channel FindChannel(uint id) => Channels.FirstOrDefault(c => c.Id == id);

    /// <summary>Bus and ISO 15765 flag of a protocol id; Bus.None when it is not supported.</summary>
    private static Bus ProtocolBus(uint protocol, out bool iso)
    {
        iso = protocol == J.ISO15765 || protocol == J.ISO15765_PS || protocol == J.SW_ISO15765_PS;
        switch (protocol)
        {
            case J.J1850VPW: return Bus.VPW;
            case J.CAN: case J.ISO15765: case J.CAN_PS: case J.ISO15765_PS: return Bus.HSCAN;
            case J.SW_CAN_PS: case J.SW_ISO15765_PS: return Bus.SWCAN;
            case J.GM_UART_PS: case J.GM_UART_VENDOR: return Bus.ALDL;
            default: return Bus.None;
        }
    }

    /// <summary>The raw CAN protocol id that goes with an ISO 15765 protocol id (mixed format).</summary>
    private static uint RawCanOf(uint protocol) => protocol switch
    {
        J.ISO15765 => J.CAN,
        J.ISO15765_PS => J.CAN_PS,
        J.SW_ISO15765_PS => J.SW_CAN_PS,
        _ => protocol,
    };

    private static string ParamName(uint id) => id switch
    {
        0x01 => "DATA_RATE", 0x03 => "LOOPBACK", 0x04 => "NODE_ADDRESS", 0x05 => "NETWORK_LINE",
        0x06 => "P1_MIN", 0x07 => "P1_MAX", 0x08 => "P2_MIN", 0x09 => "P2_MAX", 0x0A => "P3_MIN",
        0x0B => "P3_MAX", 0x0C => "P4_MIN", 0x0D => "P4_MAX", 0x0E => "W1", 0x0F => "W2", 0x10 => "W3",
        0x11 => "W4", 0x12 => "W5", 0x13 => "TIDLE", 0x14 => "TINIL", 0x15 => "TWUP", 0x16 => "PARITY",
        0x17 => "BIT_SAMPLE_POINT", 0x18 => "SYNC_JUMP_WIDTH", 0x19 => "W0", 0x1A => "T1_MAX",
        0x1B => "T2_MAX", 0x1C => "T4_MAX", 0x1D => "T5_MAX", 0x1E => "ISO15765_BS", 0x1F => "ISO15765_STMIN",
        0x20 => "DATA_BITS", 0x21 => "FIVE_BAUD_MOD", 0x22 => "BS_TX", 0x23 => "STMIN_TX", 0x24 => "T3_MAX",
        0x25 => "ISO15765_WFT_MAX", 0x8000 => "CAN_MIXED_FORMAT", 0x8001 => "J1962_PINS",
        _ => $"0x{id:X}",
    };

    private static string Hex(byte[] b) => Driver.HexBytes(b, b.Length * 3 + 8);

    private static void CopyStr(byte* dst, string s)
    {
        var b = Link.Wire.GetBytes(s);
        int n = Math.Min(b.Length, 79);
        for (int i = 0; i < n; i++)
            dst[i] = b[i];
        dst[n] = 0;
    }

    private static byte[] MsgData(PASSTHRU_MSG* m) => new ReadOnlySpan<byte>(m->Data, (int)Math.Min(m->DataSize, (uint)J.MaxData)).ToArray();

    // ---- receive path (client lines -> channel queues) ---------------------------

    /// <summary>Handles one line from a client (link thread): RX frames, VBATT; '#' lines are comments.</summary>
    public static void ProcessLine(string line)
    {
        line = line.TrimStart(' ', '\t');
        if (line.Length == 0 || line[0] == '#')
            return;
        Log.Write("<< " + line);
        int pos = 0;
        string cmd = Driver.Word(line, ref pos);
        if (cmd.Equals("RX", StringComparison.OrdinalIgnoreCase))
        {
            Driver.SkipBlanks(line, ref pos);
            string busTok = Driver.Word(line, ref pos);
            Bus bus = Driver.ParseBus(busTok);
            if (bus == Bus.None)
            {
                Log.Write($"RX: unknown bus '{busTok}'");
                return;
            }
            if (IsCan(bus))
            {
                Driver.SkipBlanks(line, ref pos);
                string idTok = Driver.Word(line, ref pos);
                uint canId = Driver.StrToUL(idTok, 16, out int end);
                if (idTok.Length == 0 || end != idTok.Length)
                {
                    Log.Write($"RX: bad CAN id '{idTok}'");
                    return;
                }
                var payload = Driver.ParseHexBytes(line.Substring(pos), J.MaxData - 4);
                if (payload == null) { Log.Write("RX: bad data bytes"); return; }
                var data = new byte[payload.Length + 4];
                data[0] = (byte)(canId >> 24); data[1] = (byte)(canId >> 16); data[2] = (byte)(canId >> 8); data[3] = (byte)canId;
                payload.CopyTo(data, 4);
                Deliver(bus, data, canId);
            }
            else
            {
                var frame = Driver.ParseHexBytes(line.Substring(pos), J.MaxData);
                if (frame == null || frame.Length == 0) { Log.Write("RX: bad data bytes"); return; }
                Deliver(bus, frame, 0);
            }
        }
        else if (cmd.Equals("VBATT", StringComparison.OrdinalIgnoreCase))
        {
            Volatile.Write(ref _vbattMv, Driver.StrToUL(line.Substring(pos), 10, out _));
        }
        else
        {
            Log.Write($"unknown command '{cmd}' ignored");
        }
    }

    /// <summary>Mask / pattern check over the filter's length; shorter messages never match.</summary>
    private static bool Match(byte[] mask, byte[] pattern, byte[] data)
    {
        if (data.Length < mask.Length)
            return false;
        for (int k = 0; k < mask.Length; k++)
            if ((data[k] & mask[k]) != (pattern[k] & mask[k]))
                return false;
        return true;
    }

    /// <summary>J2534 pass / block rule: no block filter matches and at least one pass filter does.</summary>
    private static bool PassBlock(Channel ch, byte[] data)
    {
        bool passed = false;
        foreach (var f in ch.Filters)
        {
            if (f.Type == J.BLOCK_FILTER && Match(f.Mask, f.Pattern, data))
                return false;
            if (f.Type == J.PASS_FILTER && Match(f.Mask, f.Pattern, data))
                passed = true;
        }
        return passed;
    }

    /// <summary>Caller holds Sync.</summary>
    private static void Enqueue(Channel ch, uint protocol, uint rxStatus, byte[] data)
    {
        if (ch.Rx.Count >= RxQueueMax)
        {
            if (!ch.RxFullLogged)
                Log.Write($"channel {ch.Id:X8}: receive queue full - messages dropped until the host reads");
            ch.RxFullLogged = true;
            return;
        }
        ch.RxFullLogged = false;
        ch.Rx.Enqueue(new RxMsg { Protocol = protocol, RxStatus = rxStatus, Timestamp = NowUs(), Data = data });
    }

    /// <summary>
    /// A received message (CAN: 4-byte id + complete payload) goes to every
    /// channel on its bus that accepts it. On an ISO 15765 channel, ids that
    /// match a flow control filter arrive as ISO 15765 messages of any length;
    /// with CAN_MIXED_FORMAT on, other single frames (up to 8 bytes) arrive as
    /// raw CAN through the pass / block filters. Every other channel uses only
    /// the pass / block filters.
    /// </summary>
    private static void Deliver(Bus bus, byte[] data, uint canId)
    {
        lock (Sync)
        {
            bool any = false;
            uint status = IsCan(bus) && canId > 0x7FF ? J.CAN_29BIT_ID : 0;
            foreach (var ch in Channels)
            {
                if (ch.Bus != bus)
                    continue;
                if (ch.Iso)
                {
                    if (ch.Filters.Any(f => f.Type == J.FLOW_CONTROL_FILTER && Match(f.Mask, f.Pattern, data)))
                    {
                        Enqueue(ch, ch.Protocol, status, data);
                        any = true;
                        continue;
                    }
                    if (!ch.Config.TryGetValue(J.CAN_MIXED_FORMAT, out uint mixed) || mixed == 0 || data.Length > 12)
                        continue;
                    if (PassBlock(ch, data))
                    {
                        Enqueue(ch, RawCanOf(ch.Protocol), status, data);
                        any = true;
                    }
                }
                else if (PassBlock(ch, data))
                {
                    Enqueue(ch, ch.Protocol, status, data);
                    any = true;
                }
            }
            if (any)
                Monitor.PulseAll(Sync);
            else if (IsCan(bus))
                Log.Write($"RX {BusName(bus)} {canId:X} dropped: no channel / filter accepts it");
            else
                Log.Write($"RX {BusName(bus)} dropped: no channel / filter accepts it");
        }
    }

    // ---- API (called through J2534Exports) ---------------------------------------

    public static uint Open(void* name, uint* pDeviceId)
    {
        Log.Write($"PassThruOpen Called: pName={(name == null ? "(null)" : Driver.CStr((byte*)name))}");
        if (pDeviceId == null)
            return Fail(J.ERR_NULL_PARAMETER, "pDeviceID is NULL");
        lock (Sync)
        {
            if (_open)
                return Fail(J.ERR_DEVICE_IN_USE, "the device is already open");
        }
        if (Driver.ModuleConnected)
            return Fail(J.ERR_DEVICE_IN_USE, "the D-PDU API of this DLL has the device connected");
        if (!Config.UseSerial && Config.TcpPort == 0)
        {
            Log.Write($"no valid port specified in {Config.IniPath} - no port will be opened");
            Driver.ReportPortProblem();
            return Fail(J.ERR_DEVICE_NOT_CONNECTED, "no COM port or TCP port in " + System.IO.Path.GetFileName(Config.IniPath));
        }
        Link.LineReceived = ProcessLine;
        if (!Link.Open())
            return Fail(J.ERR_DEVICE_NOT_CONNECTED, "cannot open " + Config.LinkDesc);
        lock (Sync)
        {
            _open = true;
            _curBus = Bus.None;
        }
        Link.WriteLine($"HELLO {Config.DriverName} {Config.DriverVersion} J2534");
        *pDeviceId = DeviceId;
        return J.STATUS_NOERROR;
    }

    public static uint Close(uint deviceId)
    {
        var periodics = new List<Periodic>();
        lock (Sync)
        {
            Log.Write($"PassThruClose Called: DeviceID={deviceId}");
            if (!_open || deviceId != DeviceId)
                return Fail(J.ERR_INVALID_DEVICE_ID, $"invalid device id {deviceId}");
            foreach (var ch in Channels)
                periodics.AddRange(ch.Periodics);
            Channels.Clear();
            _open = false;
            Monitor.PulseAll(Sync);
        }
        foreach (var p in periodics)
            p.Timer.Dispose();
        Link.WriteLine("BYE");
        Link.Close();
        return J.STATUS_NOERROR;
    }

    public static uint Connect(uint deviceId, uint protocol, uint flags, uint baud, uint* pChannelId)
    {
        var lines = new List<string>();
        lock (Sync)
        {
            Log.Write($"PassThruConnect Called: DeviceID={deviceId} ProtocolID=0x{protocol:X} Flags=0x{flags:X} BaudRate={baud}");
            if (!_open || deviceId != DeviceId)
                return Fail(J.ERR_INVALID_DEVICE_ID, $"invalid device id {deviceId}");
            if (pChannelId == null)
                return Fail(J.ERR_NULL_PARAMETER, "pChannelID is NULL");
            Bus bus = ProtocolBus(protocol, out bool iso);
            if (bus == Bus.None)
                return Fail(J.ERR_INVALID_PROTOCOL_ID, $"protocol 0x{protocol:X} is not supported");
            if (Channels.Any(c => c.Protocol == protocol))
                return Fail(J.ERR_CHANNEL_IN_USE, $"protocol 0x{protocol:X} is already connected");
            if (baud == 0)
                return Fail(J.ERR_INVALID_BAUDRATE, "baud rate 0");
            var ch = new Channel { Id = _nextChannel++, Protocol = protocol, Flags = flags, Bus = bus, Iso = iso };
            ch.Config[J.DATA_RATE] = baud;
            ch.Config[J.LOOPBACK] = 0;
            Channels.Add(ch);
            _curBus = bus;
            *pChannelId = ch.Id;
            Log.Write($"  ChannelID={ch.Id} bus={BusName(bus)} (total channels {Channels.Count})");
            lines.Add($"OPEN {ch.Id:X8} {BusName(bus)} PROTOCOL={protocol} RAW={(iso ? 0 : 1)} CHECKSUM=0");
            lines.Add($"PARAM {ch.Id:X8} DATA_RATE {baud}");
        }
        foreach (var l in lines)
            Link.WriteLine(l);
        return J.STATUS_NOERROR;
    }

    public static uint Disconnect(uint channelId)
    {
        Channel ch;
        lock (Sync)
        {
            Log.Write($"PassThruDisconnect Called: ChannelID={channelId}");
            ch = FindChannel(channelId);
            if (ch == null)
                return Fail(J.ERR_INVALID_CHANNEL_ID, $"invalid channel id {channelId}");
            Channels.Remove(ch);
            Monitor.PulseAll(Sync);
        }
        foreach (var p in ch.Periodics)
            p.Timer.Dispose();
        Link.WriteLine($"CLOSE {channelId:X8}");
        return J.STATUS_NOERROR;
    }

    public static uint ReadMsgs(uint channelId, PASSTHRU_MSG* msgs, uint* pNumMsgs, uint timeout)
    {
        if (msgs == null || pNumMsgs == null)
            return Fail(J.ERR_NULL_PARAMETER, "pMsg or pNumMsgs is NULL");
        uint want = *pNumMsgs;
        uint n = 0;
        *pNumMsgs = 0;
        lock (Sync)
        {
            var ch = FindChannel(channelId);
            if (ch == null)
                return Fail(J.ERR_INVALID_CHANNEL_ID, $"invalid channel id {channelId}");
            long deadline = Environment.TickCount64 + timeout;
            while (true)
            {
                while (n < want && ch.Rx.Count > 0)
                {
                    var r = ch.Rx.Dequeue();
                    var m = &msgs[n++];
                    m->ProtocolID = r.Protocol;
                    m->RxStatus = r.RxStatus;
                    m->TxFlags = 0;
                    m->Timestamp = r.Timestamp;
                    m->DataSize = (uint)r.Data.Length;
                    m->ExtraDataIndex = (uint)r.Data.Length;
                    r.Data.CopyTo(new Span<byte>(m->Data, J.MaxData));
                    Log.Write($"PassThruReadMsgs ChannelID={channelId}: Protocol=0x{r.Protocol:X} RxStatus=0x{r.RxStatus:X} {Hex(r.Data)}");
                }
                if (n >= want || timeout == 0)
                    break;
                long remaining = deadline - Environment.TickCount64;
                if (remaining <= 0)
                    break;
                Monitor.Wait(Sync, (int)Math.Min(remaining, int.MaxValue));
                if (!Channels.Contains(ch))
                {
                    *pNumMsgs = n;
                    return Fail(J.ERR_INVALID_CHANNEL_ID, $"channel {channelId} was disconnected");
                }
            }
        }
        *pNumMsgs = n;
        if (n == 0)
        {
            _lastError = timeout == 0 ? "no messages" : "no messages within the timeout";
            return timeout == 0 ? J.ERR_BUFFER_EMPTY : J.ERR_TIMEOUT;
        }
        if (n < want && timeout > 0)
        {
            _lastError = $"only {n} of {want} messages within the timeout";
            return J.ERR_TIMEOUT;
        }
        return J.STATUS_NOERROR;
    }

    /// <summary>
    /// Checks a message to send on ch (caller holds Sync): its protocol id is
    /// the channel's, or on an ISO 15765 channel also the raw CAN one (mixed
    /// format); CAN messages start with the 4-byte id.
    /// </summary>
    private static uint CheckTx(Channel ch, PASSTHRU_MSG* m, out string why)
    {
        why = null;
        if (m->ProtocolID != ch.Protocol && !(ch.Iso && m->ProtocolID == RawCanOf(ch.Protocol)))
        {
            why = $"message protocol 0x{m->ProtocolID:X} on a 0x{ch.Protocol:X} channel";
            return J.ERR_MSG_PROTOCOL_ID;
        }
        bool iso = m->ProtocolID == ch.Protocol && ch.Iso;
        uint max = !IsCan(ch.Bus) ? (uint)J.MaxData : iso ? 4 + 1 + 4095u : 12u;
        uint min = IsCan(ch.Bus) ? 4u : 1u;
        if (m->DataSize < min || m->DataSize > max)
        {
            why = $"DataSize {m->DataSize} not {min}..{max}";
            return J.ERR_INVALID_MSG;
        }
        return J.STATUS_NOERROR;
    }

    /// <summary>
    /// Sends one checked message as a TX line (PROTOCOL first when the active
    /// bus changes). Unless periodic, an ISO 15765 message gets a TX_DONE
    /// indication and, with LOOPBACK on, the message is echoed with
    /// TX_MSG_TYPE.
    /// </summary>
    private static void Send(uint channelId, uint protocol, uint txFlags, byte[] data, bool periodic)
    {
        Bus bus;
        bool busChanged = false;
        lock (Sync)
        {
            var ch = FindChannel(channelId);
            if (ch == null)
                return;
            bus = ch.Bus;
            if (_curBus != bus)
            {
                _curBus = bus;
                busChanged = true;
            }
            if (!periodic)
            {
                uint status = IsCan(bus) && (txFlags & J.CAN_29BIT_ID) != 0 ? J.CAN_29BIT_ID : 0;
                if (ch.Iso && protocol == ch.Protocol)
                    Enqueue(ch, protocol, J.TX_DONE | status, data.AsSpan(0, 4).ToArray());
                if (ch.Config.TryGetValue(J.LOOPBACK, out uint lb) && lb != 0)
                    Enqueue(ch, protocol, J.TX_MSG_TYPE | status, data);
                Monitor.PulseAll(Sync);
            }
        }
        if (busChanged)
            Link.WriteLine($"PROTOCOL {BusName(bus)}");
        // Id 0x100 is sent as a WAKEUP line by Driver.Transmit already.
        if (bus == Bus.SWCAN && (txFlags & J.SW_CAN_HV_TX) != 0 && !(data.Length >= 4 && Driver.Be32(data) == 0x100))
            Link.WriteLine("WAKEUP SWCAN");
        Driver.Transmit(bus, data);
    }

    public static uint WriteMsgs(uint channelId, PASSTHRU_MSG* msgs, uint* pNumMsgs, uint timeout)
    {
        if (msgs == null || pNumMsgs == null)
            return Fail(J.ERR_NULL_PARAMETER, "pMsg or pNumMsgs is NULL");
        uint want = *pNumMsgs;
        var batch = new List<(uint Protocol, uint TxFlags, byte[] Data)>();
        uint err = J.STATUS_NOERROR;
        lock (Sync)
        {
            Log.Write($"PassThruWriteMsgs Called: ChannelID={channelId} NumMsgs={want} Timeout={timeout}");
            var ch = FindChannel(channelId);
            if (ch == null)
                return Fail(J.ERR_INVALID_CHANNEL_ID, $"invalid channel id {channelId}");
            for (uint i = 0; i < want; i++)
            {
                var m = &msgs[i];
                err = CheckTx(ch, m, out string why);
                if (err != J.STATUS_NOERROR)
                {
                    Fail(err, $"message {i}: {why}");
                    break;
                }
                batch.Add((m->ProtocolID, m->TxFlags, MsgData(m)));
            }
        }
        foreach (var b in batch)
            Send(channelId, b.Protocol, b.TxFlags, b.Data, periodic: false);
        *pNumMsgs = (uint)batch.Count;
        return err;
    }

    public static uint StartPeriodicMsg(uint channelId, PASSTHRU_MSG* msg, uint* pMsgId, uint interval)
    {
        lock (Sync)
        {
            Log.Write($"PassThruStartPeriodicMsg Called: ChannelID={channelId} TimeInterval={interval}");
            if (msg == null || pMsgId == null)
                return Fail(J.ERR_NULL_PARAMETER, "pMsg or pMsgID is NULL");
            var ch = FindChannel(channelId);
            if (ch == null)
                return Fail(J.ERR_INVALID_CHANNEL_ID, $"invalid channel id {channelId}");
            uint err = CheckTx(ch, msg, out string why);
            if (err != J.STATUS_NOERROR)
                return Fail(err, why);
            if (interval < 5 || interval > 65535)
                return Fail(J.ERR_INVALID_TIME_INTERVAL, $"interval {interval} ms not 5..65535");
            if (ch.Periodics.Count >= J.MaxPeriodic)
                return Fail(J.ERR_EXCEEDED_LIMIT, $"more than {J.MaxPeriodic} periodic messages");
            uint protocol = msg->ProtocolID, txFlags = msg->TxFlags;
            byte[] data = MsgData(msg);
            var p = new Periodic { Id = ch.NextPeriodic++ };
            p.Timer = new Timer(_ =>
            {
                try { Send(channelId, protocol, txFlags, data, periodic: true); }
                catch (Exception ex) { Log.Write($"periodic message {p.Id} failed: {ex}"); }
            }, null, (int)interval, (int)interval);
            ch.Periodics.Add(p);
            *pMsgId = p.Id;
            Log.Write($"  MsgID={p.Id}: {Hex(data)}");
            return J.STATUS_NOERROR;
        }
    }

    public static uint StopPeriodicMsg(uint channelId, uint msgId)
    {
        Periodic p;
        lock (Sync)
        {
            Log.Write($"PassThruStopPeriodicMsg Called: ChannelID={channelId} MsgID={msgId}");
            var ch = FindChannel(channelId);
            if (ch == null)
                return Fail(J.ERR_INVALID_CHANNEL_ID, $"invalid channel id {channelId}");
            p = ch.Periodics.FirstOrDefault(x => x.Id == msgId);
            if (p == null)
                return Fail(J.ERR_INVALID_MSG_ID, $"invalid periodic message id {msgId}");
            ch.Periodics.Remove(p);
        }
        p.Timer.Dispose();
        return J.STATUS_NOERROR;
    }

    /// <summary>
    /// PASS / BLOCK filters (1-12 bytes, on CAN including the 4-byte id) and,
    /// on ISO 15765 channels, FLOW_CONTROL filters. A flow control filter is
    /// also reported as an ECU line, so the simulator learns the request /
    /// response id pair.
    /// </summary>
    public static uint StartMsgFilter(uint channelId, uint type, PASSTHRU_MSG* mask, PASSTHRU_MSG* pattern,
                                      PASSTHRU_MSG* flow, uint* pFilterId)
    {
        var lines = new List<string>();
        lock (Sync)
        {
            Log.Write($"PassThruStartMsgFilter Called: ChannelID={channelId} FilterType={type}");
            var ch = FindChannel(channelId);
            if (ch == null)
                return Fail(J.ERR_INVALID_CHANNEL_ID, $"invalid channel id {channelId}");
            if (mask == null || pattern == null || pFilterId == null || (type == J.FLOW_CONTROL_FILTER && flow == null))
                return Fail(J.ERR_NULL_PARAMETER, "pMaskMsg, pPatternMsg, pFlowControlMsg or pFilterID is NULL");
            if (type != J.PASS_FILTER && type != J.BLOCK_FILTER && type != J.FLOW_CONTROL_FILTER)
                return Fail(J.ERR_FAILED, $"invalid filter type {type}");
            if (type == J.FLOW_CONTROL_FILTER && !ch.Iso)
                return Fail(J.ERR_MSG_PROTOCOL_ID, "flow control filters need an ISO 15765 channel");
            if (mask->DataSize != pattern->DataSize || mask->DataSize == 0 || mask->DataSize > 12 ||
                (type == J.FLOW_CONTROL_FILTER && (mask->DataSize < 4 || mask->DataSize > 5 || flow->DataSize != mask->DataSize)))
                return Fail(J.ERR_INVALID_MSG, $"mask / pattern / flow control sizes {mask->DataSize} / {pattern->DataSize} / {(flow != null ? flow->DataSize : 0)}");
            if (ch.Filters.Count >= J.MaxFilters)
                return Fail(J.ERR_EXCEEDED_LIMIT, $"more than {J.MaxFilters} filters");
            var f = new Filter { Id = ch.NextFilter++, Type = type, Mask = MsgData(mask), Pattern = MsgData(pattern),
                                 Flow = type == J.FLOW_CONTROL_FILTER ? MsgData(flow) : null };
            if (IsCan(ch.Bus))
                CanFilterFixup(f);
            ch.Filters.Add(f);
            *pFilterId = f.Id;
            string kind = type == J.PASS_FILTER ? "PASS" : type == J.BLOCK_FILTER ? "BLOCK" : "FLOW";
            string line = $"FILTER {ch.Id:X8} {BusName(ch.Bus)} {kind} #{f.Id} MASK={Hex(f.Mask)} PATTERN={Hex(f.Pattern)}";
            if (f.Flow != null)
            {
                line += $" FLOW={Hex(f.Flow)}";
                uint resp = Driver.Be32(f.Pattern), req = Driver.Be32(f.Flow);
                lines.Add(line);
                lines.Add($"ECU {ch.Id:X8} {BusName(ch.Bus)} URID={f.Id:X8} REQ={req:X} RESP={resp:X} FORMAT=5 UUDT=0");
            }
            else
            {
                lines.Add(line);
            }
            Log.Write($"  FilterID={f.Id}");
        }
        foreach (var l in lines)
            Link.WriteLine(l);
        return J.STATUS_NOERROR;
    }

    /// <summary>
    /// Tech2Win compatibility for CAN filters: the top 3 bits of a CAN id are
    /// cleared (Tech2Win marks 29-bit ids with bit 29), and a 5-byte filter
    /// (extended address) compares only the 4-byte id, because Tech2Win
    /// leaves the 5th byte uninitialised.
    /// </summary>
    private static void CanFilterFixup(Filter f)
    {
        foreach (var b in new[] { f.Mask, f.Pattern, f.Flow })
        {
            if (b != null && b.Length >= 4)
                b[0] &= 0x1F;
        }
        if (f.Mask.Length == 5)
        {
            f.Mask = f.Mask[..4];
            f.Pattern = f.Pattern[..4];
            if (f.Flow != null)
                f.Flow = f.Flow[..4];
        }
    }

    public static uint StopMsgFilter(uint channelId, uint filterId)
    {
        lock (Sync)
        {
            Log.Write($"PassThruStopMsgFilter Called: ChannelID={channelId} FilterID={filterId}");
            var ch = FindChannel(channelId);
            if (ch == null)
                return Fail(J.ERR_INVALID_CHANNEL_ID, $"invalid channel id {channelId}");
            if (ch.Filters.RemoveAll(f => f.Id == filterId) == 0)
                return Fail(J.ERR_INVALID_FILTER_ID, $"invalid filter id {filterId}");
            return J.STATUS_NOERROR;
        }
    }

    /// <summary>No programmable pins: accepted and logged.</summary>
    public static uint SetProgrammingVoltage(uint deviceId, uint pin, uint voltage)
    {
        Log.Write($"PassThruSetProgrammingVoltage Called: DeviceID={deviceId} Pin={pin} Voltage={voltage} - ignored");
        lock (Sync)
        {
            if (!_open || deviceId != DeviceId)
                return Fail(J.ERR_INVALID_DEVICE_ID, $"invalid device id {deviceId}");
        }
        return J.STATUS_NOERROR;
    }

    public static uint ReadVersion(uint deviceId, byte* fw, byte* dll, byte* api)
    {
        Log.Write($"PassThruReadVersion Called: DeviceID={deviceId}");
        if (fw == null || dll == null || api == null)
            return Fail(J.ERR_NULL_PARAMETER, "a version buffer is NULL");
        lock (Sync)
        {
            if (!_open || deviceId != DeviceId)
                return Fail(J.ERR_INVALID_DEVICE_ID, $"invalid device id {deviceId}");
        }
        CopyStr(fw, Config.ModuleName + " " + Config.DriverVersion);
        CopyStr(dll, Config.DriverName + " " + Config.DriverVersion);
        CopyStr(api, "04.04");
        return J.STATUS_NOERROR;
    }

    public static uint GetLastError(byte* desc)
    {
        if (desc == null)
            return J.ERR_NULL_PARAMETER;
        CopyStr(desc, _lastError);
        return J.STATUS_NOERROR;
    }

    public static uint Ioctl(uint channelId, uint ioctlId, void* input, void* output)
    {
        var lines = new List<string>();
        var periodics = new List<Periodic>();
        lock (Sync)
        {
            Log.Write($"PassThruIoctl Called: ChannelID={channelId} IoctlID=0x{ioctlId:X}");
            if (ioctlId == J.READ_VBATT || ioctlId == J.READ_PROG_VOLTAGE)
            {
                // The device id or a channel id.
                if (!_open || (channelId != DeviceId && FindChannel(channelId) == null))
                    return Fail(J.ERR_INVALID_DEVICE_ID, $"invalid device id {channelId}");
                if (output == null)
                    return Fail(J.ERR_NULL_PARAMETER, "pOutput is NULL");
                *(uint*)output = ioctlId == J.READ_VBATT ? Volatile.Read(ref _vbattMv) : 0;
                return J.STATUS_NOERROR;
            }
            var ch = FindChannel(channelId);
            if (ch == null)
                return Fail(J.ERR_INVALID_CHANNEL_ID, $"invalid channel id {channelId}");
            switch (ioctlId)
            {
                case J.GET_CONFIG:
                case J.SET_CONFIG:
                {
                    var list = (SCONFIG_LIST*)input;
                    if (list == null || (list->NumOfParams > 0 && list->ConfigPtr == null))
                        return Fail(J.ERR_NULL_PARAMETER, "SCONFIG_LIST is NULL");
                    for (uint i = 0; i < list->NumOfParams; i++)
                    {
                        var p = &list->ConfigPtr[i];
                        if (ioctlId == J.GET_CONFIG)
                        {
                            p->Value = ch.Config.TryGetValue(p->Parameter, out uint v) ? v : 0;
                            Log.Write($"  GET {ParamName(p->Parameter)} = {p->Value}");
                        }
                        else
                        {
                            ch.Config[p->Parameter] = p->Value;
                            Log.Write($"  SET {ParamName(p->Parameter)} = {p->Value}");
                            lines.Add($"PARAM {ch.Id:X8} {ParamName(p->Parameter)} {p->Value}");
                        }
                    }
                    break;
                }
                case J.CLEAR_TX_BUFFER:
                    break;   // messages are sent straight away
                case J.CLEAR_RX_BUFFER:
                    ch.Rx.Clear();
                    break;
                case J.CLEAR_PERIODIC_MSGS:
                    periodics.AddRange(ch.Periodics);
                    ch.Periodics.Clear();
                    break;
                case J.CLEAR_MSG_FILTERS:
                    ch.Filters.Clear();
                    lines.Add($"FILTERCLR {ch.Id:X8}");
                    break;
                case J.SW_CAN_HS:
                case J.SW_CAN_NS:
                    if (ch.Bus != Bus.SWCAN)
                        return Fail(J.ERR_INVALID_IOCTL_ID, "SW_CAN_HS / SW_CAN_NS need a single wire CAN channel");
                    break;   // speed changes are not simulated
                case J.FIVE_BAUD_INIT:
                case J.FAST_INIT:
                case J.CLEAR_FUNCT_MSG_LOOKUP_TABLE:
                case J.ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
                case J.DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE:
                    return Fail(J.ERR_NOT_SUPPORTED, $"ioctl 0x{ioctlId:X} is not supported");
                default:
                    return Fail(J.ERR_INVALID_IOCTL_ID, $"invalid ioctl id 0x{ioctlId:X}");
            }
        }
        foreach (var p in periodics)
            p.Timer.Dispose();
        foreach (var l in lines)
            Link.WriteLine(l);
        return J.STATUS_NOERROR;
    }
}
