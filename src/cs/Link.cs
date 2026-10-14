// Links to the simulator(s): a serial port ([Serial] Port) and/or a TCP server
// ([Tcp] Port) that accepts several clients (see README.md, "Clients").
//
// Output is queued and written by one writer thread per link, so the D-PDU
// API never blocks. Every line goes to all links: the serial port and every
// TCP client. Lines received from any of them are echoed to the other links
// (not back to the sender) as "# COM8: ..." / "# 127.0.0.1:52133: ..." and
// then handed to Driver through LineReceived. com0com stalls serial writes
// while nobody has the other end of the pair open, so serial output is
// dropped after a short timeout.
//
// Threads: Open/Close/WriteLine are called on the API thread; the serial
// reader, serial writer, TCP network and TCP writer threads run between Open
// and Close. TxSync guards the output queues, ClientSync the TCP client table.
using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace DpduBridge;

internal static unsafe class Link
{
    private const int TxQueueMax = 2000;        // per link; further lines are dropped
    private const uint TxWriteTimeoutMs = 250;  // serial write / TCP send timeout
    private const int TcpMaxClients = 32;
    // Same limits as the C version's buffers (MAX_FRAME = 4200): received lines
    // are cut at MAX_FRAME * 4 - 1 characters, sent lines (and their ">>" log
    // lines) at MAX_FRAME * 3 + 125.
    private const int MaxLine = 4200 * 4 - 1;
    private const int MaxOutLine = 4200 * 3 + 128 - 3;

    /// <summary>
    /// Text encoding of the line protocol and the log. Latin-1 maps bytes
    /// 0-255 to chars 0-255 and back, so received bytes are echoed and logged
    /// unchanged, as the C version does with its char buffers.
    /// </summary>
    public static readonly Encoding Wire = Encoding.Latin1;

    /// <summary>One connected TCP client.</summary>
    private sealed class TcpClient
    {
        public Socket Sock;
        public int Uid;               // unique per connection (echo suppression)
        public volatile bool Dead;    // send failed: closed by the network thread
        public string Peer;
        public readonly StringBuilder Line = new();
    }

    private static volatile bool _open;
    private static nint _port = Native.INVALID_HANDLE_VALUE;
    private static Socket _listener;
    // Slots are written only by the network thread (under ClientSync); the TCP
    // writer reads them under ClientSync.
    private static readonly TcpClient[] Clients = new TcpClient[TcpMaxClients];
    private static readonly object ClientSync = new();
    // Manual-reset; set by Close to stop all link threads. Created on the first
    // Open together with the queue events and kept for the DLL's lifetime.
    private static nint _stopEvt;
    private static Thread _rxThread, _netThread;   // serial reader, TCP server

    /// <summary>Output queue of one link.  Each link has its own queue and writer
    /// thread, so a stalled serial port never delays the TCP clients (or vice versa).</summary>
    private sealed class OutQueue
    {
        public readonly Queue<OutLine> Lines = new();  // guarded by TxSync
        public bool Active;                            // link open: lines are queued
        public nint Evt;                               // auto-reset: lines queued
        public Thread Thread;
    }

    /// <summary>A queued line; Skip is the Uid of a TCP client that must not get it (0 = none).</summary>
    private sealed record OutLine(byte[] Data, int Skip);

    private static readonly OutQueue SerialQ = new(), TcpQ = new();
    private static readonly object TxSync = new();
    private static int _nextClientUid;

    /// <summary>Called on the reader / network thread for every received line.</summary>
    public static Action<string> LineReceived;

    /// <summary>True when a DOS device (e.g. COM8) with this name exists.</summary>
    public static bool PortExists(string name)
    {
        char* target = stackalloc char[512];
        fixed (char* n = name)
            return Native.QueryDosDeviceW(n, target, 512) != 0;
    }

    /// <summary>Opens every configured link (serial port and/or TCP server).
    /// Succeeds when at least one of them could be opened.</summary>
    public static bool Open()
    {
        if (_open)
            return true;
        if (!Config.UseSerial && Config.TcpPort == 0)
        {
            Log.Write("no valid port specified - not opening a port");
            return false;
        }
        if (_stopEvt == 0)
        {
            _stopEvt = Native.CreateEventW(0, 1, 0, null);
            SerialQ.Evt = Native.CreateEventW(0, 0, 0, null);
            TcpQ.Evt = Native.CreateEventW(0, 0, 0, null);
        }
        bool serialOk = Config.UseSerial && SerialSetup();
        bool tcpOk = Config.TcpPort > 0 && TcpListen();
        if (!serialOk && !tcpOk)
            return false;

        PinModule();
        Native.ResetEvent(_stopEvt);
        lock (TxSync)
        {
            SerialQ.Active = serialOk;
            TcpQ.Active = tcpOk;
            _open = true;
        }
        _rxThread = serialOk ? NewThread(SerialReaderLoop, "serial rx") : null;
        SerialQ.Thread = serialOk ? NewThread(SerialWriterLoop, "serial tx") : null;
        _netThread = tcpOk ? NewThread(TcpLoop, "tcp") : null;
        TcpQ.Thread = tcpOk ? NewThread(TcpWriterLoop, "tcp tx") : null;
        try
        {
            _rxThread?.Start();
            SerialQ.Thread?.Start();
            _netThread?.Start();
            TcpQ.Thread?.Start();
        }
        catch (Exception ex) when (ex is OutOfMemoryException || ex is ThreadStartException)
        {
            Log.Write("cannot start worker threads");
            Close();
            return false;
        }
        if (serialOk)
            Log.Write($"opened {Config.PortName} at {Config.Baud} baud");
        if (tcpOk)
            Log.Write($"listening on {Config.TcpDesc}");
        return true;
    }

    private static bool _pinned;

    /// <summary>
    /// Pins this DLL in memory for the rest of the process (once, before the
    /// first link thread starts). A host that unloads the DLL while a link
    /// thread is still running (e.g. FreeLibrary without PDUModuleDisconnect)
    /// would otherwise leave that thread executing unmapped code; the C version
    /// prevents this with a module reference per thread. A Native AOT library
    /// cannot be unloaded safely in any case.
    /// </summary>
    private static void PinModule()
    {
        if (_pinned)
            return;
        nint module;
        delegate* unmanaged[Stdcall]<uint> anyExport = &Exports.PDUDestruct;
        _pinned = Native.GetModuleHandleExW(Native.GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                                            Native.GET_MODULE_HANDLE_EX_FLAG_PIN, anyExport, &module) != 0;
    }

    /// <summary>
    /// A background link thread. An unhandled exception on any thread would
    /// terminate the host process, so the body's exceptions are logged and
    /// end only that thread.
    /// </summary>
    private static Thread NewThread(ThreadStart body, string name)
    {
        string fullName = Config.DriverName + " " + name;
        return new Thread(() =>
        {
            try
            {
                body();
            }
            catch (Exception ex)
            {
                Log.Write($"{fullName} thread failed: {ex}");
            }
        }) { IsBackground = true, Name = fullName };
    }

    /// <summary>Opens and configures the COM port (8N1, DTR/RTS on, no flow control, overlapped I/O).</summary>
    private static bool SerialSetup()
    {
        string path = @"\\.\" + Config.PortName;
        nint h;
        fixed (char* p = path)
            h = Native.CreateFileW(p, Native.GENERIC_READ | Native.GENERIC_WRITE, 0, 0,
                                   Native.OPEN_EXISTING, Native.FILE_FLAG_OVERLAPPED, 0);
        if (h == Native.INVALID_HANDLE_VALUE)
        {
            Log.Write($"cannot open {Config.PortName} (error {Native.LastError})");
            return false;
        }
        DCB dcb = default;
        dcb.DCBlength = (uint)sizeof(DCB);
        Native.GetCommState(h, &dcb);
        dcb.BaudRate = Config.Baud;
        dcb.ByteSize = 8;
        dcb.Parity = 0;   // NOPARITY
        dcb.StopBits = 0; // ONESTOPBIT
        dcb.Flags = 1u            // fBinary
                  | (1u << 4)     // fDtrControl = DTR_CONTROL_ENABLE
                  | (1u << 12);   // fRtsControl = RTS_CONTROL_ENABLE  (no CTS/DSR/XON flow control)
        Native.SetCommState(h, &dcb);
        COMMTIMEOUTS to = default;
        to.ReadIntervalTimeout = 0xFFFFFFFF;       // return as soon as data arrives
        to.ReadTotalTimeoutMultiplier = 0xFFFFFFFF;
        to.ReadTotalTimeoutConstant = 1000;
        to.WriteTotalTimeoutConstant = 1000;
        Native.SetCommTimeouts(h, &to);
        Native.PurgeComm(h, Native.PURGE_RXCLEAR | Native.PURGE_TXCLEAR);
        _port = h;
        return true;
    }

    private static bool TcpListen()
    {
        // [Tcp] Bind is an IPv4 or an IPv6 address.
        if (!IPAddress.TryParse(Config.TcpBind, out var addr) ||
            (addr.AddressFamily != AddressFamily.InterNetwork && addr.AddressFamily != AddressFamily.InterNetworkV6))
        {
            Log.Write($"invalid [Tcp] Bind address '{Config.TcpBind}'");
            return false;
        }
        Socket s;
        try
        {
            s = new Socket(addr.AddressFamily, SocketType.Stream, ProtocolType.Tcp);
        }
        catch (SocketException ex)
        {
            Log.Write($"cannot create socket ({ex.ErrorCode})");
            return false;
        }
        try
        {
            s.ExclusiveAddressUse = true;
            // An IPv6 socket bound to "::" also accepts IPv4 clients (dual stack).
            if (addr.AddressFamily == AddressFamily.InterNetworkV6)
                s.DualMode = true;
            s.Bind(new IPEndPoint(addr, Config.TcpPort));
            s.Listen();   // backlog SOMAXCONN
        }
        catch (SocketException ex)
        {
            Log.Write($"cannot listen on {Config.TcpDesc} (error {ex.ErrorCode})");
            s.Dispose();
            return false;
        }
        _listener = s;
        return true;
    }

    /// <summary>
    /// Flushes the queues briefly, stops the link threads and closes the
    /// serial port and TCP server. Must not be called while holding the
    /// driver lock: the reader threads may be waiting for it.
    /// </summary>
    public static void Close()
    {
        if (!_open)
            return;
        int t0 = Environment.TickCount;
        while (QueueCount() > 0 && Environment.TickCount - t0 < 2 * TxWriteTimeoutMs)
            Thread.Sleep(5);  // let queued lines (e.g. "BYE") go out
        Native.SetEvent(_stopEvt);
        Join(_rxThread);
        Join(_netThread);     // the TCP thread disconnects all clients
        Join(SerialQ.Thread);
        Join(TcpQ.Thread);
        _rxThread = _netThread = SerialQ.Thread = TcpQ.Thread = null;
        lock (TxSync)
        {
            _open = false;
            foreach (var q in new[] { SerialQ, TcpQ })
            {
                q.Active = false;
                q.Lines.Clear();
            }
        }
        if (_listener != null)
        {
            _listener.Dispose();
            _listener = null;
            Log.Write($"closed {Config.TcpDesc}");
        }
        if (_port != Native.INVALID_HANDLE_VALUE)
        {
            Native.CloseHandle(_port);
            _port = Native.INVALID_HANDLE_VALUE;
            Log.Write($"closed {Config.PortName}");
        }
    }

    /// <summary>Waits up to 3 s for a link thread (skipped when Close runs on that thread, e.g. from a host callback).</summary>
    private static void Join(Thread t)
    {
        if (t != null && t != Thread.CurrentThread && (t.ThreadState & ThreadState.Unstarted) == 0)
            t.Join(3000);
    }

    private static int QueueCount()
    {
        lock (TxSync)
            return (SerialQ.Thread != null ? SerialQ.Lines.Count : 0) + (TcpQ.Thread != null ? TcpQ.Lines.Count : 0);
    }

    /// <summary>Queues a line for every open link (serial port and TCP clients); never blocks.</summary>
    public static void WriteLine(string line) => Write(line, toSerial: true, skipClient: 0);

    /// <summary>Queues a line for the serial port (if toSerial) and every TCP client
    /// except the one with Uid skipClient; never blocks.</summary>
    private static void Write(string line, bool toSerial, int skipClient)
    {
        if (line.Length > MaxOutLine)
            line = line.Substring(0, MaxOutLine);
        Log.Write(">> " + line);
        var item = new OutLine(Wire.GetBytes(line + "\r\n"), skipClient);
        lock (TxSync)
        {
            if (!_open)
                return;
            foreach (var q in toSerial ? new[] { SerialQ, TcpQ } : new[] { TcpQ })
            {
                if (q.Active && q.Lines.Count < TxQueueMax)
                {
                    q.Lines.Enqueue(item);
                    Native.SetEvent(q.Evt);
                }
            }
        }
    }

    /// <summary>
    /// Splits received bytes into lines (CR and/or LF terminated; empty lines
    /// skipped), keeping a partial line in <paramref name="line"/> between
    /// calls. An exception from onLine is logged and does not stop the reader.
    /// </summary>
    private static void SplitLines(StringBuilder line, byte* data, int n, Action<string> onLine)
    {
        for (int i = 0; i < n; i++)
        {
            char ch = (char)data[i];
            if (ch == '\n' || ch == '\r')
            {
                if (line.Length > 0)
                {
                    string s = line.ToString();
                    line.Clear();
                    // The C version handles lines as C strings: a NUL byte ends the line.
                    int nul = s.IndexOf('\0');
                    if (nul >= 0)
                        s = s.Substring(0, nul);
                    try { onLine(s); }
                    catch (Exception ex) { Log.Write("error processing line: " + ex.Message); }
                }
            }
            else if (line.Length < MaxLine)
            {
                line.Append(ch);
            }
        }
    }

    // ---- writers -----------------------------------------------------------------

    private static OutLine Dequeue(OutQueue q)
    {
        lock (TxSync)
            return q.Lines.Count > 0 ? q.Lines.Dequeue() : null;
    }

    /// <summary>Writer thread of the TCP clients.</summary>
    private static void TcpWriterLoop()
    {
        nint* waits = stackalloc nint[2];
        waits[0] = _stopEvt;
        waits[1] = TcpQ.Evt;
        while (Native.WaitForMultipleObjects(2, waits, 0, Native.INFINITE) != Native.WAIT_OBJECT_0)
        {
            OutLine item;
            while ((item = Dequeue(TcpQ)) != null)
            {
                TcpBroadcast(item.Data, item.Skip);
                if (Native.WaitForSingleObject(_stopEvt, 0) == Native.WAIT_OBJECT_0)
                    break;
            }
        }
    }

    /// <summary>Writer thread of the serial port.</summary>
    private static void SerialWriterLoop()
    {
        // The OVERLAPPED lives in native memory: the kernel updates it until
        // the I/O completes, so it must not be moved by the GC.
        OVERLAPPED* ov = (OVERLAPPED*)NativeMemory.AllocZeroed((nuint)sizeof(OVERLAPPED));
        ov->hEvent = Native.CreateEventW(0, 1, 0, null);
        nint* waits = stackalloc nint[2];
        waits[0] = _stopEvt;
        waits[1] = SerialQ.Evt;
        bool stalled = false;
        try
        {
            while (Native.WaitForMultipleObjects(2, waits, 0, Native.INFINITE) != Native.WAIT_OBJECT_0)
            {
                OutLine item;
                while ((item = Dequeue(SerialQ)) != null)
                {
                    bool done = SerialWrite(ov, item.Data);
                    if (!done && !stalled)
                        Log.Write($"serial output dropped - is the simulator reading the other end of {Config.PortName}?");
                    stalled = !done;
                    if (Native.WaitForSingleObject(_stopEvt, 0) == Native.WAIT_OBJECT_0)
                        break;
                }
            }
        }
        finally
        {
            Native.CloseHandle(ov->hEvent);
            NativeMemory.Free(ov);
        }
    }

    /// <summary>
    /// Writes one line to the serial port. False when it failed or did not
    /// complete within TxWriteTimeoutMs (the write is then cancelled before
    /// returning, so <paramref name="data"/> is no longer in use).
    /// </summary>
    private static bool SerialWrite(OVERLAPPED* ov, byte[] data)
    {
        uint written = 0;
        Native.ResetEvent(ov->hEvent);
        fixed (byte* p = data)
        {
            bool done = Native.WriteFile(_port, p, (uint)data.Length, &written, ov) != 0;
            if (!done && Native.LastError == Native.ERROR_IO_PENDING)
            {
                if (Native.WaitForSingleObject(ov->hEvent, TxWriteTimeoutMs) == Native.WAIT_OBJECT_0)
                    return Native.GetOverlappedResult(_port, ov, &written, 0) != 0;
                // Nobody is reading the other end of the pair: drop it.
                Native.CancelIo(_port);
                Native.GetOverlappedResult(_port, ov, &written, 1);
                Native.PurgeComm(_port, Native.PURGE_TXABORT | Native.PURGE_TXCLEAR);
                return false;
            }
            return done;
        }
    }

    /// <summary>Sends to every client.  A failing client is only marked dead
    /// here; the network thread closes it.</summary>
    private static void TcpBroadcast(byte[] data, int skip)
    {
        lock (ClientSync)
        {
            foreach (var c in Clients)
            {
                if (c == null || c.Dead || (skip != 0 && c.Uid == skip))
                    continue;
                try
                {
                    int sent = 0;
                    while (sent < data.Length)
                        sent += c.Sock.Send(data, sent, data.Length - sent, SocketFlags.None);
                }
                catch (SocketException ex)
                {
                    c.Dead = true;
                    Log.Write($"{c.Peer}: send failed ({ex.ErrorCode}) - disconnecting");
                }
            }
        }
    }

    // ---- serial reader -----------------------------------------------------------

    /// <summary>Reader thread of the serial port: overlapped reads until Close sets the stop event.</summary>
    private static void SerialReaderLoop()
    {
        OVERLAPPED* ov = (OVERLAPPED*)NativeMemory.AllocZeroed((nuint)sizeof(OVERLAPPED));
        byte* buf = (byte*)NativeMemory.Alloc(512);
        ov->hEvent = Native.CreateEventW(0, 1, 0, null);
        nint* waits = stackalloc nint[2];
        waits[0] = _stopEvt;
        waits[1] = ov->hEvent;
        var line = new StringBuilder();
        try
        {
            while (true)
            {
                uint got = 0;
                Native.ResetEvent(ov->hEvent);
                if (Native.ReadFile(_port, buf, 512, &got, ov) == 0)
                {
                    if (Native.LastError != Native.ERROR_IO_PENDING)
                    {
                        if (Native.WaitForSingleObject(_stopEvt, 100) == Native.WAIT_OBJECT_0)
                            break;
                        continue;
                    }
                    if (Native.WaitForMultipleObjects(2, waits, 0, Native.INFINITE) == Native.WAIT_OBJECT_0)
                    {
                        Native.CancelIo(_port);
                        Native.GetOverlappedResult(_port, ov, &got, 1);
                        break;
                    }
                    if (Native.GetOverlappedResult(_port, ov, &got, 0) == 0)
                        continue;
                }
                if (Native.WaitForSingleObject(_stopEvt, 0) == Native.WAIT_OBJECT_0)
                    break;
                SplitLines(line, buf, (int)got, s => FromClient(Config.PortName, null, s));
            }
        }
        finally
        {
            Native.CloseHandle(ov->hEvent);
            NativeMemory.Free(buf);
            NativeMemory.Free(ov);
        }
    }

    // ---- TCP network thread ------------------------------------------------------

    /// <summary>Accepts clients and reads their lines.  It is the only thread
    /// that adds, removes or closes clients.</summary>
    private static void TcpLoop()
    {
        try
        {
            TcpPoll();
        }
        finally
        {
            for (int i = 0; i < TcpMaxClients; i++)
                DropClient(i);
        }
    }

    /// <summary>Body of the network thread: select() on the listener and the clients until Close.</summary>
    private static void TcpPoll()
    {
        byte[] buf = new byte[1024];
        var readList = new List<Socket>();
        while (Native.WaitForSingleObject(_stopEvt, 0) != Native.WAIT_OBJECT_0)
        {
            readList.Clear();
            readList.Add(_listener);
            for (int i = 0; i < TcpMaxClients; i++)
            {
                if (Clients[i] != null && Clients[i].Dead)
                    DropClient(i);
                if (Clients[i] != null)
                    readList.Add(Clients[i].Sock);
            }
            try
            {
                Socket.Select(readList, null, null, 200_000);
            }
            catch (SocketException)
            {
                Thread.Sleep(50);
                continue;
            }
            if (readList.Contains(_listener))
                AcceptClient();
            for (int i = 0; i < TcpMaxClients; i++)
            {
                var c = Clients[i];
                if (c == null || !readList.Contains(c.Sock))
                    continue;
                int r;
                try { r = c.Sock.Receive(buf); }
                catch (SocketException) { r = 0; }
                if (r <= 0)
                {
                    DropClient(i);
                    continue;
                }
                fixed (byte* p = buf)
                    SplitLines(c.Line, p, r, s => ClientLine(c, s));
            }
        }
    }

    private static void ClientLine(TcpClient c, string line) => FromClient(c.Peer, c, line);

    /// <summary>A line from the serial port (client == null) or a TCP client: echoed
    /// as a comment to the other clients (not back to its sender), so every client
    /// sees the complete traffic, then delivered to the host.</summary>
    private static void FromClient(string source, TcpClient client, string line)
    {
        line = line.TrimStart(' ', '\t');
        if (line.Length == 0)
            return;
        Write($"# {source}: {line}", toSerial: client != null, skipClient: client?.Uid ?? 0);
        LineReceived?.Invoke(line);
    }

    private static void AcceptClient()
    {
        Socket s;
        try { s = _listener.Accept(); }
        catch (SocketException) { return; }
        string peer;
        try
        {
            // Both can fail when the client has already reset the connection.
            peer = FormatPeer(s.RemoteEndPoint as IPEndPoint);
            s.SendTimeout = (int)TxWriteTimeoutMs;   // a client that stops reading can't stall the writer
            s.NoDelay = true;
        }
        catch (SocketException)
        {
            s.Dispose();
            return;
        }
        int slot = Array.IndexOf(Clients, null);
        if (slot < 0)
        {
            try { s.Send(Wire.GetBytes("# too many clients\r\n")); } catch (SocketException) { }
            s.Dispose();
            Log.Write($"TCP connection from {peer} refused - too many clients");
            return;
        }
        int uid = ++_nextClientUid;
        if (uid == 0)
            uid = ++_nextClientUid;   // never 0
        var c = new TcpClient { Sock = s, Uid = uid, Peer = peer };
        lock (ClientSync)
            Clients[slot] = c;
        WriteLine($"# {c.Peer} connected");
    }

    /// <summary>"a.b.c.d:port" for IPv4 (including IPv4-mapped IPv6), "[v6]:port" for IPv6.</summary>
    private static string FormatPeer(IPEndPoint ep)
    {
        if (ep == null)
            return "?";
        var a = ep.Address.IsIPv4MappedToIPv6 ? ep.Address.MapToIPv4() : ep.Address;
        return a.AddressFamily == AddressFamily.InterNetworkV6 ? $"[{a}]:{ep.Port}" : $"{a}:{ep.Port}";
    }

    /// <summary>Removes and closes a client (network thread only) and tells the other clients.</summary>
    private static void DropClient(int slot)
    {
        TcpClient c;
        lock (ClientSync)
        {
            c = Clients[slot];
            Clients[slot] = null;
        }
        if (c == null)
            return;
        c.Sock.Dispose();
        WriteLine($"# {c.Peer} disconnected");
    }
}
