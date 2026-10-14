// Configuration and logging.
//
// Config reads dpdu-bridge-cs.ini, or the dpdu-bridge.ini shared with the C
// version when that doesn't exist, from the directory the DLL is in. The keys,
// defaults and parsing match the C version (see README.md, "Configuration").
// Log appends timestamped lines to the log file named in the ini.
using System;
using System.IO;
using System.Linq;
using System.Text;

namespace DpduBridge;

/// <summary>
/// Settings from the ini file. Loaded once, by the static constructor of
/// Driver, and read-only afterwards.
/// </summary>
internal static class Config
{
    public const string DriverName = "dpdu-bridge-cs";  // HELLO line, message box title, log
    public static string PortName = "";   // [Serial] Port; empty: no serial link
    public static bool SerialBad;          // [Serial] Port is a number, not a COM port name
    public static string TcpPortStr = "";  // [Tcp] Port as written
    public static int TcpPort;             // > 0: TCP server on this port
    public static bool TcpBad;             // [Tcp] Port is not 1..65535
    public static string TcpBind = "127.0.0.1";
    public static string TcpDesc = "";     // "TCP 127.0.0.1:5000" or "TCP [::1]:5000"
    public static string LinkDesc = "";    // e.g. "COM8 + TCP 127.0.0.1:5000"

    /// <summary>A usable serial port is configured.</summary>
    public static bool UseSerial => PortName.Length > 0 && !SerialBad;
    public static string IniPath = "";    // the ini file that was read (or would have been)
    public static uint Baud = 115200;
    public static bool LogEnabled;         // [Log] Enabled; off unless the ini sets Enabled=1
    public static string LogPath = "";
    public static string ModuleName = "D-PDU Serial/TCP Bridge";
    public static uint VBattMilliVolts = 13000;
    public static bool SoftwareFilters = true;

    public static unsafe void Load()
    {
        // The ini named after the DLL (dpdu-bridge-cs.dll -> dpdu-bridge-cs.ini)
        // is used if it exists, else the dpdu-bridge.ini shared with the C
        // version, both next to the DLL.  The log is named after the DLL
        // (dpdu-bridge-cs.log).
        string dir = "", stem = "dpdu-bridge-cs";
        nint module;
        // The address of any export identifies this DLL's module (a Native AOT
        // library has no managed Assembly.Location to ask).
        delegate* unmanaged[Stdcall]<uint> anyExport = &Exports.PDUDestruct;
        if (Native.GetModuleHandleExW(Native.GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                                      Native.GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                      anyExport, &module) != 0)
        {
            char* buf = stackalloc char[1024];
            uint n = Native.GetModuleFileNameW(module, buf, 1024);
            string path = new string(buf, 0, (int)n);
            dir = Path.GetDirectoryName(path) ?? "";
            stem = Path.Combine(dir, Path.GetFileNameWithoutExtension(path));
        }

        // Same Win32 INI functions, keys, defaults and parsing as the C version.
        string ini = stem + ".ini";
        if (!File.Exists(ini))
            ini = Path.Combine(dir, "dpdu-bridge.ini");
        IniPath = ini;
        // There are no default ports.  [Serial] Port names a COM port and
        // [Tcp] Port a TCP port number; either or both can be used.
        static bool Digits(string s) => s.Length > 0 && s.All(ch => ch >= '0' && ch <= '9');
        PortName = GetString(ini, "Serial", "Port", "");
        SerialBad = Digits(PortName);
        TcpPortStr = GetString(ini, "Tcp", "Port", "");
        TcpPort = 0;
        TcpBad = false;
        if (TcpPortStr.Length > 0)
        {
            long v = Digits(TcpPortStr) && TcpPortStr.Length <= 5 ? long.Parse(TcpPortStr) : 0;
            if (v >= 1 && v <= 65535)
                TcpPort = (int)v;
            else
                TcpBad = true;
        }
        TcpBind = GetString(ini, "Tcp", "Bind", "127.0.0.1");
        // IPv6 addresses are written as [addr]:port
        TcpDesc = TcpBind.Contains(':') ? $"TCP [{TcpBind}]:{TcpPort}" : $"TCP {TcpBind}:{TcpPort}";
        LinkDesc = UseSerial ? PortName : "";
        if (TcpPort > 0)
            LinkDesc += (LinkDesc.Length > 0 ? " + " : "") + TcpDesc;
        Baud = GetInt(ini, "Serial", "Baud", 115200);
        LogEnabled = GetInt(ini, "Log", "Enabled", 0) != 0;
        LogPath = GetString(ini, "Log", "File", stem + ".log");
        ModuleName = GetString(ini, "Device", "ModuleName", "D-PDU Serial/TCP Bridge");
        VBattMilliVolts = GetInt(ini, "Device", "VBattMilliVolts", 13000);
        SoftwareFilters = GetInt(ini, "Device", "SoftwareFilters", 1) != 0;
    }

    /// <summary>GetPrivateProfileString; a missing or empty value gives the default.</summary>
    private static unsafe string GetString(string ini, string section, string key, string def)
    {
        char* buf = stackalloc char[512];
        uint n;
        fixed (char* s = section, k = key, d = "", f = ini)
            n = Native.GetPrivateProfileStringW(s, k, d, buf, 512, f);
        return n > 0 ? new string(buf, 0, (int)n) : def;
    }

    private static unsafe uint GetInt(string ini, string section, string key, uint def)
    {
        fixed (char* s = section, k = key, f = ini)
            return Native.GetPrivateProfileIntW(s, k, (int)def, f);
    }
}

/// <summary>
/// The driver's log file. Called from the API thread and the link threads;
/// the file is opened per line so it can be read (or deleted) while the DLL
/// is loaded, as with the C version.
/// </summary>
internal static class Log
{
    private static readonly object Sync = new();

    /// <summary>Appends one line; never throws. Does nothing until Config is loaded.</summary>
    public static void Write(string msg)
    {
        if (!Config.LogEnabled || Config.LogPath.Length == 0)
            return;
        try
        {
            lock (Sync)
                // Latin-1 writes received bytes back unchanged (see Link.Wire).
                File.AppendAllText(Config.LogPath, $"[{DateTime.Now:HH:mm:ss.fff}] {msg}\r\n", Encoding.Latin1);
        }
        catch
        {
            // logging must never break the API
        }
    }
}
