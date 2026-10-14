// Win32 interop used by Config and Link: serial port I/O, events and waits,
// module path lookup, INI file access and the configuration error message box.
// Every signature is blittable (pointers and integers only), so Native AOT
// generates direct calls without marshalling stubs.
using System.Runtime.InteropServices;

namespace DpduBridge;

/// <summary>Win32 DCB. The C bit fields (fBinary, fDtrControl, ...) are packed into <see cref="Flags"/>.</summary>
[StructLayout(LayoutKind.Sequential)]
internal struct DCB
{
    public uint DCBlength, BaudRate, Flags;
    public ushort wReserved, XonLim, XoffLim;
    public byte ByteSize, Parity, StopBits;
    public byte XonChar, XoffChar, ErrorChar, EofChar, EvtChar;
    public ushort wReserved1;
}

[StructLayout(LayoutKind.Sequential)]
internal struct COMMTIMEOUTS
{
    public uint ReadIntervalTimeout, ReadTotalTimeoutMultiplier, ReadTotalTimeoutConstant;
    public uint WriteTotalTimeoutMultiplier, WriteTotalTimeoutConstant;
}

[StructLayout(LayoutKind.Sequential)]
internal struct OVERLAPPED
{
    public nuint Internal, InternalHigh;
    public uint Offset, OffsetHigh;
    public nint hEvent;
}

internal static unsafe class Native
{
    public const uint GENERIC_READ = 0x80000000, GENERIC_WRITE = 0x40000000;
    public const uint OPEN_EXISTING = 3, FILE_FLAG_OVERLAPPED = 0x40000000;
    public const int ERROR_IO_PENDING = 997;
    public const uint WAIT_OBJECT_0 = 0, INFINITE = 0xFFFFFFFF;
    public const uint PURGE_TXABORT = 1, PURGE_TXCLEAR = 4, PURGE_RXCLEAR = 8;
    public const uint GET_MODULE_HANDLE_EX_FLAG_PIN = 1, GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT = 2,
                      GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS = 4;
    public static readonly nint INVALID_HANDLE_VALUE = -1;

    [DllImport("kernel32", SetLastError = true)]
    public static extern nint CreateFileW(char* name, uint access, uint share, nint sa, uint creation, uint flags, nint template);
    [DllImport("kernel32", SetLastError = true)]
    public static extern int ReadFile(nint h, byte* buf, uint n, uint* read, OVERLAPPED* ov);
    [DllImport("kernel32", SetLastError = true)]
    public static extern int WriteFile(nint h, byte* buf, uint n, uint* written, OVERLAPPED* ov);
    [DllImport("kernel32", SetLastError = true)]
    public static extern int GetOverlappedResult(nint h, OVERLAPPED* ov, uint* n, int wait);
    [DllImport("kernel32")] public static extern int CancelIo(nint h);
    [DllImport("kernel32")] public static extern int PurgeComm(nint h, uint flags);
    [DllImport("kernel32")] public static extern int GetCommState(nint h, DCB* dcb);
    [DllImport("kernel32")] public static extern int SetCommState(nint h, DCB* dcb);
    [DllImport("kernel32")] public static extern int SetCommTimeouts(nint h, COMMTIMEOUTS* t);
    [DllImport("kernel32")] public static extern int CloseHandle(nint h);
    [DllImport("kernel32")] public static extern nint CreateEventW(nint sa, int manualReset, int initialState, char* name);
    [DllImport("kernel32")] public static extern int SetEvent(nint h);
    [DllImport("kernel32")] public static extern int ResetEvent(nint h);
    [DllImport("kernel32")] public static extern uint WaitForSingleObject(nint h, uint ms);
    [DllImport("kernel32")] public static extern uint WaitForMultipleObjects(uint n, nint* handles, int waitAll, uint ms);
    [DllImport("kernel32")] public static extern uint QueryDosDeviceW(char* name, char* target, uint max);
    [DllImport("kernel32")] public static extern int GetModuleHandleExW(uint flags, void* address, nint* module);
    [DllImport("kernel32")] public static extern uint GetModuleFileNameW(nint module, char* buf, uint size);
    [DllImport("kernel32")] public static extern uint GetPrivateProfileStringW(char* section, char* key, char* def, char* buf, uint size, char* file);
    [DllImport("kernel32")] public static extern uint GetPrivateProfileIntW(char* section, char* key, int def, char* file);
    [DllImport("user32")] public static extern int MessageBoxW(nint hwnd, char* text, char* caption, uint type);

    public const uint MB_OK = 0, MB_ICONERROR = 0x10, MB_SETFOREGROUND = 0x10000, MB_TOPMOST = 0x40000;

    /// <summary>GetLastError() of the last P/Invoke declared with SetLastError = true.</summary>
    public static int LastError => Marshal.GetLastPInvokeError();
}
