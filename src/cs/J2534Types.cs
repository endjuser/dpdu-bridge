// SAE J2534-1 (04.04) and J2534-2 constants and structures used by the
// PassThru front end. 32-bit layout: "unsigned long" is 4 bytes, pointers are
// 4 bytes.
using System.Runtime.InteropServices;

namespace DpduBridge;

internal static class J
{
    // Return codes
    public const uint STATUS_NOERROR = 0x00, ERR_NOT_SUPPORTED = 0x01, ERR_INVALID_CHANNEL_ID = 0x02,
                      ERR_INVALID_PROTOCOL_ID = 0x03, ERR_NULL_PARAMETER = 0x04, ERR_INVALID_IOCTL_VALUE = 0x05,
                      ERR_INVALID_FLAGS = 0x06, ERR_FAILED = 0x07, ERR_DEVICE_NOT_CONNECTED = 0x08,
                      ERR_TIMEOUT = 0x09, ERR_INVALID_MSG = 0x0A, ERR_INVALID_TIME_INTERVAL = 0x0B,
                      ERR_EXCEEDED_LIMIT = 0x0C, ERR_INVALID_MSG_ID = 0x0D, ERR_DEVICE_IN_USE = 0x0E,
                      ERR_INVALID_IOCTL_ID = 0x0F, ERR_BUFFER_EMPTY = 0x10, ERR_BUFFER_FULL = 0x11,
                      ERR_BUFFER_OVERFLOW = 0x12, ERR_PIN_INVALID = 0x13, ERR_CHANNEL_IN_USE = 0x14,
                      ERR_MSG_PROTOCOL_ID = 0x15, ERR_INVALID_FILTER_ID = 0x16, ERR_NO_FLOW_CONTROL = 0x17,
                      ERR_NOT_UNIQUE = 0x18, ERR_INVALID_BAUDRATE = 0x19, ERR_INVALID_DEVICE_ID = 0x1A;

    // Protocol ids (J2534-1, and the J2534-2 pin-selected variants)
    public const uint J1850VPW = 1, J1850PWM = 2, ISO9141 = 3, ISO14230 = 4, CAN = 5, ISO15765 = 6;
    public const uint CAN_PS = 0x8004, ISO15765_PS = 0x8005, SW_ISO15765_PS = 0x8007, SW_CAN_PS = 0x8008,
                      GM_UART_PS = 0x8009;
    // GM UART (ALDL) protocol id of the GM / MDI 2 vendor extensions
    public const uint GM_UART_VENDOR = 0x10003;

    // Ioctl ids
    public const uint GET_CONFIG = 0x01, SET_CONFIG = 0x02, READ_VBATT = 0x03, FIVE_BAUD_INIT = 0x04,
                      FAST_INIT = 0x05, CLEAR_TX_BUFFER = 0x07, CLEAR_RX_BUFFER = 0x08,
                      CLEAR_PERIODIC_MSGS = 0x09, CLEAR_MSG_FILTERS = 0x0A, CLEAR_FUNCT_MSG_LOOKUP_TABLE = 0x0B,
                      ADD_TO_FUNCT_MSG_LOOKUP_TABLE = 0x0C, DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE = 0x0D,
                      READ_PROG_VOLTAGE = 0x0E, SW_CAN_HS = 0x8000, SW_CAN_NS = 0x8001;

    // Configuration parameters (GET_CONFIG / SET_CONFIG)
    public const uint DATA_RATE = 0x01, LOOPBACK = 0x03, CAN_MIXED_FORMAT = 0x8000, J1962_PINS = 0x8001;

    // Filter types
    public const uint PASS_FILTER = 1, BLOCK_FILTER = 2, FLOW_CONTROL_FILTER = 3;

    // TxFlags / connect flags
    public const uint ISO15765_ADDR_TYPE = 0x80, CAN_29BIT_ID = 0x100, SW_CAN_HV_TX = 0x400;

    // RxStatus
    public const uint TX_MSG_TYPE = 0x01, TX_DONE = 0x08;

    public const int MaxData = 4128;     // PASSTHRU_MSG.Data
    // Per channel. J2534-1 only requires 10, but Tech2Win sets one filter per
    // CANdi table entry: 64 for the default GMLAN table.
    public const int MaxFilters = 128;
    public const int MaxPeriodic = 10;   // per channel (J2534-1 minimum)
}

/// <summary>PASSTHRU_MSG (4152 bytes).</summary>
[StructLayout(LayoutKind.Sequential)]
public unsafe struct PASSTHRU_MSG
{
    public uint ProtocolID, RxStatus, TxFlags, Timestamp, DataSize, ExtraDataIndex;
    public fixed byte Data[J.MaxData];
}

[StructLayout(LayoutKind.Sequential)]
public struct SCONFIG { public uint Parameter, Value; }

[StructLayout(LayoutKind.Sequential)]
public unsafe struct SCONFIG_LIST { public uint NumOfParams; public SCONFIG* ConfigPtr; }
