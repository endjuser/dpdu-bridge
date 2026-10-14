// ISO 22900-2 (MVCI D-PDU API) constants and structures used by the driver.
// The layouts match src/c/pdu_api.h for the 32-bit (x86) build: sequential,
// natural alignment, 4-byte pointers. The structures are public because they
// appear in the signatures of the public Exports methods.
using System.Runtime.InteropServices;

namespace DpduBridge;

/// <summary>D-PDU API constant values (PDU_ prefix dropped) used by this driver.</summary>
internal static class Pdu
{
    public const uint ID_UNDEF = 0xFFFFFFFE;
    public const uint HANDLE_UNDEF = 0xFFFFFFFF;

    // T_PDU_ERROR
    public const uint STATUS_NOERROR = 0x00;
    public const uint ERR_FCT_FAILED = 0x01;
    public const uint ERR_PDUAPI_NOT_CONSTRUCTED = 0x20;
    public const uint ERR_SHARING_VIOLATION = 0x21;
    public const uint ERR_INVALID_PARAMETERS = 0x50;
    public const uint ERR_INVALID_HANDLE = 0x60;
    public const uint ERR_ID_NOT_SUPPORTED = 0x62;
    public const uint ERR_EVENT_QUEUE_EMPTY = 0x71;
    public const uint ERR_CLL_CONNECTED = 0x84;
    public const uint ERR_MODULE_NOT_CONNECTED = 0xA3;

    // T_PDU_IT
    public const uint IT_IO_UNUM32 = 0x1000;
    public const uint IT_RESULT = 0x1300;
    public const uint IT_STATUS = 0x1301;
    public const uint IT_MODULE_ID = 0x1600;

    // T_PDU_COPT
    public const uint COPT_STARTCOMM = 0x8001;
    public const uint COPT_STOPCOMM = 0x8002;
    public const uint COPT_UPDATEPARAM = 0x8003;
    public const uint COPT_SENDRECV = 0x8004;
    public const uint COPT_DELAY = 0x8005;
    public const uint COPT_RESTORE_PARAM = 0x8006;

    // T_PDU_OBJT
    public const uint OBJT_PROTOCOL = 0x8021;
    public const uint OBJT_BUSTYPE = 0x8022;
    public const uint OBJT_IO_CTRL = 0x8023;
    public const uint OBJT_COMPARAM = 0x8024;
    public const uint OBJT_PINTYPE = 0x8025;

    // T_PDU_STATUS
    public const uint COPST_IDLE = 0x8010;
    public const uint COPST_EXECUTING = 0x8011;
    public const uint COPST_FINISHED = 0x8012;
    public const uint COPST_CANCELLED = 0x8013;
    public const uint CLLST_OFFLINE = 0x8050;
    public const uint CLLST_ONLINE = 0x8051;
    public const uint CLLST_COMM_STARTED = 0x8052;
    public const uint MODST_READY = 0x8060;
    public const uint MODST_AVAIL = 0x8063;

    // T_PDU_EVT_DATA: the only event the driver reports to callbacks
    public const uint EVT_DATA_AVAILABLE = 0x0801;

    // T_PDU_PT: ComParam data types; only the scalar types are read as values
    public const uint PT_UNUM8 = 0x101, PT_SNUM8 = 0x102, PT_UNUM16 = 0x103, PT_SNUM16 = 0x104,
                      PT_UNUM32 = 0x105, PT_SNUM32 = 0x106;

    // T_PDU_FILTER
    public const uint FLT_PASS = 1, FLT_BLOCK = 2, FLT_PASS_UUDT = 0x11, FLT_BLOCK_UUDT = 0x12;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_FLAG_DATA { public uint NumFlagBytes; public byte* pFlagData; }

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_PARAM_ITEM
{
    public uint ItemType, ComParamId, ComParamDataType, ComParamClass;
    public void* pComParamData;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_VERSION_DATA
{
    public uint MVCI_Part1StandardVersion, MVCI_Part2StandardVersion, HwSerialNumber;
    public fixed byte HwName[64];
    public uint HwVersion, HwDate, HwInterface;
    public fixed byte FwName[64];
    public uint FwVersion, FwDate;
    public fixed byte VendorName[64];
    public fixed byte PDUApiSwName[64];
    public uint PDUApiSwVersion, PDUApiSwDate;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_MODULE_DATA
{
    public uint ModuleTypeId, hMod;
    public byte* pVendorModuleName;
    public byte* pVendorAdditionalInfo;
    public uint ModuleStatus;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_MODULE_ITEM { public uint ItemType, NumEntries; public PDU_MODULE_DATA* pModuleData; }

[StructLayout(LayoutKind.Sequential)]
public struct PDU_PIN_DATA { public uint DLCPinNumber, DLCPinTypeId; }

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_RSC_DATA { public uint BusTypeId, ProtocolId, NumPinData; public PDU_PIN_DATA* pDLCPinData; }

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_EXP_RESP_DATA
{
    public uint ResponseType, AcceptanceId, NumMaskPatternBytes;
    public byte* pMaskData;
    public byte* pPatternData;
    public uint NumUniqueRespIds;
    public uint* pUniqueRespIds;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_COP_CTRL_DATA
{
    public uint Time;
    public int NumSendCycles, NumReceiveCycles;
    public uint TempParamUpdate;
    public PDU_FLAG_DATA TxFlag;
    public uint NumPossibleExpectedResponses;
    public PDU_EXP_RESP_DATA* pExpectedResponseArray;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_EXTRA_INFO { public uint NumHeaderBytes, NumFooterBytes; public byte* pHeaderBytes; public byte* pFooterBytes; }

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_RESULT_DATA
{
    public PDU_FLAG_DATA RxFlag;
    public uint UniqueRespIdentifier, AcceptanceId;
    public PDU_FLAG_DATA TimestampFlags;
    public uint TxMsgDoneTimestamp, StartMsgTimestamp;
    public PDU_EXTRA_INFO* pExtraInfo;
    public uint NumDataBytes;
    public byte* pDataBytes;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_EVENT_ITEM { public uint ItemType, hCop; public void* pCoPTag; public uint Timestamp; public void* pData; }

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_DATA_ITEM { public uint ItemType; public void* pData; }

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_IO_FILTER_DATA
{
    public uint FilterType, FilterNumber, FilterCompareSize;
    public fixed byte FilterMaskMessage[12];
    public fixed byte FilterPatternMessage[12];
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_IO_FILTER_LIST { public uint NumFilterEntries; public PDU_IO_FILTER_DATA* pFilterData; }

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_ECU_UNIQUE_RESP_DATA { public uint UniqueRespIdentifier, NumParamItems; public PDU_PARAM_ITEM* pParams; }

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PDU_UNIQUE_RESP_ID_TABLE_ITEM { public uint ItemType, NumEntries; public PDU_ECU_UNIQUE_RESP_DATA* pUniqueData; }
