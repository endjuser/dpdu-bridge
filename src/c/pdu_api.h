/*
 * pdu_api.h - ISO 22900-2 (MVCI D-PDU API) types, constants and the event
 * callback signature, used by the C driver (dpdu_bridge.c) and by
 * tests/pdu_test.c.
 *
 * A self-contained subset of the standard header: layouts and numeric values
 * follow ISO 22900-2 for a 32-bit build (4-byte pointers, natural alignment,
 * enums as 32-bit values, __stdcall API functions).  Object ids returned by
 * PDUGetObjectId are not part of this header; they are defined by the driver.
 */
#ifndef PDU_API_H
#define PDU_API_H

#include <stdint.h>

#define PDU_API_CALL __stdcall

typedef uint8_t  UNUM8;
typedef int8_t   SNUM8;
typedef uint16_t UNUM16;
typedef int16_t  SNUM16;
typedef uint32_t UNUM32;
typedef int32_t  SNUM32;
typedef char     CHAR8;

#define PDU_ID_UNDEF      0xFFFFFFFEu
#define PDU_HANDLE_UNDEF  0xFFFFFFFFu

/* ---- Error codes (T_PDU_ERROR) ---- */
typedef enum {
    PDU_STATUS_NOERROR             = 0x00000000,
    PDU_ERR_FCT_FAILED             = 0x00000001,
    PDU_ERR_RESERVED_1             = 0x00000010,
    PDU_ERR_COMM_PC_TO_VCI_FAILED  = 0x00000011,
    PDU_ERR_PDUAPI_NOT_CONSTRUCTED = 0x00000020,
    PDU_ERR_SHARING_VIOLATION      = 0x00000021,
    PDU_ERR_RESOURCE_BUSY          = 0x00000030,
    PDU_ERR_RESOURCE_TABLE_CHANGED = 0x00000031,
    PDU_ERR_RESOURCE_ERROR         = 0x00000032,
    PDU_ERR_CLL_NOT_CONNECTED      = 0x00000040,
    PDU_ERR_CLL_NOT_STARTED        = 0x00000041,
    PDU_ERR_INVALID_PARAMETERS     = 0x00000050,
    PDU_ERR_INVALID_HANDLE         = 0x00000060,
    PDU_ERR_VALUE_NOT_SUPPORTED    = 0x00000061,
    PDU_ERR_ID_NOT_SUPPORTED       = 0x00000062,
    PDU_ERR_COMPARAM_NOT_SUPPORTED = 0x00000063,
    PDU_ERR_COMPARAM_LOCKED        = 0x00000064,
    PDU_ERR_TX_QUEUE_FULL          = 0x00000070,
    PDU_ERR_EVENT_QUEUE_EMPTY      = 0x00000071,
    PDU_ERR_VOLTAGE_NOT_SUPPORTED  = 0x00000080,
    PDU_ERR_MUX_RSC_NOT_SUPPORTED  = 0x00000081,
    PDU_ERR_CABLE_UNKNOWN          = 0x00000082,
    PDU_ERR_NO_CABLE_DETECTED      = 0x00000083,
    PDU_ERR_CLL_CONNECTED          = 0x00000084,
    PDU_ERR_TEMPPARAM_NOT_ALLOWED  = 0x00000090,
    PDU_ERR_RSC_LOCKED             = 0x000000A0,
    PDU_ERR_RSC_LOCKED_BY_OTHER_CLL= 0x000000A1,
    PDU_ERR_RSC_NOT_LOCKED         = 0x000000A2,
    PDU_ERR_MODULE_NOT_CONNECTED   = 0x000000A3,
    PDU_ERR_API_SW_OUT_OF_DATE     = 0x000000A4,
    PDU_ERR_MODULE_FW_OUT_OF_DATE  = 0x000000A5,
    PDU_ERR_PIN_NOT_CONNECTED      = 0x000000A6
} T_PDU_ERROR;

/* ---- Item types (T_PDU_IT) ---- */
typedef enum {
    PDU_IT_IO_UNUM32              = 0x1000,
    PDU_IT_IO_PROG_VOLTAGE        = 0x1001,
    PDU_IT_IO_BYTEARRAY           = 0x1002,
    PDU_IT_IO_FILTER              = 0x1003,
    PDU_IT_IO_EVENT_QUEUE_PROPERTY= 0x1004,
    PDU_IT_RSC_STATUS             = 0x1100,
    PDU_IT_PARAM                  = 0x1200,
    PDU_IT_RESULT                 = 0x1300,
    PDU_IT_STATUS                 = 0x1301,
    PDU_IT_ERROR                  = 0x1302,
    PDU_IT_INFO                   = 0x1303,
    PDU_IT_RSC_ID                 = 0x1400,
    PDU_IT_RSC_CONFLICT           = 0x1500,
    PDU_IT_MODULE_ID              = 0x1600,
    PDU_IT_UNIQUE_RESP_ID_TABLE   = 0x1700
} T_PDU_IT;

/* ---- ComPrimitive types (T_PDU_COPT) ---- */
typedef enum {
    PDU_COPT_STARTCOMM     = 0x8001,
    PDU_COPT_STOPCOMM      = 0x8002,
    PDU_COPT_UPDATEPARAM   = 0x8003,
    PDU_COPT_SENDRECV      = 0x8004,
    PDU_COPT_DELAY         = 0x8005,
    PDU_COPT_RESTORE_PARAM = 0x8006
} T_PDU_COPT;

/* ---- Object types (T_PDU_OBJT) ---- */
typedef enum {
    PDU_OBJT_PROTOCOL = 0x8021,
    PDU_OBJT_BUSTYPE  = 0x8022,
    PDU_OBJT_IO_CTRL  = 0x8023,
    PDU_OBJT_COMPARAM = 0x8024,
    PDU_OBJT_PINTYPE  = 0x8025,
    PDU_OBJT_RESOURCE = 0x8026
} T_PDU_OBJT;

/* ---- Status codes (T_PDU_STATUS) ---- */
typedef enum {
    PDU_COPST_IDLE         = 0x8010,
    PDU_COPST_EXECUTING    = 0x8011,
    PDU_COPST_FINISHED     = 0x8012,
    PDU_COPST_CANCELLED    = 0x8013,
    PDU_COPST_WAITING      = 0x8014,
    PDU_CLLST_OFFLINE      = 0x8050,
    PDU_CLLST_ONLINE       = 0x8051,
    PDU_CLLST_COMM_STARTED = 0x8052,
    PDU_MODST_READY        = 0x8060,
    PDU_MODST_NOT_READY    = 0x8061,
    PDU_MODST_NOT_AVAIL    = 0x8062,
    PDU_MODST_AVAIL        = 0x8063
} T_PDU_STATUS;

/* ---- Info codes (T_PDU_INFO) ---- */
typedef enum {
    PDU_INFO_MODULE_LIST_CHG   = 0x8070,
    PDU_INFO_RSC_LOCK_CHG      = 0x8071,
    PDU_INFO_PHYS_COMPARAM_CHG = 0x8072
} T_PDU_INFO;

/* ---- Event callback data codes ---- */
typedef enum {
    PDU_EVT_DATA_AVAILABLE = 0x0801,
    PDU_EVT_DATA_LOST      = 0x0802
} T_PDU_EVT_DATA;

/* ---- ComParam data types / classes ---- */
typedef enum {
    PDU_PT_UNUM8       = 0x00000101,
    PDU_PT_SNUM8       = 0x00000102,
    PDU_PT_UNUM16      = 0x00000103,
    PDU_PT_SNUM16      = 0x00000104,
    PDU_PT_UNUM32      = 0x00000105,
    PDU_PT_SNUM32      = 0x00000106,
    PDU_PT_BYTEFIELD   = 0x00000107,
    PDU_PT_STRUCTFIELD = 0x00000108,
    PDU_PT_LONGFIELD   = 0x00000109
} T_PDU_PT;

typedef enum {
    PDU_PC_TIMING = 1, PDU_PC_INIT, PDU_PC_COM, PDU_PC_ERRHDL,
    PDU_PC_BUSTYPE, PDU_PC_UNIQUE_ID, PDU_PC_TESTER_PRESENT
} T_PDU_PC;

/* ---- Filter types ---- */
typedef enum {
    PDU_FLT_PASS = 1, PDU_FLT_BLOCK = 2,
    PDU_FLT_PASS_UUDT = 0x11, PDU_FLT_BLOCK_UUDT = 0x12
} T_PDU_FILTER;

/* ---- Lock masks ---- */
#define PDU_LOCK_ACCESS_PHYS_COMPARAM 0x00000001u
#define PDU_LOCK_ACCESS_TRANSMIT      0x00000002u

/* ---- Structures (natural alignment, no packing pragmas) ---- */
typedef struct { UNUM32 NumFlagBytes; UNUM8 *pFlagData; } PDU_FLAG_DATA;

typedef struct {
    T_PDU_IT ItemType;
    UNUM32   ComParamId;
    T_PDU_PT ComParamDataType;
    T_PDU_PC ComParamClass;
    void    *pComParamData;
} PDU_PARAM_ITEM;

typedef struct { UNUM32 ParamMaxEntries; UNUM32 ParamActEntries; UNUM8  *pDataArray; } PDU_PARAM_BYTEFIELD_DATA;
typedef struct { UNUM32 ParamMaxEntries; UNUM32 ParamActEntries; UNUM32 *pDataArray; } PDU_PARAM_LONGFIELD_DATA;
typedef struct { UNUM32 ComParamStructType; UNUM32 ParamMaxEntries; UNUM32 ParamActEntries; void *pStructArray; } PDU_PARAM_STRUCTFIELD_DATA;

typedef struct {
    UNUM32 MVCI_Part1StandardVersion;
    UNUM32 MVCI_Part2StandardVersion;
    UNUM32 HwSerialNumber;
    CHAR8  HwName[64];
    UNUM32 HwVersion;
    UNUM32 HwDate;
    UNUM32 HwInterface;
    CHAR8  FwName[64];
    UNUM32 FwVersion;
    UNUM32 FwDate;
    CHAR8  VendorName[64];
    CHAR8  PDUApiSwName[64];
    UNUM32 PDUApiSwVersion;
    UNUM32 PDUApiSwDate;
} PDU_VERSION_DATA;

typedef struct {
    UNUM32       ModuleTypeId;
    UNUM32       hMod;
    CHAR8       *pVendorModuleName;
    CHAR8       *pVendorAdditionalInfo;
    T_PDU_STATUS ModuleStatus;
} PDU_MODULE_DATA;

typedef struct {
    T_PDU_IT         ItemType;
    UNUM32           NumEntries;
    PDU_MODULE_DATA *pModuleData;
} PDU_MODULE_ITEM;

typedef struct { UNUM32 DLCPinNumber; UNUM32 DLCPinTypeId; } PDU_PIN_DATA;

typedef struct {
    UNUM32        BusTypeId;
    UNUM32        ProtocolId;
    UNUM32        NumPinData;
    PDU_PIN_DATA *pDLCPinData;
} PDU_RSC_DATA;

typedef struct { UNUM32 hMod; UNUM32 NumIds; UNUM32 *pResourceIdArray; } PDU_RSC_ID_ITEM_DATA;
typedef struct { T_PDU_IT ItemType; UNUM32 NumModules; PDU_RSC_ID_ITEM_DATA *pResourceIdDataArray; } PDU_RSC_ID_ITEM;

typedef struct { UNUM32 hMod; UNUM32 ResourceId; UNUM32 ResourceStatus; } PDU_RSC_STATUS_DATA;
typedef struct { T_PDU_IT ItemType; UNUM32 NumEntries; PDU_RSC_STATUS_DATA *pResourceStatusData; } PDU_RSC_STATUS_ITEM;

typedef struct { UNUM32 hMod; UNUM32 ResourceId; } PDU_RSC_CONFLICT_DATA;
typedef struct { T_PDU_IT ItemType; UNUM32 NumEntries; PDU_RSC_CONFLICT_DATA *pRscConflictData; } PDU_RSC_CONFLICT_ITEM;

typedef struct {
    UNUM32 ResponseType;
    UNUM32 AcceptanceId;
    UNUM32 NumMaskPatternBytes;
    UNUM8 *pMaskData;
    UNUM8 *pPatternData;
    UNUM32 NumUniqueRespIds;
    UNUM32 *pUniqueRespIds;
} PDU_EXP_RESP_DATA;

typedef struct {
    UNUM32             Time;
    SNUM32             NumSendCycles;
    SNUM32             NumReceiveCycles;
    UNUM32             TempParamUpdate;
    PDU_FLAG_DATA      TxFlag;
    UNUM32             NumPossibleExpectedResponses;
    PDU_EXP_RESP_DATA *pExpectedResponseArray;
} PDU_COP_CTRL_DATA;

typedef struct {
    UNUM32 NumHeaderBytes;
    UNUM32 NumFooterBytes;
    UNUM8 *pHeaderBytes;
    UNUM8 *pFooterBytes;
} PDU_EXTRA_INFO;

typedef struct {
    PDU_FLAG_DATA   RxFlag;
    UNUM32          UniqueRespIdentifier;
    UNUM32          AcceptanceId;
    PDU_FLAG_DATA   TimestampFlags;
    UNUM32          TxMsgDoneTimestamp;
    UNUM32          StartMsgTimestamp;
    PDU_EXTRA_INFO *pExtraInfo;
    UNUM32          NumDataBytes;
    UNUM8          *pDataBytes;
} PDU_RESULT_DATA;

typedef struct { UNUM32 ErrorCodeId; UNUM32 ExtraErrorInfoId; } PDU_ERROR_DATA;
typedef struct { UNUM32 InfoCode; UNUM32 ExtraInfoData; } PDU_INFO_DATA;

typedef struct {
    T_PDU_IT ItemType;
    UNUM32   hCop;
    void    *pCoPTag;
    UNUM32   Timestamp;
    void    *pData;
} PDU_EVENT_ITEM;

typedef struct { T_PDU_IT ItemType; } PDU_ITEM;

typedef struct { T_PDU_IT ItemType; void *pData; } PDU_DATA_ITEM;

typedef struct { UNUM32 DataSize; UNUM8 *pData; } PDU_IO_BYTEARRAY_DATA;

typedef struct {
    T_PDU_FILTER FilterType;
    UNUM32       FilterNumber;
    UNUM32       FilterCompareSize;
    UNUM8        FilterMaskMessage[12];
    UNUM8        FilterPatternMessage[12];
} PDU_IO_FILTER_DATA;

typedef struct { UNUM32 NumFilterEntries; PDU_IO_FILTER_DATA *pFilterData; } PDU_IO_FILTER_LIST;

typedef struct {
    UNUM32          UniqueRespIdentifier;
    UNUM32          NumParamItems;
    PDU_PARAM_ITEM *pParams;
} PDU_ECU_UNIQUE_RESP_DATA;

typedef struct {
    T_PDU_IT                  ItemType;
    UNUM32                    NumEntries;
    PDU_ECU_UNIQUE_RESP_DATA *pUniqueData;
} PDU_UNIQUE_RESP_ID_TABLE_ITEM;

typedef void (PDU_API_CALL *CALLBACKFNC)(T_PDU_EVT_DATA eventType, UNUM32 hMod,
                                          UNUM32 hCLL, void *pCllTag, void *pAPITag);

#endif /* PDU_API_H */
