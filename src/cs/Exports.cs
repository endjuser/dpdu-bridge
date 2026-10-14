// Native __stdcall exports of the ISO 22900-2 D-PDU API (the 29 functions
// listed in src/c/dpdu_bridge.def, plus the PDUUnlockResource alias).
//
// Native AOT turns each [UnmanagedCallersOnly(EntryPoint = ...)] method into a
// real DLL export with the given name. The methods here only forward to
// Driver; the stubs that need no state are implemented inline, as in the C
// version. A managed exception that escapes an [UnmanagedCallersOnly] method
// terminates the host process, so every entry point runs inside Guard.
using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace DpduBridge;

public static unsafe class Exports
{
    /// <summary>
    /// Runs one API call and converts any exception into PDU_ERR_FCT_FAILED
    /// (logged), so nothing can unwind into the native caller.
    /// </summary>
    private static uint Guard(string name, Func<uint> body)
    {
        try
        {
            return body();
        }
        catch (Exception ex)
        {
            Log.Write($"{name} failed: {ex}");
            return Pdu.ERR_FCT_FAILED;
        }
    }

    [UnmanagedCallersOnly(EntryPoint = "PDUConstruct", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUConstruct(byte* optionStr, void* apiTag) =>
        Guard("PDUConstruct", () => Driver.Construct(optionStr, apiTag));

    [UnmanagedCallersOnly(EntryPoint = "PDUDestruct", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUDestruct() => Guard("PDUDestruct", Driver.Destruct);

    [UnmanagedCallersOnly(EntryPoint = "PDUModuleConnect", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUModuleConnect(uint hMod) => Guard("PDUModuleConnect", () => Driver.ModuleConnect(hMod));

    [UnmanagedCallersOnly(EntryPoint = "PDUModuleDisconnect", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUModuleDisconnect(uint hMod) => Guard("PDUModuleDisconnect", () => Driver.ModuleDisconnect(hMod));

    [UnmanagedCallersOnly(EntryPoint = "PDURegisterEventCallback", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDURegisterEventCallback(uint hMod, uint hCll, nint callback) =>
        Guard("PDURegisterEventCallback", () => Driver.RegisterEventCallback(hMod, hCll, callback));

    [UnmanagedCallersOnly(EntryPoint = "PDUIoCtl", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUIoCtl(uint hMod, uint hCll, uint id, PDU_DATA_ITEM* input, PDU_DATA_ITEM** output) =>
        Guard("PDUIoCtl", () => Driver.IoCtl(hMod, hCll, id, input, output));

    [UnmanagedCallersOnly(EntryPoint = "PDUGetVersion", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUGetVersion(uint hMod, PDU_VERSION_DATA* data) =>
        Guard("PDUGetVersion", () => Driver.GetVersion(hMod, data));

    [UnmanagedCallersOnly(EntryPoint = "PDUGetStatus", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUGetStatus(uint hMod, uint hCll, uint hCop, uint* status, uint* ts, uint* extra) =>
        Guard("PDUGetStatus", () => Driver.GetStatus(hMod, hCll, hCop, status, ts, extra));

    /// <summary>Stub: returns PDU_STATUS_NOERROR without touching the outputs.</summary>
    [UnmanagedCallersOnly(EntryPoint = "PDUGetLastError", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUGetLastError(uint hMod, uint hCll, uint* code, uint* hCop, uint* ts, uint* extra) =>
        Guard("PDUGetLastError", () => Stub($"PDUGetLastError Called: hMod={hMod:X8} hCLL={hCll:X8}"));

    [UnmanagedCallersOnly(EntryPoint = "PDUGetTimestamp", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUGetTimestamp(uint hMod, uint* ts) => Guard("PDUGetTimestamp", () => Driver.GetTimestamp(hMod, ts));

    [UnmanagedCallersOnly(EntryPoint = "PDUGetObjectId", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUGetObjectId(uint type, byte* shortName, uint* id) =>
        Guard("PDUGetObjectId", () => Driver.GetObjectId(type, shortName, id));

    // The resource functions are stubs: PDU_STATUS_NOERROR, outputs untouched.

    [UnmanagedCallersOnly(EntryPoint = "PDUGetResourceIds", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUGetResourceIds(uint hMod, void* rscData, void** list) =>
        Guard("PDUGetResourceIds", () => Stub($"PDUGetResourceIds Called: hMod={hMod:X8}"));

    [UnmanagedCallersOnly(EntryPoint = "PDUGetResourceStatus", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUGetResourceStatus(void* status) =>
        Guard("PDUGetResourceStatus", () => Stub("PDUGetResourceStatus Called"));

    [UnmanagedCallersOnly(EntryPoint = "PDUGetConflictingResources", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUGetConflictingResources(uint resourceId, void* inputList, void** outputList) =>
        Guard("PDUGetConflictingResources", () => Stub($"PDUGetConflictingResources Called: resourceId={resourceId}"));

    [UnmanagedCallersOnly(EntryPoint = "PDUGetModuleIds", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUGetModuleIds(PDU_MODULE_ITEM** list) => Guard("PDUGetModuleIds", () => Driver.GetModuleIds(list));

    [UnmanagedCallersOnly(EntryPoint = "PDULockResource", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDULockResource(uint hMod, uint hCll, uint mask) =>
        Guard("PDULockResource", () => Stub($"PDULockResource Called: hMod={hMod:X8} hCLL={hCll:X8} mask={mask:X}"));

    // Exported under both spellings, like the C version: PDUUnLockResource and
    // PDUUnlockResource (the ISO 22900-2 spelling). Both log the same line.

    [UnmanagedCallersOnly(EntryPoint = "PDUUnLockResource", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUUnLockResource(uint hMod, uint hCll, uint mask) =>
        Guard("PDUUnLockResource", () => Stub($"PDUUnLockResource Called: hMod={hMod:X8} hCLL={hCll:X8} mask={mask:X}"));

    [UnmanagedCallersOnly(EntryPoint = "PDUUnlockResource", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUUnlockResource(uint hMod, uint hCll, uint mask) =>
        Guard("PDUUnlockResource", () => Stub($"PDUUnLockResource Called: hMod={hMod:X8} hCLL={hCll:X8} mask={mask:X}"));

    [UnmanagedCallersOnly(EntryPoint = "PDUCreateComLogicalLink", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUCreateComLogicalLink(uint hMod, PDU_RSC_DATA* rsc, uint resourceId, void* cllTag,
                                               uint* phCll, PDU_FLAG_DATA* createFlag) =>
        Guard("PDUCreateComLogicalLink", () => Driver.CreateComLogicalLink(hMod, rsc, resourceId, cllTag, phCll, createFlag));

    [UnmanagedCallersOnly(EntryPoint = "PDUDestroyComLogicalLink", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUDestroyComLogicalLink(uint hMod, uint hCll) =>
        Guard("PDUDestroyComLogicalLink", () => Driver.DestroyComLogicalLink(hMod, hCll));

    [UnmanagedCallersOnly(EntryPoint = "PDUConnect", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUConnect(uint hMod, uint hCll) => Guard("PDUConnect", () => Driver.SetLinkState(hMod, hCll, true));

    [UnmanagedCallersOnly(EntryPoint = "PDUDisconnect", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUDisconnect(uint hMod, uint hCll) => Guard("PDUDisconnect", () => Driver.SetLinkState(hMod, hCll, false));

    /// <summary>Stub: returns PDU_STATUS_NOERROR without filling the parameter item.</summary>
    [UnmanagedCallersOnly(EntryPoint = "PDUGetComParam", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUGetComParam(uint hMod, uint hCll, uint paramId, PDU_PARAM_ITEM** item) =>
        Guard("PDUGetComParam", () => Stub($"PDUGetComParam Called: hMod={hMod:X8} hCLL={hCll:X8} ParamId={paramId}"));

    [UnmanagedCallersOnly(EntryPoint = "PDUSetComParam", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUSetComParam(uint hMod, uint hCll, PDU_PARAM_ITEM* item) =>
        Guard("PDUSetComParam", () => Driver.SetComParam(hMod, hCll, item));

    /// <summary>Stub: returns PDU_STATUS_NOERROR without filling the table.</summary>
    [UnmanagedCallersOnly(EntryPoint = "PDUGetUniqueRespIdTable", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUGetUniqueRespIdTable(uint hMod, uint hCll, PDU_UNIQUE_RESP_ID_TABLE_ITEM** table) =>
        Guard("PDUGetUniqueRespIdTable", () => Stub($"PDUGetUniqueRespIdTable Called: hMod={hMod:X8} hCLL={hCll:X8}"));

    [UnmanagedCallersOnly(EntryPoint = "PDUSetUniqueRespIdTable", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUSetUniqueRespIdTable(uint hMod, uint hCll, PDU_UNIQUE_RESP_ID_TABLE_ITEM* table) =>
        Guard("PDUSetUniqueRespIdTable", () => Driver.SetUniqueRespIdTable(hMod, hCll, table));

    [UnmanagedCallersOnly(EntryPoint = "PDUStartComPrimitive", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUStartComPrimitive(uint hMod, uint hCll, uint type, uint size, byte* data,
                                            PDU_COP_CTRL_DATA* ctrl, void* tag, uint* phCop) =>
        Guard("PDUStartComPrimitive", () => Driver.StartComPrimitive(hMod, hCll, type, size, data, ctrl, tag, phCop));

    [UnmanagedCallersOnly(EntryPoint = "PDUCancelComPrimitive", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUCancelComPrimitive(uint hMod, uint hCll, uint hCop) =>
        Guard("PDUCancelComPrimitive", () => Driver.CancelComPrimitive(hMod, hCll, hCop));

    [UnmanagedCallersOnly(EntryPoint = "PDUGetEventItem", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUGetEventItem(uint hMod, uint hCll, PDU_EVENT_ITEM** item) =>
        Guard("PDUGetEventItem", () => Driver.GetEventItem(hMod, hCll, item));

    [UnmanagedCallersOnly(EntryPoint = "PDUDestroyItem", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PDUDestroyItem(void* item) => Guard("PDUDestroyItem", () => Driver.DestroyItem(item));

    /// <summary>Body of the stub functions: log the call, report success.</summary>
    private static uint Stub(string logLine)
    {
        Log.Write(logLine);
        return Pdu.STATUS_NOERROR;
    }
}
