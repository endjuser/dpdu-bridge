// Native __stdcall exports of the SAE J2534-1 (04.04) PassThru API, next to
// the D-PDU API exports in Exports.cs. The methods only forward to J2534; as
// there, every entry point runs inside Guard so no managed exception can
// unwind into the native caller.
using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace DpduBridge;

public static unsafe class J2534Exports
{
    /// <summary>Runs one API call and converts any exception into ERR_FAILED (logged).</summary>
    private static uint Guard(string name, Func<uint> body)
    {
        try
        {
            return body();
        }
        catch (Exception ex)
        {
            Log.Write($"{name} failed: {ex}");
            return J.ERR_FAILED;
        }
    }

    [UnmanagedCallersOnly(EntryPoint = "PassThruOpen", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruOpen(void* name, uint* pDeviceId) =>
        Guard("PassThruOpen", () => J2534.Open(name, pDeviceId));

    [UnmanagedCallersOnly(EntryPoint = "PassThruClose", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruClose(uint deviceId) => Guard("PassThruClose", () => J2534.Close(deviceId));

    [UnmanagedCallersOnly(EntryPoint = "PassThruConnect", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruConnect(uint deviceId, uint protocolId, uint flags, uint baudRate, uint* pChannelId) =>
        Guard("PassThruConnect", () => J2534.Connect(deviceId, protocolId, flags, baudRate, pChannelId));

    [UnmanagedCallersOnly(EntryPoint = "PassThruDisconnect", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruDisconnect(uint channelId) => Guard("PassThruDisconnect", () => J2534.Disconnect(channelId));

    [UnmanagedCallersOnly(EntryPoint = "PassThruReadMsgs", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruReadMsgs(uint channelId, PASSTHRU_MSG* msgs, uint* pNumMsgs, uint timeout) =>
        Guard("PassThruReadMsgs", () => J2534.ReadMsgs(channelId, msgs, pNumMsgs, timeout));

    [UnmanagedCallersOnly(EntryPoint = "PassThruWriteMsgs", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruWriteMsgs(uint channelId, PASSTHRU_MSG* msgs, uint* pNumMsgs, uint timeout) =>
        Guard("PassThruWriteMsgs", () => J2534.WriteMsgs(channelId, msgs, pNumMsgs, timeout));

    [UnmanagedCallersOnly(EntryPoint = "PassThruStartPeriodicMsg", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruStartPeriodicMsg(uint channelId, PASSTHRU_MSG* msg, uint* pMsgId, uint interval) =>
        Guard("PassThruStartPeriodicMsg", () => J2534.StartPeriodicMsg(channelId, msg, pMsgId, interval));

    [UnmanagedCallersOnly(EntryPoint = "PassThruStopPeriodicMsg", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruStopPeriodicMsg(uint channelId, uint msgId) =>
        Guard("PassThruStopPeriodicMsg", () => J2534.StopPeriodicMsg(channelId, msgId));

    [UnmanagedCallersOnly(EntryPoint = "PassThruStartMsgFilter", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruStartMsgFilter(uint channelId, uint filterType, PASSTHRU_MSG* mask, PASSTHRU_MSG* pattern,
                                              PASSTHRU_MSG* flowControl, uint* pFilterId) =>
        Guard("PassThruStartMsgFilter", () => J2534.StartMsgFilter(channelId, filterType, mask, pattern, flowControl, pFilterId));

    [UnmanagedCallersOnly(EntryPoint = "PassThruStopMsgFilter", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruStopMsgFilter(uint channelId, uint filterId) =>
        Guard("PassThruStopMsgFilter", () => J2534.StopMsgFilter(channelId, filterId));

    [UnmanagedCallersOnly(EntryPoint = "PassThruSetProgrammingVoltage", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruSetProgrammingVoltage(uint deviceId, uint pinNumber, uint voltage) =>
        Guard("PassThruSetProgrammingVoltage", () => J2534.SetProgrammingVoltage(deviceId, pinNumber, voltage));

    [UnmanagedCallersOnly(EntryPoint = "PassThruReadVersion", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruReadVersion(uint deviceId, byte* firmware, byte* dll, byte* api) =>
        Guard("PassThruReadVersion", () => J2534.ReadVersion(deviceId, firmware, dll, api));

    [UnmanagedCallersOnly(EntryPoint = "PassThruGetLastError", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruGetLastError(byte* description) =>
        Guard("PassThruGetLastError", () => J2534.GetLastError(description));

    [UnmanagedCallersOnly(EntryPoint = "PassThruIoctl", CallConvs = new[] { typeof(CallConvStdcall) })]
    public static uint PassThruIoctl(uint channelId, uint ioctlId, void* input, void* output) =>
        Guard("PassThruIoctl", () => J2534.Ioctl(channelId, ioctlId, input, output));
}
