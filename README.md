# dpdu-bridge — D-PDU API + J2534 PassThru Serial/TCP bridge with J1850 VPW ECU simulator

A D-PDU API (ISO 22900-2) driver for Tech2Win that talks to ECU simulators instead of a
vehicle interface. Every message Tech2Win transmits is sent as a text line,
and lines received back are delivered to Tech2Win as responses. Clients
connect in two ways, which can be used separately or at the same time:

- **Serial** (`[Serial] Port=COM8`): with a [com0com](https://com0com.sourceforge.net/)
  virtual pair, e.g. if COM8 is paired with CNCA8, a simulator uses CNCA8.
- **TCP** (`[Tcp] Port=5000`): the DLL listens on that TCP port, and several
  clients (simulators, monitors, loggers) can connect at once.

```
Tech2Win ──D-PDU API──> dpdu-bridge-c.dll ──text lines──> COM8 ⇄ (com0com) ⇄ CNCA8 ──> ECU simulator
                        (or -cs)         └─text lines──> TCP :5000 <── client 1, client 2, ...
```

There are two implementations with identical behaviour and wire protocol:
one in C (`src/c/`), built as `dpdu-bridge-c.dll`, and one in C# (`src/cs/`),
built as `dpdu-bridge-cs.dll`. Both are native 32-bit DLLs exporting the 29
`__stdcall` D-PDU functions (the unlock function under both spellings,
`PDUUnlockResource` and `PDUUnLockResource`), and they can be installed side
by side.

Both DLLs are also **J2534 (PassThru 04.04) devices**: they export the 14
`PassThru*` functions as well, so Tech2Win (or another J2534 host) can use
the same bridge, simulator and line protocol through J2534 instead of the
D-PDU API. See [J2534](#j2534).

## Requirements

- Windows with a 32-bit D-PDU API host such as Tech2Win.
- For a serial link: a virtual COM port pair such as com0com.
- For the simulator: Python 3 (and `pip install pyserial` for serial links).
- To build: see [Building](#building).

## Installation

1. Download `dpdu-bridge-c.dll` and/or `dpdu-bridge-cs.dll`, plus
   `dpdu-bridge.ini` and `pdu_api_root.example.xml`, from the
   [latest release](../../releases/latest), or build them yourself (see
   [Building](#building)).
2. Put the DLL(s) and `dpdu-bridge.ini` together in one folder, e.g.
   `C:\dpdu-bridge\`. Logging is off by default; if you turn it on
   (`[Log] Enabled=1`), each DLL writes its log file next to itself, so pick
   a folder you can write to (not under `C:\Program Files`) or set
   `[Log] File`.
3. Set `[Serial] Port` and/or `[Tcp] Port` in `dpdu-bridge.ini` (see
   [Configuration](#configuration)).
4. Register the driver(s) with the D-PDU API. Make a backup copy of
   `C:\Program Files (x86)\D-PDU API\pdu_api_root.xml`, then open it in a text
   editor run as administrator. From `pdu_api_root.example.xml`, copy the
   `<MVCI_PDU_API>` … `</MVCI_PDU_API>` section of each version you want to
   use (the first is the C version, the second the C# version) and paste it
   inside the existing `<MVCI_PDU_API_ROOT>` element, next to the
   `<MVCI_PDU_API>` sections already there. Don't change or remove the
   existing sections.
5. In each section you pasted, change the `LIBRARY_FILE` URI to where you put
   the DLL, written as a `file:///` URL with forward slashes. Each
   `SHORT_NAME` must be unique in the file. For example, with the C version
   in `C:\dpdu-bridge\`:

   ```xml
   <MVCI_PDU_API_ROOT ...>
     <!-- ... existing <MVCI_PDU_API> sections ... -->
     <MVCI_PDU_API>
       <SHORT_NAME>D-PDU Bridge C</SHORT_NAME>
       <DESCRIPTION>D-PDU Serial/TCP Bridge (C)</DESCRIPTION>
       <SUPPLIER_NAME>dpdu-bridge</SUPPLIER_NAME>
       <LIBRARY_FILE URI="file:///C:/dpdu-bridge/dpdu-bridge-c.dll"/>
       <MODULE_DESCRIPTION_FILE URI=""/>
       <CABLE_DESCRIPTION_FILE URI=""/>
     </MVCI_PDU_API>
   </MVCI_PDU_API_ROOT>
   ```

6. Restart Tech2Win and select `D-PDU Bridge C` or `D-PDU Bridge C#` as the
   D-PDU API.

## Files

| Path | Purpose |
|---|---|
| `build.cmd` | builds either version (or the test) and copies its DLL to `build/` |
| `build/` | build output: `dpdu-bridge-c.dll`, `dpdu-bridge-cs.dll` and `dpdu-bridge.ini` (not in git) |
| `dpdu-bridge.sln` | Visual Studio solution with both versions (see [Visual Studio](#visual-studio)) |
| `src/c/` | C version: `dpdu_bridge.c`, `pdu_api.h`, `dpdu_bridge.def` (`dpdu-bridge-c.vcxproj` for Visual Studio) |
| `src/cs/` | C# version (.NET Native AOT, `DpduBridge.csproj`) |
| `run_sim.bat` | runs the simulator on the port from the driver's ini (`run_sim.bat [-c\|-cs] [serial\|tcp]`) |
| `sim/ecu_sim.py` | ECU simulator (serial: `pip install pyserial`; TCP: no extra packages) |
| `sim/vpw.py` | J1850 VPW module simulation (J2178 / J2190 / J1979) used by `ecu_sim.py` |
| `sim/ecu.json` | the vehicle's VPW modules: address, supported commands, PIDs, DPIDs, blocks |
| `tests/pdu_test.c` | Tech2Win-like end-to-end test |
| `tests/j2534_test.c` | end-to-end test of the DLLs' J2534 side |
| `j2534_register.example.reg` | J2534 registration of both DLLs (see [J2534](#j2534)) |
| `src/c/j2534.c`, `src/cs/J2534*.cs` | the DLLs' J2534 front end |
| `tests/test_vpw.py` | unit tests for the VPW simulation against `sim/ecu.json` (`python tests\test_vpw.py`) |
| `pdu_api_root.example.xml` | sections to add to the D-PDU API's `pdu_api_root.xml` (see [Installation](#installation)) |
| `src/dpdu-bridge.ini` | default settings, copied to `build/` (see [Configuration](#configuration)) |
| `.github/workflows/release.yml` | builds both versions on GitHub and publishes releases (see [Releases](#releases)) |

## Building

```
build.cmd c        C version, build\dpdu-bridge-c.dll   (needs the Visual Studio C++ tools)
build.cmd cs       C# version, build\dpdu-bridge-cs.dll (needs the .NET 9 SDK and the Visual Studio C++ tools)
build.cmd test     tests\pdu_test.exe and tests\j2534_test.exe (needs the Visual Studio C++ tools)
build.cmd clean    deletes obj\, src\cs\bin\, src\cs\obj\, src\cs\publish\ and the test programs
```

The parameter is required. The C++ tools are taken from the VS 2022 Build
Tools if installed, else from the newest Visual Studio (any edition) that has
them.

`c` copies `dpdu-bridge-c.dll`, and `cs` copies `dpdu-bridge-cs.dll`, to
**`build\`**, replacing that version's previous DLL. Nothing is deleted, so
the other version's DLL stays and both can be used side by side. The other
build output (`.lib`, `.exp`, `.pdb`) stays in the output folder, `obj\c\`
(C) or `src\cs\publish\` (C#). If the copy fails, close any application (e.g.
Tech2Win) that still has the DLL loaded.

`src\dpdu-bridge.ini` is copied to the output folder and to `build\`, but only
where there is no `dpdu-bridge.ini` yet: edit the one in `build\` for your
setup, and the build never overwrites it. Log files are never touched. Git
keeps the `build\` folder but ignores everything in it.

`clean` keeps `build\`, and never deletes the directory you run it from.

The C build is about 150 KB and depends only on Windows system DLLs. The C#
build is about 1.2 MB (Native AOT, no .NET runtime needed on the target). Its
export ordinals differ, but hosts look the functions up by name.

### Visual Studio

`dpdu-bridge.sln` holds both versions: `dpdu-bridge-cs` (the default project)
and `dpdu-bridge-c`. It needs Visual Studio 2022 with the ".NET desktop
development" and "Desktop development with C++" workloads (the C# version
also needs the .NET 9 SDK). Building in Visual Studio gives the same DLLs as
`build.cmd` and copies the files the same way: `dpdu-bridge-c.dll` (output
folder `obj\vs\<Configuration>\`) and `dpdu-bridge-cs.dll` (output folder
`src\cs\publish\`; the C# project publishes with Native AOT after each
build, which a plain build doesn't do) go to `build\`, and
`src\dpdu-bridge.ini` to the output folder and `build\` where there is none
yet. Debug and Release configurations are available.

## Releases

`.github/workflows/release.yml` builds both versions on GitHub's Windows
runners for every push to `main` and every pull request; the run's
`dpdu-bridge` artifact holds `dpdu-bridge-c.dll`, `dpdu-bridge-cs.dll`,
`dpdu-bridge.ini`, `pdu_api_root.example.xml` and
`j2534_register.example.reg`. Pushing a version tag also publishes a GitHub
release with those five files and generated release notes:

```
git tag v1.0.0
git push origin v1.0.0
```

## Configuration

Each version first looks for an ini named after its DLL in the directory the
DLL is in (`dpdu-bridge-c.ini` / `dpdu-bridge-cs.ini`). If that doesn't
exist, it reads the shared `dpdu-bridge.ini` there, so usually one
`dpdu-bridge.ini` serves both. Each writes its own log, named after the DLL
(`dpdu-bridge-c.log` / `dpdu-bridge-cs.log`). If both versions use the same
ports, load only one of them at a time. Both read the ini with the same
Windows functions (`GetPrivateProfileString` / `GetPrivateProfileInt`), so
the keys, quoting and number parsing are identical. A missing or empty value
uses the default.

| Section | Key | Default | Meaning |
|---|---|---|---|
| `[Serial]` | `Port` | none | COM port name, e.g. `COM8`; empty disables the serial link |
| `[Serial]` | `Baud` | `115200` | baud rate (ignored by com0com) |
| `[Tcp]` | `Port` | none | TCP port number (1–65535) to listen on, e.g. `5000`; empty disables the TCP server |
| `[Tcp]` | `Bind` | `127.0.0.1` | IPv4 or IPv6 address the TCP server listens on: `127.0.0.1` / `::1` local only, `0.0.0.0` all IPv4 interfaces, `::` all interfaces, IPv4 and IPv6 |
| `[Log]` | `Enabled` | `0` | `1` turns logging on |
| `[Log]` | `File` | `dpdu-bridge-c.log` / `dpdu-bridge-cs.log` next to the DLL | log file path (shared by both versions if set) |
| `[Device]` | `ModuleName` | `D-PDU Serial/TCP Bridge` | module/hardware name reported to the host |
| `[Device]` | `VBattMilliVolts` | `13000` | `PDU_IOCTL_READ_VBATT` value |
| `[Device]` | `SoftwareFilters` | `1` | apply `START_MSG_FILTER` filters to received VPW/ALDL frames |

At least one of `[Serial] Port` and `[Tcp] Port` is needed, and with both set
they work at the same time. An invalid value is logged and treated as not
set. Invalid values are a number in `[Serial] Port`, or a `[Tcp] Port` that
isn't 1–65535. If neither port is usable (or the INI file doesn't exist),
`PDUConstruct` shows an error message box naming the INI file and any invalid
value. No port is opened, no module is listed, and `PDUModuleConnect` fails.

When both are configured, `PDUModuleConnect` succeeds if at least one of them
opens. For example, if COM8 is missing, the TCP server still runs; the log
says which link failed.

## Testing with the simulator

1. `build.cmd c` and/or `build.cmd cs`, and set `[Serial] Port` and/or
   `[Tcp] Port` in `build\dpdu-bridge.ini` (or in `build\dpdu-bridge-c.ini` /
   `build\dpdu-bridge-cs.ini` for one version only).
2. Start the simulator with `run_sim.bat` (`run_sim.bat -c` for the C
   driver, `run_sim.bat -cs` for the C# driver). Without `-c` / `-cs` it
   uses the C driver if `dpdu-bridge-c.dll` is built, else the C# driver,
   and stops with an error if neither is built. It looks for the DLL in the
   current folder, else `build\`, reads the same ini the DLL
   reads (`dpdu-bridge-c.ini` / `dpdu-bridge-cs.ini` next to it if present,
   else `dpdu-bridge.ini`) and connects to the DLL:
   - `[Serial] Port=COM8` → the simulator uses the other end of the com0com
     pair, looked up in the registry (here `CNCA8`).
   - `[Tcp] Port=5000` → the simulator connects to that port. A `Bind` of
     `0.0.0.0` / `::` means 127.0.0.1 / ::1. It waits until the DLL is
     listening, and reconnects after each Tech2Win session.
   - With both configured, serial is used. Connecting over both would answer
     every request twice, because the DLL sends each request to all clients.
     `run_sim.bat tcp` or `run_sim.bat serial` picks one explicitly (e.g.
     `run_sim.bat -cs tcp`).
   - `run_sim.bat tcp 6000` / `run_sim.bat tcp 10.0.0.5:5000` ignore the INI.
     Any other arguments go straight to `sim\ecu_sim.py` (`run_sim.bat --help`),
     e.g. `run_sim.bat --port CNCA9 --ecus other.json`.
3. In Tech2Win, select the C or C# driver's D-PDU API, registered as in
   [Installation](#installation) steps 4–5. The `LIBRARY_FILE` paths in
   `pdu_api_root.example.xml` already point at `C:\projects\dpdu\build\`, so
   for a checkout there the sections can be pasted unchanged.
4. Watch traffic in the simulator console and, with `[Log] Enabled=1`, in
   the driver's log (`dpdu-bridge-c.log` / `dpdu-bridge-cs.log`). In the
   simulator window, ESC clears the screen and Ctrl+C stops it. While
   repeating `$2A` data packets are streaming, every other TX/RX line
   (including other clients' echoed traffic) is shown in green. Lines that
   define, request or carry DPIDs (`$2A`/`$6A`, `$2C`/`$6C`, and negative
   responses to them) keep the normal colour. Pressing SPACE while streaming
   hides or shows those DPID lines in the simulator console. The setting
   stays until the next SPACE, but only takes effect while something is
   streaming. The log, when enabled, always records everything.

Test without Tech2Win: `build.cmd test`, then, with the simulator running,
`tests\pdu_test.exe -c` (C driver) or `tests\pdu_test.exe -cs` (C# driver),
which loads the DLL from the current folder, else `build\`. Without `-c` /
`-cs` it uses `dpdu-bridge-c.dll` if it exists, else `dpdu-bridge-cs.dll`;
a DLL path also works. In TCP mode, add a wait so the simulator can connect:
`tests\pdu_test.exe -c 1500`. The J2534 side is tested the same way with
`tests\j2534_test.exe`, with the same arguments (e.g.
`tests\j2534_test.exe -cs 1500`). The simulator's unit tests run with
`python tests\test_vpw.py`.

## ECU simulator

`sim/ecu_sim.py` simulates every module in `sim/ecu.json` on the J1850 VPW
(GM Class 2) bus. It loads `ecu.json` from its own folder (`sim\`);
`--ecus FILE` picks another file. HS-CAN / SW-CAN and ALDL
requests get simple generic answers.

`ecu.json` holds the vehicle's VIN and a `nodes` array with one object per
module. Each object's `id` is the module's VPW address and must be unique.
(The comments below are explanation only; the file is plain JSON.)

```jsonc
{
  "VIN": "1GNFK16ZX4J123456",
  "nodes": [
    {
      "id": "10", "name": "PCM",
      "supportedCommands": ["10", "22", "27", "2A", "3C", "A0"],
      "data": {
        "pids":   { "0005": "3C" },            // $22 PID -> data
        "dpids":  { "F5": "2A9847CA4600" },    // $2A data packet -> 6 bytes
        "blocks": {                            // $3C / $3B block -> data
          "01": "00vAvBvCvDvE", "02": "vFvGvHvIvJvK", "03": "vLvMvNvOvPvQ"
        }
      }
    }
  ]
}
```

Data values can use the placeholders `vA` … `vQ` in place of a hex byte, for
VIN characters 1 … 17. The simulator replaces them with the bytes of the
top-level `"VIN"` when it loads the file. The PCM, BCM, IPC and RSE VIN blocks
use these placeholders. The RSE's block
`01` is `00FFvBvCvDvE`: like the real module, its first VIN character reads
as `FF`. RADIO and VCIM have blank VIN blocks. Generic OBD-II mode `$09`
reports the VIN from the PCM's blocks. Placeholders are case-insensitive, and
using one without a `"VIN"` is an error.

Node alive: every module broadcasts `08 FF <id> 03` every 2 seconds, so the
tester sees it as present on the bus. The 16 modules are spread evenly over
the 2 s, and their heartbeats aren't printed in the simulator window unless
you pass `--show-heartbeats`. Like all other simulator frames, they are sent
without the J1850 CRC byte.

Heartbeats are only sent while the DLL can deliver them: while it has a
logical link on VPW and VPW is its active bus. The simulator follows this
from the DLL's `OPEN` / `CLOSE` / `PROTOCOL` / `HELLO` / `BYE` lines, and
notes in its window when heartbeats start and stop. If the simulator starts
while Tech2Win already has a VPW link open, it hasn't seen the `OPEN`, so
heartbeats begin with Tech2Win's next link.

When Tech2Win disconnects (the DLL's `BYE` line, or the end of the TCP
connection to the DLL), every module returns to normal operation as after a
`$20` request: `$2A` streaming stops, `$2C` definitions are cleared and
security access is locked again. The simulator window notes this with
`Tech2Win disconnected: all modules returned to normal`. Blocks written with
`$3B` are kept until the simulator restarts.

Addressing (SAE J2178):

- A request is answered by the module it is addressed to (e.g. `6C 10 F0 …`
  → `6C F0 10 …`), or by every module for the all-nodes address `$FE`.
- The reply goes back to the requester: `F0` for Tech2, `F1` for a scan tool.

SAE J2190 modes — a module only handles the modes listed in its
`supportedCommands`:

| Mode | Response |
|---|---|
| `10` initiate diagnostics, `14` clear diagnostic info, `20` return to normal, `25` stop data, `26` data rates, `28`/`29` disable/enable normal messages | positive response (`20`/`25` also stop the module's `$2A` streaming; `20` returns the module to normal operation: it also clears its `$2C` definitions and locks security access again, for the addressed module or, with `$FE`, every module) |
| `22` data by PID | value from `pids`; unknown PID → `7F 22 … 31` |
| `2A` data packets | `2A <rate> <dpid>…` (`FF` = unused slot) → `6A <dpid> <6 bytes>` per DPID: rate `01` once, `00` stops; repeated every 1 s / 200 ms / 50 ms for `02`/`03`/`04`, every 50 ms for `12`/`13`/`14`, and every 200 ms for any other rate. Each request adds its DPIDs to what is already streaming (a DPID requested again takes the new rate); `2A 00 <dpid>…` stops only those DPIDs, `2A 00` stops all |
| `2C` define data packet | `2C <dpid> <def> <sources>`. The definition byte gives the source type in bits 7–6, the start position 1–6 in bits 5–3, and the number of bytes in bits 2–0. Type `01` is followed by up to two 2-byte PIDs (`FFFF` = unused slot), whose values fill the bytes in order. Examples: `4A 21A4 21B1` = two 1-byte PIDs at positions 1–2; `4D FD0D FFFF` = one 5-byte PID at position 1. Type `10` is followed by a 3-byte memory address, which reads as zeros (placeholder). A defined DPID is built from the PIDs' current values each time it is sent; undefined bytes keep the stored `dpids` value. Each definition replaces whatever covered the same bytes. A PID with no data in `ecu.json` is accepted and reads as zeros (placeholder; the simulator prints a note naming it). Bad layout → `12` |
| `3C` read block / `3B` write block | from / into `blocks` (writes are kept until the simulator restarts) |
| `27` security access | odd sub-function: random seed (`0000` once unlocked); even: any key is accepted. The module stays unlocked until `20` or until Tech2Win disconnects |
| `3F` tester present | no response |
| `12`, `17`, `19`, `23`, `34`, `35`, `36` | placeholders: no freeze frames, no DTCs, memory reads as zeros, programming accepted and discarded |
| GM-specific (`A0`, `A1`, `A8`, `AE`, `B0`, `B4`, …) | placeholder: acknowledged (mode + `$40`) |
| any mode not in `supportedCommands` | `7F <mode> … 11` (service not supported); ignored for `$FE` broadcasts |

SAE J1979 generic OBD-II (functional `68 6A F1 …` → `48 6B <module> …`):

- Only modules with PIDs `0001`–`00FF` in `ecu.json` answer (currently the
  PCM). Mode `$01` values come from those PIDs; the "PIDs supported" bitmaps
  are built from them.
- Unknown values are `00`, and DTCs (modes `$03`/`$07`) are none. Both are
  placeholders until DTC data is added.
- Mode `$09` returns the VIN from the module's blocks `01`–`03` (InfoTypes
  `01`/`02`). It returns the calibration ID from block `08` (InfoTypes
  `03`/`04`; e.g. `00C01298` → `12587672`, as 16 zero-padded ASCII characters
  in 4 messages). The CVN (InfoTypes `05`/`06`) is `00000000`, a placeholder
  because `ecu.json` has no CVN.

## Clients (serial and TCP)

The serial port and each TCP connection are all treated as clients:

- Every line the DLL sends goes to **all** clients: the serial port and
  every TCP client. Lines sent while no TCP client is connected only go to
  the serial port (if configured).
- A line from **any** client is delivered to Tech2Win. It is also echoed as a
  comment to all the **other** clients (never back to its sender), so every
  client sees the complete traffic, and other clients' lines only ever arrive
  as `#` lines:

  ```
  # COM8: RX HSCAN 7E8 62 F1 87 31 32 ...             (received on the serial port)
  # 127.0.0.1:52133: RX HSCAN 7E8 62 F1 87 31 32 ...  (received from an IPv4 TCP client)
  # [::1]:52134: RX HSCAN 7E8 62 F1 87 31 32 ...      (received from an IPv6 TCP client)
  ```

  The label is the serial port name, or the TCP client's remote address and
  port. IPv6 addresses are in brackets, and IPv4 clients of a dual-stack
  server are shown as plain IPv4.
- Clients also receive `# <address:port> connected` and
  `# <address:port> disconnected` notices.
- All of these are `#` comment lines, which the DLL and simulators ignore.
  This means a simulator never reacts to another client's echoed responses.
- The TCP server listens while Tech2Win has the module connected
  (`PDUModuleConnect` to `PDUModuleDisconnect`). Up to 32 TCP clients can be
  connected at once.
- A TCP client that stops reading is disconnected after a 250 ms send
  timeout, so it can't stall Tech2Win.

## Driver behaviour

- Supported buses (`PDUCreateComLogicalLink`): `ISO_11898_2_DWCAN` (HS-CAN, pins 6+14),
  `SAE_J2411_SWCAN` (GMLAN, pin 1), `SAE_J1850_VPW`, `SAE_J2740_UART` (ALDL).
  Anything else returns `PDU_ERR_INVALID_PARAMETERS`.
- Single module, hMod 1. It is listed when the TCP server is configured, or
  when the configured COM port exists. The module's additional info names the
  links, e.g. `COM8 + TCP 127.0.0.1:5000`.
- Handles: CLL `0x02000002…`, RX CoP `0x02100002…`, TX CoP `0x02200004…`,
  STARTCOMM CoP `0x02500001…`, UPDATEPARAM `0x02100007`.
- One event queue is shared by the module and all CLLs.
- A `SENDRECV` with **no data** creates the receive ComPrimitive. Its expected
  responses (mask/pattern on the 4-byte CAN ID) and the unique response ID
  table (`CP_CanPhysReqId`, `CP_CanRespUSDTId`, `CP_CanRespUSDTFormat`,
  `CP_CanRespUUDTId`) decide which received CAN messages are delivered.
- A `SENDRECV` **with data** transmits it (CAN: the first 4 bytes are the CAN
  ID). ISO 15765-2 segmentation is not performed: complete messages are
  exchanged in both directions. CAN ID `0x100` on SW-CAN is a high-voltage
  wake-up.
- Received data is only delivered while the host can take it: the logical
  link needs a running receive ComPrimitive and a registered event callback
  (`PDURegisterEventCallback` for that link). Otherwise the message is
  discarded (and logged), so unread results never pile up in the queue.
- `CP_TransmitIndEnable=1` produces a transmit-indication result (RxFlag
  `00 00 00 88`), under the same condition.
- ComParam values are read according to their data type (8/16/32-bit,
  signed or unsigned). Values of other types (byte, struct and long fields)
  are logged and ignored.
- Result RxFlag: `00 00 00 40` (USDT, format 5) or `80 00 00 00` (UUDT, format 0).
  VPW/ALDL results use `00 00 00 00`, UniqueRespIdentifier `0xFFFFFFFE` and AcceptanceId 1.
- IOCTLs: `PDU_IOCTL_READ_VBATT` (`[Device] VBattMilliVolts`, default 13000 mV),
  `START_MSG_FILTER`, `CLEAR_MSG_FILTER`.
  Others return `PDU_ERR_ID_NOT_SUPPORTED`.
- Stubs that return `PDU_STATUS_NOERROR` without filling outputs: `PDUGetComParam`,
  `PDUGetUniqueRespIdTable`, the resource functions, lock/unlock, and `PDUGetLastError`.

## J2534

Both DLLs also export the SAE J2534-1 04.04 API (`PassThruOpen` …
`PassThruIoctl`, all 14 functions, `__stdcall`, 32-bit), with identical
behaviour (`src/c/j2534.c`, `src/cs/J2534*.cs`). They use the same ini, log,
serial / TCP links, simulator and line protocol as the D-PDU side.

**Registration.** J2534 hosts find devices in the registry under
`HKLM\SOFTWARE\PassThruSupport.04.04` (for 32-bit hosts such as Tech2Win on
64-bit Windows: `HKLM\SOFTWARE\WOW6432Node\PassThruSupport.04.04`).
`j2534_register.example.reg` has one entry per DLL (`Name` `dpdu-c` /
`dpdu-cs`). Keep the one(s) you want, change `FunctionLibrary` to where the
DLL is, and import the file as administrator. Then select the device in the
host. Keep `Name` to 7 characters or fewer: Tech2Win shows a longer one as
empty.

**One API at a time.** The D-PDU API and J2534 share the links. While a
D-PDU host has the module connected, `PassThruOpen` fails with
`ERR_DEVICE_IN_USE`; while a J2534 host has the device open,
`PDUModuleConnect` fails.

**Behaviour:**

- One device, DeviceID 1. `PassThruOpen` opens the links and sends
  `HELLO <name> 2.0 J2534`; `PassThruClose` sends `BYE`. Without a
  usable port it shows the same message box as `PDUConstruct` and returns
  `ERR_DEVICE_NOT_CONNECTED`.
- Protocols (one channel per protocol id): `J1850VPW` → VPW; `CAN`,
  `ISO15765`, `CAN_PS`, `ISO15765_PS` → HSCAN; `SW_CAN_PS`, `SW_ISO15765_PS`
  → SWCAN; `GM_UART_PS` (`0x8009`) and GM's vendor id `0x10003` → ALDL. Others (J1850 PWM,
  K-line, …) return `ERR_INVALID_PROTOCOL_ID`. `PassThruConnect` sends
  `OPEN <ch> <BUS> PROTOCOL=<J2534 protocol id> …` and
  `PARAM <ch> DATA_RATE <baud>`; `PassThruDisconnect` sends `CLOSE <ch>`.
- `PassThruWriteMsgs` sends each message as a `TX` line, after a `PROTOCOL`
  line when the active bus changes. CAN messages start with the 4-byte id.
  **ISO 15765** messages are complete messages (up to 4095 bytes after the id,
  no PCI bytes), exactly what the line protocol carries: the simulator plays
  the device's segmentation. No flow control filter is needed to send one
  (Tech2Win also sends functional requests and UUDT frames this way). With
  `ISO15765_ADDR_TYPE`, the extended address is the first byte after the id
  and is passed on as part of the payload. Each one is followed by a `TX_DONE`
  indication (RxStatus `0x08`, the 4-byte id). **Raw CAN** messages (up to 8 bytes) are
  sent as they are. On SWCAN, the `SW_CAN_HV_TX` TxFlag sends a
  `WAKEUP SWCAN` line first, and id `0x100` is a wake-up as on the D-PDU side
  (one `WAKEUP` line when both apply).
- Received `RX` lines go to every channel on that bus that accepts them, as
  J2534 filters decide:
  - ISO 15765 channel: a message whose id matches a `FLOW_CONTROL_FILTER`
    pattern arrives as an ISO 15765 message of any length. With
    `CAN_MIXED_FORMAT` (`0x8000`) set to non-zero, other frames of up to 8 bytes
    arrive as raw CAN (protocol `CAN` / `CAN_PS` / `SW_CAN_PS`) through the
    pass / block filters.
  - Every other channel: a message is received when no `BLOCK_FILTER` and at
    least one `PASS_FILTER` matches. Without filters nothing is received. A
    1-byte all-zero pass filter passes every VPW frame. Mask and pattern
    compare the first 1–12 bytes, on CAN including the id.
  - Up to 128 filters per channel (Tech2Win sets one per CANdi table entry,
    64 for the default GMLAN table). The filter's protocol id and TxFlags are
    ignored. On CAN, the top 3 bits of the id in the mask, pattern and flow
    control message are cleared (Tech2Win marks 29-bit ids with bit 29), and a
    5-byte (extended address) filter compares only the 4-byte id, because
    Tech2Win leaves the 5th byte uninitialised.
  - Received messages carry no RxStatus bits other than those listed here
    (`TX_DONE`, `TX_MSG_TYPE` with LOOPBACK, `CAN_29BIT_ID`): no
    first-frame or padding indications.
  - RxStatus `CAN_29BIT_ID` (`0x100`) marks ids above `0x7FF`. VPW frames
    carry no CRC, in both directions.
  - Each channel queues up to 2000 messages; further ones are dropped (and
    logged) until the host reads.
- Filters are reported as `FILTER` lines. A flow control filter is also sent
  as an `ECU <ch> <BUS> URID=<filter id> REQ=<flow control id> RESP=<pattern id>
  FORMAT=5 UUDT=0` line, so the simulator learns the request / response ids.
  `CLEAR_MSG_FILTERS` sends `FILTERCLR <ch>`.
- `PassThruReadMsgs` waits up to Timeout ms for the requested number of
  messages. If none arrive, it returns `ERR_BUFFER_EMPTY` (Timeout 0) or
  `ERR_TIMEOUT`. If only some arrive, it returns `ERR_TIMEOUT` with those
  messages. `LOOPBACK` (`SET_CONFIG` parameter 3) echoes sent messages with
  RxStatus `TX_MSG_TYPE`. All functions are thread-safe: a reader thread can
  block in `PassThruReadMsgs` while another thread writes.
- `PassThruStartPeriodicMsg` sends a message every 5–65535 ms (up to 10 per
  channel) until it is stopped, the channel is disconnected or
  `CLEAR_PERIODIC_MSGS` runs.
- Ioctls:
  - `READ_VBATT`: `[Device] VBattMilliVolts`, or `VBATT` from the simulator.
  - `READ_PROG_VOLTAGE`: 0.
  - `GET_CONFIG` / `SET_CONFIG`: any parameter is stored and read back.
    `SET_CONFIG` sends `PARAM <ch> <name> <value>`, e.g. `DATA_RATE 41600` for
    VPW 4x mode.
  - `CLEAR_RX_BUFFER`, `CLEAR_TX_BUFFER`, `CLEAR_PERIODIC_MSGS`,
    `CLEAR_MSG_FILTERS`.
  - `SW_CAN_HS` / `SW_CAN_NS`: accepted on SWCAN channels.
  - `FIVE_BAUD_INIT`, `FAST_INIT` and the J1850 PWM lookup table:
    `ERR_NOT_SUPPORTED`. Other ids: `ERR_INVALID_IOCTL_ID`.
- `PassThruSetProgrammingVoltage` is accepted and ignored. `PassThruReadVersion`
  reports `<ModuleName> 2.0`, `<name> 2.0` and `04.04` (`<name>` is
  `dpdu-bridge-c` / `dpdu-bridge-cs`). Every error sets the text returned by
  `PassThruGetLastError`.

**Tech2Win.** In Tech2Win, the J2534 path may not be reachable: its interface switch is never set to
J2534, so it always uses the D-PDU API. The J2534 side is meant for use with other programs.

## Line protocol

The same protocol is used over serial and TCP: ASCII, one message per line,
`\r\n` terminated, hex bytes separated by spaces.

**DLL → simulator** (informational lines can be ignored):

```
HELLO <name> 2.0 [J2534]                       module connected (<name>: dpdu-bridge-c / dpdu-bridge-cs; J2534 when opened via PassThruOpen)
OPEN <hCLL> <BUS> PROTOCOL=<id> RAW=<0|1> CHECKSUM=<0|1>
PARAM <hCLL> <CP_name> <value>                 every PDUSetComParam
ECU <hCLL> <BUS> URID=<hex> REQ=<id> RESP=<id> FORMAT=<n> UUDT=<id>
CONNECT / DISCONNECT / CLOSE <hCLL>
STARTCOMM / STOPCOMM <hCLL>
PROTOCOL <BUS>                                 active bus changed before a send
FILTER <hCLL> <BUS> <PASS|BLOCK> #<n> MASK=.. PATTERN=..    /   FILTERCLR <hCLL>
FILTER <ch> <BUS> FLOW #<n> MASK=.. PATTERN=.. FLOW=..      J2534 flow control filter
WAKEUP SWCAN                                   GMLAN high-voltage wake-up
TX HSCAN 7E0 22 F1 90                          CAN: <BUS> <CAN id> <payload, no ISO-TP PCI>
TX SWCAN 241 1A 90
TX VPW 6C 10 F1 3C 01                          VPW/ALDL: complete frame bytes
TX ALDL F4 56 01 B5
BYE
# ...                                          client notices and echoes (see "Clients")
```

`<BUS>` is one of `HSCAN`, `SWCAN`, `VPW`, `ALDL`.

**Simulator → DLL:**

```
RX HSCAN 7E8 62 F1 90 31 32 ...   CAN response: id + complete payload (any length up to 4095)
RX SWCAN 641 5A 90 ...            (GMLAN accepted as alias for SWCAN)
RX VPW 6C F1 10 7C 01 ...         complete frame
RX ALDL F4 95 01 ... chk
VBATT 12500                       sets the PDU_IOCTL_READ_VBATT value
# comment                         ignored
```

A CAN `RX` is delivered only when its ID matches an `ECU` entry (`RESP` for
FORMAT 5, `UUDT` for FORMAT 0) and the receive ComPrimitive's expected
responses accept it. Frames for a bus that isn't active are dropped (see the log).

## Notes

- com0com stalls writes while nothing has CNCA8 open. Each link (serial port,
  TCP clients) has its own background writer, and stalled serial output is
  dropped after 250 ms. Tech2Win therefore never blocks, and a serial port
  nobody is reading never delays the TCP clients.

## License

GNU General Public License v3.0; see [LICENSE](LICENSE).
