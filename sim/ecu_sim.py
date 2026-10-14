"""ECU simulator for the D-PDU bridge DLLs (dpdu-bridge-c.dll / dpdu-bridge-cs.dll).

The DLL writes every message Tech2Win transmits as a text line, and lines
written back are delivered to Tech2Win as received messages.  See README.md
for the complete line protocol.

J1850 VPW (GM Class 2): every module in ecu.json is simulated - J2178
addressing, J2190 enhanced diagnostic modes and J1979 generic OBD-II
(see vpw.py).  HS-CAN / SW-CAN and ALDL get simple generic answers.

Port from the ini next to the built driver (dpdu-bridge-c.dll /
dpdu-bridge-cs.dll in the current directory, else the build folder): the
driver's own dpdu-bridge-c.ini / dpdu-bridge-cs.ini if it exists, else the
shared dpdu-bridge.ini, the same file the DLL reads:
    python ecu_sim.py [-c | -cs | --ini FILE] [--use auto|serial|tcp] [--ecus ecu.json] [--quiet]
Without -c / -cs / --ini, the C driver is used if it is built, else the C#
driver.

Serial ([Serial] Port=COM8; a com0com pair delivers the lines on CNCA8):
    python ecu_sim.py --port CNCA8 [--baud 115200] [--ecus ecu.json] [--quiet]

TCP ([Tcp] Port=5000; the DLL listens, the simulator connects):
    python ecu_sim.py --tcp 127.0.0.1:5000 [--ecus ecu.json] [--quiet]

Lines starting with "#" (connect notices and other clients' echoed traffic)
are only logged.  Press ESC to clear the screen, Ctrl+C to stop.  While
$2A data packets are streaming, other traffic is shown in green and SPACE
hides / shows the DPID traffic.  The modules' node-alive heartbeats are only
logged with --show-heartbeats.
"""
import argparse
import contextlib
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vpw  # noqa: E402  (J1850 VPW module simulation)


# ---------------------------------------------------------------------------
# Generic CAN data (HS-CAN / SW-CAN); VPW modules come from ecu.json
# ---------------------------------------------------------------------------

# GMLAN $1A ReadDataByIdentifier (1 byte DID) -> data bytes
GM_DIDS = {
    0x90: b"1G1YY22G965100001",          # VIN
    0xB0: bytes([0x11]),                 # ECU diagnostic address
    0xC1: bytes([0x01, 0x23, 0x45, 0x67]),  # end model part number
    0xCC: bytes([0x00, 0x00, 0x00, 0x01]),  # base model part number
}

# UDS $22 ReadDataByIdentifier (2 byte DID) -> data bytes
UDS_DIDS = {
    0xF190: b"1G1YY22G965100001",
    0xF187: b"12345678",
}


def check_keys(on_key=None):
    """ESC clears the console window; other keys go to on_key(key)
    (Windows console only)."""
    if os.name != "nt" or sys.stdin is None or not sys.stdin.isatty():
        return
    import msvcrt
    while msvcrt.kbhit():
        key = msvcrt.getwch()
        if key == "\x1b":
            os.system("cls")
        elif key == "\x03":                 # Ctrl+C read as a key
            raise KeyboardInterrupt
        elif on_key:
            on_key(key)


GREEN, RESET = "\x1b[92m", "\x1b[0m"


def enable_color():
    """True if the console shows ANSI colours (turns them on for Windows consoles)."""
    if not sys.stdout.isatty():
        return False
    if os.name == "nt":
        import ctypes
        from ctypes import wintypes
        # Own kernel32 instance with full prototypes, so 64-bit handles are not
        # truncated to int and the process-wide ctypes.windll stays untouched.
        kernel32 = ctypes.WinDLL("kernel32")
        kernel32.GetStdHandle.argtypes = (wintypes.DWORD,)
        kernel32.GetStdHandle.restype = wintypes.HANDLE
        kernel32.GetConsoleMode.argtypes = (wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD))
        kernel32.GetConsoleMode.restype = wintypes.BOOL
        kernel32.SetConsoleMode.argtypes = (wintypes.HANDLE, wintypes.DWORD)
        kernel32.SetConsoleMode.restype = wintypes.BOOL
        handle = kernel32.GetStdHandle(-11 & 0xFFFFFFFF)    # STD_OUTPUT_HANDLE
        mode = wintypes.DWORD()
        if not kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
            return False                                    # not a console (redirected)
        # ENABLE_VIRTUAL_TERMINAL_PROCESSING; fails on consoles without ANSI support
        return bool(kernel32.SetConsoleMode(handle, mode.value | 0x0004))
    return True


# VPW modes that define or carry data packets ($2A/$6A, $2C/$6C)
DPID_MODES = {0x2A, 0x6A, 0x2C, 0x6C}


def is_dpid_traffic(line):
    """True for a "TX VPW" / "RX VPW" line that requests, defines or carries a
    DPID, including a negative response to such a request."""
    words = line.split()
    if len(words) < 6 or words[1].upper() != "VPW":
        return False
    try:
        mode = int(words[5], 16)                    # <TX|RX> VPW <hdr> <tgt> <src> <mode>
        if mode == 0x7F and len(words) > 6:
            mode = int(words[6], 16)
    except ValueError:
        return False
    return mode in DPID_MODES


def hexs(data):
    """Bytes as the line protocol's space-separated hex, e.g. "6C F1 10"."""
    return " ".join("%02X" % b for b in data)


def negative(sid, nrc=0x11):
    """GMLAN / UDS negative response (default NRC $11: service not supported)."""
    return bytes([0x7F, sid, nrc])


def diag_response(req):
    """Answer a GMLAN / UDS style request; return None for no response."""
    if not req:
        return None
    sid = req[0]
    if sid == 0x3E:                                  # TesterPresent
        return None if len(req) > 1 and req[1] & 0x80 else bytes([0x7E])
    if sid == 0x10:                                  # DiagnosticSessionControl / InitiateDiagnosticOperation
        return bytes([0x50]) + req[1:2]
    if sid == 0x20:                                  # ReturnToNormalMode
        return bytes([0x60])
    if sid == 0x28:                                  # DisableNormalCommunication
        return bytes([0x68])
    if sid == 0xA2:                                  # ReportProgrammedState
        return bytes([0xE2, 0x00])
    if sid == 0x27 and len(req) >= 2:                # SecurityAccess: zero seed = unlocked
        return bytes([0x67, req[1], 0x00, 0x00]) if req[1] & 1 else bytes([0x67, req[1]])
    if sid == 0x1A and len(req) >= 2:                # GMLAN ReadDataByIdentifier
        data = GM_DIDS.get(req[1])
        return bytes([0x5A, req[1]]) + data if data is not None else negative(sid, 0x31)
    if sid == 0x22 and len(req) >= 3:                # UDS ReadDataByIdentifier
        did = (req[1] << 8) | req[2]
        data = UDS_DIDS.get(did)
        return bytes([0x62]) + req[1:3] + data if data is not None else negative(sid, 0x31)
    if sid == 0xA9:                                  # GMLAN ReadDiagnosticInformation (DTCs): none
        return None
    if sid == 0x19:                                  # UDS ReadDTCInformation: no DTCs
        return bytes([0x59, req[1] if len(req) > 1 else 0x02, 0xFF])
    return negative(sid)


# ---------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------

class Simulator:
    """Answers the DLL's TX lines over one link (SerialLink or TcpLink) until
    the link closes."""

    def __init__(self, link, quiet, vpw_ecus, show_heartbeats=False):
        self.ser = link     # SerialLink or TcpLink
        self.quiet = quiet
        self.ecus = {}      # (bus, request id) -> response id, learnt from "ECU" lines
        self.vpw = vpw.VpwNetwork(
            vpw_ecus, lambda frame: self.send("RX VPW %s" % hexs(frame)), self.log,
            heartbeat_send=lambda frame: self.send("RX VPW %s" % hexs(frame), log=show_heartbeats))
        # The DLL only delivers VPW frames while it has a logical link on VPW and
        # VPW is its active bus; track both from its OPEN / CLOSE / PROTOCOL /
        # HELLO / BYE lines and only send heartbeats while they hold.
        self.links = {}         # hCLL -> bus
        self.active_bus = None
        self.vpw.set_heartbeats(False)
        self.color = enable_color()
        self.hide_dpids = False     # SPACE while streaming: hide DPID traffic
        self.session_ended = False  # end_session() already ran since the last HELLO

    def log(self, text):
        if not self.quiet:
            print(time.strftime("%H:%M:%S"), text, flush=True)

    def on_key(self, key):
        if key == " " and self.vpw.streams:
            self.hide_dpids = not self.hide_dpids
            self.log("DPID messages %s (SPACE toggles)" % ("hidden" if self.hide_dpids else "shown"))

    def log_traffic(self, prefix, line):
        """Logs a TX/RX line.  While repeating DPIDs are streaming, other
        traffic is shown in green so it stands out, and DPID traffic is left
        out if SPACE has hidden it."""
        msg = line
        if msg.startswith("#") and ": " in msg:     # another client's traffic: "# <src>: RX ..."
            msg = msg.split(": ", 1)[1]
        words = msg.split(None, 1)
        streaming = bool(self.vpw.streams) and bool(words) and words[0].upper() in ("TX", "RX")
        if streaming and is_dpid_traffic(msg):
            if not self.hide_dpids:
                self.log(prefix + line)
        elif streaming and self.color:
            self.log(GREEN + prefix + line + RESET)
        else:
            self.log(prefix + line)

    def send(self, line, log=True):
        if log:
            self.log_traffic("<< ", line)
        self.ser.write((line + "\r\n").encode("ascii"))

    def reply_can(self, bus, can_id, data):
        self.send("RX %s %03X %s" % (bus, can_id, hexs(data)) if can_id <= 0x7FF
                  else "RX %s %08X %s" % (bus, can_id, hexs(data)))

    def on_ecu(self, words):
        # ECU <hCLL> <BUS> URID=.. REQ=.. RESP=.. FORMAT=.. UUDT=..
        if len(words) < 3:
            return
        fields = dict(w.split("=", 1) for w in words[3:] if "=" in w)
        try:
            req, resp = int(fields.get("REQ", "0"), 16), int(fields.get("RESP", "0"), 16)
        except ValueError as exc:
            self.log("bad ECU line (%s)" % exc)
            return
        if req and resp:
            self.ecus[(words[2], req)] = resp

    def on_tx_can(self, bus, can_id, data):
        resp_id = self.ecus.get((bus, can_id))
        if resp_id is None and 0x7E0 <= can_id <= 0x7E7:
            resp_id = can_id + 8                      # OBD/UDS convention
        if resp_id is None and 0x240 <= can_id <= 0x25F:
            resp_id = can_id + 0x400                  # GMLAN physical request -> USDT response
        if resp_id is None:
            return                                    # functional/broadcast: stay quiet
        response = diag_response(data)
        if response is not None:
            self.reply_can(bus, resp_id, response)

    def on_tx_vpw(self, frame):
        self.vpw.handle(frame, time.monotonic())

    def on_tx_aldl(self, frame):
        # ALDL: [device id, 0x55 + length, mode, data..., checksum]
        if len(frame) < 4:
            return
        device, mode = frame[0], frame[2]
        body = bytes([mode]) + (bytes(64) if mode == 0x01 else b"")
        out = bytes([device, 0x55 + len(body) + 1]) + body
        out += bytes([(-sum(out)) & 0xFF])
        self.send("RX ALDL %s" % hexs(out))

    def end_session(self):
        """Tech2Win disconnected (BYE, or the connection to the DLL ended): every
        module returns to normal operation, as after a $20 request."""
        self.vpw.return_all_to_normal()
        if not self.session_ended:          # BYE followed by the TCP close: note it once
            self.session_ended = True
            self.log("Tech2Win disconnected: all modules returned to normal")

    def on_link_state(self, cmd, words):
        if cmd in ("HELLO", "BYE"):                  # module (dis)connected: no links
            self.links.clear()
            self.active_bus = None
            if cmd == "HELLO":
                self.session_ended = False
            else:
                self.end_session()
        elif cmd == "OPEN" and len(words) >= 3:     # OPEN <hCLL> <BUS> ...
            self.links[words[1]] = words[2].upper()
            self.active_bus = words[2].upper()
        elif cmd == "CLOSE" and len(words) >= 2:    # CLOSE <hCLL>
            self.links.pop(words[1], None)
        elif cmd == "PROTOCOL" and len(words) >= 2:  # PROTOCOL <BUS>
            self.active_bus = words[1].upper()
        deliverable = self.active_bus == "VPW" and "VPW" in self.links.values()
        if deliverable != self.vpw.heartbeats_enabled:
            self.vpw.set_heartbeats(deliverable)
            self.log("VPW logical link %s - heartbeats %s" % (
                "active" if deliverable else "gone", "started" if deliverable else "stopped"))

    def handle(self, line):
        words = line.split()
        if not words:
            return
        cmd = words[0].upper()
        if cmd in ("HELLO", "BYE", "OPEN", "CLOSE", "PROTOCOL"):
            self.on_link_state(cmd, words)
        elif cmd == "ECU":
            self.on_ecu(words)
        elif cmd == "TX" and len(words) >= 2:
            bus = words[1].upper()
            try:
                if bus in ("HSCAN", "SWCAN"):
                    can_id = int(words[2], 16)
                    self.on_tx_can(bus, can_id, bytes(int(w, 16) for w in words[3:]))
                elif bus == "VPW":
                    self.on_tx_vpw(bytes(int(w, 16) for w in words[2:]))
                elif bus == "ALDL":
                    self.on_tx_aldl(bytes(int(w, 16) for w in words[2:]))
            except (IndexError, ValueError) as exc:
                self.log("bad TX line (%s)" % exc)

    def run(self):
        self.log("listening on %s" % self.ser.name)
        buf = b""
        while True:
            chunk = self.ser.read(256)
            if chunk is None:
                self.log("connection closed")
                self.end_session()
                return
            self.vpw.poll(time.monotonic())     # periodic $2A data packets
            check_keys(self.on_key)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if line:
                    self.log_traffic(">> ", line)
                    self.handle(line)


class SerialLink:
    """com0com / serial port transport (needs pyserial)."""

    def __init__(self, port, baud):
        try:
            import serial  # pip install pyserial
        except ImportError:
            sys.exit("pyserial is needed for serial mode: %s -m pip install pyserial" % os.path.basename(sys.executable))
        # pyserial only adds the \\.\ device prefix for COMnn names; com0com
        # ports such as CNCA8 need it as well.
        if sys.platform == "win32" and not port.startswith("\\\\.\\"):
            port = "\\\\.\\" + port
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.name = port

    def read(self, n):
        return self.ser.read(n)        # b"" on timeout

    def write(self, data):
        self.ser.write(data)

    def close(self):
        self.ser.close()


class TcpLink:
    """TCP client transport: connects to the DLL's listening port."""

    def __init__(self, address):
        host, _, port = address.rpartition(":")
        host = host.strip("[]") or "127.0.0.1"      # "[::1]:5000" -> ::1
        if not (port.isdigit() and 1 <= int(port) <= 65535):
            sys.exit("invalid TCP address %s: expected HOST:PORT with a port 1-65535" % address)
        # The DLL only listens while Tech2Win has the module connected, so
        # keep trying until it is there.
        while True:
            try:
                self.sock = socket.create_connection((host, int(port)), timeout=2)
                break
            except OSError:
                check_keys()
                time.sleep(0.2)
        self.sock.settimeout(0.02)
        self.name = "tcp %s" % address

    def read(self, n):
        """Received bytes, b"" if none arrived in time, None once the
        connection is closed."""
        if self.sock is None:
            return None
        try:
            data = self.sock.recv(n)
        except socket.timeout:
            return b""
        except OSError:
            return None
        return data if data else None

    def write(self, data):
        if self.sock is None:
            return
        try:
            self.sock.sendall(data)
        except OSError:
            # The DLL dropped the connection (end of the Tech2Win session) or
            # stopped reading.  Close it; the next read() reports it and the
            # simulator reconnects, as when the DLL closes it cleanly.
            self.close()

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None


def com0com_partner(port):
    """The other end of a com0com pair (CNCAn <-> CNCBn), e.g. COM8 -> CNCA8; None if unknown."""
    try:
        import winreg
        names = {}
        base = r"SYSTEM\CurrentControlSet\Enum\com0com\port"
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, base) as key:
            for i in range(winreg.QueryInfoKey(key)[0]):
                dev = winreg.EnumKey(key, i)
                try:
                    with winreg.OpenKey(key, dev + r"\Device Parameters") as params:
                        names[dev.upper()] = winreg.QueryValueEx(params, "PortName")[0]
                except OSError:
                    pass                    # device without a port name: not usable
    except (ImportError, OSError):        # not Windows, or com0com not installed
        return None
    for dev, name in names.items():
        if name.upper() == port.upper() and dev[:4] in ("CNCA", "CNCB"):
            partner = ("CNCB" if dev.startswith("CNCA") else "CNCA") + dev[4:]
            return names.get(partner)
    return None


DRIVERS = ("c", "cs")  # dpdu-bridge-<name>.dll, built with "build.cmd <name>"


def driver_ini(driver):
    """The ini the driver reads: dpdu-bridge-<driver>.ini next to
    dpdu-bridge-<driver>.dll if it exists, else the shared dpdu-bridge.ini
    there.  The DLL is looked up in the current directory, else the project's
    build folder.  With driver None, the C driver is used if it is built, else
    the C# driver; exits when none is built."""
    build = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "build"))
    dirs = [os.getcwd()]
    if os.path.normcase(build) != os.path.normcase(dirs[0]):
        dirs.append(build)
    drivers = [d for d in DRIVERS if driver in (None, d)]
    for name in drivers:
        for d in dirs:
            dll = os.path.join(d, "dpdu-bridge-%s.dll" % name)
            if os.path.exists(dll):
                print("driver: %s" % dll)
                own = os.path.join(d, "dpdu-bridge-%s.ini" % name)
                return own if os.path.exists(own) else os.path.join(d, "dpdu-bridge.ini")
    sys.exit("%s not found in %s - build %s first (%s)" % (
        " or ".join("dpdu-bridge-%s.dll" % name for name in drivers), " or ".join(dirs),
        "the driver" if driver else "one of the drivers",
        " or ".join("build.cmd %s" % name for name in drivers)))


def link_from_ini(path, use):
    """Picks the simulator's link from the driver's ini.

    [Serial] Port=COM8 -> the other end of the com0com pair (e.g. CNCA8);
    [Tcp] Port / Bind  -> connect to that TCP port.  With both configured,
    serial is used unless use == "tcp" (connecting to both would answer
    every request twice).  Returns ("serial", port) or ("tcp", "host:port").
    """
    import configparser
    cp = configparser.ConfigParser(interpolation=None, strict=False)
    try:
        found = cp.read(path, encoding="latin-1")
    except configparser.Error as exc:
        sys.exit("cannot parse %s: %s" % (path, exc))
    if not found:
        sys.exit("cannot read %s" % path)

    # Section names are case-insensitive, as for GetPrivateProfileString
    # (configparser already lowercases keys); the first spelling wins.
    sections = {}
    for name in cp.sections():
        sections.setdefault(name.lower(), name)

    def get(section, key):  # same rules as the DLL: empty = not set, quotes stripped
        name = sections.get(section.lower())
        return cp.get(name, key, fallback="").strip().strip('"') if name else ""

    serial_port = get("Serial", "Port")
    if serial_port.isdigit():
        print("%s: [Serial] Port=%s is not a COM port name - ignored" % (path, serial_port))
        serial_port = ""
    tcp_port, bind = get("Tcp", "Port"), get("Tcp", "Bind") or "127.0.0.1"
    if tcp_port and not (tcp_port.isdigit() and 1 <= int(tcp_port) <= 65535):
        print("%s: [Tcp] Port=%s is not a valid TCP port - ignored" % (path, tcp_port))
        tcp_port = ""

    if use == "tcp" or (use == "auto" and not serial_port and tcp_port):
        if not tcp_port:
            sys.exit("%s has no [Tcp] Port" % path)
        host = {"0.0.0.0": "127.0.0.1", "::": "::1"}.get(bind, bind)
        address = "[%s]:%s" % (host, tcp_port) if ":" in host else "%s:%s" % (host, tcp_port)
        print("%s: [Tcp] Port=%s -> connecting to %s" % (path, tcp_port, address))
        return "tcp", address
    if not serial_port:
        sys.exit("%s has no [Serial] Port%s" % (path, "" if use == "serial" else " or [Tcp] Port"))
    partner = com0com_partner(serial_port)
    if not partner:
        sys.exit("%s: [Serial] Port=%s is not one end of a com0com pair; start the simulator "
                 "with --port <other end> instead" % (path, serial_port))
    print("%s: [Serial] Port=%s -> simulator on the other end of the com0com pair, %s" % (path, serial_port, partner))
    return "serial", partner


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", help="serial port, e.g. CNCA8 (instead of the driver's ini)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--tcp", metavar="HOST:PORT", help="connect to the DLL over TCP (instead of the driver's ini)")
    drv = ap.add_mutually_exclusive_group()
    drv.add_argument("-c", dest="driver", action="store_const", const="c",
                     help="use the C driver's ini: dpdu-bridge-c.ini (else dpdu-bridge.ini) next to "
                     "dpdu-bridge-c.dll (default: C if built, else C#)")
    drv.add_argument("-cs", dest="driver", action="store_const", const="cs",
                     help="use the C# driver's ini: dpdu-bridge-cs.ini (else dpdu-bridge.ini) next to "
                     "dpdu-bridge-cs.dll")
    drv.add_argument("--ini", metavar="FILE", help="take the port from this ini file")
    ap.add_argument("--use", choices=("auto", "serial", "tcp"), default="auto",
                    help="which link of the ini to use when both are configured (auto = serial)")
    ap.add_argument("--ecus", metavar="FILE", help="VPW module definitions (default: ecu.json next to "
                    "ecu_sim.py)")
    ap.add_argument("--show-heartbeats", action="store_true",
                    help="also log the node-alive messages (08 FF <node> 03, every 2 s per module)")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()
    path = args.ecus or os.path.join(os.path.dirname(os.path.abspath(__file__)), "ecu.json")
    try:
        ecus = vpw.load_ecus(path)
    except (OSError, ValueError) as exc:
        sys.exit("cannot load %s: %s" % (path, exc))
    print("loaded %d VPW modules from %s: %s" % (len(ecus), path, " ".join(
        "%02X=%s" % (e.addr, e.name) for e in ecus)), flush=True)
    tcp, port = args.tcp, args.port
    if not tcp and not port:  # --port / --tcp win over the ini
        kind, where = link_from_ini(args.ini or driver_ini(args.driver), args.use)
        tcp, port = (where, None) if kind == "tcp" else (None, where)
    try:
        if tcp:
            print("waiting for the DLL on %s ..." % tcp, flush=True)
            while True:  # reconnect whenever the DLL closes the connection
                with contextlib.closing(TcpLink(tcp)) as link:
                    Simulator(link, args.quiet, ecus, args.show_heartbeats).run()
        else:
            with contextlib.closing(SerialLink(port, args.baud)) as link:
                Simulator(link, args.quiet, ecus, args.show_heartbeats).run()
    except KeyboardInterrupt:
        pass
    except OSError as exc:  # serial.SerialException and socket errors
        sys.exit("connection error: %s" % exc)


if __name__ == "__main__":
    main()
