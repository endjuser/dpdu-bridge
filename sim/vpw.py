"""J1850 VPW ECU simulation for ecu_sim.py.

Simulates the modules described in ecu.json on a J1850 VPW (GM Class 2) bus:

- SAE J2178 addressing: a request is handled by the module it is physically
  addressed to (header Y bit set), by every module for the GM "all nodes"
  address $FE, and by the OBD-II modules for the functional legislated
  diagnostics address $6A.
- SAE J2190 enhanced diagnostic modes ($10-$3F) using the module's
  supportedCommands, pids, dpids and blocks; GM-specific modes ($A0 and up)
  are only acknowledged.
- SAE J1979 generic OBD-II modes $01-$09 (functional 68 6A F1 requests,
  48 6B xx responses).
- GM node-alive heartbeats (08 FF <node> 03) every 2 s per module.

Frames are complete J1850 messages without the CRC byte:
    [header][target][source][mode][data...]

Usage:
    net = VpwNetwork(load_ecus("ecu.json"), send)   # send(frame) transmits a response
    net.handle(request_frame, time.monotonic())     # answer one request
    net.poll(time.monotonic())                      # call often: heartbeats, $2A streams
"""
import json
import random
import re

BROADCAST = 0xFE            # GM: all nodes (physical addressing)
OBD_REQUEST = 0x6A          # J2178 functional address: legislated diagnostics request
OBD_RESPONSE = 0x6B         # J2178 functional address: legislated diagnostics response
OBD_RESPONSE_HEADER = 0x48
MAX_DATA = 8                # J1850: 12-byte frame = 3 header + 8 data + CRC

# J2190 negative response codes
NRC_SERVICE_NOT_SUPPORTED = 0x11
NRC_INVALID_FORMAT = 0x12
NRC_OUT_OF_RANGE = 0x31

# J2190 $27 security access: positive response code for an accepted key
SECURITY_ACCESS_GRANTED = 0x34

# Node-alive heartbeat: every module broadcasts 08 FF <node> 03 every 2 s so
# that the tester sees it as present on the bus.
HEARTBEAT_INTERVAL = 2.0
HEARTBEAT_HEADER = 0x08     # priority 0, 3-byte header, no IFR, functional
HEARTBEAT_TARGET = 0xFF     # functional target: node alive
HEARTBEAT_DATA = 0x03
HEARTBEAT_CRC = False       # frames are sent without their J1850 CRC byte, like all other responses

# $2A data rates -> repeat interval in seconds ($00 = stop, $01 = send once).
# $02/$03/$04 are the standard slow/medium/fast rates, $12-$14 repeat every
# 50 ms, and any other rate repeats every 200 ms (DPID_DEFAULT_RATE).
DPID_RATES = {0x02: 1.0, 0x03: 0.2, 0x04: 0.05, 0x12: 0.05, 0x13: 0.05, 0x14: 0.05}
DPID_DEFAULT_RATE = 0.2

# J1979 mode $01/$02 PID data lengths, used to send zeros for PIDs without data
OBD_PID_LENGTHS = {
    0x01: 4, 0x02: 2, 0x03: 2, 0x0C: 2, 0x10: 2, 0x14: 2, 0x15: 2, 0x16: 2, 0x17: 2,
    0x18: 2, 0x19: 2, 0x1A: 2, 0x1B: 2, 0x1F: 2, 0x21: 2, 0x22: 2, 0x23: 2,
    0x24: 4, 0x25: 4, 0x26: 4, 0x27: 4, 0x28: 4, 0x29: 4, 0x2A: 4, 0x2B: 4,
    0x31: 2, 0x32: 2, 0x34: 4, 0x35: 4, 0x36: 4, 0x37: 4, 0x38: 4, 0x39: 4,
    0x3A: 4, 0x3B: 4, 0x3C: 2, 0x3D: 2, 0x3E: 2, 0x3F: 2, 0x41: 4, 0x42: 2,
    0x43: 2, 0x44: 2, 0x4D: 2, 0x4E: 2,
}


class Negative(Exception):
    """Raised by a mode handler to send a J2190 negative response."""

    def __init__(self, code):
        super().__init__(code)
        self.code = code


def _hex(s):
    return bytes.fromhex(s)


class Ecu:
    """One module from ecu.json, plus its diagnostic state (security access,
    $2C definitions; $3B block writes change blocks in place)."""

    def __init__(self, cfg):
        self.addr = int(cfg["id"], 16)
        self.name = cfg.get("name", cfg["id"])
        self.commands = {int(c, 16) for c in cfg.get("supportedCommands", [])}
        data = cfg.get("data", {})
        self.pids = {int(k, 16): _hex(v) for k, v in data.get("pids", {}).items()}
        self.dpids = {int(k, 16): _hex(v) for k, v in data.get("dpids", {}).items()}
        self.blocks = {int(k, 16): _hex(v) for k, v in data.get("blocks", {}).items()}
        self.unlocked = False
        self.dpid_defs = {}     # $2C definitions: dpid -> [(position, length, source type, [PIDs or address])]

    @property
    def obd_pids(self):
        """J1979 PIDs: the module's $22 PIDs $0001-$00FF are the generic OBD-II PIDs."""
        return {pid: v for pid, v in self.pids.items() if pid <= 0xFF}

    @property
    def is_obd(self):
        """Only modules with generic OBD-II PIDs answer J1979 requests."""
        return bool(self.obd_pids)

    @property
    def vin(self):
        """GM modules keep the VIN in blocks $01-$03 (leading pad byte removed)."""
        raw = b"".join(self.blocks.get(b, b"") for b in (1, 2, 3)).lstrip(b"\x00")
        return raw.decode("ascii", "replace") if len(raw) == 17 else None

    @property
    def calibration_id(self):
        """GM block $08 holds the calibration part number (4 bytes, e.g.
        00C01298 = 12587672); J1979 reports it as up to 16 ASCII characters."""
        block = self.blocks.get(0x08)
        if not block or len(block) != 4 or not any(block):
            return None
        return str(int.from_bytes(block, "big")).encode("ascii")


VIN_POSITIONS = "ABCDEFGHIJKLMNOPQ"   # placeholder vA..vQ -> VIN character 1..17
VIN_TOKEN = re.compile(r"[vV]([A-Qa-q])")


def expand_vin(value, vin, where):
    """Replaces VIN placeholders in a hex string: vA..vQ stand for the bytes of
    VIN characters 1..17 (e.g. "00vAvBvCvDvE" -> "0031474E464B")."""
    def byte(m):
        if not vin:
            raise ValueError("%s uses VIN placeholder %s but ecu.json has no \"VIN\"" % (where, m.group(0)))
        return "%02X" % ord(vin[VIN_POSITIONS.index(m.group(1).upper())])
    return VIN_TOKEN.sub(byte, value)


# Largest key in each "data" section: $22 PIDs are 2 bytes, DPIDs and blocks 1 byte
DATA_KEY_LIMITS = {"pids": 0xFFFF, "dpids": 0xFF, "blocks": 0xFF}


def _hex_number(value, where, limit):
    """Parses a hex number string from ecu.json, e.g. "2A"; raises ValueError
    naming where it is unless it is a hex string in 0..limit."""
    if isinstance(value, str):
        try:
            number = int(value, 16)
        except ValueError:
            number = -1
        if 0 <= number <= limit:
            return number
    raise ValueError("%s: %r is not a hex number 00-%X" % (where, value, limit))


def load_ecus(path):
    """Loads and validates the modules from ecu.json:

        {"VIN": "1GNFK16ZX4J123456", "nodes": [{"id": "10", "name": "PCM", ...}, ...]}

    The "VIN" fills the vA..vQ placeholders in the modules' data.  Raises
    OSError if the file can't be read, ValueError (naming the offending
    module and entry) if it isn't valid."""
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)
    if not isinstance(cfg, dict):
        raise ValueError("ecu.json must hold a JSON object")
    vin = cfg.get("VIN")
    if vin is not None and not (isinstance(vin, str) and len(vin) == 17 and vin.isascii()):
        raise ValueError("\"VIN\" must be 17 ASCII characters, got %r" % (vin,))
    nodes = cfg.get("nodes")
    if not isinstance(nodes, list):
        raise ValueError("ecu.json needs a \"nodes\" array")
    seen = set()
    for i, module in enumerate(nodes):
        if not isinstance(module, dict) or "id" not in module:
            raise ValueError("nodes[%d] has no \"id\"" % i)
        addr = _hex_number(module["id"], "nodes[%d] \"id\"" % i, 0xFF)
        if addr in seen:
            raise ValueError("module id %02X appears more than once in \"nodes\"" % addr)
        seen.add(addr)
        where = "module %02X" % addr
        commands = module.get("supportedCommands", [])
        if not isinstance(commands, list):
            raise ValueError("%s: \"supportedCommands\" must be an array" % where)
        for command in commands:
            _hex_number(command, where + " supportedCommands", 0xFF)
        data = module.get("data", {})
        if not isinstance(data, dict):
            raise ValueError("%s: \"data\" must be an object" % where)
        for section, values in data.items():
            if not isinstance(values, dict):
                raise ValueError("%s: \"data\" \"%s\" must be an object" % (where, section))
            for key, value in values.items():
                place = "%s %s %s" % (where, section, key)
                if section in DATA_KEY_LIMITS:
                    _hex_number(key, place, DATA_KEY_LIMITS[section])
                if not isinstance(value, str):
                    raise ValueError("%s: %r is not a hex string" % (place, value))
                values[key] = expand_vin(value, vin, place)
                try:
                    bytes.fromhex(values[key])
                except ValueError:
                    raise ValueError("%s: %r is not a string of hex bytes" % (place, value)) from None
    return [Ecu(e) for e in nodes]


def j1850_crc(data):
    """SAE J1850 CRC-8 (polynomial $1D, initial value $FF, result inverted)."""
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def heartbeat_frame(addr):
    """GM Class 2 node-alive message: 08 FF <node> 03 (plus its CRC if HEARTBEAT_CRC)."""
    frame = bytes([HEARTBEAT_HEADER, HEARTBEAT_TARGET, addr, HEARTBEAT_DATA])
    return frame + bytes([j1850_crc(frame)]) if HEARTBEAT_CRC else frame


class VpwNetwork:
    """All simulated VPW modules.  send(frame) transmits one response frame;
    heartbeat_send(frame), if given, is used for the periodic node-alive
    messages instead (e.g. to keep them out of the console log)."""

    def __init__(self, ecus, send, log=print, heartbeat_send=None):
        self.ecus = {e.addr: e for e in ecus}
        self.send = send
        self.heartbeat_send = heartbeat_send or send
        self.log = log
        self.streams = {}   # (module, requester, dpid) -> [interval, next due time]
        self.heartbeat_due = None   # module address -> next node-alive time
        self.heartbeats_enabled = True

    # ---- dispatch ---------------------------------------------------------

    def handle(self, frame, now):
        """Answers one request frame (sent by the tester) received at time now."""
        if len(frame) < 4:
            return
        header, target, source, mode, data = frame[0], frame[1], frame[2], frame[3], bytes(frame[4:])
        physical = bool(header & 0x04)      # J2178 Y bit: 1 = physical addressing
        if not physical:
            if target == OBD_REQUEST:
                for ecu in self.ecus.values():
                    if ecu.is_obd:
                        self._obd(ecu, mode, data)
            return                          # other functional addresses are not simulated
        if target == BROADCAST:
            targets, broadcast = list(self.ecus.values()), True
        elif target in self.ecus:
            targets, broadcast = [self.ecus[target]], False
        else:
            return                          # no such module on the bus
        for ecu in targets:
            self._enhanced(ecu, header, source, mode, data, broadcast, now)

    def set_heartbeats(self, enabled):
        """Starts or stops the node-alive messages (e.g. only while the DLL has a
        VPW logical link that can deliver them).  They restart spread out."""
        if enabled != self.heartbeats_enabled:
            self.heartbeats_enabled = enabled
            self.heartbeat_due = None

    def poll(self, now):
        """Sends what is due: node-alive messages and $2A data packets."""
        if self.heartbeats_enabled:
            if self.heartbeat_due is None:
                # spread the modules' heartbeats evenly over the interval
                step = HEARTBEAT_INTERVAL / max(len(self.ecus), 1)
                self.heartbeat_due = {addr: now + i * step for i, addr in enumerate(self.ecus)}
            for addr, due in self.heartbeat_due.items():
                if now >= due:
                    self.heartbeat_send(heartbeat_frame(addr))
                    self.heartbeat_due[addr] = max(due + HEARTBEAT_INTERVAL, now)
        for (addr, requester, dpid), stream in list(self.streams.items()):
            interval, due = stream
            if now >= due:
                ecu = self.ecus[addr]
                self._reply(ecu, 0x6C, requester, bytes([0x6A, dpid]) + self._dpid_value(ecu, dpid))
                stream[1] = max(due + interval, now)

    # ---- responses --------------------------------------------------------

    def _reply(self, ecu, header, requester, payload):
        self.send(bytes([header, requester, ecu.addr]) + payload[:MAX_DATA])

    def _negative(self, ecu, header, requester, mode, data, code):
        # J2190: $7F, the request mode and its data (as much as fits), response code
        self._reply(ecu, header, requester, bytes([0x7F, mode]) + data[:MAX_DATA - 3] + bytes([code]))

    def _stop_streams(self, ecu):
        for key in [k for k in self.streams if k[0] == ecu.addr]:
            del self.streams[key]

    # ---- diagnostic session -------------------------------------------------

    def return_to_normal(self, ecu):
        """Ends the module's diagnostic session ($20 return to normal operation):
        stops its $2A streams, clears its $2C definitions and locks security
        access again.  Blocks written with $3B are kept."""
        self._stop_streams(ecu)
        ecu.dpid_defs.clear()
        ecu.unlocked = False

    def return_all_to_normal(self):
        """Returns every module to normal operation (the tester disconnected)."""
        for ecu in self.ecus.values():
            self.return_to_normal(ecu)

    # ---- J2190 enhanced diagnostics -----------------------------------------

    def _enhanced(self, ecu, header, src, mode, data, broadcast, now):
        if mode == 0x3F:
            return                          # Test Device Present: no response
        if mode not in ecu.commands:
            # Only a request addressed to this module gets "service not supported";
            # a broadcast is just ignored by modules that don't support the mode.
            if not broadcast:
                self._negative(ecu, header, src, mode, data, NRC_SERVICE_NOT_SUPPORTED)
            return
        handler = self.HANDLERS.get(mode)
        try:
            if handler is None:
                # Not simulated: manufacturer-specific modes (GM $A0 / $A1 high-speed
                # mode, $A8, $AE device control, $B0, $B4, ...) are only
                # acknowledged with a positive response echoing the first data byte;
                # their real request/response formats are not modelled.
                payloads = [bytes([(mode + 0x40) & 0xFF]) + data[:1]]
            else:
                payloads = handler(self, ecu, src, data, now)
        except Negative as neg:
            self._negative(ecu, header, src, mode, data, neg.code)
            return
        for p in payloads or []:
            self._reply(ecu, header, src, p)

    def _mode10(self, ecu, src, data, now):        # Initiate Diagnostic Operation
        return [b"\x50" + data[:1]]

    def _mode12(self, ecu, src, data, now):        # Request Freeze Frame Data
        # Not simulated: ecu.json has no freeze frame data; respond with an empty frame.
        return [b"\x52" + data[:2]]

    def _mode14(self, ecu, src, data, now):        # Clear Diagnostic Information
        return [b"\x54"]

    def _mode17(self, ecu, src, data, now):        # Request Status of DTCs
        # Not simulated: ecu.json has no DTCs; report the requested DTC as not set.
        return [b"\x57" + (data[:2] + b"\x00\x00")[:2] + b"\x00"]

    def _mode19(self, ecu, src, data, now):        # Request DTCs by Status (GM Class 2)
        # Not simulated: ecu.json has no DTCs; send only the end-of-list message.
        return [b"\x59\x00\x00\xFF"]

    def _mode20(self, ecu, src, data, now):        # Return to Normal Operation
        self.return_to_normal(ecu)
        return [b"\x60"]

    def _mode22(self, ecu, src, data, now):        # Request Diagnostic Data by PID
        if len(data) < 2:
            raise Negative(NRC_INVALID_FORMAT)
        pid = (data[0] << 8) | data[1]
        if pid not in ecu.pids:
            raise Negative(NRC_OUT_OF_RANGE)
        return [b"\x62" + data[:2] + ecu.pids[pid]]

    def _mode23(self, ecu, src, data, now):        # Request Diagnostic Data by Memory Address
        if len(data) < 3:
            raise Negative(NRC_INVALID_FORMAT)
        # Not simulated: ecu.json has no memory image; every address reads as zeros.
        return [b"\x63" + data[:3] + bytes(4)]

    def _mode25(self, ecu, src, data, now):        # Stop Transmitting Requested Data
        self._stop_streams(ecu)
        return [b"\x65"]

    def _mode26(self, ecu, src, data, now):        # Specify Data Rates
        return [b"\x66" + data[:1]]

    def _mode27(self, ecu, src, data, now):        # Security Access
        sub = data[0] if data else 0x01
        if sub & 1:                                 # odd: request seed
            seed = b"\x00\x00" if ecu.unlocked else random.randint(1, 0xFFFF).to_bytes(2, "big")
            return [bytes([0x67, sub]) + seed]
        ecu.unlocked = True                         # even: send key - any key is accepted
        return [bytes([0x67, sub, SECURITY_ACCESS_GRANTED])]

    def _mode28(self, ecu, src, data, now):        # Disable Normal Message Transmission
        return [b"\x68"]

    def _mode29(self, ecu, src, data, now):        # Enable Normal Message Transmission
        return [b"\x69"]

    def _mode2A(self, ecu, src, data, now):        # Request Diagnostic Data Packets
        if not data:
            raise Negative(NRC_INVALID_FORMAT)
        # 2A <rate> <dpid>... - unused DPID slots are padded with FF
        rate, dpids = data[0], [d for d in data[1:] if d != 0xFF]
        if rate == 0x00:                            # stop the listed DPIDs, or all of them
            for key in [k for k in self.streams if k[:2] == (ecu.addr, src)]:
                if not dpids or key[2] in dpids:
                    del self.streams[key]
            return []
        if not dpids or any(d not in ecu.dpids for d in dpids):
            raise Negative(NRC_OUT_OF_RANGE)
        if rate != 0x01:                            # repeat until stopped
            # each request adds to the schedule (Tech2Win starts F6-FA and
            # FB-FE with two requests); a DPID asked for again takes the new rate
            interval = DPID_RATES.get(rate, DPID_DEFAULT_RATE)
            for d in dpids:
                self.streams[(ecu.addr, src, d)] = [interval, now + interval]
        return [bytes([0x6A, d]) + self._dpid_value(ecu, d) for d in dpids]

    def _mode2C(self, ecu, src, data, now):        # Dynamically Define Data Packet
        # 2C <dpid> <definition byte> <source...>
        # Definition byte: bits 7-6 source type, bits 5-3 start position in the
        # 6-byte packet (1-6), bits 2-0 number of bytes.
        #   type 01 (PID):            up to two 2-byte PIDs, FFFF = unused slot;
        #                             their values fill the bytes in order
        #                             (e.g. 4A 21A4 21B1 = two 1-byte PIDs at 1-2)
        #   type 10 (memory address): one 3-byte address
        if len(data) < 2:
            raise Negative(NRC_INVALID_FORMAT)
        dpid, d, rest = data[0], data[1], data[2:]
        kind, pos, length = d >> 6, (d >> 3) & 7, d & 7
        if not 1 <= pos <= 6 or length < 1 or pos + length - 1 > 6:
            raise Negative(NRC_INVALID_FORMAT)
        if kind == 1:
            sources = []
            for i in range(0, len(rest) - 1, 2):
                pid = (rest[i] << 8) | rest[i + 1]
                if pid == 0xFFFF:                   # unused slot (padding)
                    continue
                sources.append(pid)
            if not sources:
                raise Negative(NRC_INVALID_FORMAT)
            # Not simulated: PIDs missing from ecu.json are accepted and read as
            # zeros so data screens still work; add real values to ecu.json.
            unknown = [pid for pid in sources if pid not in ecu.pids]
            if unknown:
                self.log("  note: %s has no data for PID %s; DPID %02X uses 00"
                         % (ecu.name, ", ".join("%04X" % pid for pid in unknown), dpid))
        elif kind == 2:
            if len(rest) < 3:
                raise Negative(NRC_INVALID_FORMAT)
            sources = [int.from_bytes(rest[:3], "big")]
        else:
            raise Negative(NRC_INVALID_FORMAT)
        defs = ecu.dpid_defs.setdefault(dpid, [])
        # a new definition replaces whatever covered the same bytes
        defs[:] = [x for x in defs if x[0] + x[1] <= pos or pos + length <= x[0]]
        defs.append((pos, length, kind, sources))
        ecu.dpids.setdefault(dpid, bytes(6))
        return [bytes([0x6C, dpid])]

    @staticmethod
    def _dpid_value(ecu, dpid):
        """The 6 data bytes of a DPID: its stored bytes from ecu.json, with any
        $2C-defined parts filled from the module's current PID values."""
        value = bytearray(ecu.dpids.get(dpid, bytes(6)))
        for pos, length, kind, sources in ecu.dpid_defs.get(dpid, []):
            if kind == 1:
                share = max(1, length // len(sources))   # size of an unknown PID
                part = b"".join(ecu.pids.get(pid, bytes(share)) for pid in sources)
            else:
                # Not simulated: memory-address sources read as zeros (no memory image).
                part = b""
            value[pos - 1:pos - 1 + length] = part[:length].ljust(length, b"\x00")
        return bytes(value)

    def _mode34(self, ecu, src, data, now):        # Request Download
        # Not simulated: programming is accepted but nothing is stored.
        return [b"\x74\x00"]

    def _mode35(self, ecu, src, data, now):        # Request Upload
        # Not simulated: there is no memory image to upload.
        return [b"\x75\x00"]

    def _mode36(self, ecu, src, data, now):        # Data Transfer
        # Not simulated: transferred data is discarded.
        return [b"\x76" + data[:1]]

    def _mode3B(self, ecu, src, data, now):        # Write Data Block
        if not data:
            raise Negative(NRC_INVALID_FORMAT)
        ecu.blocks[data[0]] = bytes(data[1:])
        return [bytes([0x7B, data[0]])]

    def _mode3C(self, ecu, src, data, now):        # Read Data Block
        if not data:
            raise Negative(NRC_INVALID_FORMAT)
        if data[0] not in ecu.blocks:
            raise Negative(NRC_OUT_OF_RANGE)
        return [bytes([0x7C, data[0]]) + ecu.blocks[data[0]]]

    HANDLERS = {
        0x10: _mode10, 0x12: _mode12, 0x14: _mode14, 0x17: _mode17, 0x19: _mode19,
        0x20: _mode20, 0x22: _mode22, 0x23: _mode23, 0x25: _mode25, 0x26: _mode26,
        0x27: _mode27, 0x28: _mode28, 0x29: _mode29, 0x2A: _mode2A, 0x2C: _mode2C,
        0x34: _mode34, 0x35: _mode35, 0x36: _mode36, 0x3B: _mode3B, 0x3C: _mode3C,
    }

    # ---- J1979 generic OBD-II -------------------------------------------------

    def _obd_reply(self, ecu, payload):
        self.send(bytes([OBD_RESPONSE_HEADER, OBD_RESPONSE, ecu.addr]) + payload[:MAX_DATA])

    @staticmethod
    def _support_bitmap(base, supported):
        """J1979 'PIDs supported' bitmap for PIDs base+1 .. base+$20."""
        bits = 0
        for pid in supported:
            if base < pid <= base + 0x20:
                bits |= 1 << (32 - (pid - base))
        if any(pid > base + 0x20 for pid in supported):
            bits |= 1                               # next range is supported too
        return bits.to_bytes(4, "big")

    def _pid_value(self, ecu, pid):
        if pid % 0x20 == 0:
            supported = [p for p in ecu.obd_pids if p % 0x20 != 0]
            return self._support_bitmap(pid, supported)
        if pid in ecu.obd_pids:
            return ecu.obd_pids[pid]
        return bytes(OBD_PID_LENGTHS.get(pid, 1))   # no data: zeros

    def _obd(self, ecu, mode, data):
        if mode == 0x01:                            # current data, up to 6 PIDs
            for pid in data[:6]:
                self._obd_reply(ecu, bytes([0x41, pid]) + self._pid_value(ecu, pid))
        elif mode == 0x02:                          # freeze frame data: (PID, frame) pairs
            for i in range(0, len(data) - 1, 2):
                pid, frame = data[i], data[i + 1]
                # Not simulated: no freeze frame data; every value is zero, so
                # PID $02 reports that no DTC caused a freeze frame.
                value = self._support_bitmap(pid, []) if pid % 0x20 == 0 else bytes(OBD_PID_LENGTHS.get(pid, 1))
                self._obd_reply(ecu, bytes([0x42, pid, frame]) + value)
        elif mode in (0x03, 0x07):                  # stored / pending DTCs
            # Not simulated: no DTCs in ecu.json; an all-zero message means no DTCs.
            self._obd_reply(ecu, bytes([mode + 0x40]) + bytes(6))
        elif mode == 0x04:                          # clear DTCs
            self._obd_reply(ecu, b"\x44")
        elif mode == 0x05:                          # O2 sensor monitoring: zeros
            tid = data[0] if data else 0
            self._obd_reply(ecu, bytes([0x45, tid, data[1] if len(data) > 1 else 0]) + bytes(1))
        elif mode in (0x06, 0x08):                  # on-board monitoring / control: zeros
            tid = data[0] if data else 0
            self._obd_reply(ecu, bytes([mode + 0x40, tid]) + bytes(5))
        elif mode == 0x09:                          # vehicle information
            self._obd_mode09(ecu, data[0] if data else 0)

    def _obd_mode09(self, ecu, info):
        vin, calid = ecu.vin, ecu.calibration_id
        if info == 0x00:
            supported = ([0x01, 0x02] if vin else []) + ([0x03, 0x04, 0x05, 0x06] if calid else [])
            self._obd_reply(ecu, b"\x49\x00\x01" + self._support_bitmap(0, supported))
        elif info == 0x03 and calid:                # number of CALID messages (4 per CALID)
            self._obd_reply(ecu, b"\x49\x03\x04")
        elif info == 0x04 and calid:                # CALID: 16 characters, zero padded, 4 messages
            padded = calid[:16].ljust(16, b"\x00")
            for n in range(4):
                self._obd_reply(ecu, bytes([0x49, 0x04, n + 1]) + padded[n * 4:n * 4 + 4])
        elif info == 0x05 and calid:                # number of CVN messages (1 per CALID)
            self._obd_reply(ecu, b"\x49\x05\x01")
        elif info == 0x06 and calid:                # CVN: 4 bytes
            # Not simulated: ecu.json has no calibration verification number.
            self._obd_reply(ecu, b"\x49\x06\x01" + bytes(4))
        elif info == 0x01 and vin:                  # number of VIN messages
            self._obd_reply(ecu, b"\x49\x01\x05")
        elif info == 0x02 and vin:                  # VIN: 5 messages of 4 bytes, padded in front
            padded = b"\x00\x00\x00" + vin.encode("ascii")
            for n in range(5):
                self._obd_reply(ecu, bytes([0x49, 0x02, n + 1]) + padded[n * 4:n * 4 + 4])
        else:
            # Other InfoTypes, or a VIN / calibration ID the module doesn't
            # have: one message of zeros (not simulated).
            self._obd_reply(ecu, bytes([0x49, info, 0x01]) + bytes(4))
