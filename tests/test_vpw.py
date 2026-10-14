"""Unit tests for the ECU simulator: sim/vpw.py against sim/ecu.json, and the
link-state tracking and console colouring of sim/ecu_sim.py.  No port or
connection is opened.

    python tests/test_vpw.py
"""
import json
import os
import sys
import unittest

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import vpw  # noqa: E402

TESTER = 0xF0   # Tech2


def frame(text):
    return bytes.fromhex(text.replace(" ", ""))


class VpwTest(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.heartbeats = []
        self.ecus = {e.addr: e for e in vpw.load_ecus(os.path.join(ROOT, "sim", "ecu.json"))}
        self.logged = []
        self.net = vpw.VpwNetwork(self.ecus.values(), self.sent.append, self.logged.append,
                                  heartbeat_send=self.heartbeats.append)

    def request(self, text, now=0.0):
        self.sent.clear()
        self.net.handle(frame(text), now)
        return [f.hex(" ").upper() for f in self.sent]

    # ---- J2190 ------------------------------------------------------------

    def test_read_block(self):
        b = self.ecus[0x10].blocks[0x01].hex(" ").upper()
        self.assertEqual(self.request("6C 10 F0 3C 01"), ["6C F0 10 7C 01 " + b])

    def test_read_unknown_block_is_out_of_range(self):
        self.assertEqual(self.request("6C 10 F0 3C EE"), ["6C F0 10 7F 3C EE 31"])

    def test_write_then_read_block(self):
        self.assertEqual(self.request("6C 40 F0 3B 90 12 34"), ["6C F0 40 7B 90"])
        self.assertEqual(self.request("6C 40 F0 3C 90"), ["6C F0 40 7C 90 12 34"])

    def test_read_pid(self):
        v = self.ecus[0x10].pids[0x0005].hex(" ").upper()
        self.assertEqual(self.request("6C 10 F0 22 00 05"), ["6C F0 10 62 00 05 " + v])
        self.assertEqual(self.request("6C 10 F0 22 77 77"), ["6C F0 10 7F 22 77 77 31"])

    def test_unsupported_mode_physical(self):
        # the PCM does not list $19
        self.assertEqual(self.request("6C 10 F0 19 FF 00"), ["6C F0 10 7F 19 FF 00 11"])

    def test_unsupported_mode_broadcast_is_silent(self):
        replies = self.request("6C FE F0 14")
        listing = [a for a, e in self.ecus.items() if 0x14 in e.commands]
        self.assertLess(len(listing), len(self.ecus))
        self.assertEqual(len(replies), len(listing))
        self.assertTrue(all(r.startswith("6C F0 ") and r.endswith(" 54") for r in replies))
        self.assertNotIn("6C F0 10 54", replies)   # PCM does not list $14

    def test_every_module_supports_return_to_normal(self):
        self.assertTrue(all(0x20 in e.commands for e in self.ecus.values()))
        self.assertEqual(self.request("6C 10 F1 20"), ["6C F1 10 60"])   # Tech2Win's request to the PCM
        self.assertEqual(len(self.request("6C FE F0 20")), len(self.ecus))

    def test_tester_present_no_response(self):
        self.assertEqual(self.request("6C FE F0 3F"), [])

    def test_security_access_accepts_any_key(self):
        seed = self.request("6C 40 F0 27 01")[0].split()
        self.assertEqual(seed[3:5], ["67", "01"])
        self.assertNotEqual(seed[5:7], ["00", "00"])
        self.assertEqual(self.request("6C 40 F0 27 02 BE EF"), ["6C F0 40 67 02 34"])
        self.assertEqual(self.request("6C 40 F0 27 01"), ["6C F0 40 67 01 00 00"])

    def test_dpid_once_and_streaming(self):
        d = self.ecus[0x10].dpids[0xF5].hex(" ").upper()
        self.assertEqual(self.request("6C 10 F0 2A 01 F5"), ["6C F0 10 6A F5 " + d])
        self.assertEqual(self.request("6C 10 F0 2A 04 F5", now=10.0), ["6C F0 10 6A F5 " + d])
        self.sent.clear()
        self.net.poll(10.06)
        self.assertEqual(len(self.sent), 1)
        self.request("6C 10 F0 2A 00")
        self.sent.clear()
        self.net.poll(20.0)
        self.assertEqual(self.sent, [])

    def test_dpid_rate_intervals(self):
        expected = {0x02: 1.0, 0x03: 0.2, 0x04: 0.05, 0x12: 0.05, 0x13: 0.05, 0x14: 0.05,
                    0x05: 0.2, 0x10: 0.2, 0x24: 0.2, 0xFF: 0.2}
        for rate, interval in expected.items():
            self.request("6C 10 F0 2A %02X F5" % rate, now=100.0)
            self.assertAlmostEqual(self.net.streams[(0x10, TESTER, 0xF5)][0], interval, msg="rate %02X" % rate)
            self.sent.clear()
            self.net.poll(100.0 + interval - 0.001)       # not yet due
            self.assertEqual(self.sent, [], "rate %02X sent early" % rate)
            self.net.poll(100.0 + interval)               # due
            self.assertEqual(len(self.sent), 1, "rate %02X not sent on time" % rate)
            self.request("6C 10 F0 2A 00")

    def test_tech2win_dynamic_dpid_exchange(self):
        # the exact DRR exchange from Tech2Win
        self.assertEqual(self.request("6C 89 F1 20"), ["6C F1 89 60"])
        self.assertEqual(self.request("6C 89 F1 2C FE 4D FD 0D FF FF"), ["6C F1 89 6C FE"])
        pid = self.ecus[0x89].pids[0xFD0D]                           # 5 bytes at position 1
        sixth = self.ecus[0x89].dpids[0xFE][5:6]
        expected = "6C F1 89 6A FE " + (pid + sixth).hex(" ").upper()
        self.assertEqual(self.request("6C 89 F1 2A 13 FE FF FF FF", now=50.0), [expected])
        self.sent.clear()
        self.net.poll(50.05)                                         # rate 13: every 50 ms
        self.assertEqual([f.hex(" ").upper() for f in self.sent], [expected])

    def test_tech2win_two_pid_definition(self):
        # the exact DRR sequence from Tech2Win's data display
        drr = self.ecus[0x89]
        self.assertEqual(self.request("6C 89 F1 2C FE 4A 21 A4 21 B1"), ["6C F1 89 6C FE"])
        self.assertEqual(self.request("6C 89 F1 2C FE 59 22 69 FF FF"), ["6C F1 89 6C FE"])
        self.assertEqual(self.request("6C 89 F1 2C FD 49 FD 16 FF FF"), ["6C F1 89 6C FD"])
        fe = bytes.fromhex(self.request("6C 89 F1 2A 01 FE")[0].replace(" ", ""))[5:]
        self.assertEqual(fe[:3], drr.pids[0x21A4] + drr.pids[0x21B1] + drr.pids[0x2269])
        self.assertEqual(fe[3:], drr.dpids[0xFE][3:])
        fd = bytes.fromhex(self.request("6C 89 F1 2A 01 FD")[0].replace(" ", ""))[5:]
        self.assertEqual(fd[:1], drr.pids[0xFD16])
        # both packets in one request
        self.assertEqual(len(self.request("6C 89 F1 2A 13 FE FD FF FF")), 2)

    def test_dynamic_dpid_built_from_several_definitions(self):
        pcm = self.ecus[0x10]
        # 4A = PID, position 1, 2 bytes; 59 = PID, position 3, 1 byte
        self.assertEqual(self.request("6C 10 F0 2C FE 4A 00 0C"), ["6C F0 10 6C FE"])
        self.assertEqual(self.request("6C 10 F0 2C FE 59 00 05"), ["6C F0 10 6C FE"])
        value = bytes.fromhex(self.request("6C 10 F0 2A 01 FE")[0].replace(" ", ""))[5:]
        self.assertEqual(value[0:2], pcm.pids[0x000C])
        self.assertEqual(value[2:3], pcm.pids[0x0005])
        self.assertEqual(value[3:], pcm.dpids[0xFE][3:])                  # untouched bytes kept
        # the packet follows the PID's current value
        pcm.pids[0x0005] = b"\x77"
        value = bytes.fromhex(self.request("6C 10 F0 2A 01 FE")[0].replace(" ", ""))[5:]
        self.assertEqual(value[2], 0x77)
        # a definition over the same bytes replaces the old one
        self.request("6C 10 F0 2C FE 4A 00 05")                         # 1 byte value, padded to 2
        value = bytes.fromhex(self.request("6C 10 F0 2A 01 FE")[0].replace(" ", ""))[5:]
        self.assertEqual(value[0:2], b"\x77\x00")

    def test_tech2win_pcm_definition_with_unknown_pids(self):
        # the exact PCM sequence from Tech2Win; the PCM has no data for 1321/1322
        pcm = self.ecus[0x10]
        self.assertNotIn(0x1321, pcm.pids)
        self.assertNotIn(0x1322, pcm.pids)
        self.assertEqual(self.request("6C 10 F1 10 01"), ["6C F1 10 50 01"])
        for dpid in ("F7", "F8", "F9", "FA", "FB", "FC", "FD", "FE"):
            self.assertEqual(self.request("6C 10 F1 2C %s 4A 00 0C FF FF" % dpid), ["6C F1 10 6C " + dpid])
        self.assertEqual(self.request("6C 10 F1 2C F1 4A 13 21 13 22"), ["6C F1 10 6C F1"])
        self.assertEqual(self.logged, ["  note: PCM has no data for PID 1321, 1322; DPID F1 uses 00"])
        value = bytes.fromhex(self.request("6C 10 F1 2A 01 F1")[0].replace(" ", ""))[5:]
        self.assertEqual(value[0:2], b"\x00\x00")                            # one zero byte per PID

    def test_tech2win_two_stream_requests_both_keep_streaming(self):
        # Tech2Win's PCM data display starts two groups with separate requests
        self.assertEqual(len(self.request("6C 10 F1 2A 14 F6 F7 F9 FA", now=300.0)), 4)
        self.assertEqual(len(self.request("6C 10 F1 2A 24 FB FC FD FE", now=300.0)), 4)
        self.assertEqual(len(self.request("6C 10 F1 2A 01 F0 FF FF FF", now=300.0)), 1)   # one-shot
        streamed = set()
        for i in range(1, 11):                                                 # 2 seconds
            self.sent.clear()
            self.net.poll(300.0 + i * 0.2)
            streamed.update(f[4] for f in self.sent)
        self.assertEqual(streamed, {0xF6, 0xF7, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE})
        # stopping listed DPIDs leaves the others running; 2A 00 alone stops the rest
        self.request("6C 10 F1 2A 00 F6 F7")
        self.sent.clear()
        self.net.poll(310.0)
        self.assertEqual(sorted(f[4] for f in self.sent), [0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE])
        self.request("6C 10 F1 2A 00")
        self.sent.clear()
        self.net.poll(320.0)
        self.assertEqual(self.sent, [])

    def test_dynamic_dpid_errors(self):
        self.assertEqual(self.request("6C 10 F0 2C FE 7F 00 0C")[0].split()[-1], "12")             # 7 bytes at pos 7
        self.assertEqual(self.request("6C 10 F0 2C FE FF FF")[0].split()[-1], "12")                # no definition

    def test_dynamic_dpids_are_per_module(self):
        pcm, drr = self.ecus[0x10], self.ecus[0x89]
        # same DPID number FE, different definitions on two modules
        self.request("6C 10 F0 2C FE 4A 00 0C")      # PCM: PID 000C at position 1
        self.request("6C 89 F0 2C FE 4D FD 0D")      # DRR: PID FD0D at position 1

        def dpid(addr):
            return bytes.fromhex(self.request("6C %02X F0 2A 01 FE" % addr)[0].replace(" ", ""))[5:]

        self.assertEqual(dpid(0x10)[:2], pcm.pids[0x000C])
        self.assertEqual(dpid(0x89)[:5], drr.pids[0xFD0D])
        # streams are per module too
        self.request("6C 10 F0 2A 13 FE", now=200.0)
        self.request("6C 89 F0 2A 13 FE", now=200.0)
        self.sent.clear()
        self.net.poll(200.05)
        self.assertEqual(sorted(f[2] for f in self.sent), [0x10, 0x89])
        # return to normal on the PCM leaves the DRR's definition and stream alone
        self.request("6C 10 F0 20")
        self.assertEqual(dpid(0x10), pcm.dpids[0xFE])
        self.assertEqual(dpid(0x89)[:5], drr.pids[0xFD0D])
        self.sent.clear()
        self.net.poll(200.11)                         # next 50 ms tick
        self.assertEqual([f[2] for f in self.sent], [0x89])

    def test_return_to_normal_clears_dynamic_dpids(self):
        self.request("6C 10 F0 2C FE 4A 00 0C")
        self.request("6C 10 F0 20")
        value = bytes.fromhex(self.request("6C 10 F0 2A 01 FE")[0].replace(" ", ""))[5:]
        self.assertEqual(value, self.ecus[0x10].dpids[0xFE])

    # ---- return to normal operation ---------------------------------------

    STREAM_DPID = {0x10: 0xF5, 0x40: 0x01}       # PCM, BCM

    def start_session(self, addr):
        """Unlocks the module, streams one of its DPIDs and defines DPID $FE."""
        self.request("6C %02X F0 27 01" % addr)
        self.assertEqual(self.request("6C %02X F0 27 02 BE EF" % addr), ["6C F0 %02X 67 02 34" % addr])
        self.request("6C %02X F0 2A 04 %02X" % (addr, self.STREAM_DPID[addr]))
        self.assertEqual(self.request("6C %02X F0 2C FE 4A 00 05" % addr), ["6C F0 %02X 6C FE" % addr])
        self.assertTrue(self.in_session(addr))

    def in_session(self, addr):
        """(unlocked, has $2C definitions, streaming) for one module."""
        ecu = self.ecus[addr]
        return ecu.unlocked, bool(ecu.dpid_defs), any(k[0] == addr for k in self.net.streams)

    def assert_normal(self, addr):
        self.assertEqual(self.in_session(addr), (False, False, False), "module %02X" % addr)
        seed = self.request("6C %02X F0 27 01" % addr)[0].split()
        self.assertNotEqual(seed[5:7], ["00", "00"])             # locked: a real seed again

    def test_return_to_normal_physical(self):
        self.start_session(0x10)
        self.start_session(0x40)
        self.assertEqual(self.request("6C 10 F0 20"), ["6C F0 10 60"])
        self.assert_normal(0x10)
        self.assertEqual(self.in_session(0x40), (True, True, True))   # not addressed: unaffected
        self.sent.clear()
        self.net.poll(1000.0)
        self.assertEqual([f[2] for f in self.sent], [0x40])           # only the BCM still streams

    def test_return_to_normal_all_nodes(self):
        self.start_session(0x10)
        self.start_session(0x40)
        self.assertIn("6C F0 40 60", self.request("6C FE F0 20"))
        self.assert_normal(0x10)
        self.assert_normal(0x40)
        self.assertEqual(self.net.streams, {})

    def test_return_all_to_normal_keeps_block_writes(self):
        self.start_session(0x10)
        self.start_session(0x40)
        self.request("6C 40 F0 3B 90 12 34")
        self.net.return_all_to_normal()
        self.assert_normal(0x10)
        self.assert_normal(0x40)
        self.assertEqual(self.request("6C 40 F0 3C 90"), ["6C F0 40 7C 90 12 34"])

    def test_dtcs_none(self):
        self.assertEqual(self.request("6C 40 F0 19 FF FF 00"), ["6C F0 40 59 00 00 FF"])

    def test_gm_specific_acknowledged(self):
        self.assertEqual(self.request("6C 10 F0 A0"), ["6C F0 10 E0"])
        self.assertEqual(self.request("6C 40 F0 AE 01 02"), ["6C F0 40 EE 01"])

    def test_unknown_module_ignored(self):
        self.assertEqual(self.request("6C 55 F0 3C 01"), [])

    # ---- node-alive heartbeat ---------------------------------------------

    def test_j1850_crc(self):
        self.assertEqual(vpw.j1850_crc(b"123456789"), 0x4B)    # CRC-8/SAE-J1850 check value

    def test_heartbeat_frame(self):
        frame = vpw.heartbeat_frame(0x10)
        self.assertEqual(frame[:4], bytes([0x08, 0xFF, 0x10, 0x03]))
        if vpw.HEARTBEAT_CRC:
            self.assertEqual(frame[4:], bytes([vpw.j1850_crc(frame[:4])]))
        else:
            self.assertEqual(len(frame), 4)       # no CRC, like the other responses

    def test_heartbeat_every_two_seconds_per_module(self):
        times = {}
        t = 100.0
        while t < 110.0:                       # 10 s of polling every 10 ms
            self.heartbeats.clear()
            self.net.poll(t)
            for f in self.heartbeats:
                times.setdefault(f[2], []).append(t)
            t = round(t + 0.01, 2)
        self.assertEqual(set(times), set(self.ecus))
        for addr, ts in times.items():
            self.assertEqual(len(ts), 5, "module %02X" % addr)
            gaps = [round(b - a, 2) for a, b in zip(ts, ts[1:])]
            self.assertTrue(all(1.99 <= g <= 2.01 for g in gaps), "module %02X gaps %s" % (addr, gaps))
        firsts = sorted(ts[0] for ts in times.values())   # spread out, not one burst
        self.assertGreater(firsts[-1] - firsts[0], 1.5)
        self.assertEqual(self.sent, [])                   # heartbeats use their own sender

    # ---- J1979 ------------------------------------------------------------

    def test_obd_pid_values(self):
        coolant = self.ecus[0x10].pids[0x0005].hex(" ").upper()
        self.assertEqual(self.request("68 6A F1 01 05"), ["48 6B 10 41 05 " + coolant])
        self.assertEqual(self.request("68 6A F1 01 2F"), ["48 6B 10 41 2F 00"])   # no data: zeros

    def test_obd_supported_pids(self):
        bitmap = bytes.fromhex(self.request("68 6A F1 01 00")[0].split(" ", 5)[5].replace(" ", ""))
        bits = int.from_bytes(bitmap, "big")
        self.assertTrue(bits & (1 << (32 - 0x05)))    # PID $05 supported
        self.assertFalse(bits & (1 << (32 - 0x02)))   # PID $02 not in ecu.json

    def test_obd_no_dtcs(self):
        self.assertEqual(self.request("68 6A F1 03"), ["48 6B 10 43 00 00 00 00 00 00"])

    def test_obd_vin(self):
        frames = self.request("68 6A F1 09 02")
        self.assertEqual(len(frames), 5)
        vin = b"".join(bytes.fromhex("".join(f.split()[6:])) for f in frames).lstrip(b"\x00")
        self.assertEqual(vin.decode(), self.ecus[0x10].vin)

    def test_obd_calibration_id_and_cvn(self):
        # Tech2Win's VIN screen: 09 00, 09 02, then 09 05 / 09 04 / 09 06
        self.assertEqual(self.request("68 6A F1 09 00"), ["48 6B 10 49 00 01 FC 00 00 00"])  # 01-06
        self.assertEqual(self.request("68 6A F1 09 03"), ["48 6B 10 49 03 04"])
        self.assertEqual(self.request("68 6A F1 09 05"), ["48 6B 10 49 05 01"])
        self.assertEqual(self.request("68 6A F1 09 04"), [
            "48 6B 10 49 04 01 31 32 35 38",        # "1258"
            "48 6B 10 49 04 02 37 36 37 32",        # "7672" = block 08 00C01298
            "48 6B 10 49 04 03 00 00 00 00",
            "48 6B 10 49 04 04 00 00 00 00"])
        self.assertEqual(self.request("68 6A F1 09 06"), ["48 6B 10 49 06 01 00 00 00 00"])


class HeartbeatLinkStateTest(unittest.TestCase):
    """The simulator only sends heartbeats while the DLL has a VPW logical link
    on the active bus (it tracks OPEN / CLOSE / PROTOCOL / HELLO / BYE)."""

    class FakeLink:
        name = "fake"

        def __init__(self):
            self.lines = []

        def write(self, data):
            self.lines.append(data.decode().strip())

    def setUp(self):
        import ecu_sim
        self.link = self.FakeLink()
        self.sim = ecu_sim.Simulator(self.link, True, vpw.load_ecus(os.path.join(ROOT, "sim", "ecu.json")))
        self.t = 0.0

    def heartbeats_in_next_2s(self, *lines):
        for line in lines:
            self.sim.handle(line)
        self.link.lines.clear()
        for _ in range(201):
            self.sim.vpw.poll(self.t)
            self.t = round(self.t + 0.01, 2)
        return sum(1 for l in self.link.lines if l.startswith("RX VPW 08 FF"))

    def test_heartbeats_follow_vpw_link(self):
        modules = len(self.sim.vpw.ecus)
        self.assertEqual(self.heartbeats_in_next_2s(), 0)                               # nothing open yet
        self.assertEqual(self.heartbeats_in_next_2s("HELLO dpdu-bridge-c 1.0"), 0)
        self.assertEqual(self.heartbeats_in_next_2s("OPEN 02000002 HSCAN PROTOCOL=16 RAW=0 CHECKSUM=0"), 0)
        self.assertGreaterEqual(self.heartbeats_in_next_2s("OPEN 02000003 VPW PROTOCOL=40 RAW=1 CHECKSUM=0"),
                                modules)                                                # VPW link: every module
        self.assertEqual(self.heartbeats_in_next_2s("PROTOCOL HSCAN"), 0)                # active bus switched
        self.assertGreaterEqual(self.heartbeats_in_next_2s("PROTOCOL VPW"), modules)     # and back
        self.assertEqual(self.heartbeats_in_next_2s("CLOSE 02000003"), 0)                # VPW link closed
        self.assertGreaterEqual(self.heartbeats_in_next_2s("OPEN 02000004 VPW PROTOCOL=40"), modules)
        self.assertEqual(self.heartbeats_in_next_2s("BYE"), 0)                           # module disconnected

    def test_echo_lines_do_not_change_state(self):
        self.sim.handle("OPEN 02000003 VPW PROTOCOL=40")
        self.assertGreater(self.heartbeats_in_next_2s("# COM8: RX VPW 6C F1 10 60"), 0)


class StreamColorTest(unittest.TestCase):
    """While repeating DPIDs stream, other TX/RX lines are logged in green."""

    def setUp(self):
        import ecu_sim
        self.ecu_sim = ecu_sim
        self.sim = ecu_sim.Simulator(HeartbeatLinkStateTest.FakeLink(), False,
                                     vpw.load_ecus(os.path.join(ROOT, "sim", "ecu.json")))
        self.sim.color = True
        self.logged = []
        self.sim.log = self.logged.append

    def tx(self, line):
        self.logged.clear()
        self.sim.log_traffic(">> ", line)
        self.sim.handle(line)
        return self.logged

    def green(self, lines):
        return [l.startswith(self.ecu_sim.GREEN) for l in lines]

    def test_green_only_while_streaming(self):
        self.assertEqual(self.green(self.tx("TX VPW 6C 10 F1 20")), [False, False])          # not streaming
        self.assertEqual(self.green(self.tx("TX VPW 6C 10 F1 2A 14 F6 F7")), [False, False, False])
        self.sim.vpw.poll(10 ** 9)                                                           # stream tick
        self.assertTrue(self.logged and not any(self.green(self.logged)))                    # streamed 6A
        self.assertEqual(self.green(self.tx("TX VPW 6C 10 F1 22 00 05")), [True, True])      # request + reply
        self.assertEqual(self.green(self.tx("TX VPW 6C 10 F1 2C F6 4A 00 0C FF FF")), [False, False])
        self.assertEqual(self.green(self.tx("TX VPW 6C 10 F1 2C F6 7F 00 0C")), [False, False])  # 7F 2C ..
        self.assertEqual(self.green(self.tx("# COM8: RX VPW 6C F1 10 50 01")), [True])       # other client
        self.assertEqual(self.green(self.tx("TX VPW 6C 10 F1 2A 00")), [False])              # stop: no reply
        self.assertEqual(self.green(self.tx("TX VPW 6C 10 F1 22 00 05")), [False, False])

    def test_space_toggles_dpid_logging_while_streaming(self):
        self.sim.on_key(" ")                                          # not streaming: no effect
        self.assertFalse(self.sim.hide_dpids)
        self.tx("TX VPW 6C 10 F1 2A 14 F6 F7")
        self.sim.on_key(" ")
        self.assertTrue(self.sim.hide_dpids)
        self.logged.clear()
        self.sim.vpw.poll(10 ** 9)
        self.assertEqual(self.logged, [])                             # streamed 6A hidden
        self.assertEqual(self.tx("TX VPW 6C 10 F1 2C F6 4A 00 0C FF FF"), [])
        self.assertEqual(self.green(self.tx("TX VPW 6C 10 F1 22 00 05")), [True, True])  # others still shown
        self.sim.on_key(" ")                                          # shown again
        self.logged.clear()
        self.sim.vpw.poll(2 * 10 ** 9)
        self.assertEqual(len(self.logged), 2)
        self.sim.on_key(" ")                                          # hide, then stop streaming
        self.assertEqual(self.tx("TX VPW 6C 10 F1 2A 00"), [])
        self.assertEqual(len(self.tx("TX VPW 6C 10 F1 2A 01 F6")), 2)  # not streaming: logged

    def test_is_dpid_traffic(self):
        f = self.ecu_sim.is_dpid_traffic
        self.assertTrue(f("RX VPW 6C F1 10 6A F6 00 00 00 00 00 00"))
        self.assertTrue(f("RX VPW 6C F1 10 7F 2A 14 F6 31"))
        self.assertFalse(f("RX VPW 6C F1 10 62 00 05 40"))
        self.assertFalse(f("TX HSCAN 7E0 02 01 00"))


class SessionEndTest(unittest.TestCase):
    """When Tech2Win disconnects (BYE, or the TCP connection ends), every
    module returns to normal operation."""

    DISCONNECTED = "Tech2Win disconnected: all modules returned to normal"

    class ClosedLink(HeartbeatLinkStateTest.FakeLink):
        def read(self, n):
            return None                         # the DLL closed the connection

    def setUp(self):
        import ecu_sim
        self.link = self.ClosedLink()
        self.sim = ecu_sim.Simulator(self.link, False, vpw.load_ecus(os.path.join(ROOT, "sim", "ecu.json")))
        self.logged = []
        self.sim.log = self.logged.append
        self.pcm = self.sim.vpw.ecus[0x10]
        self.sim.handle("HELLO dpdu-bridge-c 1.0")
        self.sim.handle("OPEN 02000002 VPW PROTOCOL=40 RAW=1 CHECKSUM=0")
        for line in ("TX VPW 6C 10 F1 27 01", "TX VPW 6C 10 F1 27 02 BE EF",
                     "TX VPW 6C 10 F1 2A 14 F5", "TX VPW 6C 10 F1 2C FE 4A 00 05"):
            self.sim.handle(line)
        self.assertTrue(self.pcm.unlocked and self.pcm.dpid_defs and self.sim.vpw.streams)

    def assert_normal(self):
        self.assertFalse(self.pcm.unlocked)
        self.assertEqual(self.pcm.dpid_defs, {})
        self.assertEqual(self.sim.vpw.streams, {})

    def test_bye_returns_all_modules_to_normal(self):
        self.sim.handle("BYE")
        self.assert_normal()
        self.assertEqual(self.logged.count(self.DISCONNECTED), 1)
        self.sim.run()                          # the TCP close after BYE: not noted again
        self.assertEqual(self.logged.count(self.DISCONNECTED), 1)
        self.sim.handle("HELLO dpdu-bridge-c 1.0")
        self.sim.handle("BYE")                  # next session
        self.assertEqual(self.logged.count(self.DISCONNECTED), 2)

    def test_connection_end_returns_all_modules_to_normal(self):
        self.sim.run()                          # connection closed without BYE
        self.assert_normal()
        self.assertEqual(self.logged[-2:], ["connection closed", self.DISCONNECTED])


class IniTest(unittest.TestCase):
    """The simulator reads the driver's ini like GetPrivateProfileString:
    section and key names are case-insensitive."""

    def link(self, text, use="auto"):
        import contextlib
        import io
        import tempfile
        from unittest import mock
        import ecu_sim
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "dpdu-bridge.ini")
            with open(path, "w", encoding="latin-1") as f:
                f.write(text)
            with mock.patch.object(ecu_sim, "com0com_partner", lambda port: "CNCA" + port[3:]), \
                    contextlib.redirect_stdout(io.StringIO()):
                return ecu_sim.link_from_ini(path, use)

    def test_section_names_case_insensitive(self):
        self.assertEqual(self.link("[serial]\nport=COM8\n"), ("serial", "CNCA8"))
        self.assertEqual(self.link("[SERIAL]\nPORT=COM8\n[tcp]\nPort=5000\n"), ("serial", "CNCA8"))
        self.assertEqual(self.link("[TCP]\nPORT=5000\nbind=::\n"), ("tcp", "[::1]:5000"))
        self.assertEqual(self.link("[Serial]\nPort=COM8\n[TCP]\nPort=5000\n", "tcp"), ("tcp", "127.0.0.1:5000"))


class VinTest(unittest.TestCase):
    """The top-level "VIN" in ecu.json fills the vA..vQ placeholders."""

    def setUp(self):
        with open(os.path.join(ROOT, "sim", "ecu.json"), encoding="utf-8") as f:
            self.vin = json.load(f)["VIN"]
        self.ecus = {e.addr: e for e in vpw.load_ecus(os.path.join(ROOT, "sim", "ecu.json"))}

    def test_vin_check_digit(self):
        translit = {**{str(d): d for d in range(10)}, **dict(zip("ABCDEFGH", range(1, 9))),
                    **dict(zip("JKLMN", range(1, 6))), "P": 7, "R": 9, **dict(zip("STUVWXYZ", range(2, 10)))}
        weights = [8, 7, 6, 5, 4, 3, 2, 10, 0, 9, 8, 7, 6, 5, 4, 3, 2]
        total = sum(translit[c] * w for c, w in zip(self.vin, weights))
        self.assertEqual(self.vin[8], "X" if total % 11 == 10 else str(total % 11))

    def test_vin_blocks(self):
        for addr in (0x10, 0x40, 0x60):                       # PCM, BCM, IPC
            b = self.ecus[addr].blocks
            self.assertEqual(b[1] + b[2] + b[3], b"\x00" + self.vin.encode(), "module %02X" % addr)
        rse = self.ecus[0x85].blocks                         # RSE keeps its FF first character
        self.assertEqual(rse[1] + rse[2] + rse[3], b"\x00\xFF" + self.vin[1:].encode())
        for addr in (0x80, 0x97):                             # RADIO, VCIM stay blank
            self.assertEqual(self.ecus[addr].blocks[1], bytes(6))

    def test_obd_vin_is_json_vin(self):
        self.assertEqual(self.ecus[0x10].vin, self.vin)

    def test_placeholders(self):
        self.assertEqual(vpw.expand_vin("00vAvBvCvDvE", "1GNFK16ZX4J123456", "t"), "0031474E464B")
        # vI = character 9 ('X'), vK = 11 ('J'), vQ = 17 ('6'); case-insensitive
        self.assertEqual(vpw.expand_vin("VivkvQ", "1GNFK16ZX4J123456", "t"), "584A36")
        with self.assertRaises(ValueError):
            vpw.expand_vin("00vA", None, "t")

    def test_nodes_format(self):
        with open(os.path.join(ROOT, "sim", "ecu.json"), encoding="utf-8") as f:
            cfg = json.load(f)
        self.assertEqual(set(cfg), {"VIN", "nodes"})
        self.assertEqual([n["id"] for n in cfg["nodes"]], ["%02X" % e for e in self.ecus])

    @staticmethod
    def load(cfg):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "ecu.json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump(cfg, f)
            return vpw.load_ecus(path)

    def test_duplicate_id_rejected(self):
        with self.assertRaises(ValueError):
            self.load({"nodes": [{"id": "10"}, {"id": "10"}]})

    def test_invalid_files_rejected(self):
        bad = [
            [],                                                             # not an object
            {"nodes": {}},                                                  # nodes not an array
            {"VIN": "TOO SHORT", "nodes": []},
            {"nodes": ["10"]},                                              # node not an object
            {"nodes": [{"id": "1G"}]},                                      # id not hex
            {"nodes": [{"id": "100"}]},                                     # id out of range
            {"nodes": [{"id": "10", "supportedCommands": "22"}]},
            {"nodes": [{"id": "10", "supportedCommands": ["XX"]}]},
            {"nodes": [{"id": "10", "data": []}]},
            {"nodes": [{"id": "10", "data": {"pids": []}}]},
            {"nodes": [{"id": "10", "data": {"pids": {"0005": 60}}}]},      # value not a string
            {"nodes": [{"id": "10", "data": {"pids": {"0005": "3G"}}}]},    # value not hex
            {"nodes": [{"id": "10", "data": {"blocks": {"100": "00"}}}]},   # key out of range
        ]
        for cfg in bad:
            with self.subTest(cfg=cfg), self.assertRaises(ValueError):
                self.load(cfg)
        ecu, = self.load({"VIN": self.vin, "nodes": [{"id": "10", "data": {"blocks": {"01": "00vA"}}}]})
        self.assertEqual(ecu.blocks[1], b"\x00" + self.vin[:1].encode())


if __name__ == "__main__":
    unittest.main(verbosity=2)
