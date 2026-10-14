# SPDX-License-Identifier: GPL-3.0-or-later
"""Check diagnostic probe framing and that its modes do not send ESC writes."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

sys.modules.setdefault("serial", types.SimpleNamespace(Serial=object))
spec = importlib.util.spec_from_file_location("probe_esc", Path(__file__).resolve().parents[1] / "tools/probe_esc.py")
probe_esc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe_esc)


class FakeProbe:
    fail_channel = None

    def __init__(self, port):
        self.calls = []
        self.closed = False

    def msp(self, command, payload=b""):
        self.calls.append(("msp", command, payload))
        if command == 245:
            return bytes([4])
        if command == 131:
            return bytes([0] * 6 + [4])
        if command == 253:
            return b"diagnostic\n"
        return b"test"

    def fourway(self, command, param=0, address=0):
        self.calls.append(("4way", command, param, address))
        if command == 0x37:
            if param == self.fail_channel:
                raise RuntimeError("ACK 0x0f")
            return bytes([6, 0x35, 2, 4])
        return b"interface"

    def diagnostics(self, channel):
        self.msp(253, bytes([channel]))

    def close(self, path):
        self.closed = True


class ProbeTest(unittest.TestCase):
    def test_reply_framing_skips_console_without_hiding_bad_checksum(self):
        class Port:
            def __init__(self, data):
                self.data = bytearray(data)

            def write(self, packet):
                pass

            def flush(self):
                pass

            def read(self, count):
                data = bytes(self.data[:count])
                del self.data[:count]
                return data

        probe = object.__new__(probe_esc.Probe)
        probe.event = lambda *args: None
        reply = b"$M>\x02\xfdok\xfb"
        probe.port = Port(b"rc=lost $prefix $M?ignored\n" + reply)
        self.assertEqual(probe.msp(253), b"ok")
        probe.port = Port(b"console\n" + reply[:-1] + b"\x00")
        with self.assertRaisesRegex(RuntimeError, "checksum"):
            probe.msp(253)

    def test_channel_query_checksum(self):
        probe = object.__new__(probe_esc.Probe)
        calls = []
        probe.exchange = lambda *args: calls.append(args) or b"ok"
        self.assertEqual(probe.msp(253, bytes([3])), b"ok")
        self.assertEqual(calls[0][0], b"$M<\x01\xfd\x03\xff")
        with self.assertRaises(ValueError):
            probe.fourway(0x3b)

    def run_probe(self, option, fail_channel=None):
        probe = FakeProbe("unused")
        probe.fail_channel = fail_channel
        args = ["probe_esc.py", "--port", "unused", option]
        if option == "--identify-only":
            args.append("--all-channels")
        with patch.object(sys, "argv", args), patch.object(probe_esc, "Probe", return_value=probe), contextlib.redirect_stdout(io.StringIO()):
            if fail_channel is None:
                probe_esc.main()
            else:
                with self.assertRaisesRegex(RuntimeError, "identification failed"):
                    probe_esc.main()
        self.assertTrue(probe.closed)
        return probe.calls

    def test_identification_success_and_failure_exit_without_settings(self):
        for failure in (None, 1):
            calls = self.run_probe("--identify-only", failure)
            self.assertEqual([c[2] for c in calls if c[:2] == ("4way", 0x37)], [0, 1, 2, 3])
            self.assertEqual([c[2] for c in calls if c[:2] == ("msp", 253)], [bytes([c]) for c in range(4)])
            self.assertTrue(all(c[1] in (0x31, 0x32, 0x33, 0x37, 0x34) for c in calls if c[0] == "4way"))
            self.assertEqual(calls[-2:], [("4way", 0x34, 0, 0), ("msp", 1, b"")])

    def test_diagnostic_read_does_not_enter_or_acquire(self):
        calls = self.run_probe("--diagnostics-only")
        self.assertEqual([c[1] for c in calls], [5, 131, 253, 253, 253, 253])
        self.assertTrue(all(c[0] == "msp" for c in calls))


if __name__ == "__main__":
    unittest.main()
