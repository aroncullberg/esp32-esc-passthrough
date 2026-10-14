#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
import importlib.util
from pathlib import Path
import struct
import unittest

spec = importlib.util.spec_from_file_location("dshot_control", Path(__file__).resolve().parents[1] / "tools/dshot_control.py")
control = importlib.util.module_from_spec(spec)
spec.loader.exec_module(control)


def frame(payload, error=False):
    body = bytes((len(payload), control.COMMAND)) + payload
    crc = 0
    for byte in body:
        crc ^= byte
    return b"$M" + (b"!" if error else b">") + body + bytes((crc,))


def status_payload(operation_error=0):
    return control.MAGIC + struct.pack("<BBHIII", 2, 0, 0, 0, operation_error, 0) + bytes((1, 0, 1, 1))


class Port:
    def __init__(self, response):
        self.response = bytearray(response)
        self.written = bytearray()

    def write(self, data):
        self.written.extend(data)
        return len(data)

    def flush(self):
        pass

    def read(self, size):
        del size
        if not self.response:
            return b""
        chunk = bytes(self.response[:1])
        del self.response[:1]
        return chunk


class ControlTests(unittest.TestCase):
    def test_framing_and_no_discovery(self):
        packet = control.request_packet("run", 75)
        self.assertEqual(packet[:5], b"$M<\x07\xf0")
        self.assertEqual(packet[5:12], b"DSH1\x02\x4b\x00")
        crc = 0
        for byte in packet[3:]:
            crc ^= byte
        self.assertEqual(crc, 0)
        port = Port(b"noise" + frame(status_payload()))
        result = control.Client(port).request("status")
        self.assertEqual(result["state"], "ready")
        self.assertEqual(result["last_end"], "normal_exit")
        self.assertEqual(bytes(port.written), control.request_packet("status"))

    def test_rejection_and_checksum(self):
        result = control.Client(Port(frame(status_payload(0x103), error=True))).request("run", 48)
        self.assertEqual(result["operation_error"], 0x103)
        bad_crc = bytearray(frame(status_payload()))
        bad_crc[-1] ^= 1
        with self.assertRaisesRegex(ValueError, "checksum"):
            control.Client(Port(bad_crc)).request("status")
        with self.assertRaisesRegex(ValueError, "unsupported"):
            control.Client(Port(frame(b"", error=True))).request("arm")

    def test_invalid_wire_format(self):
        with self.assertRaises(ValueError):
            control.decode_status(status_payload()[:-1])
        bad = bytearray(status_payload())
        bad[4] = 99
        with self.assertRaises(ValueError):
            control.decode_status(bad)


if __name__ == "__main__":
    unittest.main()
