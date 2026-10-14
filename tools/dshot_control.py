#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Control examples/dshot over native USB without MSP discovery or a text console."""
import argparse
import json
import struct
import sys
import time

COMMAND = 240
MAGIC = b"DSH1"
OPERATIONS = {"status": 0, "arm": 1, "run": 2, "stop": 3}
STATES = ("stopped", "arming", "ready", "running", "configuration", "fault")
REASONS = ("none", "normal_exit", "transport_loss", "timeout", "setup_failure", "reply_failure", "shutdown")


def request_packet(operation, value=0):
    payload = MAGIC + struct.pack("<BH", OPERATIONS[operation], value)
    body = bytes((len(payload), COMMAND)) + payload
    checksum = 0
    for byte in body:
        checksum ^= byte
    return b"$M<" + body + bytes((checksum,))


def decode_status(payload):
    if len(payload) != 24 or payload[:4] != MAGIC or payload[23] != 1:
        raise ValueError("Unexpected firmware reply; flash examples/dshot with the matching control format")
    state, held, throttle, remaining, operation_error, motor_error = struct.unpack_from("<BBHIii", payload, 4)
    if state >= len(STATES) or payload[20] >= len(REASONS):
        raise ValueError("Invalid state or session reason")
    return {"state": STATES[state], "session_held": bool(held), "raw_throttle": throttle,
            "arming_remaining_ms": remaining, "operation_error": operation_error,
            "motor_error": motor_error, "last_end": REASONS[payload[20]],
            "needs_reboot_mask": payload[21],
            "io_released": bool(payload[22]) if payload[20] else None}


class Client:
    def __init__(self, port, timeout=2.0):
        self.port = port
        self.timeout = timeout
        self.buffer = bytearray()

    def request(self, operation, value=0):
        packet = request_packet(operation, value)
        if self.port.write(packet) != len(packet):
            raise OSError("Incomplete USB write")
        self.port.flush()
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            self.buffer.extend(self.port.read(264))
            while True:
                start = self.buffer.find(b"$M")
                if start < 0:
                    # Preserve a possible split '$M' prefix.
                    self.buffer[:] = b"$" if self.buffer.endswith(b"$") else b""
                    break
                if start:
                    del self.buffer[:start]
                if len(self.buffer) < 6:
                    break
                if self.buffer[2] not in (ord(">"), ord("!")):
                    del self.buffer[0]
                    continue
                size = self.buffer[3] + 6
                if len(self.buffer) < size:
                    break
                frame = bytes(self.buffer[:size])
                del self.buffer[:size]
                checksum = 0
                for byte in frame[3:-1]:
                    checksum ^= byte
                if checksum != frame[-1]:
                    raise ValueError("Bad MSP checksum")
                if frame[4] != COMMAND:
                    continue
                if not frame[3]:
                    raise ValueError("Control command unsupported; use examples/dshot firmware")
                status = decode_status(frame[5:-1])
                return status
        raise TimeoutError("No control reply; close ESC Configurator and verify the firmware/USB port")


def raw_throttle(text):
    try:
        value = int(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("raw throttle must be an integer") from error
    if not 48 <= value <= 2047:
        raise argparse.ArgumentTypeError("raw throttle must be between 48 and 2047")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Native USB serial port; close the configurator first")
    sub = parser.add_subparsers(dest="operation", required=True)
    sub.add_parser("status")
    sub.add_parser("arm", help="Send zeros and wait through the fixed five-second arming interval")
    run = sub.add_parser("run", help="Publish a fresh raw DShot throttle request")
    run.add_argument("value", type=raw_throttle, metavar="48..2047")
    sub.add_parser("stop", help="Return to zero frames")
    args = parser.parse_args(argv)
    try:
        import serial
        port = serial.Serial(timeout=0.1, write_timeout=2.0)
        port.port = args.port
        port.baudrate = 115200
        port.dtr = False
        port.rts = False
        with port:
            client = Client(port)
            status = client.request(args.operation, getattr(args, "value", 0))
            if args.operation == "arm" and status["operation_error"] == 0:
                deadline = time.monotonic() + 8
                while status["state"] == "arming":
                    if time.monotonic() >= deadline:
                        raise TimeoutError("Arming did not finish")
                    time.sleep(0.1)
                    status = client.request("status")
            print(json.dumps(status, indent=2))
            return 1 if status["operation_error"] or status["state"] == "fault" else 0
    except ImportError:
        print("Install pyserial: python3 -m pip install pyserial", file=sys.stderr)
    except (OSError, ValueError, TimeoutError) as error:
        print(str(error), file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
