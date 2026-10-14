#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Read-only MSP/4-way bench probe."""
import argparse
import binascii
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import serial


class Probe:
    def __init__(self, port):
        self.started = time.monotonic()
        self.trace = {"format": "esc-webserial-trace-v1", "url": "local:probe_esc.py",
                      "startedAt": datetime.now(timezone.utc).isoformat(), "events": []}
        self.port = serial.Serial(port=None, baudrate=115200, timeout=0.05)
        self.port.dtr = False
        self.port.rts = False
        self.port.port = port
        self.port.open()
        time.sleep(0.3)
        self.port.reset_input_buffer()
        self.event("opened")

    def event(self, kind, data=None):
        item = {"tMs": round((time.monotonic() - self.started) * 1000, 3),
                "port": 1, "kind": kind}
        if data is not None:
            item["bytes"] = list(data)
        self.trace["events"].append(item)

    def exchange(self, packet, protocol, command, address=0):
        self.event("tx", packet)
        self.port.write(packet)
        self.port.flush()
        response = bytearray()
        wanted = None
        # Allow for the adapter's bootloader entry delay.
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            data = self.port.read(1 if wanted is None else wanted - len(response))
            if not data:
                continue
            self.event("rx", data)
            response.extend(data)
            if protocol == "msp" and wanted is None:
                # Ignore console output between MSP frames.
                start = response.find(b"$M")
                if start < 0:
                    response = response[-1:] if response.endswith(b"$") else bytearray()
                    continue
                if start:
                    del response[:start]
                if len(response) >= 3 and response[2] not in b">!":
                    del response[0]
                    continue
            if len(response) >= 5 and wanted is None:
                if protocol == "msp":
                    if response[:2] != b"$M" or response[2] not in b">!":
                        raise RuntimeError(f"Unexpected MSP bytes: {response.hex(' ')}")
                    wanted = response[3] + 6
                else:
                    if response[0] != 0x2e:
                        raise RuntimeError(f"Unexpected 4-way bytes: {response.hex(' ')}")
                    wanted = (response[4] or 256) + 8
            if wanted is not None and len(response) == wanted:
                break
        else:
            raise TimeoutError(f"No complete {protocol} reply for 0x{command:02x}: {response.hex(' ')}")
        if protocol == "msp":
            crc = 0
            for byte in response[3:-1]:
                crc ^= byte
            if response[4] != command or crc != response[-1]:
                raise RuntimeError("Invalid MSP reply command/checksum")
            if response[2] == ord("!"):
                raise RuntimeError(f"MSP command {command} rejected")
            return bytes(response[5:-1])
        if (response[1] != command or int.from_bytes(response[2:4], "big") != address or
                binascii.crc_hqx(response[:-2], 0) != int.from_bytes(response[-2:], "big")):
            raise RuntimeError("Invalid 4-way reply command/address/checksum")
        if response[-3] != 0:
            raise RuntimeError(f"4-way command 0x{command:02x}: ACK 0x{response[-3]:02x}")
        return bytes(response[5:-3])

    def msp(self, command, payload=b""):
        checksum = len(payload) ^ command
        for byte in payload:
            checksum ^= byte
        packet = b"$M<" + bytes([len(payload), command]) + payload + bytes([checksum])
        return self.exchange(packet, "msp", command)

    def diagnostics(self, channel):
        try:
            print(self.msp(253, bytes([channel])).decode("ascii", errors="replace"), end="")
        except (RuntimeError, TimeoutError) as error:
            print(f"Diagnostics unavailable: {error}")

    def fourway(self, command, param=0, address=0):
        if command not in (0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x37, 0x3a):
            raise ValueError("Not a read-only probe command")
        packet = bytes([0x2f, command, address >> 8, address & 255, 1, param])
        packet += binascii.crc_hqx(packet, 0).to_bytes(2, "big")
        return self.exchange(packet, "4way", command, address)

    def close(self, trace_path):
        self.port.close()
        self.event("closed")
        if trace_path:
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            trace_path.write_text(json.dumps(self.trace, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/cu.usbmodem1101")
    parser.add_argument("--channel", type=int, choices=range(4), default=0)
    parser.add_argument("--trace", type=Path,
                        help="Trace output; diagnostic modes save no file unless supplied")
    parser.add_argument("--settings", type=Path, default=Path("backups/bluejay-settings.bin"))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--msp-only", action="store_true")
    mode.add_argument("--identify-only", action="store_true",
                      help="Identify ESCs and print raw init diagnostics; never read or write settings")
    mode.add_argument("--diagnostics-only", action="store_true",
                      help="Read retained init diagnostics without entering passthrough or touching ESC wires")
    parser.add_argument("--all-channels", action="store_true",
                        help="With --identify-only, probe each configured channel")
    args = parser.parse_args()
    if args.all_channels and not args.identify_only:
        parser.error("--all-channels requires --identify-only")
    if args.trace is None and not (args.identify_only or args.diagnostics_only):
        args.trace = Path("tools/traces/c3-bluejay-probe.json")
    probe = Probe(args.port)
    entered = connected = False
    try:
        if args.diagnostics_only:
            build = probe.msp(5).decode("ascii", errors="replace")
            print(f"Adapter build: {build}")
            count = probe.msp(131)[6]
            for channel in range(count):
                probe.diagnostics(channel)
            return
        for command in (1, 2, 3, 5, 4, 160, 104, 36, 101, 130):
            data = probe.msp(command)
            print(f"MSP {command:3}: {data.hex(' ')}")
        if args.msp_only:
            return
        count = probe.msp(245)[0]
        entered = True
        print(f"Passthrough: {count} configured ESC channel(s)")
        if args.channel >= count:
            raise RuntimeError("Requested channel is not configured")
        for command in (0x31, 0x32, 0x33):
            print(f"4-way 0x{command:02x}: {probe.fourway(command).hex(' ')}")
        if args.identify_only:
            failures = []
            for channel in range(count) if args.all_channels else [args.channel]:
                try:
                    info = probe.fourway(0x37, channel)
                    print(f"Channel {channel}: identity={info.hex(' ')} mode={info[3]}")
                except (RuntimeError, TimeoutError) as error:
                    print(f"Channel {channel}: {error}")
                    failures.append(channel)
                probe.diagnostics(channel)
            if failures:
                raise RuntimeError(f"ESC identification failed on channel(s) {failures}")
            return
        try:
            info = probe.fourway(0x37, args.channel)
        except RuntimeError:
            probe.diagnostics(args.channel)
            raise
        connected = True
        signature = int.from_bytes(info[:2], "little")
        print(f"ESC identity: signature=0x{signature:04x}, boot='{chr(info[2])}', mode={info[3]}")
        offsets = {0xe8b1: 0x1a00, 0xe8b2: 0x1a00, 0xe8b5: 0x3000}
        if info[3] != 1 or signature not in offsets:
            raise RuntimeError("Probe settings decoder supports Bluejay EFM8BB1/BB2/BB51 only")
        offset = offsets[signature]
        data = probe.fourway(0x3a, 0, offset)  # 256-byte framing test
        if len(data) != 256:
            raise RuntimeError("Wrong settings read length")
        settings = data[:255]
        text = lambda start: settings[start:start+16].decode("ascii", errors="replace").strip("\x00\xff ")
        print(f"Firmware v{settings[0]}.{settings[1]}, layout revision {settings[2]}")
        print(f"Layout: {text(0x40)}; MCU: {text(0x50)}; name: {text(0x60)}")
        again = probe.fourway(0x3a, 0, offset)
        if again != data:
            raise RuntimeError("Repeated settings reads differ")
        part = probe.fourway(0x3a, 128, offset)
        if part != settings[:128]:
            raise RuntimeError("128-byte settings read differs from 256-byte read")
        probe.fourway(0x30)
        args.settings.parent.mkdir(parents=True, exist_ok=True)
        args.settings.write_bytes(settings)
        print(f"PASS: two matching 256-byte reads, matching 128-byte read and keepalive. Saved {args.settings}.")
    finally:
        if entered:
            try:
                if connected:
                    probe.fourway(0x35, args.channel, 1)
                probe.fourway(0x34)
                probe.msp(1)
                print("PASS: ESC reset, passthrough exit and MSP reconnect.")
            except Exception as error:
                print(f"Cleanup: {error}")
        probe.close(args.trace)


if __name__ == "__main__":
    main()
