# SPDX-License-Identifier: GPL-3.0-or-later
"""Decode captured MSP v1 and 4-way byte streams"""

import argparse
import binascii
import json
from collections import Counter, defaultdict, deque
from pathlib import Path


COMMANDS = {
    "MSPv1": {
        1: "API_VERSION", 2: "FC_VARIANT", 3: "FC_VERSION",
        4: "BOARD_INFO", 5: "BUILD_INFO", 36: "FEATURE_CONFIG",
        101: "STATUS", 104: "MOTOR", 124: "MOTOR_3D_CONFIG",
        130: "BATTERY_STATE", 131: "MOTOR_CONFIG", 160: "UID",
        214: "SET_MOTOR", 245: "SET_PASSTHROUGH",
    },
    "4way": {
        0x30: "InterfaceTestAlive", 0x31: "ProtocolGetVersion",
        0x32: "InterfaceGetName", 0x33: "InterfaceGetVersion",
        0x34: "InterfaceExit", 0x35: "DeviceReset",
        0x37: "DeviceInitFlash", 0x38: "DeviceEraseAll",
        0x39: "DevicePageErase", 0x3A: "DeviceRead",
        0x3B: "DeviceWrite", 0x3C: "DeviceC2CK_LOW",
        0x3D: "DeviceReadEEprom", 0x3E: "DeviceWriteEEprom",
        0x3F: "InterfaceSetMode", 0x40: "DeviceVerify",
    },
}

ACKS = {
    0: "ACK_OK", 2: "ACK_I_INVALID_CMD", 3: "ACK_I_INVALID_CRC",
    4: "ACK_I_VERIFY_ERROR", 8: "ACK_I_INVALID_CHANNEL",
    9: "ACK_I_INVALID_PARAM", 15: "ACK_D_GENERAL_ERROR",
}


def decode_trace(trace):
    """Decode MSP v1 and 4-way frames by port and direction."""
    if trace.get("format") != "esc-webserial-trace-v1":
        raise ValueError("Unsupported trace format")
    pending = defaultdict(list)
    frames = []
    for event_index, event in enumerate(trace["events"]):
        if event["kind"] not in ("tx", "rx"):
            continue
        data = event["bytes"]
        if any(type(byte) is not int or not 0 <= byte <= 255 for byte in data):
            raise ValueError(f"Invalid byte array at event {event_index}")
        key = (event["port"], event["kind"])
        stream = pending[key]
        stream.extend((byte, event["tMs"], event_index) for byte in data)
        while len(stream) >= 5:
            buffered = bytes(item[0] for item in stream)
            if buffered.startswith(b"$M"):
                direction = buffered[2]
                expected = b"<" if key[1] == "tx" else b">!"
                if direction not in expected:
                    raise ValueError(f"Unexpected MSP direction at event {stream[0][2]}")
                size = buffered[3] + 6
                if len(stream) < size:
                    break
                packet = buffered[:size]
                checksum = 0
                for byte in packet[3:-1]:
                    checksum ^= byte
                frame = {
                    "protocol": "MSPv1", "command": packet[4],
                    "direction": chr(direction), "payload": list(packet[5:-1]),
                    "checksumValid": checksum == packet[-1],
                }
            elif buffered[0] in (0x2F, 0x2E):
                reply = buffered[0] == 0x2E
                if reply != (key[1] == "rx"):
                    raise ValueError(f"Unexpected 4-way direction at event {stream[0][2]}")
                length = buffered[4] or 256
                size = 5 + length + (3 if reply else 2)
                if len(stream) < size:
                    break
                packet = buffered[:size]
                frame = {
                    "protocol": "4way", "command": packet[1],
                    "address": int.from_bytes(packet[2:4], "big"),
                    "payload": list(packet[5:5 + length]),
                    "checksumValid": binascii.crc_hqx(packet[:-2], 0)
                    == int.from_bytes(packet[-2:], "big"),
                }
                if reply:
                    frame["ack"] = packet[-3]
                    frame["ackName"] = ACKS.get(packet[-3], "UNKNOWN_ACK")
            else:
                raise ValueError(
                    f"Unsupported protocol/data at event {stream[0][2]}: "
                    f"{buffered[:12].hex(' ')}"
                )
            frame.update({
                "name": COMMANDS[frame["protocol"]].get(frame["command"], "UNKNOWN_COMMAND"),
                "port": key[0], "kind": key[1],
                "startMs": stream[0][1], "endMs": stream[size - 1][1],
                "eventIndex": stream[0][2], "bytes": list(packet),
            })
            frames.append(frame)
            del stream[:size]

    frames.sort(key=lambda frame: frame["eventIndex"])
    counts = Counter((frame["protocol"], frame["command"], frame["kind"]) for frame in frames)
    command_keys = sorted({(frame["protocol"], frame["command"]) for frame in frames})
    commands = [{
        "protocol": protocol, "command": command,
        "name": COMMANDS[protocol].get(command, "UNKNOWN_COMMAND"),
        "tx": counts[protocol, command, "tx"],
        "rx": counts[protocol, command, "rx"],
    } for protocol, command in command_keys]

    waiting = defaultdict(deque)
    exchanges = []
    unmatched_replies = 0
    for index, frame in enumerate(frames):
        if not frame["checksumValid"]:
            continue
        key = (frame["port"], frame["protocol"], frame["command"])
        if frame["kind"] == "tx":
            waiting[key].append(index)
        elif waiting[key]:
            request_index = waiting[key].popleft()
            exchanges.append({
                "requestFrame": request_index, "replyFrame": index,
                "elapsedMs": round(frame["endMs"] - frames[request_index]["startMs"], 3),
            })
        else:
            unmatched_replies += 1

    return {
        "metadata": {key: value for key, value in trace.items() if key != "events"},
        "summary": {
            "frameCount": len(frames),
            "checksumFailures": sum(not frame["checksumValid"] for frame in frames),
            "incompleteStreams": [{
                "port": key[0], "kind": key[1], "bytes": [item[0] for item in stream],
            } for key, stream in pending.items() if stream],
            "unansweredRequests": sum(len(requests) for requests in waiting.values()),
            "unmatchedReplies": unmatched_replies,
            "commands": commands,
        },
        "frames": frames,
        "exchanges": exchanges,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--output", type=Path, help="Save decoded frames and summary as JSON")
    args = parser.parse_args()
    try:
        result = decode_trace(json.loads(args.trace.read_text()))
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f"Cannot decode trace: {error}\n")
    result["source"] = args.trace.name
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result["summary"], indent=2))
    summary = result["summary"]
    return int(bool(summary["checksumFailures"] or summary["incompleteStreams"]
                    or result["metadata"].get("droppedEvents", 0)))


if __name__ == "__main__":
    raise SystemExit(main())
