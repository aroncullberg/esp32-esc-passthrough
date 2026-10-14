#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Run sanitized protocol/backend and session lifecycle host tests."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dshot-dir", type=Path, help="Resolved DShot component source for example tests")
    args = parser.parse_args()
    trace = json.loads((ROOT / "tools/traces/fc-no-esc.decoded.json").read_text())
    with tempfile.TemporaryDirectory(prefix="esc-tests-") as temp:
        temp = Path(temp)
        rows = []
        for exchange in trace["exchanges"]:
            request = trace["frames"][exchange["requestFrame"]]
            reply = trace["frames"][exchange["replyFrame"]]
            req = ",".join(map(str, request["bytes"]))
            rep = ",".join(map(str, reply["bytes"]))
            rows.append("{%d, {%s}, %d, {%s}}" % (len(request["bytes"]), req, len(reply["bytes"]), rep))
        (temp / "trace_fixture.h").write_text(
            "static const struct { size_t req_size; uint8_t req[263]; "
            "size_t rep_size; uint8_t rep[264]; } trace[] = {\n" + ",\n".join(rows) + "\n};\n"
        )
        flags = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-g",
                 "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                 "-I", str(ROOT / "include"), "-I", str(temp)]
        suites = {
            "test_mvp": ["src/passthrough.c", "src/esc_backend.c"],
            "test_session": ["src/passthrough.c", "src/esc_backend.c", "src/esc_passthrough.c"],
            "test_uart": ["src/esc_uart.c"],
            "test_uart_no_boot_wait": ["src/esc_uart.c"],
        }
        for name, sources in suites.items():
            extra = [] if name == "test_mvp" else ["-I", str(ROOT / "tests/fakes"), "-I", str(ROOT / "src")]
            test_source = "test_uart" if name == "test_uart_no_boot_wait" else name
            if name == "test_uart_no_boot_wait":
                extra += ["-DCONFIG_ESC_PASSTHROUGH_BOOT_WAIT_MS=0"]
            command = [os.environ.get("CC", "cc"), *flags, *extra,
                       str(ROOT / f"tests/{test_source}.c"),
                       *(str(ROOT / source) for source in sources), "-o", str(temp / name)]
            subprocess.run(command, check=True)
            subprocess.run([str(temp / name)], check=True)
        subprocess.run([os.environ.get("PYTHON", "python3"), str(ROOT / "tests/test_dshot_control.py")], check=True)
        subprocess.run([os.environ.get("PYTHON", "python3"), str(ROOT / "tests/test_probe.py")], check=True)
        dshot = args.dshot_dir or ROOT / "examples/dshot/managed_components/dshot"
        if (dshot / "include/dshot/passthrough.hpp").is_file():
            includes = ["-I", str(ROOT / "tests/fakes"), "-I", str(ROOT / "include"),
                        "-I", str(ROOT / "src"), "-I", str(ROOT / "examples/dshot/main"),
                        "-I", str(dshot / "include"), "-I", str(dshot / "src")]
            objects = []
            for source in ("passthrough", "esc_backend", "esc_passthrough"):
                obj = temp / f"{source}.o"
                subprocess.run([os.environ.get("CC", "cc"), *flags, *includes,
                                "-c", str(ROOT / f"src/{source}.c"), "-o", str(obj)], check=True)
                objects.append(str(obj))
            binary = temp / "test_dshot_example"
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined", "-fno-omit-frame-pointer", *includes,
                            str(ROOT / "tests/test_dshot_example.cpp"), str(dshot / "src/config.cpp"),
                            str(dshot / "src/passthrough.cpp"), *objects, "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
        elif args.dshot_dir:
            parser.error("DShot source must contain dshot/passthrough.hpp")
        else:
            print("DShot example policy test skipped: resolve its managed dependency or pass --dshot-dir PATH")



if __name__ == "__main__":
    main()
