#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Run host tests against the captured FC traffic."""
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
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
        command = [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                   "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                   "-I", str(ROOT / "components/protocol/include"),
                   "-I", str(ROOT / "components/esc_backend/include"), "-I", str(temp),
                   str(ROOT / "tests/test_mvp.c"),
                   str(ROOT / "components/protocol/passthrough.c"),
                   str(ROOT / "components/esc_backend/esc_backend.c"),
                   "-o", str(temp / "test_mvp")]
        subprocess.run(command, check=True)
        subprocess.run([str(temp / "test_mvp")], check=True)


if __name__ == "__main__":
    main()
