#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cmake -S "$ROOT" -B "$ROOT/build/o0" -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_COMPILER=g++-13 -DCMAKE_CXX_FLAGS="-O0" -DEKF_BUILD_TESTS=OFF -DEKF_BUILD_WASM=OFF
cmake --build "$ROOT/build/o0" --target ekf_cli
cmake -S "$ROOT" -B "$ROOT/build/o2" -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=g++-13 -DCMAKE_CXX_FLAGS_RELEASE="-O2 -DNDEBUG" -DEKF_BUILD_TESTS=OFF -DEKF_BUILD_WASM=OFF
cmake --build "$ROOT/build/o2" --target ekf_cli
python3 - "$ROOT" <<'PY'
import subprocess
import sys

root = sys.argv[1]

def dump(build, scenario="matched", seed=1):
    out = subprocess.check_output(
        [f"{root}/build/{build}/ekf_cli", scenario, str(seed), "--parity-full"],
        text=True,
    )
    lines = []
    for line in out.strip().splitlines():
        if line.startswith("S "):
            lines.append(("S", tuple(line.split()[1:])))
        elif line.startswith("G "):
            lines.append(("G", tuple(line.split()[1:])))
        elif line.startswith("RMSE "):
            lines.append(("RMSE", float(line.split()[1])))
    return lines

a = dump("o0")
b = dump("o2")
assert len(a) == len(b)
for (ka, va), (kb, vb) in zip(a, b):
    assert ka == kb
    if ka == "RMSE":
        assert abs(va - vb) < 1e-9
    elif ka == "G":
        assert va[0] == vb[0]
        assert int(va[1]) == int(vb[1])
        assert int(va[2]) == int(vb[2])
    else:
        for x, y in zip(va, vb):
            assert abs(float(x) - float(y)) < 1e-9
PY
