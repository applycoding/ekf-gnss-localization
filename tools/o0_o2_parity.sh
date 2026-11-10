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

def dump(build):
    out = subprocess.check_output(
        [f"{root}/build/{build}/ekf_cli", "matched", "1", "--parity"], text=True
    )
    rows = []
    for line in out.strip().splitlines():
        if line.strip():
            rows.append(tuple(map(float, line.split())))
    return rows

a = dump("o0")
b = dump("o2")
assert len(a) == len(b)
for row_a, row_b in zip(a, b):
    assert row_a[0] == row_b[0]
    assert abs(row_a[1] - row_b[1]) < 1e-9
    assert abs(row_a[2] - row_b[2]) < 1e-9
    assert int(row_a[3]) == int(row_b[3])
PY
