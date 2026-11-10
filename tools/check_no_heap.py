#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fail if ekf_core sources use heap containers or banned includes."""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CORE_DIRS = [ROOT / "cpp/include/ekf", ROOT / "cpp/src/core"]

BANNED = (
    re.compile(r"\bvector\s*<"),
    re.compile(r"\bstring\b"),
    re.compile(r"\bunique_ptr\s*<"),
    re.compile(r"\bshared_ptr\s*<"),
    re.compile(r"#\s*include\s*<memory>"),
    re.compile(r"\bnlohmann\b"),
    re.compile(r"\bnew\b"),
    re.compile(r"\bmalloc\b"),
)


def main() -> int:
    errors: list[str] = []
    for directory in CORE_DIRS:
        for path in directory.rglob("*"):
            if path.suffix not in {".hpp", ".cpp", ".h", ".cc"}:
                continue
            text = path.read_text(encoding="utf-8")
            for pattern in BANNED:
                for match in pattern.finditer(text):
                    line = text.count("\n", 0, match.start()) + 1
                    errors.append(f"{path.relative_to(ROOT)}:{line}: banned `{match.group()}`")
    if errors:
        sys.stderr.write("check_no_heap failed:\n")
        sys.stderr.write("\n".join(errors) + "\n")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
