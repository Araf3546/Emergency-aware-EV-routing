#!/usr/bin/env python3
"""Read-only checks for a fresh clone before any simulation (handover, 2026-09-17). Runs no simulation.

    python scripts/diagnostics/handover_checks.py frozen --root ../v1-frozen
        # in a worktree of tag v1.0-frozen-chain: every file in V1_MANIFEST.sha256 is byte-exact, and the frozen
        # chain's code fingerprint is c39f2a937667f63c
    python scripts/diagnostics/handover_checks.py comparison
        # on main: the V1 comparison code digest is 769b328325ffb4fb and nothing differs from the plan
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
FROZEN_DIGEST = "c39f2a937667f63c"
COMPARISON_DIGEST = "769b328325ffb4fb"
PLAN = "results/comparison_v1_2026-09-17/plan.json"


def cmd_frozen(args) -> int:
    root = Path(args.root).resolve()
    manifest = root / "V1_MANIFEST.sha256"
    if not manifest.is_file():
        raise SystemExit(f"{manifest} not found; is {root} a worktree of tag v1.0-frozen-chain?")
    entries = [line.rstrip("\n").split("  ", 1) for line in manifest.read_text(encoding="utf-8").splitlines() if line]
    bad = [p for h, p in entries if not (root / p).is_file() or hashlib.sha256((root / p).read_bytes()).hexdigest() != h]
    print(f"manifest mismatches: {len(bad)} of {len(entries)}" + (f" -> {bad[:10]}" if bad else ""))
    # The fingerprint must be computed by the tagged code itself, so it runs in a separate process rooted there.
    code = ("import sys; sys.path[:0]=['.','scripts']; import tune_e3_weights as t; from pathlib import Path; "
            "print(t._digest(t.evaluation_fingerprint(20260721, Path('simulation.sumocfg'))))")
    digest = subprocess.run([sys.executable, "-c", code], cwd=root, capture_output=True, text=True).stdout.strip()
    print(f"frozen chain fingerprint: {digest}")
    ok = not bad and digest == FROZEN_DIGEST
    print("FROZEN CHAIN OK" if ok else f"FAILED: expected 0 mismatches and {FROZEN_DIGEST}")
    return 0 if ok else 1


def cmd_comparison(args) -> int:
    sys.path[:0] = [str(REPO_ROOT / "scripts" / "diagnostics")]
    import comparison_evaluation as C
    import heldout_evaluation as H

    files = C.comparison_fingerprint(20260723)
    plan = json.loads((REPO_ROOT / PLAN).read_text(encoding="utf-8"))
    digest = C.code_digest(files)
    differences = H.code_differences(files, plan["code_fingerprint"]["files"])
    print(f"comparison code digest: {digest}")
    print(f"differences from the plan: {differences}")
    ok = digest == COMPARISON_DIGEST == plan["code_fingerprint"]["code_digest"] and not differences
    print("COMPARISON CODE OK" if ok else f"FAILED: expected {COMPARISON_DIGEST} and no differences")
    return 0 if ok else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    f = sub.add_parser("frozen")
    f.add_argument("--root", required=True)
    sub.add_parser("comparison")
    args = parser.parse_args()
    return {"frozen": cmd_frozen, "comparison": cmd_comparison}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
