#!/usr/bin/env python3
"""Record the frozen campaign design and its code/input fingerprint, and check a campaign against it (2026-09-15).

Diagnostic only. It creates no study, runs no simulation, and writes only to the file given by --out.

    python scripts/diagnostics/campaign_fingerprint.py record --out results/final_code_fingerprint_2026-09-15/campaign_settings.json
    python scripts/diagnostics/campaign_fingerprint.py compare results/final_code_fingerprint_2026-09-15/campaign_settings.json <campaign dir>

`record` computes exactly what `tune_e3_weights.py --new` would store for the real campaign: the tuner's own
`campaign_settings()` for the contracted design (tuning seed, 3,600 s, CONTRACT_TUNING_TRIALS, 10 start-up trials,
simulation.sumocfg, the canonical 3,600 s reference). That includes the SHA-256 of every file a trial runs, and
its 16-character digest.

`compare` checks a campaign's stored `campaign.json` against the record.
- Every setting must match except the reference path, which is printed.
- The code fingerprint must match file for file.
- It prints MATCH or every difference. For a smoke campaign (other sim_time, trials, reference) only the code
  fingerprint is meaningful, and the settings differences are expected.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO_ROOT), str(REPO_ROOT / "scripts")]
os.chdir(REPO_ROOT)

import tune_e3_weights as tuner  # noqa: E402

CANONICAL_REFERENCE = Path("results/tuning_reference/seed_20260721_3600s/reference.json")


def record(out: Path) -> None:
    import yaml
    config = yaml.safe_load(Path("config/scenario.yaml").read_text(encoding="utf-8"))
    seed, sim_time = int(config["seeds"]["tuning"]), int(config["tuning"]["duration_seconds"])
    settings = tuner.campaign_settings(seed, sim_time, tuner.DEFAULT_TRIALS, tuner.DEFAULT_STARTUP_TRIALS,
                                       Path("simulation.sumocfg"), CANONICAL_REFERENCE)
    document = {"settings": settings, "code_fingerprint_digest": tuner._digest(settings["code_fingerprint"]),
                "files_fingerprinted": len(settings["code_fingerprint"])}
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        raise SystemExit(f"{out} exists; evidence is never overwritten")
    out.write_text(json.dumps(document, indent=1, sort_keys=True), encoding="utf-8")
    print(f"recorded {out}: {document['files_fingerprinted']} files, digest {document['code_fingerprint_digest']}, "
          f"seed {settings['seed']}, {settings['sim_time']} s, {settings['trials_target']} trials, "
          f"{settings['startup_trials']} start-up")


def compare(recorded: Path, campaign: Path) -> int:
    rec = json.loads(recorded.read_text(encoding="utf-8"))["settings"]
    got = json.loads((campaign / "campaign.json").read_text(encoding="utf-8"))
    problems = []
    for key in sorted(set(rec) | set(got)):
        # the reference path is printed below; storage and study name are added per campaign when it is created
        if key in ("code_fingerprint", "reference", "storage", "study_name"):
            continue
        if rec.get(key) != got.get(key):
            problems.append(f"setting {key}: recorded {rec.get(key)!r}, campaign {got.get(key)!r}")
    a, b = rec["code_fingerprint"], got.get("code_fingerprint", {})
    for path in sorted(set(a) | set(b)):
        if a.get(path) != b.get(path):
            problems.append(f"fingerprint {path}: recorded {str(a.get(path))[:12]}, campaign {str(b.get(path))[:12]}")
    print(f"reference: recorded {rec['reference']}, campaign {got.get('reference')}")
    print(f"code fingerprint: {len(a)} recorded files, {len(b)} in the campaign, digest "
          f"{tuner._digest(a)} / {tuner._digest(b) if b else '-'}")
    for line in problems:
        print("  DIFF", line)
    print("MATCH" if not problems else f"{len(problems)} difference(s)")
    return 0 if not problems else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("record")
    r.add_argument("--out", required=True, type=Path)
    c = sub.add_parser("compare")
    c.add_argument("recorded", type=Path)
    c.add_argument("campaign", type=Path)
    args = parser.parse_args()
    if args.command == "record":
        record(args.out)
        return 0
    return compare(args.recorded, args.campaign)


if __name__ == "__main__":
    sys.exit(main())
