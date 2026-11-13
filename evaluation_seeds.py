"""Which SUMO seed, frozen demand file and route-choice seed a run uses (contract 13.13, 13.15).

Contract decision 13.13 freezes every ordinary car's speed factor into the demand file, as SUMO
itself drew it under one SUMO seed. A frozen file therefore belongs to exactly one seed. Running
it under another seed would hand one seed's drivers to another seed's randomness (dawdling,
insertion), which is neither realisation. So a run is selected by one number, the evaluation
seed, and this module turns it into everything that has to agree with it:

* SUMO's `--seed`;
* the frozen demand file harvested under that seed, passed as `--route-files` in place of
  `demand/porto.rou.xml`, with the other route files of `simulation.sumocfg` unchanged;
* the route-choice seed of ACO, ACO-PSO and E3 (decision 13.15: it follows the evaluation seed).

No seed means exactly what `simulation.sumocfg` and `config/scenario.yaml` say, unchanged. The
canonical frozen file `demand/porto.rou.xml` was harvested under SUMO seed 20260722, which is
also `seeds.route_choice`, so evaluation seed 20260722 reproduces the default run exactly.

A frozen file is used only if its manifest names the requested seed and its SHA-256 still
matches the manifest. A file regenerated or edited after freezing is refused, never used.

Frozen files: `demand/porto.rou.xml` for 20260722 (manifest `demand/porto.speed_factors.json`),
and `demand/seeds/porto.rou.<seed>.xml` with `demand/seeds/porto.speed_factors.<seed>.json` for
every other seed. They are written by `scripts/08b_freeze_speed_factors.py --seed <seed>`.
"""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple

ROOT = Path(__file__).resolve().parent
SUMO_CFG = ROOT / "simulation.sumocfg"
SCENARIO_CONFIG = ROOT / "config" / "scenario.yaml"
CANONICAL_DEMAND = ROOT / "demand" / "porto.rou.xml"
CANONICAL_MANIFEST = ROOT / "demand" / "porto.speed_factors.json"
SEED_DIR = ROOT / "demand" / "seeds"
# The entry of simulation.sumocfg's route-files that a frozen file replaces.
ORDINARY_DEMAND_ENTRY = "demand/porto.rou.xml"


class FrozenDemandError(RuntimeError):
    """The frozen demand file for a seed is missing, belongs to another seed, or was edited."""


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_manifest(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def frozen_demand_paths(seed: int) -> Tuple[Path, Path]:
    """(frozen route file, manifest) for a SUMO seed. Does not check that they exist."""
    if CANONICAL_MANIFEST.is_file() and int(_read_manifest(CANONICAL_MANIFEST)["sumo_seed"]) == int(seed):
        return CANONICAL_DEMAND, CANONICAL_MANIFEST
    return SEED_DIR / f"porto.rou.{int(seed)}.xml", SEED_DIR / f"porto.speed_factors.{int(seed)}.json"


def checked_frozen_demand(seed: int) -> Path:
    """The frozen route file for `seed`, after checking it really is that seed's file."""
    route_file, manifest_path = frozen_demand_paths(seed)
    rerun = f"run: python scripts/08b_freeze_speed_factors.py --seed {int(seed)}"
    if not route_file.is_file() or not manifest_path.is_file():
        raise FrozenDemandError(f"no frozen demand file for seed {seed} ({route_file}); {rerun}")
    manifest = _read_manifest(manifest_path)
    if int(manifest["sumo_seed"]) != int(seed):
        raise FrozenDemandError(
            f"{manifest_path} was harvested under seed {manifest['sumo_seed']}, not {seed}"
        )
    if _sha256(route_file) != manifest["frozen_sha256"]:
        raise FrozenDemandError(
            f"{route_file} no longer matches its manifest - it was changed after freezing; {rerun}"
        )
    return route_file


def config_route_files(sumo_cfg: Path = SUMO_CFG) -> List[str]:
    """The route-files entries of a SUMO configuration, as written in it."""
    element = ET.parse(sumo_cfg).getroot().find(".//route-files")
    return [entry.strip() for entry in element.get("value").split(",") if entry.strip()]


def sumo_seed_args(seed: Optional[int], sumo_cfg: Path = SUMO_CFG) -> List[str]:
    """Extra SUMO command-line options for an evaluation seed; [] means the config as it is.

    Command-line options override the configuration file. Paths are made absolute because a
    command-line path resolves against the working directory, not against the config file.
    """
    if seed is None:
        return []
    sumo_cfg = Path(sumo_cfg)
    frozen = checked_frozen_demand(seed)
    entries = config_route_files(sumo_cfg)
    if ORDINARY_DEMAND_ENTRY not in entries:
        raise FrozenDemandError(f"{sumo_cfg} does not load {ORDINARY_DEMAND_ENTRY}; cannot substitute it")
    base = sumo_cfg.resolve().parent
    route_files = [str(frozen) if entry == ORDINARY_DEMAND_ENTRY else str(base / entry) for entry in entries]
    return ["--seed", str(int(seed)), "--route-files", ",".join(route_files)]


def route_choice_seed(seed: Optional[int]) -> Optional[int]:
    """The route-choice seed for an evaluation seed: the same number (decision 13.15).

    None means "read seeds.route_choice from config/scenario.yaml", as every run did before.
    """
    return None if seed is None else int(seed)


def _seeds_config() -> dict:
    import yaml

    with open(SCENARIO_CONFIG, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)["seeds"]


def evaluation_seeds() -> List[int]:
    """The predefined evaluation seed list, from config/scenario.yaml (decision 13.15)."""
    return [int(seed) for seed in _seeds_config()["evaluation_runs"]]


def tuning_seed() -> int:
    """The held-out weight-tuning seed, never one of the evaluation seeds (decision 13.15)."""
    return int(_seeds_config()["tuning"])
