"""An evaluation seed selects SUMO's seed, its own frozen demand file and the route-choice seed.

Contract 13.13 froze each ordinary car's speed factor as SUMO drew it under one seed; 13.15 fixes
the five evaluation seeds and a held-out tuning seed. A frozen file must never run under another
seed, and an edited frozen file must never run at all - these tests pin both, and fail if any
listed seed has no valid frozen file.
"""

import hashlib
import json
from pathlib import Path

import pytest

import evaluation_seeds
from evaluation_seeds import (
    FrozenDemandError,
    checked_frozen_demand,
    evaluation_seeds as listed_evaluation_seeds,
    frozen_demand_paths,
    route_choice_seed,
    sumo_seed_args,
    tuning_seed,
)

ROOT = Path(__file__).resolve().parent.parent


def test_no_seed_means_the_config_as_it_is():
    assert sumo_seed_args(None) == []
    assert route_choice_seed(None) is None


def test_the_canonical_seed_is_the_default_run():
    """20260722 resolves to demand/porto.rou.xml: the default run IS that evaluation seed."""
    route_file, manifest = frozen_demand_paths(20260722)
    assert route_file == ROOT / "demand" / "porto.rou.xml"
    args = sumo_seed_args(20260722)
    assert args[:2] == ["--seed", "20260722"]
    files = args[3].split(",")
    assert Path(files[0]) == ROOT / "demand" / "porto.rou.xml"
    assert [Path(f).name for f in files[1:]] == ["porto.emergency.rou.xml", "porto.dispatch.rou.xml"]


def test_route_choice_follows_the_evaluation_seed():
    assert route_choice_seed(20260724) == 20260724


def test_the_seed_lists():
    assert listed_evaluation_seeds() == [20260722, 20260723, 20260724, 20260725, 20260726]
    assert tuning_seed() == 20260721
    assert tuning_seed() not in listed_evaluation_seeds()


@pytest.mark.parametrize("seed", [20260721, 20260722, 20260723, 20260724, 20260725, 20260726])
def test_every_listed_seed_has_its_own_valid_frozen_file(seed):
    route_file = checked_frozen_demand(seed)
    manifest = json.loads(frozen_demand_paths(seed)[1].read_text(encoding="utf-8"))
    assert manifest["sumo_seed"] == seed
    assert len(manifest["speed_factors"]) == 3760


def test_different_seeds_have_different_drivers():
    """One realisation reused for every seed would remove the between-seed variation.

    A handful of equal values is chance: SUMO draws speed factors to four decimals, and the
    placeholder car carries 1.0 in every file. Measured 2026-09-12: 1 to 4 per pair of seeds.
    """
    values = {}
    for seed in listed_evaluation_seeds():
        manifest = json.loads(frozen_demand_paths(seed)[1].read_text(encoding="utf-8"))
        values[seed] = manifest["speed_factors"]
    seeds = list(values)
    for i, a in enumerate(seeds):
        for b in seeds[i + 1:]:
            shared = sum(1 for v in values[a] if values[a][v] == values[b][v])
            assert shared < 10, f"seeds {a} and {b} share {shared} speed factors"


@pytest.fixture
def seed_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(evaluation_seeds, "SEED_DIR", tmp_path)
    return tmp_path


def _write(seed_dir, seed, manifest_seed=None, edit=False):
    route_file = seed_dir / f"porto.rou.{seed}.xml"
    route_file.write_text("<routes/>\n", encoding="utf-8")
    digest = hashlib.sha256(route_file.read_bytes()).hexdigest()
    if edit:
        route_file.write_text("<routes><!-- edited --></routes>\n", encoding="utf-8")
    (seed_dir / f"porto.speed_factors.{seed}.json").write_text(json.dumps({
        "sumo_seed": manifest_seed if manifest_seed is not None else seed,
        "frozen_sha256": digest,
    }), encoding="utf-8")
    return route_file


def test_a_missing_frozen_file_is_refused(seed_dir):
    with pytest.raises(FrozenDemandError, match="08b_freeze_speed_factors.py --seed 99"):
        sumo_seed_args(99)


def test_a_file_harvested_under_another_seed_is_refused(seed_dir):
    _write(seed_dir, 99, manifest_seed=98)
    with pytest.raises(FrozenDemandError, match="harvested under seed 98"):
        checked_frozen_demand(99)


def test_a_frozen_file_edited_after_freezing_is_refused(seed_dir):
    _write(seed_dir, 99, edit=True)
    with pytest.raises(FrozenDemandError, match="changed after freezing"):
        checked_frozen_demand(99)


def test_a_valid_per_seed_file_replaces_only_the_ordinary_demand(seed_dir):
    route_file = _write(seed_dir, 99)
    args = sumo_seed_args(99)
    files = args[3].split(",")
    assert args[:2] == ["--seed", "99"]
    assert Path(files[0]) == route_file
    assert [Path(f).name for f in files[1:]] == ["porto.emergency.rou.xml", "porto.dispatch.rou.xml"]
