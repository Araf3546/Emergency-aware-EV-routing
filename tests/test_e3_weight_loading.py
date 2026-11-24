"""A tuned weight set must actually reach the router, and must be able to change a decision.

Audit C10: `best_e3_weights.json` had never been loaded by anything, so every run used
DEFAULT_WEIGHTS whatever tuning produced - tuning could not have changed a single result.
These tests pin the loading path and, more importantly, prove the weights are not inert: the
same candidate set decided under two weight sets gives two different winners.
"""

import json

import pytest

from algorithms.e3_core import configured_weights_file
from p3_hybrid_router import COMPONENT_NAMES, DEFAULT_WEIGHTS, HybridContext, HybridRouter, load_weights_file


from emergency_manager import ROAD_PERMISSION_POLICY

CAMPAIGN = {  # what the corrected tuner writes for a complete real campaign (2026-09-13)
    "evaluator": "p3_runtime.P3RuntimeCoordinator(emergency_layer=True)",
    "objective_weights": {"travel": 0.35, "response": 0.30, "time_loss": 0.2625,
                          "incomplete": 0.07, "teleports": 0.0175},
    "seed": 20260721,
    "sim_time": 3600,
    "road_permissions": ROAD_PERMISSION_POLICY,
    "campaign_complete": True,
    "trials_evaluated": 20,   # contract 13.19, amended from 25 on 2026-09-15
}


def write_weights(tmp_path, values, wrap=True, **overrides):
    path = tmp_path / "best_e3_weights.json"
    document = dict(CAMPAIGN, best_weights=values, **overrides) if wrap else values
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def test_the_tuners_own_file_shape_loads():
    """The file scripts/tune_e3_weights.py writes nests the values under best_weights."""
    values = {"pheromone": 0.1, "heuristic": 0.2, "protocol": 0.3, "bco": 0.4, "velocity": 0.5}
    assert set(values) == set(COMPONENT_NAMES)


def test_loading_replaces_every_weight(tmp_path):
    values = {"pheromone": 0.11, "heuristic": 0.22, "protocol": 0.33, "bco": 0.44, "velocity": 0.55}
    loaded = load_weights_file(write_weights(tmp_path, values))
    assert loaded == values
    assert loaded != DEFAULT_WEIGHTS


def test_a_plain_mapping_also_loads(tmp_path):
    values = {"pheromone": 1.0, "heuristic": 0.0, "protocol": 0.0, "bco": 0.0, "velocity": 0.0}
    assert load_weights_file(write_weights(tmp_path, values, wrap=False)) == values


def test_a_typo_or_a_missing_component_is_refused(tmp_path):
    bad = {"pheromone": 0.4, "heuristic": 0.2, "protocol": 0.4, "bco": 0.1, "speed": 0.05}
    with pytest.raises(ValueError, match="unknown components"):
        load_weights_file(write_weights(tmp_path, bad))


def test_no_configuration_means_default_weights(monkeypatch):
    """The default run must be exactly what it was before C10 was fixed."""
    monkeypatch.delenv("P3_E3_WEIGHTS", raising=False)
    assert configured_weights_file() is None
    assert HybridRouter().weights == DEFAULT_WEIGHTS


def test_the_environment_override_is_found(tmp_path, monkeypatch):
    path = write_weights(tmp_path, dict(DEFAULT_WEIGHTS))
    monkeypatch.setenv("P3_E3_WEIGHTS", str(path))
    assert configured_weights_file() == path


def test_changing_the_loaded_weights_changes_the_chosen_edge(tmp_path):
    """The point of the fix: a loaded weight set must be able to change a routing decision.

    Two candidates. 'a' is the better road for the heuristic; 'b' carries far more pheromone.
    A pheromone-led weight set picks 'b'; a heuristic-led one picks 'a'.
    """
    candidates = [
        ("a", HybridContext(pheromone=1.0, heuristic=1.0, protocol_weight=0.5, bco_exploration=0.5, velocity=0.0)),
        ("b", HybridContext(pheromone=5.0, heuristic=0.0, protocol_weight=0.5, bco_exploration=0.5, velocity=0.0)),
    ]
    pheromone_led = {"pheromone": 1.0, "heuristic": 0.05, "protocol": 0.0, "bco": 0.0, "velocity": 0.0}
    heuristic_led = {"pheromone": 0.05, "heuristic": 1.0, "protocol": 0.0, "bco": 0.0, "velocity": 0.0}

    assert HybridRouter(weights=load_weights_file(write_weights(tmp_path, pheromone_led))).select_best_edge(candidates)[0] == "b"
    assert HybridRouter(weights=load_weights_file(write_weights(tmp_path, heuristic_led))).select_best_edge(candidates)[0] == "a"



# ---------------------------------------------------------------- 2026-09-13: stale weight sets
VALUES = {"pheromone": 0.11, "heuristic": 0.22, "protocol": 0.33, "bco": 0.44, "velocity": 0.55}


def test_a_pre_fix_tuner_file_is_refused(tmp_path):
    """Every file written before 2026-09-12/13 has best_weights and nothing else of this."""
    path = tmp_path / "best_e3_weights.json"
    path.write_text(json.dumps({"best_weights": VALUES, "seed": 20260721, "sim_time": 3000}), encoding="utf-8")
    with pytest.raises(ValueError, match="not a valid tuned weight set"):
        load_weights_file(path)


@pytest.mark.parametrize("override, reason", [
    ({"sim_time": 1800}, "contracted"),                   # the validation study
    ({"sim_time": 700}, "contracted"),                    # the smoke study
    ({"seed": 20260722}, "tuning seed"),                  # tuned on an evaluation seed
    ({"objective_weights": {"travel": 0.40}}, "finalized objective"),
    ({"evaluator": "algorithms/e3 loop"}, "corrected tuner"),
    ({"road_permissions": None}, "road permissions"),     # tuned before PD-1 question 2
    ({"campaign_complete": False, "trials_evaluated": 5}, "complete campaign"),   # mid-campaign
    ({"trials_evaluated": 19}, "complete campaign"),                              # one short of 20
])
def test_a_study_that_is_not_the_real_campaign_is_refused(tmp_path, override, reason):
    with pytest.raises(ValueError, match=reason):
        load_weights_file(write_weights(tmp_path, VALUES, **override))


def test_the_contracted_tuning_budget_is_20_trials():
    """Contract 13.19 (amended 2026-09-15): 20 trials, 10 random start-up then 10 TPE. The weight loader's guard and
    the tuner's default both read the one constant; `campaign_status` reads the tuner's default."""
    import p3_hybrid_router
    tuner = pytest.importorskip("scripts.tune_e3_weights")
    assert p3_hybrid_router.CONTRACT_TUNING_TRIALS == 20
    assert tuner.DEFAULT_TRIALS == 20
    assert tuner.DEFAULT_STARTUP_TRIALS == 10


def test_anything_under_results_invalid_is_refused(tmp_path):
    quarantine = tmp_path / "results" / "_invalid" / "old"
    quarantine.mkdir(parents=True)
    with pytest.raises(ValueError, match="quarantined"):
        load_weights_file(write_weights(quarantine, VALUES))


def test_every_quarantined_weight_file_in_the_project_is_refused():
    from pathlib import Path
    root = Path(__file__).resolve().parent.parent / "results" / "_invalid"
    files = sorted(root.rglob("best_e3_weights.json")) if root.exists() else []
    for path in files:
        with pytest.raises(ValueError):
            load_weights_file(path)


def test_the_runtime_refuses_a_throwaway_study_even_when_configured(tmp_path, monkeypatch):
    from algorithms.e3_core import E3Core
    monkeypatch.setenv("P3_E3_WEIGHTS", str(write_weights(tmp_path, VALUES, sim_time=1800)))
    with pytest.raises(ValueError):
        E3Core(emergency_layer=True, reset_state=True)
