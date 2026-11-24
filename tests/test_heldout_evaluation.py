"""The carried-forward weight files and the held-out evaluation's rules (2026-09-16).

The files in `results/final_weight_candidates_2026-09-16/` are what the final evaluation runs, so these tests pin
what they must contain: the campaign's own weights for their trial, the campaign's provenance, and no claim to be
the campaign's best trial. The rules of the evaluation itself are pinned before it is run.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
CAMPAIGN = REPO_ROOT / "results/e3_tuning_campaign_2026-09-15"
CANDIDATES = REPO_ROOT / "results/final_weight_candidates_2026-09-16"
WEIGHT_NAMES = ("pheromone", "heuristic", "protocol", "bco", "velocity")


def _module():
    spec = importlib.util.spec_from_file_location(
        "heldout_evaluation", REPO_ROOT / "scripts/diagnostics/heldout_evaluation.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


heldout = _module()


def _candidate(number: int) -> dict:
    return json.loads((CANDIDATES / f"trial_{number:03d}_weights.json").read_text(encoding="utf-8"))


def _trial(number: int) -> dict:
    return json.loads((CAMPAIGN / f"trials/trial_{number:03d}/trial.json").read_text(encoding="utf-8"))


# ---------------------------------------------------------------- the weight files

@pytest.mark.parametrize("number", (16, 17))
def test_the_weights_are_exactly_the_campaign_trials(number):
    assert _candidate(number)["best_weights"] == {k: _trial(number)["weights_requested"][k] for k in WEIGHT_NAMES}


@pytest.mark.parametrize("number", (16, 17))
def test_the_production_loader_accepts_the_file(number):
    from p3_hybrid_router import load_weights_file
    loaded = load_weights_file(CANDIDATES / f"trial_{number:03d}_weights.json")
    assert loaded == {k: float(_trial(number)["weights_requested"][k]) for k in WEIGHT_NAMES}


@pytest.mark.parametrize("number", (16, 17))
def test_the_file_does_not_claim_to_be_the_campaign_best(number):
    document = _candidate(number)
    assert document["is_campaign_best_trial"] is False
    assert document["source_trial_number"] == number
    assert document["campaign_best_trial_number"] == 10
    # "best_weights" is the loader's schema key; no other key may assert that this trial was the best.
    assert [k for k in document if k.startswith("best_") and k != "best_weights"] == []


@pytest.mark.parametrize("number", (16, 17))
def test_the_campaign_provenance_is_carried_unchanged(number):
    campaign = json.loads((CAMPAIGN / "best_e3_weights.json").read_text(encoding="utf-8"))
    document = _candidate(number)
    for key in ("study_name", "seed", "evaluator", "objective", "objective_weights", "road_permissions",
                "reference", "frozen_demand", "storage", "sampler", "startup_trials", "code_fingerprint_digest",
                "campaign_complete", "trials_evaluated", "trials_complete", "failed_trials", "sim_time"):
        assert document[key] == campaign[key], key


def test_the_two_files_hold_different_weight_sets():
    assert _candidate(16)["best_weights"] != _candidate(17)["best_weights"]


def test_the_roles_name_the_robustness_outcome():
    assert "winner" in _candidate(16)["weights_role"]
    assert "near-tied" in _candidate(17)["weights_role"]


# ---------------------------------------------------------------- the evaluation's rules

def test_the_matrix_is_five_baselines_then_two_runs_per_seed():
    specs = heldout.evaluation_matrix([1, 2, 3, 4, 5])
    assert len(specs) == 15
    assert [s["kind"] for s in specs[:5]] == ["baseline"] * 5
    assert sorted(s["configuration"] for s in specs[5:]) == sorted(["trial_016", "trial_017"] * 5)
    for seed in (1, 2, 3, 4, 5):                      # every seed runs both configurations, so the test is paired
        assert sorted(s["configuration"] for s in specs if s.get("seed") == seed and s["kind"] == "evaluation") == \
               ["trial_016", "trial_017"]


def test_the_matrix_never_includes_the_tuning_seed():
    assert heldout.TUNING_SEED not in heldout.seeds()


def test_only_the_seed_demand_file_may_differ_from_the_frozen_code():
    frozen = {"p3_runtime.py": "a", "demand/seeds/porto.rou.20260721.xml": "b", "config/scenario.yaml": "c"}
    same_code_other_seed = {"p3_runtime.py": "a", "demand/seeds/porto.rou.20260723.xml": "z",
                            "config/scenario.yaml": "c"}
    assert heldout.code_differences(same_code_other_seed, frozen) == []
    changed = dict(same_code_other_seed)
    changed["p3_runtime.py"] = "x"
    assert heldout.code_differences(changed, frozen) == ["p3_runtime.py: changed since the freeze"]
    assert heldout.code_differences({"demand/porto.rou.xml": "z", "config/scenario.yaml": "c"}, frozen) == \
        ["p3_runtime.py: missing from this run's fingerprint"]


def test_the_tool_cannot_select_between_the_two_configurations():
    # Section 13 of the pre-registration: held-out data selects nothing. The tool must expose no such machinery.
    for forbidden in ("decide", "rank_configurations", "select", "winner"):
        assert not hasattr(heldout, forbidden), forbidden
    assert heldout.FINAL_CONFIGURATION == "trial_016"
    assert heldout.SENSITIVITY_CONFIGURATION == "trial_017"
    assert "any winner, ranking or selection between T16 and T17" in         heldout.report_specification()["not_reported"][2]


# ---------------------------------------------------------------- the censoring rule

def test_a_defined_ratio_is_the_plain_division():
    ratio, reason = heldout.normalized_response(164.0, "valid", 328.0, "valid_post_window")
    assert ratio == 0.5 and reason == "defined"


@pytest.mark.parametrize("status", ("teleported", "not reached", None))
def test_a_censored_run_response_makes_the_ratio_undefined(status):
    ratio, reason = heldout.normalized_response(400.0, status, 328.0, "valid")
    assert ratio is None and "run response censored" in reason


@pytest.mark.parametrize("status", ("teleported", "not reached", None))
def test_a_censored_baseline_response_makes_the_ratio_undefined(status):
    ratio, reason = heldout.normalized_response(400.0, "valid", 328.0, status)
    assert ratio is None and "baseline response censored" in reason


def test_an_undefined_incident_is_excluded_and_counted_never_substituted():
    block = heldout.mean_normalized_response({"404": 0.8, "401": None, "405": 0.6})
    assert block["mean"] == pytest.approx(0.7)             # the mean of the two defined ratios only
    assert block["eligible_count"] == 2
    assert block["eligible_incidents"] == ["404", "405"]
    assert block["excluded_incidents"] == ["401"]


def test_every_incident_undefined_gives_no_mean_rather_than_zero():
    block = heldout.mean_normalized_response({"404": None, "401": None, "405": None})
    assert block["mean"] is None and block["eligible_count"] == 0


def test_the_worst_incident_is_the_largest_defined_ratio():
    assert heldout.worst_incident({"404": 0.9, "401": 1.2, "405": None}) == "401"
    assert heldout.worst_incident({"404": None}) is None


# ---------------------------------------------------------------- aggregates, quadrants, decomposition

def test_paired_statistics_uses_the_student_t_multiplier_for_five_observations():
    stats = heldout.paired_statistics([10.0, 12.0, 8.0, 14.0, 6.0])
    assert stats["n_defined"] == 5 and stats["mean"] == pytest.approx(10.0)
    assert stats["sd"] == pytest.approx(3.1622776601683795)
    assert stats["t_multiplier"] == 2.776 and stats["degrees_of_freedom"] == 4
    assert stats["ci95_halfwidth"] == pytest.approx(2.776 * 3.1622776601683795 / 5 ** 0.5)


def test_paired_statistics_skips_undefined_observations_and_reports_the_count():
    stats = heldout.paired_statistics([1.0, None, 3.0])
    assert stats["n_defined"] == 2 and stats["mean"] == pytest.approx(2.0)
    assert stats["observations"] == [1.0, None, 3.0]


def test_quadrants_follow_the_t17_minus_t16_convention():
    assert heldout.quadrant(-0.1, -0.1)["quadrant"] == "A"   # T17 lower J, lower response
    assert heldout.quadrant(-0.1, +0.1)["quadrant"] == "B"
    assert heldout.quadrant(+0.1, -0.1)["quadrant"] == "C"
    assert heldout.quadrant(+0.1, +0.1)["quadrant"] == "D"


def test_a_tie_is_reported_as_a_tie_and_never_forced_into_a_quadrant():
    tie = heldout.quadrant(0.0, -0.1)
    assert tie["quadrant"] is None and "tie" in tie["description"]
    undefined = heldout.quadrant(0.1, None)
    assert undefined["quadrant"] is None and "undefined" in undefined["description"]


def test_the_objective_is_additive_so_delta_j_equals_the_sum_of_its_term_differences():
    a = {"travel": 0.35, "response": 0.22, "time_loss": 0.26, "incomplete": 0.07, "teleports": 0.02}
    b = {"travel": 0.33, "response": 0.24, "time_loss": 0.25, "incomplete": 0.08, "teleports": 0.01}
    delta = sum(a.values()) - sum(b.values())
    check = heldout.decomposition(a, b, delta)
    assert check["additive_within_tolerance"] is True
    assert check["per_term"]["response"] == pytest.approx(-0.02)


def test_the_decomposition_reports_a_residual_when_it_does_not_add_up():
    a = {"travel": 0.35}
    check = heldout.decomposition(a, {"travel": 0.30}, 0.09)
    assert check["additive_within_tolerance"] is False and check["residual"] == pytest.approx(0.04)


# ---------------------------------------------------------------- the frozen specification

def test_the_report_specification_is_fixed_with_the_plan():
    spec = heldout.report_specification()
    assert spec["sign_conventions"]["primary_baseline_minus_T16"].startswith("positive = T16 faster")
    assert spec["sign_conventions"]["secondary_T17_minus_T16"].startswith("negative = T17 lower")
    assert "1.0 by construction" in spec["not_reported"][0]
    assert set(spec["per_run_layers"]) == {"raw", "baseline_relative", "weighted_terms"}
    assert spec["valid_response_statuses"] == ["valid", "valid_post_window"]


def test_the_gate_names_only_fields_the_checker_actually_compares():
    for field in ("J", "weights_requested", "metrics", "objective"):
        assert any(field in entry for entry in heldout.COMPARED_FIELDS)
    assert "trip records" not in heldout.COMPARED_FIELDS    # the exact wording is 'every tripinfo record'


def test_an_unknown_configuration_is_refused():
    with pytest.raises(SystemExit):
        heldout.candidate_file("trial_010")


def test_the_baseline_of_each_seed_is_its_own():
    paths = {s: heldout.reference_path(s) for s in heldout.seeds()}
    assert len(set(paths.values())) == len(paths)
    canonical = REPO_ROOT / "results/tuning_reference/seed_20260721_3600s/reference.json"
    assert canonical not in paths.values()          # the tuning seed's baseline is never reused
