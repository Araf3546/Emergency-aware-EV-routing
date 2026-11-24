"""The two supplementary response metrics (2026-09-16).

Post-hoc descriptive metrics over completed runs. These tests pin their arithmetic and their sign convention, and
check that the generated percentages agree with the pre-registered held-out report rather than drifting from it.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
REPORT = REPO_ROOT / "results/heldout_eval_2026-09-16/heldout_report.json"
SUPPLEMENT = REPO_ROOT / "results/supplementary_metrics_2026-09-16/supplementary_metrics.json"


def _module():
    spec = importlib.util.spec_from_file_location(
        "response_delay_metrics", REPO_ROOT / "scripts/diagnostics/response_delay_metrics.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


metrics = _module()


# ---------------------------------------------------------------- metric 1: delay decomposition

def test_the_decomposition_adds_back_up_to_the_response():
    split = metrics.delay_decomposition(duration_s=568.0, time_loss_s=487.3, dispatch_delay_s=60.0)
    assert split["free_flow_s"] == pytest.approx(80.7)
    assert split["delay_s"] == pytest.approx(487.3)
    assert split["accounted_response_s"] == pytest.approx(60.0 + 568.0)


def test_a_trip_with_no_delay_is_all_free_flow():
    split = metrics.delay_decomposition(duration_s=100.0, time_loss_s=0.0, dispatch_delay_s=60.0)
    assert split["free_flow_s"] == pytest.approx(100.0) and split["delay_s"] == 0.0


# ---------------------------------------------------------------- metric 2: percentage change

def test_faster_than_baseline_is_positive_seconds_and_positive_percent():
    change = metrics.percentage_change(335.0, 179.0)
    assert change["difference_s"] == pytest.approx(156.0)
    assert change["percent"] == pytest.approx(46.567164, abs=1e-5)
    assert change["ratio"] == pytest.approx(179.0 / 335.0)


def test_slower_than_baseline_is_negative_seconds_and_negative_percent():
    change = metrics.percentage_change(335.0, 629.0)
    assert change["difference_s"] == pytest.approx(-294.0)
    assert change["percent"] == pytest.approx(-87.76119, abs=1e-4)
    assert change["ratio"] == pytest.approx(1.877611, abs=1e-5)


def test_an_identical_response_is_zero_change():
    change = metrics.percentage_change(300.0, 300.0)
    assert change["difference_s"] == 0.0 and change["percent"] == 0.0 and change["ratio"] == 1.0


@pytest.mark.parametrize("status", ("teleported", "not reached", None))
def test_a_censored_measurement_gives_no_percentage(status):
    assert metrics.percentage_change(335.0, 179.0, "valid", status)["percent"] is None
    assert metrics.percentage_change(335.0, 179.0, status, "valid")["percent"] is None


def test_both_valid_statuses_are_accepted():
    assert metrics.percentage_change(335.0, 179.0, "valid_post_window", "valid")["percent"] is not None


def test_undefined_values_are_excluded_from_the_summary_not_zeroed():
    summary = metrics.summarise([10.0, None, 20.0])
    assert summary["n"] == 2 and summary["mean"] == pytest.approx(15.0)
    assert metrics.summarise([None, None])["mean"] is None


# ---------------------------------------------------------------- agreement with the frozen report

@pytest.mark.skipif(not SUPPLEMENT.is_file(), reason="the supplementary metrics have not been generated")
def test_every_ratio_matches_the_pre_registered_report():
    report = json.loads(REPORT.read_text(encoding="utf-8"))
    supplement = json.loads(SUPPLEMENT.read_text(encoding="utf-8"))
    for seed, block in report["per_seed"].items():
        for label in ("trial_016", "trial_017"):
            for incident in ("404", "401", "405"):
                pre = block[label]["normalized_response"]["per_incident"][incident]
                new = supplement["per_seed"][seed][incident]["against_baseline"][label]["ratio"]
                assert new == pytest.approx(pre, abs=1e-12), (seed, label, incident)


@pytest.mark.skipif(not SUPPLEMENT.is_file(), reason="the supplementary metrics have not been generated")
def test_the_responder_route_never_changed_and_was_never_rerouted():
    supplement = json.loads(SUPPLEMENT.read_text(encoding="utf-8"))
    for seed, incidents in supplement["per_seed"].items():
        for incident, entry in incidents.items():
            lengths = [arm["route_length_m"] for arm in entry["arms"].values()]
            assert max(lengths) - min(lengths) < 5.0, (seed, incident, lengths)
            assert all(arm["responder_reroutes"] == 0 for arm in entry["arms"].values()), (seed, incident)
