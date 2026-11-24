"""Focused tests for the E3 Optuna tuning harness."""

import math

import pytest

from scripts.tune_e3_weights import EvaluationResult, normalize_weights


def test_normalize_weights_returns_simplex():
    weights = normalize_weights([4, 2, 4, 1, 1])

    assert set(weights) == {"pheromone", "heuristic", "protocol", "bco", "velocity"}
    assert math.isclose(sum(weights.values()), 1.0)
    assert weights["pheromone"] == pytest.approx(4 / 12)


def test_normalize_weights_rejects_invalid_values():
    with pytest.raises(ValueError):
        normalize_weights([0, 0, 0, 0, 0])
    with pytest.raises(ValueError):
        normalize_weights([1, -1, 1, 1, 1])
    with pytest.raises(ValueError):
        normalize_weights([1, 1, 1])


def test_evaluation_result_is_serializable():
    result = EvaluationResult(
        objective=0.98,
        relative={"travel": 1.01, "response": 0.9, "time_loss": 1.0, "incomplete": 0.95, "teleports": 0.7},
        metrics={"travel_s": 300.0, "time_loss_s": 150.0, "incomplete_share": 0.15, "teleports": 21,
                 "finished": 3300, "responses": {"404": {"value_s": 400.0, "status": "valid"}}},
        reroutes=3,
    )

    payload = result.to_dict()
    assert payload["objective"] == 0.98
    assert payload["rel_response"] == 0.9
    assert payload["finished"] == 3300
    assert payload["teleports"] == 21
    assert payload["response_404_s"] == 400.0


def test_normalize_weights_never_returns_a_negative_weight():
    """Audit T8: the sum fix-up on the last weight is clamped at zero."""
    weights = normalize_weights([1.0, 1.0, 1.0, 1.0, 1e-18])
    assert min(weights.values()) >= 0.0


def test_default_tuning_window_is_the_contracted_one():
    """The tuner used to default to 3000 s while config/scenario.yaml says tune at 3600 s."""
    from scripts.tune_e3_weights import contract_tuning_duration

    assert contract_tuning_duration() == 3600
