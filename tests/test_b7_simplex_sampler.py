"""Audit B7 (2026-09-14): the tuner samples the weight simplex uniformly and one-to-one.

It used to draw five U(0, 1) values and divide by their sum. That puts little mass near the
corners: any one weight exceeds 0.5 in only 4.2% of draws, against 31.25% for a uniform draw. And
because the score divides by the total weight, one of the five sampled dimensions carried no
information. Four stick parameters are now mapped by inverse-CDF stick-breaking, in the existing
component order (the user's decision).
"""

import argparse
import random
import statistics

import pytest

import scripts.tune_e3_weights as tuner
from scripts.tune_e3_weights import (
    SAMPLER_DESCRIPTION,
    STICK_PARAMS,
    WEIGHT_NAMES,
    EvaluationResult,
    normalize_weights,
    simplex_weights,
    trial_weights,
)


def inverse(weights):
    """The one parameter point that produces a weight set."""
    remaining, sticks = 1.0, []
    for k, name in enumerate(WEIGHT_NAMES[:-1]):
        fraction = weights[name] / remaining if remaining > 0 else 0.0
        sticks.append(1.0 - (1.0 - fraction) ** (len(WEIGHT_NAMES) - 1 - k))
        remaining -= weights[name]
    return sticks


def test_four_parameters_for_five_weights_in_the_existing_order():
    assert STICK_PARAMS == ("stick_1", "stick_2", "stick_3", "stick_4")
    assert list(simplex_weights([0.5] * 4)) == list(WEIGHT_NAMES)
    assert "stick-breaking" in SAMPLER_DESCRIPTION and ", ".join(WEIGHT_NAMES) in SAMPLER_DESCRIPTION


@pytest.mark.parametrize("sticks", [[0, 0, 0, 0], [1, 1, 1, 1], [0.5] * 4, [0.2, 0.9, 0.1, 0.7],
                                    [1, 0, 0, 0], [0, 0, 0, 1], [0.999999, 0.5, 0.5, 0.5]])
def test_weights_are_non_negative_and_sum_to_one(sticks):
    w = simplex_weights(sticks)
    assert all(v >= 0.0 for v in w.values())
    assert sum(w.values()) == pytest.approx(1.0, abs=1e-12)


def test_the_map_is_one_to_one():
    rng = random.Random(3)
    for _ in range(2000):
        u = [rng.random() for _ in range(4)]
        assert inverse(simplex_weights(u)) == pytest.approx(u, abs=1e-9)


def test_the_induced_prior_is_uniform_on_the_simplex():
    rng = random.Random(11)
    n = 40000
    draws = [simplex_weights([rng.random() for _ in range(4)]) for _ in range(n)]
    for name in WEIGHT_NAMES:                         # every component alike, whatever its position
        assert statistics.fmean(d[name] for d in draws) == pytest.approx(0.2, abs=0.005)
        assert sum(d[name] > 0.3 for d in draws) / n == pytest.approx(0.7 ** 4, abs=0.01)
    assert sum(max(d.values()) > 0.5 for d in draws) / n == pytest.approx(5 * 0.5 ** 4, abs=0.01)


def test_the_old_sampler_is_what_b7_replaced():
    rng = random.Random(11)
    n = 40000
    old = [normalize_weights([rng.random() for _ in range(5)]) for _ in range(n)]
    assert sum(max(d.values()) > 0.5 for d in old) / n == pytest.approx(0.042, abs=0.005)


def test_stored_trials_are_read_under_either_parameterisation():
    assert trial_weights({name: 0.5 for name in STICK_PARAMS}) == simplex_weights([0.5] * 4)
    assert trial_weights(dict(zip(WEIGHT_NAMES, [4, 2, 4, 1, 1]))) == normalize_weights([4, 2, 4, 1, 1])
    assert trial_weights({}) is None


def test_the_objective_samples_the_four_stick_parameters(tmp_path, monkeypatch):
    class Trial:
        number = 0

        def __init__(self):
            self.names, self.user_attrs = [], {}

        def suggest_float(self, name, low, high):
            self.names.append((name, low, high))
            return 0.3

        def set_user_attr(self, key, value):
            self.user_attrs[key] = value

    seen = {}

    def evaluate(weights, sumo_cfg, sim_time, seed, trial_dir=None, reference=None):
        seen.update(weights)
        return EvaluationResult(objective=1.0, relative={}, metrics={}, reroutes=0)

    monkeypatch.setattr(tuner, "evaluate_weights", evaluate)
    trial = Trial()
    args = argparse.Namespace(output_dir=tmp_path, sumo_cfg=None, sim_time=60, seed=1, reference_data={})
    tuner._objective(trial, args)
    assert trial.names == [(name, 0.0, 1.0) for name in STICK_PARAMS]
    assert trial.user_attrs["weights"] == seen == simplex_weights([0.3] * 4)
