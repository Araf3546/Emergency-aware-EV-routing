#!/usr/bin/env python3
"""How many tuning trials, and how many nudge repeats, are worth running? A noise model (2026-09-15).

Diagnostic and analysis only: it runs no simulation, reads nothing and changes nothing. Its output backs the
tuning-budget and robustness-protocol proposal in results/tuning_budget_analysis_2026-09-15/README.md.

Model (random search, a conservative stand-in for TPE):
- the true J of a weight set is drawn from N(0, s^2);
- one 3,600 s run observes the true J plus N(0, n^2) of chaos.
Tuning is N trials with one run each. The finalists are the top 5 by that run, plus DEFAULT (another draw).
Robustness gives r fresh nudge runs per finalist, and the winner is the lowest mean of the fresh runs.

Calibration, from existing runs:
- n about 0.06: default weights, the unperturbed run plus three one-car nudges on 2026-09-15 gave J 1.0858, 1.2097,
  1.1780 and 1.1029 (results/yield_signal_diagnosis_2026-09-14/README.md), SD 0.059;
- s about 0.06: J across weight sets has SD 0.084 (campaign 2b, five trials) and 0.088 (default, t0, t1 on the
  current code), and sqrt(0.085^2 - 0.06^2) is about 0.06. The cases s = 0.03 and 0.09, and n = 0.04, bound it.

    python scripts/diagnostics/tuning_budget_model.py
"""
import numpy as np

rng = np.random.default_rng(1)
M = 40000

def sim(N, r, s, n, top=5, reuse=False):
    mu = rng.normal(0, s, (M, N)); obs = mu + rng.normal(0, n, (M, N))
    order = np.argsort(obs, axis=1)[:, :top]
    fin_mu = np.take_along_axis(mu, order, 1)
    d_mu = rng.normal(0, s, (M, 1))
    cand = np.concatenate([fin_mu, d_mu], 1)
    fresh = cand[:, :, None] + rng.normal(0, n, (M, top + 1, r))
    score = fresh.mean(2)
    if reuse:   # also average in the (selected, optimistic) tuning run for the tuned finalists
        tun = np.take_along_axis(obs, order, 1)
        score = np.concatenate([(fresh[:, :top].sum(2) + tun) / (r + 1), score[:, top:]], 1)
    w = np.argmin(score, 1)
    win_mu = cand[np.arange(M), w]
    best_all = np.minimum(mu.min(1), d_mu[:, 0])
    return dict(win=win_mu.mean(), regret=(win_mu - best_all).mean(),
                p_best_in_top5=(np.argmin(mu, 1)[:, None] == order).any(1).mean(),
                p_pick_best_finalist=(w == np.argmin(cand, 1)).mean(),
                p_worse_than_default=(win_mu > d_mu[:, 0] + 1e-12).mean(),
                optimism=(np.take_along_axis(obs, order[:, :1], 1)[:, 0] - np.take_along_axis(mu, order[:, :1], 1)[:, 0]).mean())

for s, n in [(0.06, 0.06), (0.03, 0.06), (0.09, 0.06), (0.06, 0.04)]:
    print(f"\n=== signal SD {s}, run noise SD {n}  (J units; negative = better than an average random weight set)")
    print(" N  r | winner true J | regret vs best | P(true best in top5) | P(pick best of 6) | P(winner worse than DEFAULT) | optimism of tuning best")
    for N in (10, 15, 20, 25):
        for r in (3, 4, 6):
            x = sim(N, r, s, n)
            print(f"{N:2d} {r:2d} | {x['win']:+.3f}        | {x['regret']:.3f}          | {x['p_best_in_top5']:.2f}                 | {x['p_pick_best_finalist']:.2f}              | {x['p_worse_than_default']:.2f}                         | {x['optimism']:+.3f}")
x0, x1 = sim(20, 4, 0.06, 0.06), sim(20, 4, 0.06, 0.06, reuse=True)
print(f"\nN=20 r=4, fresh-only ranking: winner {x0['win']:+.3f}; reusing the tuning run in the mean: winner {x1['win']:+.3f}")
