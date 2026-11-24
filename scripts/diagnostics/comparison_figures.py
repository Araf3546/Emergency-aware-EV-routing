"""Figures for the V1 comparison report (C4, 2026-09-17).

Reads only the finished `report.json` and writes PNG files into a `figures/` folder, refusing any file that already
exists. It never opens a run folder, so a plotting failure cannot touch raw evidence.

Colours: the dataviz reference categorical palette in its fixed order (light mode), with the baseline in a neutral
grey. Every bar is named on its axis, so identity never rests on colour alone. One value axis per panel; measures on
different scales get separate panels.
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, List

SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e4e3df"
BASELINE_GREY = "#9b9a94"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
INCIDENTS = ("404", "401", "405")
SHORT = {"baseline": "No\nrerouting", "dijkstra": "Dijkstra", "astar": "A*", "aco": "ACO", "aco_pso": "ACO-\nPSO",
         "e3_off_t16": "E3 T16\nlayer\nOFF", "e3_on_t16": "E3 T16\nlayer ON\n(extension)"}


def _rows(report: dict) -> List[str]:
    return list(report["core_six_method_comparison"]) + list(report["emergency_aware_extension"])


def _colours(rows: List[str]) -> Dict[str, str]:
    colours, k = {}, 0
    for r in rows:
        if r == "baseline":
            colours[r] = BASELINE_GREY
        else:
            colours[r] = SERIES[k]
            k += 1
    return colours


def _style(ax, title: str, ylabel: str) -> None:
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=TEXT, fontsize=11, loc="left")
    ax.set_ylabel(ylabel, color=TEXT_2, fontsize=9)
    ax.tick_params(colors=TEXT_2, labelsize=8, length=0)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.yaxis.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def _per_seed(report: dict, row: str, getter) -> List[float]:
    values = []
    for block in report["per_seed"].values():
        try:
            v = getter(block[row])
        except (KeyError, TypeError):
            v = None
        if v is not None:
            values.append(v)
    return values


def _bars(ax, report: dict, rows: List[str], getter, colours) -> None:
    """Mean bar per row with each seed as a dot; the extension row is set apart by a gap."""
    xs = []
    for i, r in enumerate(rows):
        x = i + (0.6 if r in report["emergency_aware_extension"] else 0)
        xs.append(x)
        values = _per_seed(report, r, getter)
        if not values:
            continue
        mean = sum(values) / len(values)
        ax.bar(x, mean, width=0.62, color=colours[r], edgecolor=SURFACE, linewidth=2, zorder=2)
        ax.scatter([x] * len(values), values, s=16, color=TEXT, zorder=3, linewidths=0)
    ax.set_xticks(xs)
    ax.set_xticklabels([SHORT.get(r, r) for r in rows], color=TEXT_2, fontsize=8)


def _save(fig, path: Path, written: List[Path]) -> None:
    if path.exists():
        raise SystemExit(f"{path} exists; figures are written once")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=200, facecolor=SURFACE, bbox_inches="tight")
    written.append(path)


def make_all(report: dict, out_dir: Path) -> List[Path]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = _rows(report)
    colours = _colours(rows)
    written: List[Path] = []
    note = "Bars: mean of the five seeds. Dots: each seed."

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2), facecolor=SURFACE)
    for ax, incident in zip(axes, INCIDENTS):
        _bars(ax, report, rows, lambda x, i=incident: x["raw"]["responses"][i]["value_s"], colours)
        _style(ax, f"Incident {incident}: response time", "seconds (lower is faster)")
    fig.text(0.01, -0.04, note, color=TEXT_2, fontsize=8)
    _save(fig, out_dir / "fig1_response_per_incident.png", written)
    plt.close(fig)

    specs = [
        ("fig2_mean_normalized_response.png", "Mean normalized emergency response (baseline = 1.0)", "ratio",
         lambda x: x["normalized_response"]["mean"], 1.0),
        ("fig3_completion.png", "Trips completed (share of scheduled vehicles)", "share",
         lambda x: x["raw"]["completion_share"], None),
        ("fig4_teleports.png", "Teleports", "count", lambda x: x["raw"]["teleports"], None),
    ]
    for name, title, ylabel, getter, ref in specs:
        fig, ax = plt.subplots(figsize=(9, 4), facecolor=SURFACE)
        _bars(ax, report, rows, getter, colours)
        if ref is not None:
            ax.axhline(ref, color=TEXT_2, linewidth=1, linestyle="--", zorder=1)
        _style(ax, title, ylabel)
        fig.text(0.01, -0.06, note, color=TEXT_2, fontsize=8)
        _save(fig, out_dir / name, written)
        plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(14, 4.2), facecolor=SURFACE)
    _bars(axes[0], report, rows, lambda x: x["raw"]["travel_s"], colours)
    _style(axes[0], "Travel time, every scheduled vehicle (censored at 3,600 s)", "seconds")
    _bars(axes[1], report, rows, lambda x: x["raw"]["time_loss_s"], colours)
    _style(axes[1], "Time loss, every vehicle that entered", "seconds")
    fig.text(0.01, -0.05, note + " Read next to completion (fig3).", color=TEXT_2, fontsize=8)
    _save(fig, out_dir / "fig5_travel_and_time_loss.png", written)
    plt.close(fig)

    terms = ["travel", "response", "time_loss", "incomplete", "teleports"]
    j_rows = [r for r in rows if r != "baseline"]
    fig, ax = plt.subplots(figsize=(9, 4.4), facecolor=SURFACE)
    xs = [i + (0.6 if r in report["emergency_aware_extension"] else 0) for i, r in enumerate(j_rows)]
    bottoms = [0.0] * len(j_rows)
    for k, term in enumerate(terms):
        heights = []
        for r in j_rows:
            vals = _per_seed(report, r, lambda x, t=term: x["terms"][t])
            heights.append(sum(vals) / len(vals) if vals else 0.0)
        ax.bar(xs, heights, bottom=bottoms, width=0.62, color=SERIES[k], edgecolor=SURFACE, linewidth=2,
               label=term.replace("_", " "), zorder=2)
        bottoms = [b + h for b, h in zip(bottoms, heights)]
    ax.axhline(1.0, color=TEXT_2, linewidth=1, linestyle="--", zorder=1)
    ax.set_xticks(xs)
    ax.set_xticklabels([SHORT.get(r, r) for r in j_rows], color=TEXT_2, fontsize=8)
    _style(ax, "J and its five weighted terms (mean of five seeds; baseline J = 1.0)", "J")
    ax.legend(frameon=False, fontsize=8, labelcolor=TEXT_2, ncol=5, loc="upper center", bbox_to_anchor=(0.5, -0.18))
    _save(fig, out_dir / "fig6_J_terms.png", written)
    plt.close(fig)

    parts = [("dispatch_delay_s", "dispatch delay"), ("free_flow_s", "free-flow driving"), ("delay_s", "delay in traffic")]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4), facecolor=SURFACE)
    for ax, incident in zip(axes, INCIDENTS):
        xs = [i + (0.6 if r in report["emergency_aware_extension"] else 0) for i, r in enumerate(rows)]
        bottoms = [0.0] * len(rows)
        for k, (key, label) in enumerate(parts):
            heights = []
            for r in rows:
                vals = _per_seed(report, r, lambda x, i=incident, kk=key: x["responders"][i][kk])
                heights.append(sum(vals) / len(vals) if vals else 0.0)
            ax.bar(xs, heights, bottom=bottoms, width=0.62, color=SERIES[k], edgecolor=SURFACE, linewidth=2,
                   label=label, zorder=2)
            bottoms = [b + h for b, h in zip(bottoms, heights)]
        ax.set_xticks(xs)
        ax.set_xticklabels([SHORT.get(r, r) for r in rows], color=TEXT_2, fontsize=7)
        _style(ax, f"Incident {incident}: where the response time goes", "seconds (mean of five seeds)")
    axes[1].legend(frameon=False, fontsize=8, labelcolor=TEXT_2, ncol=3, loc="upper center",
                   bbox_to_anchor=(0.5, -0.22))
    _save(fig, out_dir / "fig7_responder_delay.png", written)
    plt.close(fig)
    return written
