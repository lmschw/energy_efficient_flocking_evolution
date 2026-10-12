"""Swarm-size sweep of the paper (figure "swarm_size"): the Stage 2 controller (evolved with 20
robots) vs. the baseline for n_agents = 1..20, 30 seeds each. Also counts, per swarm size, the
runs in which the controller beats the baseline on distance and battery at the same time
(paired by seed).

Outputs results/swarm_size_sweep.json and figures/swarm_size_distance_battery.{png,pdf,svg}.

Usage: python swarm_size_sweep.py   (runs in parallel)
"""
import json
import os
from concurrent.futures import ProcessPoolExecutor

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import paths  # puts simulation/ on sys.path
import config  # noqa: E402
from hebbian_controller import unflatten_abcd  # noqa: E402
from simulation_hebbian import simulate_hebbian_episode  # noqa: E402
from lj_baseline import simulate_lj_baseline  # noqa: E402
from leadership_metrics import _ConfigOverride, WIND_GRID  # noqa: E402

N_AGENTS_SWEEP = tuple(range(1, 21))
SEEDS = list(range(1000, 1030))
OUT_DIR_FIG = paths.FIG_DIR
OUT_JSON = os.path.join(paths.RESULTS_DIR, "swarm_size_sweep.json")

NO_CLAMP = dict(HEBBIAN_SAFETY_CLAMP_ENABLED=False, HEBBIAN_MIN_DIST_INFLATION=1.0,
                HEBBIAN_RESOLVE_COLLISIONS=False)

CONDITIONS = {
    "lj_baseline": (None, None, "#77AC30"),
    "plain_seed123": (paths.STAGE2_GENOME, NO_CLAMP, "#D95319"),
}


def _run_scalar(args):
    """(label, n_agents, seed) -> (label, n_agents, seed, distance [m], battery [% of start])."""
    label, n_agents, seed = args
    genome_path, cfg, _color = CONDITIONS[label]
    if genome_path is None:
        rules = json.load(open(paths.LJ_RULES))
        with _ConfigOverride(HEBBIAN_NX=WIND_GRID, HEBBIAN_NY=WIND_GRID):
            _, dist, batt, _ = simulate_lj_baseline(rules=rules, seed=seed, n_agents=n_agents)
        return label, n_agents, seed, float(dist), float(batt / config.MAX_BATTERY * 100.0)

    rules = unflatten_abcd(np.load(genome_path))
    with _ConfigOverride(**cfg):
        dist, batt, _, _, _, _ = simulate_hebbian_episode(
            rules, seed=seed, n_agents=n_agents, wind_enabled=True, nx=WIND_GRID, ny=WIND_GRID)
    return label, n_agents, seed, float(dist), float(batt)


def main():
    jobs = [(label, n, s) for n in N_AGENTS_SWEEP for label in CONDITIONS for s in SEEDS]
    with ProcessPoolExecutor() as pool:
        runs = list(pool.map(_run_scalar, jobs))

    results = {label: {} for label in CONDITIONS}
    for n_agents in N_AGENTS_SWEEP:
        per_label_db = {}
        for label in CONDITIONS:
            rows = sorted((s, d, b) for (l, n, s, d, b) in runs if l == label and n == n_agents)
            per_label_db[label] = (np.array([r[1] for r in rows]), np.array([r[2] for r in rows]))

        lj_d, lj_b = per_label_db["lj_baseline"]
        for label, (dists, batts) in per_label_db.items():
            both_beat = None
            if label != "lj_baseline":
                both_beat = int(np.sum((dists >= lj_d) & (batts >= lj_b)))
            results[label][n_agents] = {
                "dist_mean": float(dists.mean()), "dist_std": float(dists.std()),
                "batt_mean": float(batts.mean()), "batt_std": float(batts.std()),
                "both_beat": both_beat,
            }
            beat_str = f"  both-beat={both_beat}/{len(SEEDS)}" if both_beat is not None else ""
            print(f"  n={n_agents:2d} {label:24s} dist={dists.mean():6.2f}+-{dists.std():5.2f} "
                  f"batt={batts.mean():6.2f}+-{batts.std():5.2f}%{beat_str}")

    os.makedirs(paths.RESULTS_DIR, exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved {OUT_JSON}")

    os.makedirs(OUT_DIR_FIG, exist_ok=True)
    ns = np.array(N_AGENTS_SWEEP)
    fig, (ax_d, ax_b) = plt.subplots(1, 2, figsize=(13, 5.5))
    for label, (_gp, _cfg, color) in CONDITIONS.items():
        dmean = np.array([results[label][n]["dist_mean"] for n in ns])
        dstd = np.array([results[label][n]["dist_std"] for n in ns])
        bmean = np.array([results[label][n]["batt_mean"] for n in ns])
        bstd = np.array([results[label][n]["batt_std"] for n in ns])
        ax_d.plot(ns, dmean, "o-", color=color, label=label)
        ax_d.fill_between(ns, dmean - dstd, dmean + dstd, color=color, alpha=0.15)
        ax_b.plot(ns, bmean, "o-", color=color, label=label)
        ax_b.fill_between(ns, bmean - bstd, bmean + bstd, color=color, alpha=0.15)
    ax_d.set_xlabel("n_agents"); ax_d.set_ylabel("Distance travelled [m]")
    ax_d.set_title("Distance vs. swarm size (mean ± 1 std, 30 seeds)")
    ax_d.set_xticks(ns); ax_d.grid(True, linestyle=":", alpha=0.6); ax_d.legend(fontsize=9)
    ax_b.set_xlabel("n_agents"); ax_b.set_ylabel("Remaining battery [% of full charge]")
    ax_b.set_title("Battery vs. swarm size (mean ± 1 std, 30 seeds)")
    ax_b.set_xticks(ns); ax_b.grid(True, linestyle=":", alpha=0.6); ax_b.legend(fontsize=9)
    fig.tight_layout()
    stem = os.path.join(OUT_DIR_FIG, "swarm_size_distance_battery")
    fig.savefig(stem + ".png", dpi=150); fig.savefig(stem + ".pdf"); fig.savefig(stem + ".svg")
    plt.close(fig)
    print(f"Saved {stem}.{{png,pdf,svg}}")


if __name__ == "__main__":
    main()
