"""Leading/following turn-taking metrics from the collective-motion literature (paper Section
"Leadership metrics"), applied to the evolved Stage 2 controller (condition id "plain_seed123":
the 2-stage curriculum run with training seed 123) and the baseline ("lj_baseline").

The simulator does not store per-step trajectories, so this script regenerates them from the
saved genome/rules (wind grid 50x50, matching training) and computes, per episode:
  1. Voelkl et al. (2015, PNAS) leading/following time-matching (reciprocity): correlation
     across agents between each agent's cumulative time in the front rank and in the back rank.
  2. Nagy, Akos, Biro & Vicsek (2010, Nature) directional correlation delay: pairwise
     cross-correlation of turning-rate time series at a range of time lags gives a directed,
     weighted leadership network, reduced to two scalars per trial: leadership entropy (flat =
     everyone initiates equally) and hierarchy steepness (steep = one agent dominates).
  3. Front-rank occupancy fraction + position-exchange rate: fraction of total time each agent
     spends as rank-1 (front-most) in the direction of travel (normalised Shannon entropy) and
     how often that identity changes hands (Mirzaeinia et al. 2019).
  4. Persistence-filtered front-position changes: a lightweight proxy for Butail & Porfiri
     (2019, Chaos)'s switch detector -- a change only counts when the new front robot keeps
     the position for at least 3 steps (1.5 s); shorter spells are merged into the holder.

Usage:
    python leadership_metrics.py      # 30-seed sweeps at n = 7, 10, 20 -> results/seed_sweep_table_n{N}.json
    python leadership_metrics.py --seed-sweep-n-agents 20
"""
import argparse
import json
import os

import numpy as np
from scipy import stats

import paths  # puts simulation/ on sys.path
import config  # noqa: E402
from hebbian_controller import unflatten_abcd  # noqa: E402
from simulation_hebbian import simulate_hebbian_episode  # noqa: E402
from lj_baseline import simulate_lj_baseline  # noqa: E402

SEED = config.HEBBIAN_DEFAULT_SEED  # 42
WIND_GRID = 50  # matches the training-time wind grid (train.py --wind-grid 50)
OUTPUT_DIR = paths.RESULTS_DIR

# The controller was trained (and is evaluated) without any of the simulator's optional
# collision-safety mechanisms.
_CONDITION_OVERRIDES = {
    "plain_seed123": dict(safety_clamp=False, min_dist_inflation=1.0, resolve_collisions=False),
}
_GENOME_PATH = {
    "plain_seed123": paths.STAGE2_GENOME,
}
_BASELINE_RULES_PATH = paths.LJ_RULES

CONDITIONS = ("plain_seed123", "lj_baseline")


class _ConfigOverride:
    """Temporarily patches module-level config.* attributes, restoring them on exit --
    simulate_hebbian_episode/simulate_lj_baseline read several safety/physics flags straight
    off the config module rather than accepting them as parameters."""

    def __init__(self, **overrides):
        self.overrides = overrides
        self._saved = {}

    def __enter__(self):
        for name, value in self.overrides.items():
            self._saved[name] = getattr(config, name)
            setattr(config, name, value)
        return self

    def __exit__(self, *exc):
        for name, value in self._saved.items():
            setattr(config, name, value)


BATTERY_SCALE = {  # battery_pct = average_batt / this * 100
    "lj_baseline": config.MAX_BATTERY,             # 150.0, the LJ model's own scale
    "plain_seed123": config.HEBBIAN_MAX_BATTERY,   # 100.0
}


def run_trajectory(condition, n_agents, seed=SEED):
    """Returns (positions, dt, dist_travelled, average_batt) -- positions is
    (n_steps+1, n_agents, 2); average_batt is on that condition's own raw scale (see
    BATTERY_SCALE, not yet converted to a percentage)."""
    dt = config.DT
    if condition == "lj_baseline":
        rules = json.load(open(_BASELINE_RULES_PATH))
        with _ConfigOverride(HEBBIAN_NX=WIND_GRID, HEBBIAN_NY=WIND_GRID):
            _, dist, batt, _, tele = simulate_lj_baseline(
                rules=rules, seed=seed, n_agents=n_agents, record_trajectory=True)
        return tele["positions"], dt, dist, batt

    overrides = _CONDITION_OVERRIDES[condition]
    genome = np.load(_GENOME_PATH[condition])
    rules = unflatten_abcd(genome)
    cfg_patch = dict(
        HEBBIAN_SAFETY_CLAMP_ENABLED=overrides["safety_clamp"],
        HEBBIAN_MIN_DIST_INFLATION=overrides["min_dist_inflation"],
        HEBBIAN_RESOLVE_COLLISIONS=overrides["resolve_collisions"],
    )
    if "resolve_strength" in overrides:
        cfg_patch["HEBBIAN_RESOLVE_COLLISIONS_STRENGTH"] = overrides["resolve_strength"]
        cfg_patch["HEBBIAN_RESOLVE_COLLISIONS_MAX_ITER"] = overrides["resolve_max_iter"]
    with _ConfigOverride(**cfg_patch):
        dist, batt, _, _, _, _, tele = simulate_hebbian_episode(
            rules, seed=seed, n_agents=n_agents, wind_enabled=True,
            nx=WIND_GRID, ny=WIND_GRID, record_trajectory=True)
    return tele["positions"], dt, dist, batt


# --- Shared derived signals: per-step bearing (movement direction) and front/back rank ---

def _bearings(positions, eps=1e-9):
    """Movement-direction heading per agent per step, from consecutive position fixes (the
    same GPS-fix-derived heading convention used in the ibis/pigeon/stork literature this is
    modeled on -- NOT the simulator's internal body-orientation state). Shape (T-1, N).
    A step where an agent doesn't move (e.g. fully braked by the safety clamp) has an
    undefined bearing; carries the previous step's heading forward rather than snapping to 0,
    since arctan2(0, 0) == 0 would otherwise inject a fake "due east" heading into the
    turning-rate signal used by the directional-correlation-delay network."""
    disp = np.diff(positions, axis=0)  # (T-1, N, 2)
    speed = np.hypot(disp[..., 0], disp[..., 1])
    bearing = np.arctan2(disp[..., 1], disp[..., 0])
    for t in range(1, bearing.shape[0]):
        stalled = speed[t] < eps
        bearing[t, stalled] = bearing[t - 1, stalled]
    return bearing


def _front_rank_series(positions):
    """Rank-1 (front-most along instantaneous flock heading) identity at every step with a
    defined bearing. Flock heading = circular mean of individual bearings that step. Returns
    ranks array (T-1, N) where ranks[t, i] = i's rank (0 = front/leading, N-1 = back/trailing)."""
    bearing = _bearings(positions)
    T_minus_1, n_agents = bearing.shape
    pos = positions[1:]  # align with bearing's time axis

    flock_heading = np.arctan2(np.mean(np.sin(bearing), axis=1), np.mean(np.cos(bearing), axis=1))
    heading_vec = np.stack([np.cos(flock_heading), np.sin(flock_heading)], axis=1)  # (T-1, 2)
    projection = np.einsum("tad,td->ta", pos, heading_vec)  # (T-1, N)

    order = np.argsort(-projection, axis=1)  # descending: index 0 = front-most agent id
    ranks = np.empty_like(order)
    np.put_along_axis(ranks, order, np.arange(n_agents)[None, :].repeat(T_minus_1, axis=0), axis=1)
    return ranks, bearing


# --- Metric 1: Voelkl et al. (2015) leading/following time-matching ---

def reciprocity_index(ranks, dt):
    n_agents = ranks.shape[1]
    lead_time = np.array([np.sum(ranks[:, i] == 0) for i in range(n_agents)]) * dt
    follow_time = np.array([np.sum(ranks[:, i] == n_agents - 1) for i in range(n_agents)]) * dt
    if n_agents < 3 or np.std(lead_time) == 0 or np.std(follow_time) == 0:
        r, p = np.nan, np.nan
    else:
        r, p = stats.pearsonr(lead_time, follow_time)
    return {"lead_time": lead_time, "follow_time": follow_time, "r": r, "p": p}


# --- Metric 3: front-rank occupancy + exchange rate ---

def occupancy_and_exchange(ranks, dt, positions):
    n_agents = ranks.shape[1]
    front_id = np.argmin(ranks, axis=1)  # rank-0 agent at each step
    occ_counts = np.bincount(front_id, minlength=n_agents)
    occupancy_fraction = occ_counts / len(front_id)

    p = occupancy_fraction[occupancy_fraction > 0]
    entropy = -np.sum(p * np.log(p))
    normalized_entropy = entropy / np.log(n_agents) if n_agents > 1 else np.nan

    switches = np.sum(front_id[1:] != front_id[:-1])
    duration = len(front_id) * dt
    centroid = positions.mean(axis=1)
    path_length = np.sum(np.hypot(*np.diff(centroid, axis=0).T))

    return {
        "occupancy_fraction": occupancy_fraction,
        "normalized_entropy": normalized_entropy,
        "raw_switches": int(switches),
        "exchange_rate_per_sec": switches / duration if duration > 0 else np.nan,
        "exchange_rate_per_m": switches / path_length if path_length > 0 else np.nan,
        "front_id": front_id,
    }


# --- Metric 4: persistence-filtered switch rate (simplified Butail & Porfiri proxy) ---

def _filtered_runs(front_id, min_dwell_steps=3):
    """Run-length-encodes front_id, merging runs shorter than min_dwell_steps into their
    predecessor (treating a hand-off that doesn't stick as jostling noise, not a real
    switch). Returns a list of [identity, length_in_steps]."""
    runs = []  # list of [identity, length]
    for val in front_id:
        if runs and runs[-1][0] == val:
            runs[-1][1] += 1
        else:
            runs.append([val, 1])

    filtered = [runs[0]]
    for ident, length in runs[1:]:
        if length < min_dwell_steps:
            filtered[-1][1] += length  # absorb short flicker into whatever identity currently holds
            continue
        if filtered[-1][0] == ident:
            filtered[-1][1] += length
        else:
            filtered.append([ident, length])
    return filtered


def persistence_filtered_switches(front_id, dt, path_length, min_dwell_steps=3):
    """Counts transitions in the persistence-filtered run sequence -- a simplified proxy for
    Butail & Porfiri's transfer-entropy switch detector, NOT the same method; see module
    docstring."""
    filtered = _filtered_runs(front_id, min_dwell_steps)
    real_switches = max(0, len(filtered) - 1)
    duration = len(front_id) * dt

    segments = []
    t = 0
    for ident, length in filtered:
        segments.append({"agent": int(ident), "start_s": t * dt, "end_s": (t + length) * dt})
        t += length

    return {
        "real_switches": real_switches,
        "rate_per_sec": real_switches / duration if duration > 0 else np.nan,
        "rate_per_m": real_switches / path_length if path_length > 0 else np.nan,
        "segments": segments,
    }


# --- Metric 2: Nagy/Akos/Biro/Vicsek (2010) directional correlation delay network ---

def _lagged_pearson(a, b, lag):
    """corr(a[t], b[t+lag]) over the valid overlap."""
    if lag == 0:
        x, y = a, b
    elif lag > 0:
        x, y = a[:-lag], b[lag:]
    else:
        x, y = a[-lag:], b[:lag]
    if len(x) < 3 or np.std(x) == 0 or np.std(y) == 0:
        return 0.0
    return float(np.corrcoef(x, y)[0, 1])


def directional_correlation_network(bearing, dt, max_lag_seconds=5.0):
    """bearing: (T-1, N) movement-direction time series. Returns the signed leadership matrix
    (leader_strength[i, j] = correlation strength of the edge i -> j, i.e. i's turning
    predicts j's turning after a positive delay; 0 if no pair relation or lag==0/ambiguous)
    and the per-pair best delay (seconds, positive)."""
    n_agents = bearing.shape[1]
    omega = np.diff(np.unwrap(bearing, axis=0), axis=0) / dt  # (T-2, N) turning rate
    max_lag = max(1, round(max_lag_seconds / dt))

    leader_strength = np.zeros((n_agents, n_agents))
    delay = np.zeros((n_agents, n_agents))
    for i in range(n_agents):
        for j in range(i + 1, n_agents):
            lags = np.arange(-max_lag, max_lag + 1)
            corrs = [_lagged_pearson(omega[:, i], omega[:, j], lag) for lag in lags]
            best = int(np.argmax(corrs))
            best_lag, best_corr = lags[best], corrs[best]
            if best_lag > 0 and best_corr > 0:
                leader_strength[i, j] = best_corr  # i leads j
                delay[i, j] = best_lag * dt
            elif best_lag < 0 and best_corr > 0:
                leader_strength[j, i] = best_corr  # j leads i
                delay[j, i] = -best_lag * dt
    return leader_strength, delay


def hierarchy_summary(leader_strength):
    n_agents = leader_strength.shape[0]
    out_strength = leader_strength.sum(axis=1)
    total = out_strength.sum()
    if total <= 0:
        return {"leadership_score": out_strength, "entropy": np.nan, "steepness": np.nan}

    p = out_strength / total
    p_nonzero = p[p > 0]
    entropy = -np.sum(p_nonzero * np.log(p_nonzero)) / np.log(n_agents) if n_agents > 1 else np.nan

    sorted_scores = np.sort(out_strength)[::-1]
    normalized = sorted_scores / sorted_scores[0]
    rank_frac = np.arange(n_agents) / (n_agents - 1) if n_agents > 1 else np.zeros(1)
    slope = stats.linregress(rank_frac, normalized).slope if n_agents > 1 else np.nan
    steepness = -slope

    return {"leadership_score": out_strength, "entropy": entropy, "steepness": steepness}


# --- Orchestration ---

def analyze(condition, n_agents, seed=SEED):
    positions, dt, dist_travelled, average_batt = run_trajectory(condition, n_agents, seed=seed)
    ranks, bearing = _front_rank_series(positions)

    recip = reciprocity_index(ranks, dt)
    occ = occupancy_and_exchange(ranks, dt, positions)
    centroid = positions.mean(axis=1)
    path_length = np.sum(np.hypot(*np.diff(centroid, axis=0).T))
    persist = persistence_filtered_switches(occ["front_id"], dt, path_length)
    leader_strength, delay = directional_correlation_network(bearing, dt)
    hierarchy = hierarchy_summary(leader_strength)

    return {
        "n_agents": n_agents, "dt": dt, "positions": positions,
        "dist_travelled": dist_travelled, "average_batt": average_batt,
        "battery_pct": average_batt / BATTERY_SCALE[condition] * 100.0,
        "reciprocity": recip, "occupancy": occ, "persistence": persist,
        "leader_strength": leader_strength, "delay": delay, "hierarchy": hierarchy,
    }


def run_seed_sweep(n_agents, seeds):
    """Per-episode metrics for every condition at a fixed n_agents, one episode per random
    seed (spawn positions) in `seeds`."""
    results = {}
    for condition in CONDITIONS:
        for i, seed in enumerate(seeds):
            print(f"\r  seed-sweep {condition} n={n_agents}: {i + 1}/{len(seeds)}", end="", flush=True)
            results[(condition, seed)] = analyze(condition, n_agents, seed=seed)
        print()
    return results


def save_seed_sweep_table(seed_results, n_agents, seeds, path):
    rows = []
    for condition in CONDITIONS:
        for seed in seeds:
            r = seed_results[(condition, seed)]
            rows.append({
                "condition": condition, "n_agents": n_agents, "seed": seed,
                "reciprocity_r": r["reciprocity"]["r"],
                "front_occupancy_normalized_entropy": r["occupancy"]["normalized_entropy"],
                "raw_switches": r["occupancy"]["raw_switches"],
                "exchange_rate_per_sec": r["occupancy"]["exchange_rate_per_sec"],
                "real_switches": r["persistence"]["real_switches"],
                "persistence_filtered_rate_per_sec": r["persistence"]["rate_per_sec"],
                "hierarchy_leadership_entropy": r["hierarchy"]["entropy"],
                "hierarchy_steepness": r["hierarchy"]["steepness"],
            })
    with open(path, "w") as f:
        json.dump(rows, f, indent=2)
    print(f"Saved {path}")
    return rows


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output-dir", default=OUTPUT_DIR)
    parser.add_argument("--seed-sweep-n-agents", type=int, nargs="*", default=[7, 10, 20],
                         help="Swarm sizes to sweep (paper: 20 and 10 for the simulation table, "
                              "7 for the comparison with the hardware trials).")
    parser.add_argument("--n-seeds", type=int, default=30)
    parser.add_argument("--seed-sweep-start", type=int, default=1000)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    seeds = list(range(args.seed_sweep_start, args.seed_sweep_start + args.n_seeds))
    for n_agents in args.seed_sweep_n_agents:
        print(f"\nSeed sweep: n_agents={n_agents}, {len(seeds)} seeds x {len(CONDITIONS)} conditions")
        seed_results = run_seed_sweep(n_agents, seeds)
        save_seed_sweep_table(seed_results, n_agents, seeds,
                               os.path.join(args.output_dir, f"seed_sweep_table_n{n_agents}.json"))
