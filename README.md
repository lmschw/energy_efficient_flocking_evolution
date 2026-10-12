# Energy-Efficient Flocking in Self-Organized Robot Swarms

Code and data to reproduce the results of the paper *Energy-Efficient Flocking in Self-Organized
Robot Swarms* (Schwarzenbach, Mahdavi, et al.). Every robot runs a small neural network whose
weights are updated online by Hebbian (ABCD) rules. The rules are shared by the whole swarm and
evolved with CMA-ES in a 2D kinematic simulator with wind, drag and a battery model. The evolved
controller is compared with a hand-designed flocking baseline (Ferrante et al., 2012) in
simulation and on 7 real robots (Thymio + Crazyflie with Lighthouse positioning).

## Layout

```
simulation/         simulator, controller, baseline and CMA-ES training (train.py)
data/
  training_run/     output of the paper's training run: best genome + per-generation history per stage
                    (hebbian_save_battery_avoid_all_best.npy is the Stage 2 controller used everywhere)
  lj_baseline_rules.json   baseline parameters (paper Table "Simulation parameters for the baseline")
analysis/           one script per group of paper results; results/ (JSON) and figures/ are their outputs
hardware/
  deployment/       code that ran on the robots (Raspberry Pi side), calibration, launch tools
  firmware/         Crazyflie app that shares the robots' positions (GPL-3.0, see header)
  runs/             raw hardware trials: 40 baseline (lj_rep*) + 40 Stage 2 (plain_rep*) runs of 60 s
```

In the code the Stage 2 controller is called `plain_seed123` (the training run used seed 123)
and the baseline `lj_baseline`; in the hardware runs they are `plain` and `lj`.

## Setup

```bash
python -m venv venv && source venv/bin/activate
pip install -r requirements.txt
```

## Reproducing the paper

All analysis scripts are run from `analysis/` and are deterministic: re-running them on the
included genome gives exactly the numbers in the paper. Running the scripts in the order below
regenerates everything from scratch (about 7 minutes on a 22-core machine).

```bash
cd analysis
python leadership_metrics.py         # 30-seed leadership sweeps at n = 7, 10, 20 (inputs to the next two)
python paper_leadership.py
python hardware_analysis.py          # needs results/seed_sweep_table_n7.json
python paper_results.py
python swarm_size_sweep.py
python fitness_curves.py
python paper_methodology_figures.py
```

| Paper | Script | Output |
|---|---|---|
| Fig. Overview of the methodology | `paper_methodology_figures.py` | `figures/paper_method_overview` |
| Fig. Sensing (quadrants) | `paper_methodology_figures.py` | `figures/paper_method_sensing` |
| Fig. Wind-field model | `paper_methodology_figures.py` | `figures/paper_method_wind` |
| Tables: swarm experiment parameters, fitness per stage | — | `simulation/config.py` (`HEBBIAN_*`, `HEBBIAN_STAGE_FITNESS_WEIGHTS`) |
| Table: baseline parameters | — | `data/lj_baseline_rules.json` |
| Fig. Fitness evolution | `fitness_curves.py` | `figures/fitness_curves` |
| Fig. (a) distance vs. battery, Table: performance over 100 simulations | `paper_results.py` | `figures/paper_dist_vs_battery_n20`, `results/paper_results.json` |
| Fig. (b) battery awareness, Table: battery awareness | `paper_results.py` | `figures/paper_battery_awareness_boxplot`, `results/paper_results.json` (`battery_awareness`) |
| Fig. Trajectories Stage 1 vs. Stage 2 (1 and 5 robots) | `paper_results.py` | `figures/paper_trajectories_stage1_vs_stage2` |
| Table + Fig. Leadership metrics (n = 20; n = 10 numbers in the text) | `leadership_metrics.py`, `paper_leadership.py` | `results/seed_sweep_table_n{10,20}.json`, `results/paper_leadership_summary.json`, `figures/paper_leadership_n20` |
| Tables: leadership on real robots and matched n = 7 simulations, Fig. real robots, hardware movement numbers | `hardware_analysis.py` | `results/paper_hardware_lighthouse_summary.json`, `figures/hardware_lighthouse` |
| Fig. Swarm size | `swarm_size_sweep.py` | `results/swarm_size_sweep.json`, `figures/swarm_size_distance_battery` |

`hardware_analysis.py` caches the first-60-s variant of the n = 7 simulations in
`results/seed_sweep_table_n7_first60s.json`; delete that file to re-simulate it.

## Training the controller

```bash
cd simulation
python train.py --n-agents 20 --seed 123 --wind-grid 50 --output-dir training_output
```

Stage 1 (`walk_upwind`, 100 generations) then Stage 2 (`save_battery_avoid_all`, 200
generations), population 30, median of 3 simulations per candidate. The paper's run took about
16 hours; its output is `data/training_run/`. The CMA-ES library seeds its own random number
generator from the clock, so a new run will not reproduce that genome bit for bit. Point
`analysis/paths.py` (`TRAINING_RUN`) at a new run to analyse it instead. `--resume` checkpoints
every generation and continues an interrupted run.

## Hardware experiments

`hardware/deployment/README.md` describes the setup: Lighthouse calibration, firmware, radio IDs,
the Raspberry Pis. `FLEET.md` lists the fleet and the deployment adaptations. Both were written
during the experiments, so some details are specific to our lab. The launch tools read the
Pis' login user, ssh key and IP addresses from `hardware/deployment/tools/fleet.conf`, which
holds placeholders. Fill in your own values there, or keep them in a file outside the
repository and set `FLEET_CONF=/path/to/that/file`. The trials were run with

```bash
cd hardware/deployment
LAUNCHER=run_swarm_fast.sh tools/experiment_schedule.sh 30 60    # 30 x 60 s per condition, lj then plain
CONDITIONS="lj plain" INTERLEAVE=1 FIRST_REP=31 LAUNCHER=run_swarm_fast.sh tools/experiment_schedule.sh 40 60
```

and write their logs to `hardware/deployment/hardware_runs/`. `hardware/runs/README.md`
documents the format of a run. The paper uses the first 30 usable trials per controller in
repetition order (`hardware_analysis.py`). Each run's `run_info.json` refers to a code snapshot
under `hardware_runs/_code/` that is not included here. Those snapshots differ from
`hardware/deployment/` only in the launcher scripts; the controller code is the same.
