"""Repository paths shared by the analysis scripts. Importing this module also puts simulation/
on sys.path, so `import config`, `simulation_hebbian`, ... resolve to the simulator the
controller was trained with."""
import os
import sys

ANALYSIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(ANALYSIS_DIR)
SIM_DIR = os.path.join(REPO_ROOT, "simulation")
DATA_DIR = os.path.join(REPO_ROOT, "data")

TRAINING_RUN = os.path.join(DATA_DIR, "training_run")  # output of simulation/train.py
STAGE1_GENOME = os.path.join(TRAINING_RUN, "hebbian_walk_upwind_best.npy")
STAGE2_GENOME = os.path.join(TRAINING_RUN, "hebbian_save_battery_avoid_all_best.npy")
LJ_RULES = os.path.join(DATA_DIR, "lj_baseline_rules.json")

RESULTS_DIR = os.path.join(ANALYSIS_DIR, "results")
FIG_DIR = os.path.join(ANALYSIS_DIR, "figures")
HW_RUNS = os.path.join(REPO_ROOT, "hardware", "runs")

sys.path.insert(0, SIM_DIR)
