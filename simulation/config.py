"""Central configuration of the simulator, the Hebbian ABCD controller, the baseline and the
CMA-ES training for "Energy-Efficient Flocking in Self-Organized Robot Swarms".

Stage 1 of the curriculum ("walk_upwind") trains WITH wind enabled and rewards net progress
against the wind (along WIND_DIRECTION below). The robots have no wind sensor: battery level
(which drains faster the more a robot fights the wind), heading and the neighbour quadrants are
their only inputs. The paper's training (train.py) skips the intermediate stage
"save_battery_avoid_wall" and runs walk_upwind -> save_battery_avoid_all.

Several mechanisms in here (safety clamp, min-distance inflation, soft collision resolution,
instant death on collision, graduated proximity penalty, cohesion term, empty-quadrant zero
encoding) are optional extensions. Some of them default to ON below, but all of them are OFF
for the paper's controller: train.py and every analysis script override the defaults.

Paper settings that differ from the defaults in this file:
  HEBBIAN_SAFETY_CLAMP_ENABLED = False, HEBBIAN_MIN_DIST_INFLATION = 1.0,
  HEBBIAN_RESOLVE_COLLISIONS = False, and a 50 x 50 wind grid (HEBBIAN_NX/NY, passed as nx/ny).
"""

import math

# --- Core simulation parameters (shared by the wind/drag/battery physics and the baseline) ---
DT = 0.5                    # time-step [s]
ROBOT_RAD = 0.055            # robot radius [m]
WIND_RAD = 0.15              # robot's wind-occlusion radius [m]
X_RANGE = [-5.0, 5.0]        # simulation X bounds [m]
Y_RANGE = [-5.0, 5.0]        # simulation Y bounds [m] (the corridor walls)
V_WIND = 10.0                # freestream wind speed [m/s]

# --- Agent spawning ---
SPAWN_SQUARE_SIZE = 3.0      # side length of the square agents are randomly spawned in [m]
SPAWN_MIDPOINT = [0.0, 0.0]  # center of the spawn square
SPAWN_MIN_DIST_SLACK = 0.1   # extra slack (on top of 2*ROBOT_RAD) enforced between spawned agents

# --- Collision / walls ---
COLLISION_MIN_DIST_SLACK = 0.01      # min_dist = COLLISION_MIN_DIST_SLACK + 2*ROBOT_RAD
WALL_MARGIN_FACTOR = 0.5             # wall_margin = ROBOT_RAD * WALL_MARGIN_FACTOR
WALL_COLLISION_WEIGHT = 3            # each wall hit counts as this many collisions (baseline only)

# Optional (paper: 1.0). Multiplies min_dist for collision counting, instant death and the
# proximity penalty -- NOT for sensing or wind -- so that training treats robots as bigger than
# they are (a sim-to-real margin).
HEBBIAN_MIN_DIST_INFLATION = 1.3

# Optional (paper: False). A hard, non-learned velocity override: _move() (simulation_hebbian.py)
# caps each agent's FORWARD speed as a function of its current gap to the nearest neighbour,
# full speed above OUTER_GAP falling linearly to zero at INNER_GAP, whatever the network commands.
# Angular velocity is untouched, so a braked agent can still steer away. Always uses the true
# ROBOT_RAD (not HEBBIAN_MIN_DIST_INFLATION). The OUTER-INNER zone (0.25 m) is wider than the
# largest distance two agents can close in one step (2 * 0.2 m/s * 0.5 s = 0.2 m).
HEBBIAN_SAFETY_CLAMP_ENABLED = True
HEBBIAN_SAFETY_CLAMP_OUTER_GAP = 0.30   # gap at which braking begins (full speed above this)
HEBBIAN_SAFETY_CLAMP_INNER_GAP = 0.05   # gap at which forward speed reaches zero

# Same braking against the walls, switched by HEBBIAN_SAFETY_CLAMP_ENABLED as well. The zone is
# narrower than the agent-agent one because a wall does not move (worst-case closing distance
# per step: 0.1 m).
HEBBIAN_WALL_SAFETY_CLAMP_OUTER_GAP = 0.15   # gap to the wall at which braking begins
HEBBIAN_WALL_SAFETY_CLAMP_INNER_GAP = 0.02   # gap at which forward speed reaches zero (~contact)

# Optional (paper: False). If True, _move() (simulation_hebbian.py) pushes overlapping agents
# apart toward min_dist every step, after the collision-count/fitness bookkeeping and before the
# wall clamp: each of the MAX_ITER passes moves a pair STRENGTH of the way to min_dist, so with
# 0.5 / 2 a hard pile-up is only partly resolved within one step. (The MATLAB reference,
# simulation_free_global_mod_2.m's move(), contains this resolution but has it commented out.)
# The fitness's collision count is unaffected.
HEBBIAN_RESOLVE_COLLISIONS = True
HEBBIAN_RESOLVE_COLLISIONS_STRENGTH = 0.5
HEBBIAN_RESOLVE_COLLISIONS_MAX_ITER = 2

# Optional (paper: False). If True, simulate_hebbian_episode() ends the WHOLE episode the moment
# any pair of agents crosses min_dist, like the battery-empty termination. Pointless together
# with HEBBIAN_RESOLVE_COLLISIONS (which keeps pairs at or above min_dist). With 20 robots it
# ends almost every early-generation episode within a few steps, leaving CMA-ES no gradient.
HEBBIAN_COLLISION_INSTANT_DEATH = False

# --- Graduated proximity penalty (optional; active only for a stage whose proximity_w in
# HEBBIAN_STAGE_FITNESS_WEIGHTS is not None -- None in every stage below) ---
# Continuous penalty on the inter-agent surface gap (centre distance minus 2*ROBOT_RAD):
#   gap >= OUTER_GAP                 : no penalty
#   INNER_GAP <= gap < OUTER_GAP     : rises quadratically 0 -> 1 as the gap shrinks to INNER_GAP
#   contact_gap <= gap < INNER_GAP   : rises quadratically 1 -> 1 + STEEP_MULT towards contact
#   gap < contact_gap (D < min_dist) : a collision (counted as before)
# The fitness subtracts proximity_penalty / proximity_w. It is meant to replace, not add to,
# the flat inter-robot collision term (include_inter_robot_collision = False for that stage).
HEBBIAN_PROXIMITY_OUTER_GAP = 1.0 * ROBOT_RAD   # 0.055 m
HEBBIAN_PROXIMITY_INNER_GAP = 0.5 * ROBOT_RAD   # 0.0275 m
HEBBIAN_PROXIMITY_STEEP_MULT = 10.0

# --- Wind-tracking camera window (the x-range RayTraceCircularRobots is evaluated over) ---
WIND_TRACKING_WINDOW_WIDTH = 10.0    # total width of the tracking window [m] (nominally X_RANGE's span)
WIND_TRACKING_MAX_SPAN = 9.8         # cap on the swarm's own x-extent within that window [m]

# --- Wind / wake ray-tracing (RayTraceCircularRobots) ---
UINF = 100.0                  # freestream ("full power") wind value
KAPPA = 10.0                  # drag scale factor in F_drag = 0.5*DRAG_AIR_DENSITY*DRAG_COEFFICIENT_AREA*
                              # KAPPA*v_rel^2 (paper Eq. F_drag; MATLAB reference: kappa = 10)
WAKE_RECOVERY_RATE = 1.0      # fraction of wake gap recovered per grid step outside a robot's radius
WAKE_PERCENT_DROP = 0.25      # wind intensity drop on entering/switching a robot's wake
WAKE_MAX_WALL_SPAN = 0.7      # controls how sharply the wall effect kicks in (lower = more wall effect)
WAKE_MIN_POWER_X = 30.0       # floor applied to power immediately behind a robot
WAKE_MIN_POWER_Y = 10.0       # floor applied to power after the wall-effect pass
WAKE_ALPHA = 0.5              # decay rate of the first smoothing kernel
WAKE_BETA = 0.5               # decay rate of the second smoothing kernel
WAKE_X_SMOOTHING_1 = 100      # first-pass smoothing kernel size divisor (x)
WAKE_Y_SMOOTHING_1 = 50       # first-pass smoothing kernel size divisor (y)
WAKE_X_SMOOTHING_2 = 50       # second-pass smoothing kernel size divisor (x)
WAKE_Y_SMOOTHING_2 = 50       # second-pass smoothing kernel size divisor (y)
WAKE_THR_OK_DELTA = 1.0       # a cell counts as "free-stream" once within this much of UINF

# --- Drag force (dragforce) ---
DRAG_UPSTREAM_LOOKAHEAD_FACTOR = 1.1   # how far upstream (in wind_rad) to sample the wind grid
DRAG_AIR_DENSITY = 1.225                # kg/m^3
DRAG_COEFFICIENT_AREA = 0.0045          # effective drag coefficient * frontal area

# --- Battery drainage (batterydrainage) ---
BATTERY_WHEEL_POWER_DIVISOR = 4.0    # P_mot = (|v_L| + |v_R|) / this  (paper: gamma = 1/4)
BATTERY_MIN_DRAIN = 0.10             # floor on per-step drain (idle power draw, paper: P_idle)
BATTERY_DRAIN_SCALE = 2.0            # c in B(t+dt) = B(t) - c * P_use * dt (paper: c = 2; the MATLAB
                                     # reference, simulation_free_global_mod_2.m, also drains 2*batt_drain)

# --- Video output (render_hebbian_episode_video in simulation_hebbian.py; not used by the paper's scripts) ---
HEBBIAN_VIDEO_PATH = "hebbian_alone.mp4"
VIDEO_FPS = 10.0
VIDEO_SIZE = (1200, 800)
VIDEO_FIGSIZE = (12, 8)
VIDEO_VIEWPORT_HALF_WIDTH = 5.0   # camera half-width/height around the swarm's center of mass [m]
VIDEO_ARROW_LEN = 0.3             # heading-arrow length in the rendered frame [m]
VIDEO_QUIVER_WIDTH = 0.004        # heading-arrow line width

# --- Default seed for single example episodes (e.g. the paper's trajectory figures) ---
HEBBIAN_DEFAULT_SEED = 42

# =====================================================================================
# --- Hebbian ABCD neural-network controller ---
# Each robot runs a 10-10-10-2 MLP (ReLU, ReLU, tanh) whose weights are updated online by a
# Hebbian rule; the rule's coefficients (not the weights themselves) are what CMA-ES evolves,
# shared by every agent in a swarm. See hebbian_controller.py, sensor_model.py,
# simulation_hebbian.py, optimize_hebbian.py. MATLAB reference (Mahdavi et al.): hebbianStep.m
# (forward pass + update), simulation_free_global_mod_2.m's getsensordata()/W(i) init loop.
# =====================================================================================

# --- Robot & sensing ---
HEBBIAN_N_AGENTS = 20             # default swarm size (the paper trains with 20)
HEBBIAN_SENSING_RADIUS = 2.01     # R: neighbor detection radius [m]; also the "no neighbor" default distance
# Optional (paper: False). If True, an EMPTY quadrant feeds (0, 0) for its (distance, bearing)
# inputs instead of the default (+1, -1) (distance = R, bearing = 0 after rescaling). A genome
# trained with this ON must also be evaluated and deployed with it ON.
HEBBIAN_EMPTY_QUADRANT_ZERO = False
HEBBIAN_LINEAR_VEL_MAX = 0.2      # m/s, tanh output #1 rescaled to [-this, this]
HEBBIAN_ANGULAR_VEL_MAX = math.pi / 5  # rad/s, tanh output #2 rescaled to [-this, this]

# --- Battery & wind grid ---
# The battery model and the sensor's B/50-1 normalisation are defined over B in [0, 100].
HEBBIAN_MAX_BATTERY = 100.0        # starting battery for all agents but one
HEBBIAN_MIN_BATTERY = 100.0        # starting battery for the single "weakest" agent (the paper's
                                   # battery-awareness test passes min_battery=50 instead)
HEBBIAN_NX = 200                   # wind grid resolution; the paper uses 50 x 50 (passed as nx/ny)
HEBBIAN_NY = 200                   # (the O(Nx) wake-marching loop dominates per-step cost)

# --- Neural controller architecture ---
HEBBIAN_N_INPUTS = 10             # 4 quadrants x (distance, bearing) + battery + compass heading
HEBBIAN_N_HIDDEN = 10             # both hidden layers
HEBBIAN_N_OUTPUTS = 2             # (v, w)
HEBBIAN_LEARNING_RATE = 0.1       # mu in delta_w = mu*(A*ni*nj + B*ni + C*nj + D)
# Weight-matrix shapes, in flatten/unflatten order:
# W1: N_INPUTS x N_HIDDEN, W2: N_HIDDEN x N_HIDDEN, W3: N_HIDDEN x N_OUTPUTS.
# All three are initialised uniformly in [-HEBBIAN_WEIGHT_INIT_RANGE, HEBBIAN_WEIGHT_INIT_RANGE],
# independently for every robot, as in the paper (the MATLAB source samples W1 from randn()).
HEBBIAN_WEIGHT_INIT_RANGE = 1.0

# --- ABCD genotype ---
# 4 coefficients (A, B, C, D) per NN weight, shared across all agents in a swarm:
# 4 * (10*10 + 10*10 + 10*2) = 880 total parameters.
HEBBIAN_N_ABCD = 4 * (HEBBIAN_N_INPUTS * HEBBIAN_N_HIDDEN + HEBBIAN_N_HIDDEN * HEBBIAN_N_HIDDEN
                      + HEBBIAN_N_HIDDEN * HEBBIAN_N_OUTPUTS)
HEBBIAN_ABCD_INIT_RANGE = 5.0     # unused: the initial genome is sampled uniformly from HEBBIAN_ABCD_BOUNDS
HEBBIAN_ABCD_BOUNDS = [-5.0, 5.0]  # CMA-ES hard bounds (and the initial sampling range)

# --- CMA-ES hyperparameters (paper Table "Swarm experiment parameters") ---
HEBBIAN_CMAES_POPSIZE = 30        # lambda
HEBBIAN_CMAES_GEN_MAX = 100       # Ngen per stage (train.py doubles it for Stage 2)
HEBBIAN_CMAES_SIGMA0 = 0.3        # initial covariance/step-size
HEBBIAN_N_REPEATS = 3             # simulations per candidate (different seeds); fitness = median

# Seeds used by optimize_hebbian.py's --seeds flag (with no explicit values) to run the entire
# 3-stage curriculum once per seed.
HEBBIAN_BATCH_SEEDS = [42, 123, 777, 2026, 888, 99, 412, 555, 1010, 8432]

# --- Staged curriculum ---
# Each stage's CMA-ES run starts from the previous stage's best genome; stage 1 starts from a
# uniform-random genome. All stages train with wind. The paper uses walk_upwind and
# save_battery_avoid_all only (train.py); optimize_hebbian.py runs all three.
HEBBIAN_STAGES = ("walk_upwind", "save_battery_avoid_wall", "save_battery_avoid_all")
HEBBIAN_STAGE_WIND_ENABLED = {
    "walk_upwind": True,   # stage 1 trains with wind -- see module docstring
    "save_battery_avoid_wall": True,
    "save_battery_avoid_all": True,
}
# WIND_DIRECTION: unit-ish vector dist_travelled is projected onto (simulation_hebbian.py,
# lj_baseline.py). Wind blows toward +x (see wind_physics.py's wake propagation), so (-1, 0)
# is "into the wind": dist_travelled = -mean(final x).
WIND_DIRECTION = (-1.0, 0.0)
# Fitness per stage (stage_fitness() in simulation_hebbian.py):
#   eff = HEBBIAN_EFF_DISTANCE_WEIGHT*dist + batt/battery_w
#         - (collision_time + wall_col_mult * wall_collision_time) / collision_w
#         - cohesion_dist / cohesion_w - proximity_penalty / proximity_w
# A weight of None removes that term. include_inter_robot_collision=False leaves robot-robot
# collisions out of collision_time (only walls count). With HEBBIAN_EFF_DISTANCE_WEIGHT = 16 this
# gives the paper's Table "Fitness function during each evolution stage":
#   Stage 1: f = 16 x_avg
#   Stage 2: f = 16 x_avg + B_avg,end / 5 - (t_col + 3 t_wcol) / 250
HEBBIAN_STAGE_FITNESS_WEIGHTS = {
    #                             battery_w   collision_w   wall_col_mult   include_inter_robot_collision   cohesion_w   proximity_w
    "walk_upwind":                (None,        None,         3.0,            False,                          None,        None),
    "save_battery_avoid_wall":   (5.0,         500.0,        3.0,            False,                          None,        None),
    "save_battery_avoid_all":    (5.0,         250.0,        3.0,            True,                           None,        None),
}

# Weight of the distance term in every stage's fitness. Without it (an implicit weight of 1, as
# in Mahdavi et al.'s table), CMA-ES found that barely moving is a cheap way to keep battery:
# distance collapsed from ~84 m after stage 1 to ~1.5 m in the later stages, with the battery
# term making up over 90% of the fitness. Of 4, 8, 16 and 32 (3 seeds each, 10 robots), 16 gave
# the lowest collision time with the best battery.
HEBBIAN_EFF_DISTANCE_WEIGHT = 16.0

# =====================================================================================
# --- Baseline (lj_baseline.py): the flocking model of Ferrante et al. (2012) with a
# Lennard-Jones-type potential, run in the same simulator with the same wind, drag and battery
# model as the evolved controller. A trimmed, standalone copy of the original LJ model's control
# law + fitness.
# =====================================================================================
MAX_BATTERY = 150.0          # the baseline's starting battery (its native scale, paper Table
MIN_BATTERY = 150.0          # "baseline parameters"); not HEBBIAN_MAX_BATTERY's 0-100 scale

# Used when simulate_lj_baseline() is called without rules: the MATLAB LJ script's
# (simulation_free_global_mod_2_LJ.m) own starting point. NOT the paper's baseline.
DEFAULT_RULES = {
    "r0": 0.70, "epsilon": 0.5, "k_align": 0.0, "k_goal": 3.0, "K1": 0.05, "K2": 0.5, "U": 0.005,
}
# The paper's baseline (Table "Simulation parameters for the baseline controller"); identical to
# data/lj_baseline_rules.json, which the analysis scripts load.
PAPER_BASELINE_RULES = {
    "r0": 0.70, "epsilon": 1.0, "k_align": 0.0, "k_goal": 3.0, "K1": 0.05, "K2": 0.5, "U": 0.0,
}
R_CUT = 3.0        # LJ interaction cutoff radius [m] (paper: D_p)
R_MIN = 0.0         # LJ singularity guard radius [m]
R_ALIGN = 1.5       # neighbor radius used for heading alignment [m] (unused with k_align = 0)
LINEAR_VEL_MAX = 0.20            # robot's max linear speed [m/s]
ANGULAR_VEL_MAX = math.pi / 5    # robot's max angular speed [rad/s]

# Baseline fitness, eff = EFF_DISTANCE_WEIGHT*dist_travelled + avg_batt/EFF_BATTERY_WEIGHT
# - collision_time/EFF_COLLISION_WEIGHT. Returned by simulate_lj_baseline() but not used by the
# paper, which compares distance and battery directly.
EFF_DISTANCE_WEIGHT = 8.0
EFF_BATTERY_WEIGHT = 10.0
EFF_COLLISION_WEIGHT = 250.0
