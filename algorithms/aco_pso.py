import random

import traci

from algorithms.route_utils import build_destination_valid_route, get_outgoing_edges

# -------------------------
# ACO PARAMETERS
# -------------------------
ALPHA = 1.0
BETA = 2.0
RHO = 0.1
Q = 1.0

# -------------------------
# PSO PARAMETERS
# -------------------------
W = 0.5       # inertia weight
C1 = 1.5      # cognitive (personal best) coefficient
C2 = 1.5      # social (global best) coefficient
VMAX = 0.5    # maximum velocity
# E3's PSO uses the expected value of U(0, 1) for both random coefficients r1 and r2 (user
# decision, 2026-09-14). With random draws, PSO alone moved one trial's J from 1.2036 to 0.9944
# at velocity weight 0.314, so one evaluation per weight set could not be trusted. This is a
# deliberate departure from textbook PSO. The ACO-PSO comparator keeps its random draws.
E3_PSO_R = 0.5

# -------------------------
# PHEROMONE BOUNDS AND DECAY (audit A10)
# -------------------------
#
# Three defects were measured in the previous scheme:
#
# 1. Evaporation lived inside `update_pheromone`, which the P3 path calls only when a
#    reroute is actually *applied* - about 480 times an hour rather than once per second.
#    The decay rate therefore depended on how often the router happened to act.
# 2. Decay was multiplicative toward zero with no floor. An edge not re-deposited within
#    roughly 100 reroutes retained 2.7e-05 of its value, i.e. zero.
# 3. An edge absent from the table defaulted to 1.0. Combined with (2), a road the router
#    knew about but had not used recently scored *worse than a road it had never seen* -
#    a cliff in the middle of the score, the same class of defect A3 removed from the
#    protocol term.
#
# The scheme below fixes all three. Deposited edges relax exponentially back toward
# TAU_INIT rather than toward zero, so "stale" converges to "no information" instead of to
# "worst possible". Values are bounded to [TAU_MIN, TAU_MAX] in the manner of MAX-MIN Ant
# System, which is the standard ACO remedy for unbounded growth and stagnation. Because an
# untouched edge is exactly TAU_INIT, the dictionary default is consistent with the decay
# target and the cliff disappears without having to seed 21,385 edges.

TAU_INIT = 1.0   # neutral: an edge nothing is known about
TAU_MIN = 0.1    # floor, MMAS-style
TAU_MAX = 5.0    # ceiling; the unbounded scheme reached 6.45 and was still climbing

# Half-life of a deposit, in simulated seconds, chosen to match the timescale on which this
# network's congestion actually changes (running vehicles climb 162 -> 545 over the hour,
# and incident windows are 300 s). A deposit is half forgotten after five minutes.
PHEROMONE_HALF_LIFE_S = 300.0
RHO_STEP = 1.0 - 0.5 ** (1.0 / PHEROMONE_HALF_LIFE_S)

# -------------------------
# OUTCOME-BASED LEARNING (work order item 1)
# -------------------------
#
# `design_phase.md` section 2.5 requires pheromone to update "from **successful** route
# choices" and the system to "learn from actual outcomes, not just from predicted scores".
# Neither held: `deposit()` laid a flat Q on every edge of every applied route the moment it
# was applied, so pheromone measured how often an edge was chosen, not how well it worked,
# and PSO's fitness was read straight out of that same table. Both halves of ACO+PSO were
# learning frequency, and PSO could not be anything but a function of pheromone.
#
# `algorithms/route_outcomes.py` now watches each applied reroute for a short window and
# reports a **quality** in [0, 1]: realised mean speed divided by the free-flow speed of the
# roads actually driven. That is the classic ACO cost ratio - with L = actual time over
# free-flow time, quality is 1/L - so a deposit proportional to quality is the textbook
# `Q / L`, and PSO gets a fitness that is a measured travel outcome rather than a copy of
# the pheromone it is supposed to be independent of.
#
# The deposit is divided by a running mean of observed qualities before it is applied. This
# matters more than it looks: in this scenario the network runs at 12-19x free-flow, so raw
# qualities cluster near 0.05-0.15 and a raw `Q * quality` deposit would be roughly ten times
# smaller than the flat deposit it replaces. The table would drift to a single value and
# pheromone would stop discriminating - a change of scale masquerading as a change of rule.
# Normalising by the running mean keeps an *average* route depositing what the flat rule
# deposited, so what changes is which edges are reinforced, not how hard the table is driven.

# A single outcome may deposit at most this multiple of the mean. Without it one vehicle
# that happens to catch a green wave writes an edge straight to TAU_MAX.
DEPOSIT_SCALE_CAP = 3.0

# How fast the running mean quality follows the run. Slow, because it is the yardstick the
# deposits are measured against and it must not chase the thing it is normalising.
QUALITY_MEAN_ALPHA = 0.05

# How fast an edge's own realised fitness follows new observations.
FITNESS_ALPHA = 0.30

# PSO in a dynamic environment: a personal or global best recorded twenty minutes ago
# describes a road network that no longer exists, and the standard response is to forget or
# re-evaluate the memory rather than keep it forever. The bests here relax toward the
# current mean quality with the same half-life the pheromone table uses, so "best" means
# "best recently" and an edge can once again exceed it. Without this every fitness sits
# below a permanent maximum, every velocity is negative, and the term goes inert.
FITNESS_HALF_LIFE_S = 300.0
FITNESS_RHO_STEP = 1.0 - 0.5 ** (1.0 / FITNESS_HALF_LIFE_S)

# Value of an edge nobody has driven yet. It is the running mean quality, not a constant:
# an unobserved edge must read as "typical", never as "better than everything measured".
# A fixed 0.5 against observed qualities near 0.1 would make every unknown road look
# excellent - the same cliff audit A10 removed from the pheromone table and audit A3 from
# the protocol term.
FITNESS_INIT = 0.5

# -------------------------
# STATE
# -------------------------
pheromone = {}
velocity = {}
personal_best = {}

# PSO's social memory, on the realised-quality scale [0, 1]. It starts at the neutral
# FITNESS_INIT rather than 0.0: a global best of zero would sit below every fitness, making
# the social term positive and identical for every candidate until the first outcome lands.
global_best_score = FITNESS_INIT

# Realised quality per edge, fed by route_outcomes and read by the PSO velocity update.
edge_fitness = {}

# Running mean of the qualities observed so far: the yardstick for deposits and the value
# an unobserved edge reads. E3 keeps windows closed early by a new decision out of it (A2,
# 2026-09-14), so for E3 `outcomes_in_mean` is `outcomes_recorded` minus those windows.
quality_mean = FITNESS_INIT
outcomes_recorded = 0
outcomes_in_mean = 0

# Edges carrying a deposit that has not yet decayed back to TAU_INIT. Evaporation only has
# to touch these, so the per-step cost is proportional to what the router has actually
# used rather than to the size of the network.
_deposited = set()

# Edges E3 scored since the last `advance_velocities` (audit S5, 2026-09-14): each advances
# once at the next step, however many vehicles scored it and in whatever order.
_scored_since_advance = set()


# -------------------------
# Initialize pheromone
# -------------------------
def initialize_pheromone():
    global pheromone
    for edge in traci.edge.getIDList():
        pheromone[edge] = 1.0


# -------------------------
# Get outgoing edges
# -------------------------
# Shared with every other algorithm via algorithms/route_utils.py: all lanes (C14),
# passenger-accessible only (C11), sorted for determinism (C13).


# -------------------------
# Heuristic (low congestion)
# -------------------------
def heuristic(edge):
    veh = traci.edge.getLastStepVehicleNumber(edge)
    return 1.0 / (1.0 + veh)


# -------------------------
# PSO velocity - shared by every path that uses this module's state
# -------------------------
#
# Audit C8 / Step 5. The PSO half of E3 was not running at all in the P3/E3 path. That
# path (`p3_runtime._apply_hybrid_decision`) imports the `velocity` dictionary from this
# module and reads it with `velocity.get(edge, 0.0)`, but the only code that ever wrote
# that dictionary was `select_route` below, which a P3 run never calls. So every candidate
# edge scored a velocity of exactly 0.0, the PSO component was identical for all of them,
# and a component that is identical for every candidate cannot change which one wins -
# leaving the PSO weight unreachable by tuning. Measured: after 300 s of a P3/E3 run the
# velocity dictionary held 0 entries, and across 377 real route selections the PSO term
# differed between candidates in 0.0% of them.
#
# The update itself is unchanged and is now defined once, here, instead of being copied
# into each caller.


def fitness_of(edge):
    """The realised travel quality of an edge; the running mean if nobody has driven it."""
    return edge_fitness.get(edge, quality_mean)


def update_velocity(edge, r1=None, r2=None):
    """Run one PSO velocity update for one candidate edge and return the new velocity.

    `r1` and `r2` are the random coefficients. When they are not given they are drawn from
    Python's `random`, in the same order as always, which is what the ACO-PSO comparator uses.
    E3 passes `E3_PSO_R` for both (2026-09-14).

    Work order item 1. The fitness driving this used to be `pheromone[edge]`, which made
    PSO a function of the ACO table it is supposed to complement - and since that table
    reinforced frequency rather than quality, PSO was learning frequency too. It is now the
    edge's **realised** travel quality: the mean speed vehicles actually achieved on it,
    relative to its speed limit, as reported by `route_outcomes`.

    The sign convention is deliberate and is not the textbook one, and `design_phase.md`
    2.6 is what fixes it: what must be fed back is "velocity updates from route
    **improvement or deterioration**". Velocity is therefore positive when a road is
    currently doing better than the best it had shown before, negative when it is doing
    worse, and it decays back toward zero through the inertia term W once the improvement
    has been absorbed. It is a rate of change, which is what the word velocity should mean.

    That only works if the bests are updated *after* the velocity step, which is also the
    textbook PSO ordering: evaluate, move, then update the memory. Folding an observation
    into the personal best before measuring the difference from it guarantees the
    difference is never positive - measured, and it pinned every velocity to the -0.5
    floor.
    """
    global global_best_score

    if edge not in velocity:
        velocity[edge] = 0.0

    fitness = fitness_of(edge)
    pb = personal_best.get(edge, quality_mean)
    r1 = random.random() if r1 is None else r1
    r2 = random.random() if r2 is None else r2

    velocity[edge] = (
        W * velocity[edge]
        + C1 * r1 * (fitness - pb)
        + C2 * r2 * (fitness - global_best_score)
    )
    velocity[edge] = max(-VMAX, min(VMAX, velocity[edge]))

    if fitness > pb:
        personal_best[edge] = fitness
    if fitness > global_best_score:
        global_best_score = fitness

    return velocity[edge]


def velocity_of(edge):
    """An edge's PSO velocity, for E3's scoring: it leaves the PSO state as it is (audit S5).

    E3 used to call `update_velocity` once per candidate scoring. An edge scored by ten
    vehicles on the same 30 s tick then took ten iterations: inertia 0.5^10, twenty random
    draws, and its bests raised on the first. Which vehicle saw which value depended on the
    order TraCI listed them in. Now every vehicle in a step reads the same value. The edge is
    noted so the next `advance_velocities` iterates it once.
    """
    _scored_since_advance.add(edge)
    return velocity.get(edge, 0.0)


def advance_velocities():
    """One PSO iteration for each edge E3 scored since the last call. Call once per step.

    Exactly once per edge, in sorted edge order, so the best updates do not depend on how many
    vehicles scored the edge or in what order. The iteration is `update_velocity`: velocity
    first, then the bests, the textbook ordering. Its coefficients are the fixed `E3_PSO_R`,
    so E3's PSO draws no random numbers at all. The ACO-PSO comparator still calls
    `update_velocity` directly with random draws, as before.
    """
    edges = sorted(_scored_since_advance)
    _scored_since_advance.clear()
    for edge in edges:
        update_velocity(edge, r1=E3_PSO_R, r2=E3_PSO_R)
    return len(edges)


def record_outcome(edges, quality, update_mean=True):
    """File one realised travel outcome against the roads that produced it.

    Updates the per-edge fitness PSO reads and the running mean quality that is both the
    yardstick for deposits and the reading for a road nobody has driven. The personal and
    global bests are deliberately *not* touched here - they are updated in `update_velocity`
    after the velocity has been computed, which is the PSO ordering and what lets a velocity
    represent an improvement rather than always being zero or negative.

    An edge's first observation sets its fitness outright instead of being averaged toward
    the mean. Most edges in this network are driven once or twice in a run, so shrinking a
    first measurement 70% of the way back to the mean would throw away most of the signal
    the run produced.

    `update_mean=False` updates the edges' fitness but leaves the running mean alone. E3 passes
    it for a window closed early by a new decision (audit A2, user decision 2026-09-14, "mean
    protected"). The reroute trigger picks only slow cars, so those windows are a congested
    subsample, 3.4-11.3 times lower in quality than full windows in every diagnosis run. In the
    mean they pulled it down and inflated every deposit normalised by it. Every other caller,
    the frozen ACO-PSO comparator included, keeps the default.
    """
    global quality_mean, outcomes_recorded, outcomes_in_mean

    quality = max(0.0, min(1.0, float(quality)))
    outcomes_recorded += 1
    if update_mean:
        outcomes_in_mean += 1
        quality_mean += QUALITY_MEAN_ALPHA * (quality - quality_mean)

    for edge in edges:
        if edge in edge_fitness:
            edge_fitness[edge] += FITNESS_ALPHA * (quality - edge_fitness[edge])
        else:
            edge_fitness[edge] = quality


def decay_fitness(seconds=1.0):
    """Forget PSO's memory toward the current mean. Call once per simulation step.

    The same treatment audit A10 gave pheromone, for the same reason: a best that never
    decays describes traffic that has moved on, and the whole term is measured as a
    difference from it.
    """
    global global_best_score

    rate = 1.0 - (1.0 - FITNESS_RHO_STEP) ** max(seconds, 0.0)
    global_best_score += (quality_mean - global_best_score) * rate

    for edge in list(personal_best.keys()):
        value = personal_best[edge] + (quality_mean - personal_best[edge]) * rate
        if abs(value - quality_mean) < 1e-4:
            personal_best.pop(edge, None)
        else:
            personal_best[edge] = value

    for edge in list(edge_fitness.keys()):
        value = edge_fitness[edge] + (quality_mean - edge_fitness[edge]) * rate
        if abs(value - quality_mean) < 1e-4:
            edge_fitness.pop(edge, None)
        else:
            edge_fitness[edge] = value


# -------------------------
# ACO + PSO decision
# -------------------------
def select_route(veh_id, current_edge):
    global global_best_score, personal_best
    
    if not pheromone:
        initialize_pheromone()

    outgoing = get_outgoing_edges(current_edge, veh_id=veh_id)

    if not outgoing:
        return list(traci.vehicle.getRoute(veh_id))

    probabilities = []
    edges = []

    for edge in outgoing:
        # ACO component
        tau = pheromone.get(edge, TAU_INIT) ** ALPHA
        eta = heuristic(edge) ** BETA
        
        # PSO velocity update, shared with the P3/E3 path (audit C8).
        edge_velocity = update_velocity(edge)

        # Combined score: ACO component + PSO velocity boost
        # PSO adds exploration/exploitation balance via velocity
        score = tau * eta * (1.0 + edge_velocity)
        probabilities.append(max(score, 0.0001))
        edges.append(edge)

    # Normalize probabilities
    total = sum(probabilities)
    probabilities = [p / total for p in probabilities]

    # Select edge based on combined score
    selected_edge = random.choices(edges, probabilities)[0]
    # `record_selection(selected_edge)` used to sit here and raised the personal and global
    # bests to the *pheromone* of whatever edge was chosen. Both bests are now on the
    # realised-quality scale and are set by `record_outcome`, so a caller of this function
    # must feed it outcomes (see metrics/metrics_logger.py) or PSO will have nothing to
    # measure against. Updating the bests from a selection would put a pheromone value on a
    # quality scale and reinstate exactly the dependence on pheromone this change removes.

    # Build new route.
    # Audit C1: the previous version returned current_route[:idx + 1] + [selected_edge],
    # which changed the vehicle's destination instead of its path. Contract section 5
    # requires a complete, destination-valid route.
    current_route = list(traci.vehicle.getRoute(veh_id))
    return build_destination_valid_route(veh_id, current_route, current_edge, selected_edge)


# -------------------------
# Pheromone update
# -------------------------
def evaporate(seconds=1.0):
    """Decay deposited pheromone back toward TAU_INIT. Call once per simulation step.

    Audit A10: this is the half of `update_pheromone` that must run on the clock rather
    than on reroute events. Returns how many edges still carry a deposit.
    """
    if not _deposited:
        return 0

    rate = 1.0 - (1.0 - RHO_STEP) ** max(seconds, 0.0)
    settled = []
    for edge in _deposited:
        tau = pheromone.get(edge, TAU_INIT)
        tau += (TAU_INIT - tau) * rate
        if abs(tau - TAU_INIT) < 1e-4:
            # Back to neutral: drop it, because the dictionary default is TAU_INIT and an
            # absent edge and a fully-decayed edge must mean the same thing.
            pheromone.pop(edge, None)
            settled.append(edge)
        else:
            pheromone[edge] = tau

    for edge in settled:
        _deposited.discard(edge)
    return len(_deposited)


def deposit_strength(quality):
    """How much pheromone one realised outcome is worth, relative to the flat old rule.

    `Q / L` in the classic ACO sense, expressed as a multiple of the mean outcome so that
    an average route deposits what the previous flat rule deposited. See the note at the
    top of this module for why the normalisation is not cosmetic.
    """
    if quality is None:
        return Q
    reference = quality_mean if quality_mean > 1e-6 else FITNESS_INIT
    return Q * min(DEPOSIT_SCALE_CAP, max(0.0, quality) / reference)


def deposit(route, quality=None):
    """Reinforce the edges of a route just driven, bounded above by TAU_MAX (audit A10).

    `quality` is the realised outcome of that route in [0, 1] from `route_outcomes`. Passing
    None keeps the old flat deposit and is used only by the deprecated `update_pheromone`
    below, which the six-algorithm comparison table still runs.
    """
    strength = deposit_strength(quality)
    if strength <= 0.0:
        return
    for edge in route:
        tau = pheromone.get(edge, TAU_INIT) + strength
        pheromone[edge] = max(TAU_MIN, min(TAU_MAX, tau))
        _deposited.add(edge)


def reset_pheromone():
    """Clear the table between runs so one run cannot inherit another's search state."""
    global global_best_score, quality_mean, outcomes_recorded, outcomes_in_mean
    pheromone.clear()
    velocity.clear()
    personal_best.clear()
    edge_fitness.clear()
    _deposited.clear()
    _scored_since_advance.clear()
    global_best_score = FITNESS_INIT
    quality_mean = FITNESS_INIT
    outcomes_recorded = 0
    outcomes_in_mean = 0


def update_pheromone(route):
    """Deprecated: evaporate-and-deposit in one call, decay tied to reroute frequency.

    Kept unchanged for `metrics/metrics_logger.py`, which drives the ACO-PSO entry of the
    six-algorithm comparison table. The P3 path now calls `evaporate()` per step and
    `deposit()` per applied reroute instead. Unifying the two is audit C8.
    """
    for e in pheromone:
        pheromone[e] *= (1 - RHO)

    for edge in route:
        # Audit A10, in part. `p3_runtime` imports this table but never seeds it, and even
        # a seeded table misses edges that appear later, so `pheromone[edge] += Q` raised
        # KeyError on the first reroute and the deposit was lost. Matches the behaviour
        # `algorithms/e3.py` already had. Seeding the table in the protocol path, and the
        # rest of A10, remain Step 5 work.
        pheromone[edge] = pheromone.get(edge, TAU_INIT) + Q
