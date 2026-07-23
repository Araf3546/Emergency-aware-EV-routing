# Design Phase Document

## Part 1 — Protocol Design

This part defines the vehicle-to-vehicle message that will drive emergency-aware routing in the new design.

### 1.1 Message schema

Each vehicle broadcasts a compact state message to nearby vehicles. The message should be simple enough to send frequently, but rich enough to support routing decisions.

#### Mandatory fields
- `vehicle_id` (string): unique identifier for the sender.
- `timestamp` (float): local simulation time or message generation time.
- `vehicle_type` (enum): `normal`, `ambulance`, `fire_truck`, `police`, `relief_vehicle`.
- `position_edge` (string): current road/edge identifier.
- `position_lane` (string, optional in implementation but useful): current lane if available.
- `speed` (float): current speed in m/s.
- `priority_level` (integer): urgency level from `0` to `4`.
- `emergency_flag` (boolean): whether the sender is an emergency vehicle or is actively signaling emergency influence.
- `route_intent` (string): next major road/goal intent, used to help neighbors infer likely movement.

#### Optional fields
- `congestion_estimate` (float): local congestion score for the current edge, normalized to $[0,1]$.
- `yield_request` (boolean): indicates that the sender is giving way or is in a short-term yielding state.
- `next_edge` (string): immediate next edge preference if known.
- `distance_to_emergency` (float): estimated distance to a known emergency zone if available.
- `hop_count` (integer): optional simple propagation field for debugging or protocol analysis.

#### Mandatory vs. optional decision
- The protocol should be robust even if some optional fields are absent.
- The routing layer must be able to make a decision using only the mandatory fields plus local SUMO observations.
- Optional fields improve decision quality, but they must not be required for correctness.

### 1.2 Priority encoding & emergency signaling

Emergency vehicle prioritization is part of the protocol design, not an afterthought.

#### Priority encoding
- `0` = normal vehicle, no special priority.
- `1` = normal vehicle in a yield/clearance state.
- `2` = medium urgency or local congestion-aware cooperative behavior.
- `3` = high-priority cooperative behavior.
- `4` = emergency vehicle or emergency-response state.

#### Emergency signaling rules
- Emergency vehicles always set:
  - `vehicle_type` to one of the emergency categories,
  - `priority_level = 4`,
  - `emergency_flag = true`.
- Normal vehicles set:
  - `priority_level = 0` by default,
  - `emergency_flag = false` by default.
- If a normal vehicle receives an emergency message or detects a nearby emergency condition, it may switch to a temporary cooperative state:
  - `priority_level = 1`,
  - `yield_request = true`,
  - `emergency_flag = true` only if the vehicle is explicitly signaling the emergency context to neighbors.

#### Why this design works
- It gives emergency vehicles an unambiguous high-priority signal.
- It lets normal vehicles cooperatively clear space without overloading the message format.
- It allows the same protocol to be used for both routing and coordination.

### 1.3 Transmission policy

#### Update frequency
- Emergency vehicles: broadcast every 1 second or immediately after a route change.
- Normal vehicles: broadcast every 2–3 seconds, or earlier if their local congestion or route intent changes significantly.
- Triggered broadcast events:
  - entering a new edge,
  - significant speed change,
  - receiving an emergency message,
  - changing route decision.

#### Message size & bandwidth budget
- Target message size: under 128 bytes when serialized compactly.
- Keep edge identifiers short and use integer priority codes.
- The design should stay within a practical local communication budget for a dense urban network.
- A reasonable target is a low-overhead message rate that does not overwhelm the simulation or create unnecessary per-step computation.

#### Urgency handling
- Emergency messages must be processed before ordinary traffic messages when both are present.
- In the receiver logic, emergency messages should be treated as high-priority state updates.
- This prevents postponed emergency awareness from weakening the system during critical moments.

### 1.4 Assumptions & fault handling

#### Communication assumptions
- Communication range: approximately 200–300 meters in a typical urban V2V setting.
- The protocol assumes that vehicles within the same local neighborhood can exchange messages frequently enough to influence route decisions.
- The system should work even if not every vehicle is connected at every moment.

#### Latency and packet loss
- Expected latency is assumed to be low, but not zero.
- Packet loss should be handled explicitly rather than ignored.
- The receiver must tolerate missing messages and work from the last known valid state for a limited time.

#### Staleness handling
- A message older than a short freshness window should be treated as stale.
- The freshness window should be short enough to reflect dynamic traffic conditions, but not so short that ordinary message delay causes instability.
- A practical design is to keep a message for only a few seconds and then drop or discount it.

#### Fallback behavior
- If no fresh message is received, the vehicle falls back to local sensing:
  - local edge congestion,
  - current queue length,
  - current road occupancy,
  - local emergency knowledge from the simulation environment.
- Emergency vehicles should still be able to route safely even if neighbor communication is missing for a short interval.

---

## Part 2 — E3 Hybrid Design

This part defines how the E3 Hybrid system consumes the protocol and combines ACO, PSO, and BCO into a single routing framework.

### 2.1 How BCO integrates with the existing ACO-PSO core

At this stage, only ACO + PSO hybrid behavior is already implemented. BCO is not yet integrated and is now part of the main design target.

#### Role of BCO in the new design
- ACO contributes pheromone-based route reinforcement.
- PSO contributes adaptive velocity and response to changing traffic conditions.
- BCO adds a different kind of exploration and local search behavior through bee-inspired scouting and recruitment.

#### Why BCO is needed
- ACO + PSO already help with path adaptation and route reinforcement.
- BCO adds a more explicit exploration mechanism that is useful when traffic conditions change quickly, especially around emergencies.
- In practice, BCO helps the system discover alternative routes that the other two methods may not explore aggressively enough.

#### Design position of BCO
- BCO should not replace ACO or PSO.
- It should work as a third component that improves route exploration and helps the system react to new emergency patterns.
- The final E3 Hybrid decision should be a combined decision, not a simple switch between methods.

### 2.2 Input handling

The hybrid router uses both local information and protocol-based neighbor information.

#### Own state inputs
- current edge,
- current speed,
- vehicle type,
- current route and destination intent,
- local queue/congestion observations.

#### Neighbor state inputs from the protocol
- nearby vehicles’ positions,
- their priority levels,
- whether they are emergency vehicles,
- whether they are yielding,
- their latest direction or next-edge intent.

#### Emergency inputs
- emergency vehicle messages,
- local emergency awareness from the simulation environment,
- blocked or restricted edges when applicable.

#### Internal state inputs
- pheromone values from ACO,
- velocity values from PSO,
- scout/forager memory from BCO.

### 2.3 Core decision logic

The routing decision should combine several influences into one score for each candidate edge or route option.

#### Scoring idea
For each candidate next edge, the system evaluates:

$$
Score(edge) = w_1 \times Pheromone(edge) + w_2 \times Heuristic(edge) + w_3 \times ProtocolWeight(edge) + w_4 \times BeeExploration(edge)
$$

Where:
- $Pheromone(edge)$ captures learned route quality from ACO.
- $Heuristic(edge)$ captures short-term local suitability such as lower congestion and shorter distance.
- $ProtocolWeight(edge)$ reflects emergency signals and neighbor cooperation from the V2V protocol.
- $BeeExploration(edge)$ reflects the BCO scout/forager contribution and helps find alternative routes when the current route becomes poor.

#### How each component contributes
- ACO strengthens routes that have worked well before.
- PSO provides adaptive movement in the search space by updating a velocity-like tendency for promising edges.
- BCO adds exploration pressure so the system does not become too conservative.
- The protocol weight becomes especially important near emergency zones and during cooperative yielding.

#### Practical implementation note
- The system should avoid recomputing route scores from scratch unnecessarily.
- It is better to update only the relevant local edge scores and reuse previously computed context where possible.
- This is a real efficiency improvement that preserves correctness.

### 2.4 Emergency override logic

Emergency handling is now protocol-driven, not local blocked-edge-only logic.

#### For emergency vehicles
- Emergency vehicles prioritize routes that are both fast and compatible with protocol-based clearance from surrounding vehicles.
- They should strongly favor edges that are not only short, but also currently receiving cooperative signals from nearby traffic.
- Their behavior should not simply be “go to the shortest path”; it should be “go to the fastest safe path under current traffic and protocol context.”

#### For normal vehicles
- Normal vehicles should reduce their routing priority on edges that are being approached by emergency vehicles.
- If they receive an emergency message or observe protocol-based yield behavior, they should avoid blocking the path.
- In this design, emergency response is not a special case added later; it is a first-class part of route scoring.

#### Override behavior
- Emergency messages should be treated as top-priority routing signals.
- If a normal vehicle is near an emergency corridor, it should prefer a detour that reduces conflict.
- The system should be able to distinguish between ordinary congestion and genuine emergency urgency.

### 2.5 Fallback behavior

The protocol should improve routing, but the system must still work when messages are missing or delayed.

#### If messages are missing
- Use local observations instead:
  - current edge speed,
  - queue length,
  - current occupancy,
  - known emergency context from the environment.

#### If messages are stale
- Devalue older messages and rely more on current local measurements.
- The routing system should not overreact to outdated emergency signals.

#### If communication is temporarily unavailable
- The hybrid system should fall back to a robust baseline decision using ACO + PSO + local congestion information.
- Emergency vehicles should retain a safety-first fallback that avoids known dangerous or blocked paths.

### 2.6 Output / action

This is the point where the hybrid design turns into an actual route update.

#### Route update process
1. Collect local state and protocol messages.
2. Update the current neighbor table and emergency context.
3. Score candidate next edges or route options.
4. Select the best next action.
5. Apply the route update to the vehicle.
6. Feed the outcome back into the hybrid learning state.

#### What gets fed back
- pheromone updates from successful route choices,
- velocity updates from route improvement or deterioration,
- BCO scout/forager memory from explored alternatives,
- protocol-based success indicators such as reduced emergency conflict or faster clearance.

#### Design principle
- The system must learn from actual outcomes, not just from predicted scores.
- That learning loop is what makes the hybrid model meaningful in the thesis.

---

## Simulation time

The earlier 600–1000 second runs are too short for 8,000 vehicles with emergency events to fully play out. A short run would make the results noisy, especially for emergency response and congestion recovery.

### Reasoning
- With 8,000 vehicles, the network needs enough time for traffic to build up, for emergency events to appear, and for the system to respond.
- Emergency events should be allowed to start, affect surrounding traffic, and then resolve.
- Very short runs mainly capture startup effects rather than meaningful routing behavior.

### Final choice
- Use a simulation duration of 3600 seconds.
- This is long enough to produce more stable and meaningful results without making each run impractically long for the hardware available.

---

## Performance vs. correctness

The simulation will be slower at this scale, and that is expected. The design should prioritize correctness and robustness over raw speed.

### What is acceptable for efficiency improvement
- Avoiding redundant recomputation.
- Using efficient data structures for neighbor state and edge scoring.
- Reusing intermediate values when possible.
- Reducing unnecessary per-step overhead where it does not weaken the logic.

### What should not be sacrificed
- Emergency logic should not be removed.
- The protocol should not be simplified in a way that weakens emergency propagation.
- Simulation fidelity should not be reduced just to make runs faster.

### Explicit tradeoff note
- If a speed optimization would noticeably weaken routing quality or emergency response, it should not be used.
- The thesis should reflect the more reliable approach even if it costs more computation time.

---

## Long-run simulation strategy

We should not rely on many short runs because each one is expensive. A single well-logged full run gives more useful information than several shallow runs.

In simple terms: a full SUMO run costs a lot of time, so it is better to run it carefully once or twice and record as much meaningful data as possible. That gives better evidence for the thesis than running many short tests that only show a small part of the system behavior.

### Practical strategy
- Run one or two full simulations per algorithm configuration.
- Log enough data to capture:
  - travel time,
  - waiting time,
  - route changes,
  - emergency response behavior,
  - congestion and recovery patterns.
- Prefer a few rich runs over many weak ones.

---

## Metrics & visuals

### Recommended metrics
- Average travel time.
- Average waiting time.
- Average distance traveled.
- Emergency response time.
- Emergency vehicle delay.
- Congestion level over time.
- Number of reroutes.
- Message overhead and protocol freshness.
- Conflict or blockage rate near emergency zones.

### Recommended visuals
- Bar charts for comparing average travel time, waiting time, distance, and emergency response time across algorithms.
- Time-series plots for congestion and response dynamics over the full run.
- Heatmaps or edge-level congestion maps for showing where congestion and emergency impact are strongest.
- Boxplots or cumulative distribution plots for showing spread in travel time and waiting time.
- Simple protocol-overhead plots to show communication cost versus benefit.

### Why these visuals fit this thesis
- They are informative without requiring many repeated runs.
- They communicate both system performance and emergency behavior clearly.
- They are suitable for a long-run simulation where the main value comes from logging a few complete runs well.

---

## 5. Emergency-response comparison: before vs. after E3 Hybrid

This section is specifically for showing the benefit of E3 Hybrid for emergency vehicles, rather than only reporting average traffic performance.

### 5.1 Comparison definition

To keep the comparison apples-to-apples, the before and after conditions must be defined clearly.

#### Before: traditional algorithm baseline
- The “before” condition means the existing traditional routing behavior already in the repo:
  - Baseline,
  - ACO,
  - Dijkstra,
  - A*.
- In this setting, emergency handling is non-protocol-based and effectively local or reactive, with limited awareness of nearby vehicles’ cooperation or emergency intent.
- The emergency vehicle still receives priority through the routing logic, but the surrounding traffic is not coordinating through a shared V2V message protocol.

#### After: E3 Hybrid with protocol-based emergency handling
- The “after” condition means the new E3 Hybrid design using the protocol from Part 1.
- Emergency vehicles broadcast their urgency and route intent.
- Nearby vehicles use that shared information to change their routing behavior and yield or reroute cooperatively.
- This is the protocol-driven version of emergency handling, which is the main contribution of the new design.

### 5.2 Metrics to compare

The comparison should focus on emergency-vehicle-specific outcomes.

#### Primary metrics
- Emergency response time: time from emergency activation to the first emergency vehicle reaching the affected zone.
- Emergency vehicle delay: extra delay experienced by emergency vehicles compared with a free-flow or ideal reference path.
- Response time under high congestion: response time measured during periods when network congestion is high.
- Response time when multiple emergency vehicles are active simultaneously: performance under overlapping emergencies or dense emergency activity.

#### Why these are the right metrics
- They isolate the impact of the protocol-driven emergency behavior.
- They show whether the new design improves emergency mobility specifically, not just general traffic flow.
- They are especially relevant for a thesis argument because they directly support the claim that the proposed system helps emergency vehicles reach critical zones faster.

### 5.3 Results and visuals plan

#### 1. Before/after bar chart per algorithm
- Create one grouped bar chart for each traditional algorithm and E3 Hybrid.
- Each chart should compare:
  - emergency response time,
  - emergency vehicle delay,
  - response time under high congestion.
- The visual should make it obvious that E3 Hybrid improves emergency-vehicle performance relative to the traditional algorithms.

#### 2. Percentage-improvement summary table
- Produce a summary table with rows for:
  - Baseline,
  - ACO,
  - Dijkstra,
  - A*.
- Columns should show percentage improvement of E3 Hybrid over each algorithm for:
  - emergency response time,
  - emergency vehicle delay,
  - response time under high congestion.
- This gives a compact quantitative summary for the thesis.

#### 3. Time series or box plot for response time distribution
- Use a time series plot to show how response time changes over the simulation, especially during emergency windows.
- Alternatively, use a box plot to show the distribution of emergency response times before vs. after.
- The box plot is especially useful if the goal is to show consistency and reduction in variability, not only average improvement.

### 5.4 Additional logging/instrumentation needed

Yes — this requires a bit more instrumentation than the general metrics section alone.

#### Additional logging needed
- Event start time and event end time for each emergency incident.
- First-arrival time of each emergency vehicle to the active emergency zone.
- Per-vehicle emergency delay from the planned route or ideal path.
- A congestion-window flag so response time can be measured specifically under high-congestion conditions.
- A multi-emergency flag so the system can distinguish periods with one active emergency versus several active simultaneously.

#### Why this is necessary
- The general metrics section covers overall network behavior, but it does not guarantee emergency-specific timing is captured in a way that supports a before/after comparison.
- This section needs dedicated event-level logging so the thesis can make a precise claim about emergency improvement.

### 5.5 Interpretation guidance for the thesis
- The key claim should be: E3 Hybrid improves emergency responsiveness by using protocol-based cooperation rather than relying only on local or traditional routing behavior.
- The strongest thesis narrative is that the improvement is visible not only in overall traffic metrics, but specifically in emergency-vehicle response performance under congestion and multi-emergency conditions.
