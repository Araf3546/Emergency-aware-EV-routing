# P3 Implementation Checklist

## 1. Foundation and scope
- [x] Confirm the target simulation scale: 8,000 vehicles with 5–7% emergency vehicles.
- [x] Decide the final simulation duration and lock it in for all runs (recommended: 3600 seconds).
- [x] Keep the current ACO + PSO hybrid as the base implementation and add BCO as a third component rather than replacing the existing logic.
- [x] Keep emergency handling protocol-driven from the start; do not rely on local blocked-edge logic as the main mechanism.

## 2. Protocol implementation
- [x] Create a vehicle message structure for broadcast data.
- [x] Implement mandatory fields: vehicle ID, timestamp, vehicle type, current edge, speed, priority level, emergency flag, and route intent.
- [x] Implement optional fields: congestion estimate, yield request, next edge, distance to emergency, and hop count.
- [x] Define how messages are created and serialized in a compact form.
- [x] Define how messages are received and stored in a nearby-neighbor state table.
- [x] Implement staleness handling so old messages are discounted or dropped.
- [x] Implement fallback behavior when no fresh message is received.
- [x] Ensure emergency messages are prioritized over normal traffic messages.

## 3. Hybrid algorithm implementation
- [x] Create or extend the routing module so it can consume protocol messages at each decision step.
- [x] Keep ACO pheromone logic as the reinforcement component.
- [x] Keep PSO velocity-style adaptation as the dynamic adjustment component.
- [x] Add BCO-inspired scout/forager behavior as a third exploration mechanism.
- [x] Define a combined scoring function that includes pheromone, heuristic suitability, protocol weight, and BCO exploration.
- [ ] Decide how weights are initialized and tuned for the thesis experiments.
- [x] Ensure the system uses the protocol-derived emergency information in the main scoring logic rather than as a separate patch.

## 4. Emergency vehicle behavior
- [x] Implement emergency vehicle message generation with maximum priority and emergency flag enabled.
- [x] Implement normal-vehicle reaction logic for emergency messages and yield behavior.
- [x] Ensure emergency vehicles receive higher routing priority in the decision logic.
- [x] Ensure normal vehicles can yield or divert around emergency corridors when the protocol indicates urgency.
- [x] Make emergency influence explicit in the choice of next edge or route update.

## 5. Simulation integration
- [x] Connect the protocol state update loop to the SUMO step loop.
- [x] Ensure messages are updated at a controlled frequency and not too often to create unnecessary overhead.
- [x] Ensure route updates are applied only when the decision is meaningful and valid.
- [x] Connect the routing decision output to actual route changes in SUMO.
- [x] Feed successful routing outcomes back into the hybrid learning state.
- [x] Keep the simulation loop stable at the larger vehicle scale.

## 6. Efficiency and robustness
- [ ] Avoid unnecessary recomputation of route scores on every step.
- [ ] Use efficient data structures for neighbor state and edge scoring.
- [ ] Keep the implementation correct even if it is computationally heavier.
- [ ] Do not remove emergency logic or simplify the protocol just to improve runtime.
- [ ] Flag any speed-vs-accuracy tradeoff explicitly in the thesis notes if one becomes necessary.

## 7. Metrics and logging
- [ ] Log travel time, waiting time, and distance for all vehicles.
- [ ] Log emergency response time and emergency vehicle delay.
- [ ] Log congestion level over time.
- [x] Log number of reroutes and protocol-related events.
- [x] Log message overhead and freshness where possible.
- [ ] Log enough data for one or two full runs per configuration rather than many shallow runs.
- [ ] Add dedicated emergency-event logging for event start time, first-arrival time, emergency delay, high-congestion windows, and multi-emergency windows.
- [ ] Ensure the logging supports a before/after comparison between traditional algorithms and E3 Hybrid.

## 8. Visualization and thesis output
- [ ] Produce bar charts for average travel time, waiting time, distance, and emergency response time.
- [ ] Produce time-series plots for congestion and emergency behavior over the simulation.
- [ ] Produce at least one congestion or route-impact visual for emergency zones.
- [ ] Prepare a short explanation of why one or two full runs are preferred over many short runs.
- [ ] Make sure the final thesis text clearly explains the protocol, the hybrid algorithm, and the emergency override logic.

## 9. Validation milestones
- [ ] Milestone 1: protocol layer works and messages are generated/received correctly.
- [ ] Milestone 2: ACO + PSO + BCO hybrid produces valid route decisions using protocol information.
- [ ] Milestone 3: emergency vehicles and normal vehicles respond correctly to emergency signaling.
- [ ] Milestone 4: full-scale simulation runs complete with stable logging and outputs.
- [ ] Milestone 5: results are summarized and ready to be written into the thesis.
