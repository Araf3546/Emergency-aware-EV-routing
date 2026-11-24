# Experiment Contract: Porto Emergency-Aware EV Routing

**Version:** 0.5 (phase-3 draft - response-time measurement window clarified)  
**Status:** Phase 3 - Porto network/demand/emergency scenario data exist; simulation-layer defects under repair (see THESIS_CODE_AUDIT.md)  
**Last updated:** 2026-09-06

## 1. Research purpose

Evaluate whether the proposed decentralized bio-inspired hybrid routing framework improves vehicle-routing performance and emergency coordination compared with SUMO default routing, Dijkstra, A*, and ACO under identical Porto traffic scenarios.

The P3 contribution includes a lightweight vehicle-to-vehicle message protocol that shares relevant local traffic state, emergency priority, route intent, and yield requests.

## 2. Data provenance

| Item | Value |
|---|---|
| Study city | Porto, Portugal |
| Demand-calibration dataset | Taxi Service Trajectory - Prediction Challenge, ECML PKDD 2015 |
| Source archive | Official Porto taxi trajectory dataset archive in `data/raw/` |
| Raw file policy | The archive is immutable. All derived data is written outside `Data/raw/`. |
| Dataset checksum | Pending - calculate before extraction/processing. |
| Dataset licence/citation | Record from the official UCI dataset page in `data/raw/DATASET_PROVENANCE.md`. |

The Porto GPS trajectories calibrate observed departure patterns, OD patterns, trip duration, and trip distance. They are not inserted directly as SUMO routes.

## 3. Study-area decision - mandatory before OSM download

| Required item | Approved value |
|---|---|
| Boundary format | `config/porto_study_area.geojson` |
| Coordinate reference system | WGS84 / EPSG:4326 for the boundary and raw GPS data |
| Exact boundary polygon | `[[[-8.61817, 41.14282], [-8.61817, 41.15513], [-8.59798, 41.15513], [-8.59798, 41.14282], [-8.61817, 41.14282]]]` |
| Numeric buffer around boundary | 1000 m |
| Boundary metadata | CRS: EPSG:4326; feature name: Porto study area |
| Rationale | The approved study area is a compact, connected, central urban Porto boundary with a 1000 m context buffer to reduce extraction edge effects while preserving dense observed taxi coverage. |

Approximate bounding box: west -8.61817, south 41.14282, east -8.59798, north 41.15513.

**Gate A:** No OSM map may be downloaded or converted until the polygon, buffer distance, CRS, and rationale are completed and reviewed.

## 4. Simulation environment

| Parameter | Locked initial value |
|---|---:|
| SUMO version | Eclipse SUMO 1.25.0 |
| Target vehicle demand | 4,000 total vehicles (+3 scripted incident responders) |
| Simulation duration | 3,600 seconds |
| Emergency-vehicle ratio | 6% (240 vehicles) |
| Normal vehicles | 3,760 vehicles |
| Emergency types | ambulance, fire truck, police, relief vehicle |
| Base reproducibility seed | 20260721 |
| Study-area file | `config/porto_study_area.geojson` |

Before any reported result, record the complete `sumo --version` output, OSM extract source/date, `netconvert` command, `duarouter` command, and checksums of all input files.

## 5. Reproducibility and fairness rules

- Each comparison run uses the identical OD demand, departure times, vehicle types, incident scenario, and seed for every algorithm.
- Routing logic is the only permitted difference between compared runs.
- Every algorithm must produce a complete, destination-valid path before it can be evaluated.
- Dijkstra and A* baselines must be full-destination, admissible/complete implementations (unbounded search to the true destination; a consistent heuristic for A*). A bounded-horizon or inadmissible baseline is not an acceptable comparison point for the thesis result.
- Weight-tuning campaigns (e.g. Optuna for E3) must run at the full contract simulation duration (3,600 seconds - see §4), matching exactly what is reported as the final result. Tuning at a shorter duration for cost reasons requires an explicit, documented rationale and is not acceptable as the default.
- Randomness must use explicit seeds stored in `config/scenario.yaml`.
- Repeated experiments will use a predefined seed list; the number of repetitions is pending final computational-budget approval.
- All rejected trips, route failures, teleports, and incomplete trips must be reported, not silently ignored.

## 6. Demand requirements

- Demand must be derived from cleaned Porto trajectories within the approved study area.
- The normal-traffic baseline is calibrated from Porto-local departures in the 14:00-16:00 window. Those observed 7,200 seconds are linearly mapped to the 3,600-second simulation duration; this preserves relative departure timing within the selected window without claiming that the simulation represents two real-time hours.
- OD pairs must map to passenger-vehicle-accessible SUMO edges/zones.
- At least 99% of sampled candidate OD pairs must be routeable before full trip generation.
- The source/sink pool must contain enough distinct valid edges to avoid artificial concentration of demand.
- A fixed, seed-controlled demand generator creates the baseline trip file at the demand level fixed in §4.

**Gate B:** Full demand generation begins only after network topology and OD-reachability validation pass.

## 7. Emergency scenario requirements

- Incidents are defined before the simulation starts in `scenarios/emergency_scenarios.yaml`.
- Each incident has a fixed ID, seed, type, start/end time, affected edges, and explicit SUMO road-impact action.
- Each emergency dispatch vehicle has a valid origin, destination/incident target, departure time, and routeability proof.
- The same incident scenario and seed are used for every algorithm comparison.
- Protocol settings, including message radius and freshness window, are stored with every scenario.
- Emergency-vehicle population has two required parts: (1) 240 ambient emergency-class vehicles (6% of demand, per §4) inserted as ordinary background traffic that broadcast emergency priority via the protocol like any emergency vehicle; (2) the 3 seeded incident-dispatch vehicles defined in `scenarios/emergency_scenarios.yaml`, whose response times are the ones formally measured for the before/after emergency-response comparison.
- An active incident closes its affected edges (`sumo_action: close_edge`, `capacity_factor: 0.0`) to every vehicle class except `vClass="emergency"`, which remains permitted so dispatched responders can physically reach the incident.
- Emergency response time is measured from incident activation until the dispatched responder's current road ID equals the incident's `center_edge`. A response time attributed to any vehicle other than the incident's own `dispatch.vehicle_id` must not be reported as that incident's response time.
- The measurement is **not bounded by the incident window**. A responder that reaches the `center_edge` after the incident's `end_time` still yields a valid response time, which must be reported and flagged as a post-window arrival, never discarded. Incident duration governs how long the closure is applied to the network; it is not a deadline on the metric.
- The V2V protocol supports hop-limited multi-hop forwarding, with a maximum of 3 hops. Message latency and packet loss are not modeled; every message is delivered instantly and reliably within the configured radius and freshness window. This is a stated simplification, not an oversight.

**Gate C:** P3 emergency evaluation begins only after every incident and emergency dispatch route passes validation.

## 8. Evaluation metrics

| Metric | Definition | Unit |
|---|---|---|
| Travel time | Vehicle arrival time minus departure time | seconds |
| Waiting time | SUMO accumulated waiting time per completed vehicle | seconds |
| Distance travelled | SUMO route/travel distance per completed vehicle | metres |
| Congestion | Mean vehicle time loss relative to free-flow movement; report definition and aggregation with results | seconds/vehicle |
| Reroutes | Number of successful route replacements per vehicle | count |
| Emergency response time | Incident activation to arrival of that incident's dispatched responder at its `center_edge`; not bounded by the incident window (see §7) | seconds |
| Energy consumption | Optional; only report if a validated SUMO energy model is configured | Wh |

Report completed-vehicle count, incomplete-vehicle count, route errors, and teleports alongside every metric.

**Implementation note (locked 2026-09-03):** Congestion must be computed from SUMO's own `timeLoss` field in `tripinfo.xml` (or an equivalent per-vehicle time-loss-vs-free-flow computation), not from a live speed-ratio proxy. A speed-ratio time series may still be logged as a secondary diagnostic signal, but it must be labelled as such and must never be reported as the contract's congestion metric.

## 9. Required validation gates

- **Gate A - Study area:** exact Porto boundary, CRS, and buffer approved before OSM download.
- **Gate B - Network:** passenger-edge connectivity and calibrated-OD reachability support the §4 demand. (Gate B was originally passed against the superseded 8,000-vehicle figure; 4,000 is a strict subset of that demand, so the gate still holds.)
- **Gate C - Baseline:** the seeded SUMO baseline completes with documented route-error, teleport, and completion rates.
- **Gate D - Emergency:** all seeded incidents and dispatch routes are reachable and reproducible.
- **Gate E - Algorithms:** each algorithm returns full valid routes and uses the shared scenario interface.

## 10. Change-control rule

Any change to the study boundary, SUMO version, conversion command, demand volume, duration, seed, vehicle mix, metric definition, or emergency scenario requires:

1. a contract version update;
2. a short rationale;
3. regeneration of affected artifacts; and
4. re-validation of the relevant gate.

## 11. Decisions log

Definitions locked during the 2026-09-03 review of `THESIS_CODE_AUDIT.md` §13 (open decisions). Each entry supersedes any conflicting behaviour previously implemented in code; code must be corrected to match, not the reverse.

| # | Decision | Locked value | Rationale |
|---|---|---|---|
| 13.1 | Emergency-vehicle population | Both: 240 ambient emergency-class vehicles + 3 seeded incident dispatches (was 480 when demand was 8,000; scaled by decision 13.8) | The contract's 6% figure and the scenario file's 3 dispatches were previously two disconnected concepts; both are required so ambient emergency behaviour and measured incident response are both represented. |
| 13.2 | Incident physical effect | Full edge closure (`close_edge`, `capacity_factor: 0.0`), exempting `vClass="emergency"` | Already declared in `scenarios/emergency_scenarios.yaml`; the exemption is added so dispatched responders can still reach the incident once closure is actually implemented in code (see THESIS_CODE_AUDIT.md B4). |
| 13.3 | Arrival definition | Dispatched responder's road ID equals the incident's `center_edge` | Follows directly from the vClass exemption in 13.2; keeps the metric simple and matches this contract's existing §8 wording ("first valid emergency-vehicle arrival"). |
| 13.4 | Congestion metric | SUMO `tripinfo.xml` `timeLoss`, seconds/vehicle | Already this contract's §8 definition; the code's speed-ratio proxy was a deviation from the contract, not an alternate valid definition. |
| 13.5 | Weight-tuning simulation duration | Full 3,600 seconds, matching §4 | Avoids reporting weights tuned under a different traffic distribution than the one used in final experiments. |
| 13.6 | Dijkstra / A* baseline strength | Fixed to full-destination, admissible implementations | A comparison against deliberately weakened baselines is not defensible at examination. |
| 13.7 | Protocol scope (hop count, latency/loss) | Hop-limited multi-hop forwarding, max 3 hops; no latency or packet-loss model | Multi-hop forwarding is implemented so `hop_count` is meaningful; latency/loss modeling is explicitly out of scope rather than left silently unimplemented. |
| 13.8 | Experimental demand level | 4,000 vehicles per 3,600 s (3,760 normal + 240 emergency), plus 3 scripted responders | Set empirically, not arbitrarily. A capacity sweep on this network (2,000/3,000/4,000/5,000/6,000/8,003 vehicles, identical release window, drained to completion) shows 100% insertion and 100% completion up to 4,000 with 6 teleports, then sharp breakdown at 5,000 (78.6% inserted, 1,955 teleports, mean time loss 119 s -> 1,201 s). 4,000 is therefore the measured maximum stable operating point. The previous 8,000 figure was an undocumented target that ran at roughly twice network capacity and made emergency response time unmeasurable. Supersedes the §4 values from contract v0.3. |
| 13.9 | Emergency dispatch timing | Responder departs at incident `start_time` + 60 s | In scenario v1 each responder departed ~120 s *before* its own incident, which is physically impossible and produced negative response times. Incident windows are unchanged; route validity is time-independent, so Gate D still holds. |
| 13.10 | Response-time measurement window | Response time is measured whenever the responder reaches the `center_edge`, including after the incident's `end_time` | Clarification of 13.3, not a change to it: §7 and §8 never imposed a window condition, but the code gated detection on the incident still being active (audit finding M9). Measured over seeds 20260722/23/24, all three responders reach their incident after their own closure window ends (windows are 240-330 s; travel times 245-588 s), so that gate discarded every one of the nine measurements rather than a subset. No demand, scenario, timing or route artifact changes, so no gate needs re-validation. |
