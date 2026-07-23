# Experiment Contract: Porto Emergency-Aware EV Routing

**Version:** 0.2 (phase-2 draft)  
**Status:** Phase 2 setup - study area approved; network and scenario generation not started  
**Last updated:** 2026-07-21

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
| Target vehicle demand | 8,000 total vehicles |
| Simulation duration | 3,600 seconds |
| Emergency-vehicle ratio | 6% (480 vehicles) |
| Normal vehicles | 7,520 vehicles |
| Emergency types | ambulance, fire truck, police, relief vehicle |
| Base reproducibility seed | 20260721 |
| Study-area file | `config/porto_study_area.geojson` |

Before any reported result, record the complete `sumo --version` output, OSM extract source/date, `netconvert` command, `duarouter` command, and checksums of all input files.

## 5. Reproducibility and fairness rules

- Each comparison run uses the identical OD demand, departure times, vehicle types, incident scenario, and seed for every algorithm.
- Routing logic is the only permitted difference between compared runs.
- Every algorithm must produce a complete, destination-valid path before it can be evaluated.
- Randomness must use explicit seeds stored in `config/scenario.yaml`.
- Repeated experiments will use a predefined seed list; the number of repetitions is pending final computational-budget approval.
- All rejected trips, route failures, teleports, and incomplete trips must be reported, not silently ignored.

## 6. Demand requirements

- Demand must be derived from cleaned Porto trajectories within the approved study area.
- The normal-traffic baseline is calibrated from Porto-local departures in the 14:00-16:00 window. Those observed 7,200 seconds are linearly mapped to the 3,600-second simulation duration; this preserves relative departure timing within the selected window without claiming that the simulation represents two real-time hours.
- OD pairs must map to passenger-vehicle-accessible SUMO edges/zones.
- At least 99% of sampled candidate OD pairs must be routeable before full trip generation.
- The source/sink pool must contain enough distinct valid edges to avoid artificial concentration of demand.
- A fixed, seed-controlled demand generator creates the 8,000-vehicle baseline trip file.

**Gate B:** Full demand generation begins only after network topology and OD-reachability validation pass.

## 7. Emergency scenario requirements

- Incidents are defined before the simulation starts in `scenarios/emergency_scenarios.yaml`.
- Each incident has a fixed ID, seed, type, start/end time, affected edges, and explicit SUMO road-impact action.
- Each emergency dispatch vehicle has a valid origin, destination/incident target, departure time, and routeability proof.
- The same incident scenario and seed are used for every algorithm comparison.
- Protocol settings, including message radius and freshness window, are stored with every scenario.

**Gate C:** P3 emergency evaluation begins only after every incident and emergency dispatch route passes validation.

## 8. Evaluation metrics

| Metric | Definition | Unit |
|---|---|---|
| Travel time | Vehicle arrival time minus departure time | seconds |
| Waiting time | SUMO accumulated waiting time per completed vehicle | seconds |
| Distance travelled | SUMO route/travel distance per completed vehicle | metres |
| Congestion | Mean vehicle time loss relative to free-flow movement; report definition and aggregation with results | seconds/vehicle |
| Reroutes | Number of successful route replacements per vehicle | count |
| Emergency response time | Incident activation to first valid emergency-vehicle arrival | seconds |
| Energy consumption | Optional; only report if a validated SUMO energy model is configured | Wh |

Report completed-vehicle count, incomplete-vehicle count, route errors, and teleports alongside every metric.

## 9. Required validation gates

- **Gate A - Study area:** exact Porto boundary, CRS, and buffer approved before OSM download.
- **Gate B - Network:** passenger-edge connectivity and calibrated-OD reachability support the 8,000-vehicle demand.
- **Gate C - Baseline:** the seeded SUMO baseline completes with documented route-error, teleport, and completion rates.
- **Gate D - Emergency:** all seeded incidents and dispatch routes are reachable and reproducible.
- **Gate E - Algorithms:** each algorithm returns full valid routes and uses the shared scenario interface.

## 10. Change-control rule

Any change to the study boundary, SUMO version, conversion command, demand volume, duration, seed, vehicle mix, metric definition, or emergency scenario requires:

1. a contract version update;
2. a short rationale;
3. regeneration of affected artifacts; and
4. re-validation of the relevant gate.
