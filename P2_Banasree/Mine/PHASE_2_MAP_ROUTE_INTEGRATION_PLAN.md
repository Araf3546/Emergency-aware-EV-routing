# Phase 2: Porto Map and Route Integration Plan

## Purpose

Build a clean, reproducible SUMO scenario using one study city, one matching road network, and demand calibrated from the Porto Taxi Service Trajectory dataset. This phase replaces the archived mixed Banasree/NYC workflow; it does not replace the P3 protocol and emergency-coordination research components.

## Governing rules

- One geographic area only: Porto, Portugal.
- The OSM network and taxi observations must cover the same study area.
- GPS trajectories are used to calibrate and validate demand; they are not copied directly into SUMO as routes.
- Every generated OD pair must be reachable on the SUMO network.
- The random seed, demand, incident scenarios, and vehicle settings must be identical for all routing algorithms.
- Each routing algorithm must return a complete, destination-valid route.
- All generated artifacts must be reproducible from source data and configuration files.
- The approved study-area polygon and its numeric buffer must exist before any OSM download.
- Emergency incidents and emergency dispatch trips must be seeded, reachable, and pre-validated; they must not be placed randomly during a simulation run.

## Initial P3 scenario values to lock in the experiment contract

| Parameter | Initial value | Rule |
|---|---:|---|
| SUMO version | 1.19.0 | Record the complete `sumo --version` output before every reproducibility release. |
| Target demand | 8,000 vehicles | This is the initial P3 target. Any change requires an experiment-contract revision. |
| Simulation duration | 3,600 seconds | This is the initial P3 target. Any change requires an experiment-contract revision. |
| Emergency-vehicle ratio | 6% | Must remain within the approved 5–7% range unless the thesis scope is revised. |
| Random seeds | Explicit integer values | Store separate seeds for demand generation, route choice, incidents, and each repeated run. |

## Proposed active project structure

```text
Mine/
  config/
    experiment_contract.md
    porto_study_area.geojson
    scenario.yaml
  data/
    raw/                 # Downloaded Porto dataset; never edited
    processed/           # Cleaned/calibrated tables derived from raw data
  network/
    porto.osm
    porto.net.xml
    network_validation.json
  demand/
    demand_statistics.json
    od_pairs.csv
    porto.trips.xml
    porto.rou.xml
  scenarios/
    baseline.sumocfg
  scripts/
    01_prepare_porto_data.py
    02_build_network.py
    03_validate_network.py
    04_calibrate_demand.py
    05_generate_trips.py
    06_generate_routes.py
    07_validate_scenario.py
  results/               # Created only after a validated scenario exists
```

The existing P3 modules remain at the project root until their interfaces are refactored against the validated scenario.

## Step-by-step implementation plan

### 0. Freeze the experiment contract

Create `config/experiment_contract.md` before downloading or generating files.

It must record:

- study city, dataset version, licence, and data-source citation;
- coordinate reference system, exact study-area polygon, and an explicit buffer distance;
- SUMO version/build and exact `netconvert` and `duarouter` command settings;
- target demand volume (8,000 vehicles), simulation duration (3,600 seconds), and fixed random seeds;
- OD construction rules and reachability rule;
- vehicle and emergency-vehicle proportions;
- incident/event templates and message assumptions;
- metrics, units, exclusions, and aggregation rules;
- number of repeated runs and statistical summary method.

**Gate:** no implementation proceeds until this contract is reviewed and agreed.

### 1. Acquire and document the Porto trajectory data

Download the official Porto Taxi Service Trajectory dataset into `data/raw/` and record its version, checksum, licence, and citation.

Use these fields:

- `TRIP_ID`
- `TIMESTAMP`
- `MISSING_DATA`
- `POLYLINE`
- taxi/call metadata only where useful for analysis

**Output:** `data/raw/porto_taxi_train.csv` and `data/raw/DATASET_PROVENANCE.md`.

### 2. Define the study area from observed coverage

Use a reproducible sample of valid Porto trajectories to determine a compact urban boundary. The selected area must have a dense connected road network, meaningful intersections/signals, and enough observed trips.

Before downloading OSM data, create and approve the exact boundary polygon and numeric buffer around it. The buffer must include sufficient road context around observed origins and destinations so that extraction-boundary edge effects do not dominate the simulation.

**Output:** `config/porto_study_area.geojson`.

**Gate:** inspect the boundary on a map and confirm that it represents central urban Porto rather than an arbitrary rectangle. Record its bounding coordinates, CRS, area, and buffer distance in the experiment contract. No OSM data is downloaded before this gate passes.

### 3. Build the Porto SUMO network

Download/extract OSM roads only inside the approved boundary. Convert the OSM file with SUMO `netconvert`, retaining passenger-vehicle-accessible roads and traffic-light information.

**Outputs:**

- `network/porto.osm`
- `network/porto.net.xml`
- the exact conversion command and parameters in `network/BUILD_NETWORK.md`

Record the full `sumo --version` output, the complete `netconvert` command, all options, the OSM extraction date/source, and file checksums. Rebuilding the network must use these recorded settings exactly.

### 4. Validate network topology before generating demand

Check network size, strongly connected components, passenger-accessible edges, source/sink candidates, junctions, traffic lights, and disconnected/dead-end artifacts.

Do not generate trips until the network passes validation.

**Output:** `network/network_validation.json`.

**Acceptance criteria:** the selected source/sink edge pool supports a high reachable-OD rate and no systematic extraction-boundary failures are present.

### 4A. Confirm that connectivity supports the selected demand volume

Test the network against the intended 8,000-vehicle OD demand before generating the full trip file. Sample candidate OD pairs from the calibrated distribution and verify that:

- selected origins and destinations lie in the passenger-vehicle strongly connected component;
- at least 99% of sampled candidate OD pairs are routeable by `duarouter` or an equivalent network path check;
- enough distinct valid source and sink edges exist to avoid concentrating the whole demand on a few edges;
- the short baseline smoke test does not show systematic route failures caused by extraction boundaries.

**Gate:** demand generation proceeds only after the connectivity report demonstrates that the approved OD pool can support the chosen demand volume.

### 5. Clean and summarize Porto trajectories

Filter invalid or incomplete GPS traces, remove unusable records, and derive:

- departure-time distribution;
- observed duration: `(GPS point count - 1) x 15 seconds`;
- observed GPS distance using successive coordinate pairs;
- origin/destination coordinates;
- OD-zone distribution;
- trip-length and duration distributions.

Keep the cleaning thresholds and counts in a machine-readable report.

**Outputs:**

- `data/processed/porto_clean_trips.parquet`;
- `data/processed/porto_cleaning_report.json`;
- `demand/demand_statistics.json`.

### 6. Connect observed OD demand to the SUMO network

Create network zones or valid source/sink edge pools. Map the observed OD coordinates or OD zones to the approved network area, then reject or resample pairs that are outside the area, identical, invalid, or unreachable.

This is demand calibration, not a claim that every GPS path equals the simulated route.

**Output:** `demand/od_pairs.csv`.

### 7. Generate reproducible baseline trips

Generate a fixed number of passenger-vehicle trips from calibrated OD and time distributions using an explicit random seed. Store the seed and all generation parameters in `config/scenario.yaml`.

Add emergency vehicles only through a separate, documented scenario generator after the normal-traffic baseline is validated.

**Output:** `demand/porto.trips.xml`.

### 8. Produce destination-valid baseline routes

Run `duarouter` with the Porto network and generated trips. Retain only valid connected routes; report every rejected trip and its reason.

**Output:** `demand/porto.rou.xml`.

### 9. Create and validate the baseline SUMO scenario

Create one `scenarios/baseline.sumocfg` that references only `network/porto.net.xml` and `demand/porto.rou.xml`.

Run a short headless smoke test followed by the full baseline. Validate departures, arrivals, route errors, teleports, travel time, waiting time, and distance outputs.

**Outputs:**

- `scenarios/baseline.sumocfg`;
- `results/baseline_smoke_test/validation_report.json`.

**Gate:** routing algorithms and P3 integration begin only after this scenario completes with documented acceptable error rates.

### 9A. Define seeded, reachable emergency scenarios

Create emergency scenarios from pre-validated incident candidates, not from `random.choice()` during a simulation. For every scenario, store:

- incident ID, incident type, start/end time, and incident seed;
- affected edge set and the exact SUMO action used to reduce capacity or close the road;
- emergency vehicle type, dispatch origin, destination/incident target, and departure time;
- proof that the emergency vehicle can reach the incident through a valid route before the incident activates;
- message radius, freshness window, and protocol settings.

Use the same incident scenario and seed for every algorithm in a comparison run.

**Output:** `scenarios/emergency_scenarios.yaml` and an emergency-scenario validation report.

**Gate:** P3 emergency evaluation begins only after every incident and emergency response route has passed reachability validation.

### 10. Refactor the algorithm and P3 integration

Only after the baseline gate passes:

- implement destination-complete Dijkstra and A* baselines;
- implement ACO and ACO-PSO with valid full-route construction;
- refactor `p3_runtime.py` to use the new common routing interface;
- replace the legacy emergency-trip script with a seeded, reachable emergency-scenario generator;
- evaluate normal and emergency scenarios under identical seeds.

## Checklist

- [ ] Archive remaining legacy metrics and visualization artifacts.
- [ ] Create and approve `config/experiment_contract.md`.
- [ ] Obtain the Porto dataset and record provenance.
- [ ] Define and approve the exact Porto study-area polygon and buffer before downloading OSM data.
- [ ] Download matching OSM data.
- [ ] Record the exact SUMO version/build, OSM source date, and conversion commands.
- [ ] Convert OSM to `porto.net.xml` with recorded parameters.
- [ ] Validate topology and reachability.
- [ ] Prove that connectivity supports the 8,000-vehicle OD demand volume.
- [ ] Clean Porto trajectories and save the cleaning report.
- [ ] Derive demand statistics and OD distribution.
- [ ] Map OD demand to valid network edges/zones.
- [ ] Generate seeded baseline trips.
- [ ] Generate and validate baseline routes with `duarouter`.
- [ ] Run a baseline smoke test and inspect outputs.
- [ ] Approve the baseline scenario gate.
- [ ] Refactor routing algorithms and integrate the P3 protocol.
- [ ] Add seeded, reachable emergency scenarios and validate them before P3 evaluation.
- [ ] Run repeated, fair algorithm comparisons.

## Current status

Phase 1 archive completed. Phase 2 has not started; no Porto map, demand, route, or SUMO configuration file exists in the active workspace yet.
