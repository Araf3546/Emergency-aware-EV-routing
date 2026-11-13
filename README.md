# Emergency-aware routing

Research code for evaluating emergency-aware, decentralized vehicle routing in
the Porto road network. The project combines SUMO traffic simulation with
swarm-inspired routing algorithms and a P3 emergency-communication protocol.
It is a private research repository and is not packaged as a production
library.

## What this project does

The simulation compares ordinary traffic routing with emergency-aware routing
under seeded incidents and network closures. The main P3 arm is a hybrid
router that combines:

- ant-colony optimisation (ACO);
- ACO with particle-swarm optimisation (ACO-PSO);
- E3 exploration and exploitation components;
- destination-aware heuristics and bounded pheromone state;
- protocol messages from emergency and ordinary vehicles; and
- an explicit yielding rule for traffic near emergency responders.

The repository also contains Dijkstra and A* baselines, route-validity checks,
emergency dispatch and incident management, metric collection, held-out
evaluation, and Optuna-based E3 weight tuning.

The current experiment contract uses a 3,600-second Porto simulation with
4,000 vehicles: 3,760 normal vehicles and 240 ambient emergency vehicles.
Three scripted emergency events are defined in
[`scenarios/emergency_scenarios.yaml`](scenarios/emergency_scenarios.yaml).
Treat [`config/scenario.yaml`](config/scenario.yaml) and
[`config/experiment_contract.md`](config/experiment_contract.md) as the
authoritative description of experiment settings.

## Repository layout

| Path | Purpose |
| --- | --- |
| [`algorithms/`](algorithms/) | ACO, ACO-PSO, A*, Dijkstra, E3, routing utilities, and route outcomes |
| [`controller/`](controller/) | Simulation controllers, decision points, triggers, and rerouting |
| [`data_processing/`](data_processing/) | Taxi-data cleaning, temporal processing, and OD mapping |
| [`metrics/`](metrics/) | Trip, emergency-response, baseline, logging, and tuning metrics |
| [`scripts/`](scripts/) | Numbered data/build/validation pipeline and diagnostic tools |
| [`tests/`](tests/) | Unit and integration-style regression tests |
| [`config/`](config/) | Study-area geometry, scenario configuration, and experiment contract |
| [`Data/`](Data/) | Raw/processed Porto data and provenance records |
| [`network/`](network/) | OSM input, SUMO network, and network-build/validation artefacts |
| [`demand/`](demand/) | Generated OD pairs, trips, routes, speed-factor manifests, and reports |
| [`scenarios/`](scenarios/) | SUMO configuration and seeded emergency scenarios |
| [`results/`](results/) | Experiment outputs, reports, diagnostics, figures, and tuning campaigns |
| [`archive_2026-09-11/`](archive_2026-09-11/) | Historical handoffs and superseded design/audit material |

Generated demand, network validation, and result files may not be present in a
fresh checkout. Do not edit files under `Data/raw/`; the raw archive is
immutable and its provenance is recorded in
[`Data/raw/porto/DATASET_PROVENANCE.md`](Data/raw/porto/DATASET_PROVENANCE.md).

## Requirements

- Windows (the documented SUMO paths use Windows conventions);
- Python 3.10 or newer;
- Eclipse SUMO 1.25.0, including `sumo`, `netconvert`, and `duarouter`;
- Python packages used by the pipeline: `traci`, `sumolib`, `pyarrow`,
  `scipy`, `numpy`, `PyYAML`, `optuna`, and `pytest`.

The repository pins the tuning dependencies in
[`requirements-tuning.txt`](requirements-tuning.txt). The data-cleaning and
OD-mapping stages additionally require the scientific/data packages listed
above. SUMO is an external installation and is not installed by pip.

Create an environment from the project directory:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements-tuning.txt
python -m pip install pyarrow scipy numpy PyYAML pytest
```

If PowerShell prevents activation, run the commands with the environment's
Python directly, for example `.\.venv\Scripts\python.exe -m pytest`.

## Reproducing the data and network pipeline

Run commands from the `Mine` directory. The numbered scripts are intended to
be run in order; each script writes a report alongside its output.

1. **Validate the SUMO network**

   ```powershell
   python scripts/03_validate_network.py
   ```

   The default input is `network/porto.net.xml`. Network construction and the
   OSM provenance are documented in
   [`network/BUILD_NETWORK.md`](network/BUILD_NETWORK.md).

2. **Clean and calibrate the Porto taxi trajectories**

   ```powershell
   python scripts/04_calibrate_demand.py --raw-archive Data/raw/train.csv.zip
   ```

   The script streams the immutable ZIP archive, filters trajectories to the
   approved study polygon, and writes processed Parquet/statistics outputs.
   The canonical archive and expected filtering counts are documented in the
   dataset provenance file.

3. **Map cleaned OD endpoints to the SUMO network**

   ```powershell
   python scripts/06_map_od_to_network.py
   ```

4. **Validate routeability of a seeded OD sample**

   ```powershell
   python scripts/05_validate_od_routeability.py
   ```

5. **Generate normal demand and destination-valid routes**

   ```powershell
   python scripts/07_generate_baseline_trips.py
   python scripts/08_generate_baseline_routes.py
   ```

6. **Freeze per-vehicle speed factors and create emergency demand**

   ```powershell
   python scripts/08b_freeze_speed_factors.py
   python scripts/09_validate_emergency_dispatch_routes.py
   python scripts/10_generate_emergency_demand.py
   python scripts/11_generate_emergency_dispatch.py
   ```

   These steps preserve paired populations across evaluation seeds and create
   the emergency/dispatch route files consumed by SUMO.

The scripts expose additional options through `--help`. In particular, use
explicit `--raw-archive` and SUMO executable paths when local paths differ
from the defaults.

## Running simulations and evaluations

The SUMO configuration files are:

- [`simulation.sumocfg`](simulation.sumocfg) for the main seeded simulation;
- [`scenarios/baseline.sumocfg`](scenarios/baseline.sumocfg) for the baseline
  configuration.

The main runtime is implemented in
[`p3_runtime.py`](p3_runtime.py), with protocol and routing state in
[`p3_protocol.py`](p3_protocol.py), [`p3_hybrid_router.py`](p3_hybrid_router.py),
and [`emergency_manager.py`](emergency_manager.py).

After the demand and route files exist, the standard post-processing sequence
is:

```powershell
python scripts/12_measure_emergency_response.py
python scripts/13_verify_route_validity.py
python scripts/14_run_p3_protocol.py
python scripts/15_verify_metrics.py
python scripts/16_compare_baseline_vs_p3.py
```

Use the diagnostic scripts under
[`scripts/diagnostics/`](scripts/diagnostics/) for focused checks such as
route completion, closure handling, emergency-response delay, paired
population integrity, teleport invalidation, and comparison equivalence.
Most diagnostics require a result directory and expose their required inputs
with `--help`.

Evaluation seeds are coordinated through
[`evaluation_seeds.py`](evaluation_seeds.py). Do not independently change the
SUMO seed, route-choice seed, or frozen demand file: an evaluation seed must
select all three together. The current evaluation seeds and held-out tuning
seed are listed in [`config/scenario.yaml`](config/scenario.yaml).

## Tuning E3 weights

E3 tuning uses Optuna and a persistent SQLite study. The main entry point is
[`scripts/tune_e3_weights.py`](scripts/tune_e3_weights.py). A typical
invocation is:

```powershell
python scripts/tune_e3_weights.py --help
```

Read the experiment contract before starting a campaign. Tuning must use the
configured 3,600-second duration and the held-out tuning seed; do not tune on
the evaluation seeds used for reported results. Existing campaigns and their
reports are retained under [`results/`](results/).

## Testing

Run the complete Python regression suite from `Mine`:

```powershell
python -m pytest -q
```

The tests cover protocol messages, route selection and normalization,
pheromone bounds, E3 tuning, route validity, emergency metrics, seeded
evaluation behavior, yielding, and teleport/closure handling. A focused test
file can be run while developing, for example:

```powershell
python -m pytest -q tests/test_p3_hybrid_router.py
```

For a reproducibility check, also run the relevant numbered validation script
and inspect its JSON report rather than relying only on a successful process
exit.

## Data, results, and reproducibility conventions

- Porto taxi data comes from the UCI Taxi Service Trajectory dataset. See the
  [dataset provenance record](Data/raw/porto/DATASET_PROVENANCE.md) for the
  citation, license, checksums, and filtering procedure.
- OSM/SUMO network inputs and checksums are recorded in
  [`network/BUILD_NETWORK.md`](network/BUILD_NETWORK.md).
- Scenario settings, seeds, vehicle counts, protocol behavior, arrival
  definitions, and tuning rules are locked in
  [`config/experiment_contract.md`](config/experiment_contract.md).
- Results are organized by study/date under [`results/`](results/). Each
  completed study should retain its README, command/configuration evidence,
  reports, and checksums where applicable.
- Files in `results/_archive/` and `results/_invalid/` are historical or
  superseded evidence and should not be used as current results without
  checking their accompanying README.

## Citation and status

This is private research code supporting the P3/emergency-aware-routing study.
The repository contains both maintained experiment code and historical
research artefacts; it should be read alongside the experiment contract and
the result-specific READMEs when reproducing a reported experiment.

