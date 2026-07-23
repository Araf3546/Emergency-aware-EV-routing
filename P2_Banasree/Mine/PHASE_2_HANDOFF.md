# Phase 2 Handoff — Porto Scenario

**Last updated:** 2026-07-22  
**Current phase:** Porto map, calibrated normal baseline demand, and route integration.

## What has been achieved

1. Archived legacy Banasree/NYC work is separated from the active Porto pipeline.
2. The Porto core study area is defined in `config/porto_study_area.geojson`.
3. A 1,000 m buffered OSM extraction area is saved as `config/porto_osm_extract_area.geojson`.
4. The OSM query, source, buffer, bounds, and checksum are documented in `network/BUILD_NETWORK.md`.
5. The buffered OSM extract was converted with SUMO 1.25.0 to `network/porto.net.xml`.
6. Network topology validation passed: 5,659 source/sink candidate passenger edges are in the largest strongly connected component.
7. The immutable Porto taxi archive was cleaned into 63,188 study-area trips and summarized in `data/processed/` and `demand/demand_statistics.json`.
8. 59,669 observed OD pairs were snapped to the validated passenger-edge pool.
9. A seeded 8,000-pair routeability sample passed at 100% with `duarouter` (Gate B).
10. The configured normal baseline uses 7,520 passenger vehicles. The seeded `demand/porto.trips.xml` and destination-valid `demand/porto.rou.xml` have been generated; emergency vehicles remain reserved for the later seeded emergency-scenario stage so that they can be validated separately.

## Locked baseline-demand choice

- Source departure window: Porto local time 14:00-16:00.
- Simulation duration: 3,600 seconds.
- Timing treatment: linearly map the selected 7,200-second observed window into the 3,600-second simulation; preserve relative timing but do not interpret the simulation as two real-time hours.
- Normal-traffic seed: `20260721`.
- Route-generation seed: `20260722`.

## Next 10 steps

1. Create `scenarios/baseline.sumocfg` referencing only `network/porto.net.xml` and `demand/porto.rou.xml`.
2. Configure deterministic baseline SUMO outputs: tripinfo, summary, warning/error log, and seed.
3. Run a short headless baseline smoke test.
4. Inspect departures, arrivals, incomplete vehicles, route errors, teleports, travel time, waiting time, and distance.
5. Write `results/baseline_smoke_test/validation_report.json` and approve or correct Gate C.
6. Run the full 3,600-second normal-traffic baseline and retain all raw output files.
7. Define fixed incident templates and emergency dispatch trips in `scenarios/emergency_scenarios.yaml`; do not use random incident placement during a run.
8. Validate every incident and emergency dispatch route before activating the P3 protocol (Gate D).
9. Refactor the legacy algorithms and P3 runtime against the shared Porto scenario interface.
10. Run fair repeated baseline, conventional-routing, and P3/E3 comparisons under identical seeds and scenarios.

## Important boundaries

- Do not modify `Data/raw/porto/train.csv.zip`.
- Do not treat `network/validation/routeability_sample.*` as final demand; those files exist only for Gate B validation.
- Do not add the 480 emergency vehicles until the seeded emergency scenarios are designed and validated.
