# Porto Taxi Trajectory Dataset Provenance

## Dataset

- Name: Taxi Service Trajectory - Prediction Challenge, ECML PKDD 2015
- Study city: Porto, Portugal
- Source: UCI Machine Learning Repository
- Official dataset page: https://archive.ics.uci.edu/dataset/339/taxi+service+trajectory+prediction+challenge+ecml+pkdd+2015
- DOI: https://doi.org/10.24432/C55W25
- Licence: Creative Commons Attribution 4.0 International (CC BY 4.0)
- Downloaded file: `train.csv.zip`
- Local path: `Data/raw/porto/train.csv.zip`
- Download date: 2026-07-21
- File size: 533,605,607 bytes
- SHA-256: `210DD0A20DA66A8FC2DE3440AECD84670921BC257591F8365A4475E31453C5EA`

## Canonical input - verified 2026-09-13

The archive recorded above has been verified as the input every derived artefact was built
from. It is the canonical training-data input of this thesis. It is referenced where it lies and
is **not copied** (533 MB). The "Local path" line above was the location at download. The archive
is now held one level up, and `scripts/04_calibrate_demand.py` still defaults to the old
location, so re-running it needs `--raw-archive Data/raw/train.csv.zip`.

| Field | Value |
|---|---|
| Absolute path | `D:\p3\p3-20260818T160450Z-1-001\p3\Mine\Data\raw\train.csv.zip` |
| File name | `train.csv.zip` |
| Size | 533,605,607 bytes (matches the download record) |
| SHA-256 | `210dd0a20da66a8fc2de3440aecd84670921bc257591f8365a4475e31453c5ea` (matches the value recorded at download, 2026-07-21) |
| Archive integrity | `zipfile.testzip()` reports no bad member (CRC of every member correct) |
| Contents | one member, `train.csv`: 1,942,848,724 bytes uncompressed, 533,605,447 compressed, member date 2015-04-20 17:08:32 |
| Rows | **1,710,670** data rows (1,710,589 distinct `TRIP_ID`s, so 81 ids repeat, a property of the published data) |
| Columns and types | `TRIP_ID` int64, `CALL_TYPE` text (A 364,770 / B 817,881 / C 528,019), `ORIGIN_CALL` float64 (1,345,900 empty), `ORIGIN_STAND` float64 (904,091 empty), `TAXI_ID` int64, `TIMESTAMP` int64 (Unix UTC, 2013-07-01 00:00:53 to 2014-06-30 23:59:56), `DAY_TYPE` text (all "A"), `MISSING_DATA` bool (10 true), `POLYLINE` text (a JSON list of [longitude, latitude] points, one every 15 s) |
| Preprocessing before use | `scripts/04_calibrate_demand.py` streams the CSV straight from the archive (nothing is extracted to disk). It keeps a trip when `MISSING_DATA` is false, the polyline has at least two points, and both origin and destination lie inside `config/porto_study_area.geojson`: 10 rejected for missing data, 36,508 for short polylines, 1,610,964 outside the study area, **63,188 retained** |
| Direct artefacts | `Data/processed/porto_clean_trips.parquet` (SHA-256 `13d8fe5e9aba3f7e6253f98b15d1a42ef7ce41f7e9ee40b94f79f19f5a164e79`), `Data/processed/porto_cleaning_report.json`, `demand/demand_statistics.json` |
| Downstream | `scripts/06_map_od_to_network.py` reads that parquet: 63,188 trips gave 59,669 mapped OD pairs (`demand/od_mapping_report.json`). Scripts 07 and 08, then the frozen per-seed demand, are built from the mapped pairs |

**How it was verified.** The cleaning function was re-run on this archive into a scratch folder.
All five counts were identical to the recorded report. The cleaned table was identical to the
existing one, and so was the parquet file itself, byte for byte (same SHA-256). The demand
statistics were identical apart from their generation timestamp. So this archive is not only the
same download: it reproduces the project's processed data exactly. Evidence:
`results/raw_provenance_2026-09-13/`.

## Citation

Moreira-Matias, L., Ferreira, M., Mendes-Moreira, J., L. L., & J. J.
(2013). Taxi Service Trajectory - Prediction Challenge, ECML PKDD 2015
[Dataset]. UCI Machine Learning Repository.
https://doi.org/10.24432/C55W25

## Raw-data policy

This archive is immutable. All cleaning, filtering, and derived outputs must
be written outside `Data/raw/`.