# Porto OSM extraction

- Query date/time: 2026-07-21, [2026-07-21T16:36:17Z]
- Source: OpenStreetMap data via Overpass API / Overpass Turbo
- Original study area: config/porto_study_area.geojson
- Download area: config/porto_osm_extract_area.geojson
- Buffer: 1000 m
- CRS used to create buffer: EPSG:3763
- Download CRS: EPSG:4326
- Bounding box: - Bounding box (south, west, north, east):
   `41.1338157498141896,-8.6300833614775705,41.1641342168691082,-8.5860666070307037`
- Full Overpass query: [[out:xml][timeout:180];

way["highway"](41.1338157498141896,-8.6300833614775705,41.1641342168691082,-8.5860666070307037)->.roads;

node(w.roads);
out body;

.roads out body;

relation["type"="restriction"](41.1338157498141896,-8.6300833614775705,41.1641342168691082,-8.5860666070307037);
out body;]

- Output: network/porto.osm
- SHA-256: `479E41855298092E18FD473CB0C7868A1453C24AE9EE3626A9678CDDA08447A5`

## SUMO network conversion

- SUMO version: Eclipse SUMO 1.25.0
- Output: `network/porto.net.xml`
- Output SHA-256: `25D8D5B6377720892AD1DD1DB4DF7FD97D83E247115CCD8919F40A60E11051D7`
- Command:

  ```powershell
  & "C:\Program Files (x86)\Eclipse\Sumo\bin\netconvert.exe" --osm-files "D:\p3\P2_Banasree\P2_Banasree\Mine\network\porto.osm" --type-files "C:\Program Files (x86)\Eclipse\Sumo\data\typemap\osmNetconvert.typ.xml" --output-file "D:\p3\P2_Banasree\P2_Banasree\Mine\network\porto.net.xml"