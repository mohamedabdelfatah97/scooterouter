# scooterouter

C++17 field operations route planner for e-scooter collection on real OpenStreetMap data.

Given a fleet of scooters scattered across Hamburg, scooterouter computes the optimal
collection route for a field operator — prioritizing by battery level and damage status
— and visualizes it in real time using SDL2.

## Demo

Hamburg road network (25,299 nodes, 44,536 edges) with 15 scooters color-coded by urgency.
Route computed via priority-weighted A* with 2-opt VRP optimization.

## Algorithms

| Algorithm | Role | Time (Hamburg) |
|-----------|------|----------------|
| A* (Euclidean) | Primary path planner | 18ms |
| Dijkstra | Benchmark baseline | 24.8ms |
| BFS | Unweighted baseline | 12ms |
| D* Lite | Incremental replanner | — |
| Nearest neighbor + 2-opt | VRP collection order | — |

Benchmarks run on Apple M4, debug build.

## Controls

| Key | Action |
|-----|--------|
| R | Trigger replan + frontier animation |
| F | Toggle scooter markers |
| P | Toggle route path |
| Space | Pause / resume |
| ESC | Quit |

## Stack

C++17 · SDL2 · libosmium · CMake · vcpkg · Google Benchmark · GoogleTest

## Build
```bash
git clone https://github.com/mohamedabdelfatah97/scooterouter.git
cd scooterouter
cmake -B build -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=../vcpkg/scripts/buildsystems/vcpkg.cmake \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/scooterouter --map data/maps/hamburg.osm --fleet data/scooters/fleet.csv
```

## Data

OSM extract via Overpass API. Fleet CSV with id, lat, lon, battery_pct, status columns.