# scooterouter

E-scooter collection route planner built in C++17.

Given a fleet of scooters scattered across a real city (loaded from OpenStreetMap),
scooterouter computes the optimal collection route for a field operator — prioritizing
by battery level and damage status — and replans dynamically in real-time using D* Lite
when conditions change mid-mission.

## Algorithms
- **A\*** — primary path planner on the OSM road graph
- **D\* Lite** — incremental replanner for mid-mission changes
- **Dijkstra / BFS** — benchmark baselines
- **2-opt VRP** — collection order optimizer

## Dependencies
SDL2, libosmium, CLI11, fmt, nlohmann-json, Google Benchmark, GoogleTest — all managed via vcpkg.

## Build
\`\`\`bash
git clone https://github.com/mohamedabdelfatah97/scooterouter.git
cd scooterouter
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=../vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build
./build/scooterouter --map data/maps/munich.osm --fleet data/scooters/fleet.csv
\`\`\`

## Controls
| Key / Click | Action |
|---|---|
| Click scooter | Mark collected / not found |
| R | Force full replan |
| Space | Pause / resume animation |
| +/- | Speed up / slow down |
| ESC | Quit |
