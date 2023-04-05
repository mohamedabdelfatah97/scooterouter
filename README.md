# scooterouter

E-scooter collection route planner built in C++17 on real OpenStreetMap data.

## Stack
C++17 · SDL2 · libosmium · CMake · vcpkg

## Build
```bash
git clone https://github.com/mohamedabdelfatah97/scooterouter.git
cd scooterouter
cmake -B build -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=../vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build
./build/scooterouter
```

## Controls
| Key | Action |
|-----|--------|
| ESC | Quit   |