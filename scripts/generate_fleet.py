#!/usr/bin/env python3
"""
generate_fleet.py
Generates fake fleet.csv with N scooters in a city bounding box.
Usage: python3 scripts/generate_fleet.py --city munich --count 30
"""
import argparse, random, csv, os

CITIES = {
    "munich":  {"lat": (48.12, 48.16), "lon": (11.55, 11.62)},
    "berlin":  {"lat": (52.48, 52.54), "lon": (13.37, 13.45)},
    "hamburg": {"lat": (53.54, 53.58), "lon": (9.97,  10.04)},
}

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--city",   default="munich")
    p.add_argument("--count",  type=int, default=20)
    p.add_argument("--output", default="data/scooters/fleet.csv")
    args = p.parse_args()

    bounds = CITIES.get(args.city, CITIES["munich"])
    rows = []
    for i in range(1, args.count + 1):
        lat = random.uniform(*bounds["lat"])
        lon = random.uniform(*bounds["lon"])
        bat = random.randint(2, 95)
        status = random.choices(
            ["available", "available", "available", "damaged"],
            weights=[70, 70, 70, 20])[0]
        rows.append([i, f"{lat:.4f}", f"{lon:.4f}", bat, status])

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "lat", "lon", "battery_pct", "status"])
        w.writerows(rows)
    print(f"Generated {args.count} scooters → {args.output}")

if __name__ == "__main__":
    main()
