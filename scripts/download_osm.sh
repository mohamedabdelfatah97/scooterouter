#!/bin/bash
# download_osm.sh — download OSM extract via Overpass API
# Usage: ./scripts/download_osm.sh munich
# Output: data/maps/munich.osm

CITY=${1:-munich}
declare -A BBOX
BBOX[munich]="48.12,11.55,48.16,11.62"
BBOX[berlin]="52.48,13.37,52.54,13.45"
BBOX[hamburg]="53.54,9.97,53.58,10.04"

BOX=${BBOX[$CITY]}
if [ -z "$BOX" ]; then
  echo "Unknown city: $CITY. Available: munich, berlin, hamburg"
  exit 1
fi

OUTPUT="data/maps/${CITY}.osm"
URL="https://overpass-api.de/api/map?bbox=${BOX}"

echo "Downloading OSM extract for ${CITY} (bbox: ${BOX})..."
curl -L "$URL" -o "$OUTPUT"
echo "Saved to ${OUTPUT}"
