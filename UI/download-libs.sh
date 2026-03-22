#!/bin/bash
# BulletGCSS UI — third-party library downloader
#
# Run this script from the UI/ directory to fetch or update all bundled libraries.
# To upgrade a library: change its version variable below, then run the script.
# Commit the updated files afterwards.
#
# Usage:
#   cd UI/
#   bash download-libs.sh

set -e
cd "$(dirname "$0")"

OL_VERSION="6.5.0"
NOSLEEP_VERSION="0.12.0"
PAHO_VERSION="1.1.0"        # inferred — no version string in original file header
OLC_VERSION="1.0.3"         # inferred — no version string in original file header

echo "Downloading OpenLayers ${OL_VERSION}..."
curl -sLo ol/ol.js         "https://cdn.jsdelivr.net/npm/ol@${OL_VERSION}/dist/ol.js"
curl -sLo ol/ol.js.map     "https://cdn.jsdelivr.net/npm/ol@${OL_VERSION}/dist/ol.js.map"
curl -sLo css/ol.css       "https://cdn.jsdelivr.net/npm/ol@${OL_VERSION}/ol.css"

echo "Downloading NoSleep.js ${NOSLEEP_VERSION}..."
curl -sLo js/NoSleep.min.js "https://cdn.jsdelivr.net/npm/nosleep.js@${NOSLEEP_VERSION}/dist/NoSleep.min.js"

echo "Downloading Paho MQTT ${PAHO_VERSION}..."
curl -sLo mqttws31.js      "https://cdn.jsdelivr.net/npm/paho-mqtt@${PAHO_VERSION}/mqttws31.js"

echo "Downloading Open Location Code ${OLC_VERSION}..."
curl -sLo js/olc.min.js    "https://cdn.jsdelivr.net/npm/open-location-code@${OLC_VERSION}/OpenLocationCode.min.js"

echo ""
echo "Done. Remember to commit the updated files and bump the versions in package.json."
