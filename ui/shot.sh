#!/usr/bin/env bash
# Screenshot harness for the HELISIM design studio.
#
# Drives the studio through a set of deterministic states (via URL params) and
# captures each with headless Chromium/Edge — the visual feedback loop for
# iterating on the UI. Requires the studio server running (`helisim ui`).
#
# Usage:  ui/shot.sh [base_url] [out_dir]
#   ui/shot.sh                       # http://127.0.0.1:8080 -> ui/shots/
#   ui/shot.sh http://127.0.0.1:8080 /tmp/shots

set -euo pipefail
BASE="${1:-http://127.0.0.1:8080}"
OUT="${2:-ui/shots}"
W=1600; H=1000
BUDGET=9000   # ms of virtual time before the snapshot (lets fetch + render settle)

# Find a Chromium-family browser.
BROWSER=""
for c in \
  "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
  "/Applications/Chromium.app/Contents/MacOS/Chromium" \
  "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge" \
  "/Applications/Brave Browser.app/Contents/MacOS/Brave Browser" \
  "$(command -v chromium 2>/dev/null || true)" \
  "$(command -v google-chrome 2>/dev/null || true)"; do
  if [ -n "$c" ] && [ -x "$c" ]; then BROWSER="$c"; break; fi
done
[ -z "$BROWSER" ] && { echo "no Chromium-family browser found"; exit 1; }
echo "browser: $BROWSER"

mkdir -p "$OUT"

# name -> url-query pairs
shoot() {
  local name="$1" query="$2"
  local url="$BASE/${query:+?$query}"
  "$BROWSER" --headless=new --hide-scrollbars --window-size=${W},${H} \
    --enable-unsafe-swiftshader --use-gl=angle --use-angle=swiftshader \
    --force-device-scale-factor=1 \
    --screenshot="$OUT/$name.png" --virtual-time-budget=$BUDGET \
    "$url" >/dev/null 2>&1
  echo "  ✓ $OUT/$name.png   ($url)"
}

echo "capturing studio states -> $OUT/"
shoot "01-overview"  ""
shoot "02-blade"     "part=blade"
shoot "03-explode"   "explode=1"
shoot "04-fea"       "tab=fea"
shoot "05-build-6061-doublers"  "build=1"
shoot "06-build-ream-root"      "build=3"
shoot "07-build-balance"        "build=6"
shoot "08-assembly-blade-root"  "asm=4"
shoot "09-optimize"  "tab=optimize"
shoot "10-cfd"       "tab=cfd"
echo "done."
