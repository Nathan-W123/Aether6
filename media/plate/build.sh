#!/usr/bin/env bash
# Render Plate 1: the animation and the stills, from a real simulator run.
#
#   media/plate/build.sh [SCENARIO.yaml]
#
# Requires: the C++ binaries built (`make build`), node with playwright-core, the frontend's
# node_modules for Three.js and the IBM Plex faces, and an ffmpeg with libx264.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HERE="$ROOT/media/plate"
WORK="${AETHER_PLATE_WORK:-$ROOT/build/plate}"
OUT="$ROOT/results/media"
SCENARIO="${1:-$HERE/plate.yaml}"
PORT="${AETHER_PLATE_PORT:-8811}"

mkdir -p "$WORK" "$OUT"

if [ ! -d "$HERE/node_modules" ]; then
  echo "== installing the headless browser driver =="
  (cd "$HERE" && npm install --no-audit --no-fund)
fi
export NODE_PATH="$HERE/node_modules"

echo "== simulating =="
"$ROOT/build/bin/aether_sim" "$SCENARIO" --output "$WORK" --quiet

echo "== preparing the payloads =="
python3 "$HERE/prepare.py" "$WORK" "$WORK"
python3 "$HERE/prepare_figures.py" "$ROOT/results/monte_carlo" "$ROOT/results/linear" "$WORK"

echo "== staging the renderer =="
cp "$HERE/plate.js" "$HERE/stage.js" "$HERE/film.html" \
   "$HERE/figures.js" "$HERE/figures.html" "$WORK/"
MODULES="$ROOT/web/frontend/node_modules"
cp "$MODULES/three/build/three.module.js" "$MODULES/three/build/three.core.js" "$WORK/"
mkdir -p "$WORK/fonts"
for face in \
  ibm-plex-mono/files/ibm-plex-mono-latin-400-normal.woff2 \
  ibm-plex-mono/files/ibm-plex-mono-latin-500-normal.woff2 \
  ibm-plex-sans-condensed/files/ibm-plex-sans-condensed-latin-500-normal.woff2 \
  ibm-plex-sans-condensed/files/ibm-plex-sans-condensed-latin-600-normal.woff2 ; do
  cp "$MODULES/@fontsource/$face" "$WORK/fonts/"
done
# The still is the same page at a larger canvas.
sed 's/1080px/2048px/g; s/width="1080" height="1080"/width="2048" height="2048"/' \
  "$WORK/film.html" > "$WORK/still.html"

echo "== serving =="
python3 -m http.server "$PORT" --bind 127.0.0.1 --directory "$WORK" >/dev/null 2>&1 &
SERVER=$!
trap 'kill "$SERVER" 2>/dev/null || true' EXIT
sleep 2

echo "== capturing frames =="
AETHER_PLATE_PORT="$PORT" AETHER_PLATE_WORK="$WORK" node "$HERE/capture.mjs"

echo "== encoding =="
FFMPEG="${FFMPEG:-$(python3 -c 'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())')}"
"$FFMPEG" -y -loglevel error -framerate 30 -i "$WORK/frames/f%05d.png" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p -movflags +faststart \
  "$OUT/aether6-plate1.mp4"
"$FFMPEG" -y -loglevel error -i "$WORK/frames/f%05d.png" \
  -vf "fps=10,scale=480:-1:flags=lanczos,palettegen=max_colors=96:stats_mode=diff" "$WORK/pal.png"
"$FFMPEG" -y -loglevel error -framerate 30 -i "$WORK/frames/f%05d.png" -i "$WORK/pal.png" \
  -lavfi "fps=10,scale=480:-1:flags=lanczos[x];[x][1:v]paletteuse=dither=none:diff_mode=rectangle" \
  "$OUT/aether6-plate1.gif"

echo "== stills =="
AETHER_PLATE_PORT="$PORT" AETHER_PLATE_WORK="$WORK" AETHER_PLATE_OUT="$OUT" \
  node "$HERE/stills.mjs"

echo "== figures =="
AETHER_PLATE_PORT="$PORT" AETHER_PLATE_OUT="$OUT" node "$HERE/figures.mjs"

echo
echo "written to $OUT:"
ls -la "$OUT"
