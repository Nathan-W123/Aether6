# Plate 1

A cinematic render of one flight, built for sharing rather than for analysis.

The circuit is printed on a drafting sheet. The aircraft flies above it at its logged
altitude, casts a shadow on the paper, and drops a vertical projection line to its plan
position — the descriptive-geometry convention for tying a body in space to its plan view. The
track is laid down as it goes.

**The track's colour is ground speed**, on one sequential ramp. That is why there are no wind
arrows: with 6 m/s of wind on a 26 m/s aeroplane, ground speed runs from about 17 to 35 m/s
around the circuit while airspeed barely moves, so the wind draws itself as the gradient.

Everything except the aircraft's *size* is logged data. A 2.4 m aircraft on a 1.9 km sheet is
sub-pixel, so the glyph is drawn at a roughly constant angular size; its attitude is the
logged quaternion.

## Building it

```bash
make build        # the C++ binaries
make dashboard-build   # the frontend's node_modules: Three.js and the IBM Plex faces
make plate        # or: media/plate/build.sh
```

The headless browser driver installs itself into `media/plate/node_modules` on the first run.
Also needs an ffmpeg with libx264 — `pip install imageio-ffmpeg` provides one — and a Chromium,
taken from `$CHROMIUM` if set.

The figures read `results/monte_carlo/` and `results/linear/`, both committed, so they build
without re-running the campaign.

Output lands in `results/media/`:

| File | What it is |
|---|---|
| `aether6-plate1.mp4` | 22 s, 1080×1080, H.264 — the one to post |
| `aether6-plate1.gif` | 480 px fallback for places that will not take video |
| `aether6-plate1-plan.png` | 2048² still, square to the sheet |
| `aether6-plate1-quarter.png` | 2048² still, three-quarter view |
| `aether6-dispersed-tracks.png` | 2048² figure: every recorded track of the campaign |
| `aether6-modes.png` | 2048² figure: the five modes in the complex plane |

The stills and figures are committed; the video is not — it is 17 MB of regenerable binary
against a repository whose entire history is 18 MB.

## The two figures

They carry no plate furniture — no title block, no plate number, no commentary. Each is the
figure, its axes and the labels it needs, so either stands alone beside the animation.

**Dispersed tracks** answers "does it hold up?": every recorded trajectory of the 256-trial
campaign, the completed trials forming a band and the trials a safety limit stopped picked out
over it. **Modes** answers "is the physics real?": the eigenvalues of the numerically
linearised aircraft, with the spiral sitting just inside the divergent half-plane, which is
correct for an airframe of this class.

A third figure — position error against the filter's own ±1σ — was built and dropped. The
errors run *outside* the band: the filter is optimistic about its position uncertainty, as
recorded in `docs/limitations.md` §6. The figure would have had to claim a consistency the
data does not show, so it is not here.

## Files

| File | Role |
|---|---|
| `prepare.py` | a simulator run → the payload the renderer reads |
| `plate.js` | the sheet, its printed drawing, the track ribbon, the aircraft |
| `stage.js` | camera, lighting, per-frame update |
| `film.html` | the page the capture drives |
| `capture.mjs` | steps film time and writes one PNG per frame |
| `stills.mjs` | shoots the plate stills at 2048² |
| `prepare_figures.py` | a campaign and a modal analysis → the figures' payload |
| `figures.js` | the two standalone figures |
| `figures.mjs` | shoots them at 2048² |
| `build.sh` | runs the whole chain |
