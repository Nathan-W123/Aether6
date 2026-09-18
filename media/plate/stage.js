/*
 * Camera, lighting and the per-frame update for Plate 1.
 *
 * There is no overlay: everything a reader needs is printed on the sheet, which is where a
 * drawing carries its own legend, scale and title block. The camera moves slowly — an
 * engineering plate is examined, not flown through.
 */
import * as THREE from './three.module.js'
import {
  F, N, DURATION, CENTRE, SHEET, bodyAxis, buildAircraft, buildEstimate, buildPlot,
  buildSheet, buildTrack, frameAt,
} from './plate.js'

const canvas = document.getElementById('view')
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true })
renderer.setPixelRatio(1)
renderer.setSize(canvas.width, canvas.height, false)
renderer.shadowMap.enabled = true
renderer.shadowMap.type = THREE.PCFSoftShadowMap
renderer.toneMapping = THREE.ACESFilmicToneMapping
renderer.toneMappingExposure = 1.0

const scene = new THREE.Scene()
scene.background = new THREE.Color(0x14171a)
const camera = new THREE.PerspectiveCamera(30, canvas.width / canvas.height, 3, 40000)

// Studio lighting on a sheet of paper: a soft fill and one raking key that casts the shadow.
scene.add(new THREE.HemisphereLight(0xffffff, 0x2a2f33, 2.0))
const key = new THREE.DirectionalLight(0xfff8ee, 2.5)
key.position.set(-SHEET * 0.55, SHEET * 0.95, SHEET * 0.45)
key.castShadow = true
key.shadow.mapSize.set(4096, 4096)
key.shadow.camera.near = 10
key.shadow.camera.far = SHEET * 3
const H = SHEET * 0.58
Object.assign(key.shadow.camera, { left: -H, right: H, top: H, bottom: -H })
key.shadow.bias = -0.0004
key.shadow.normalBias = 1.2
key.shadow.camera.updateProjectionMatrix()
scene.add(key)
scene.add(new THREE.DirectionalLight(0xe8eef2, 0.5).translateX(SHEET * 0.5))

scene.add(buildSheet())

const track = buildTrack()
scene.add(track)
const plot = buildPlot()
scene.add(plot)
const estimate = buildEstimate()
scene.add(estimate)

const aircraft = buildAircraft()
scene.add(aircraft)

// The projection line: aircraft to its plan position, the way a drawing relates the two views.
const drop = new THREE.Line(
  new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
  new THREE.LineBasicMaterial({ color: 0x14191b, transparent: true, opacity: 0.4 }))
scene.add(drop)

const smooth = (a, b, x) => {
  const t = Math.max(0, Math.min(1, (x - a) / (b - a)))
  return t * t * (3 - 2 * t)
}

/**
 * Three held viewpoints, each drifting slowly, cross-faded by position rather than cut:
 *   a low raking view across the sheet, a three-quarter, and a near-plan of the finished plate.
 */
const HALF_PI = Math.PI / 2

// The corners of the sheet plus the top of the flight envelope: what every frame must contain.
const FIT_POINTS = (() => {
  const h = SHEET / 2
  const top = Math.max(...F.alt)
  const pts = []
  for (const dx of [-h, h]) {
    for (const dz of [-h, h]) {
      pts.push(new THREE.Vector3(CENTRE.x + dx, 0, CENTRE.z + dz))
      pts.push(new THREE.Vector3(CENTRE.x + dx * 0.55, top, CENTRE.z + dz * 0.55))
    }
  }
  return pts
})()

/**
 * Distance at which the whole plate fits the frame.
 *
 * Fitting the sheet's diagonal analytically fails at low elevation, where a square sheet
 * foreshortens and the binding constraint is its projected height, not its extent. Solving
 * it against the actual projection keeps the plate filling the frame at every viewpoint.
 */
function fitDistance(dir, target, fov, margin = 0.94, marginX = 0.94) {
  let lo = SHEET * 0.25
  let hi = SHEET * 8
  camera.fov = fov
  for (let iter = 0; iter < 22; iter += 1) {
    const mid = (lo + hi) / 2
    camera.position.copy(target).addScaledVector(dir, mid)
    camera.up.set(0, 1, 0)
    camera.lookAt(target)
    camera.updateProjectionMatrix()
    camera.updateMatrixWorld(true)
    let fits = true
    for (const point of FIT_POINTS) {
      const v = point.clone().project(camera)
      if (Math.abs(v.x) > marginX || Math.abs(v.y) > margin) { fits = false; break }
    }
    if (fits) hi = mid
    else lo = mid
  }
  return hi
}

function place(f, p) {
  // The camera sits SOUTH of the sheet looking north (+z is south in this frame), which puts
  // east to screen-right and north up. From the north the plate renders mirrored.
  const views = [
    { az: HALF_PI + 0.58, el: 0.34, fov: 31 },
    { az: HALF_PI + 0.32, el: 0.64, fov: 30 },
    { az: HALF_PI,        el: 1.06, fov: 29 },
  ]
  const x = p * (views.length - 1)
  const i = Math.min(views.length - 2, Math.floor(x))
  const k = smooth(0, 1, x - i)
  const a = views[i]
  const b = views[i + 1]
  const lerp = (u, v) => u + (v - u) * k

  // A slow drift that eases out, so nothing is static but the plate settles square at the end.
  const az = lerp(a.az, b.az) + 0.14 * (1 - smooth(0.5, 1, p))
  const el = Math.min(1.42, lerp(a.el, b.el))
  const fov = lerp(a.fov, b.fov)

  const dir = new THREE.Vector3(
    Math.cos(az) * Math.cos(el), Math.sin(el), Math.sin(az) * Math.cos(el)).normalize()
  // Lean the framing toward the aircraft only while the track is still being drawn.
  const target = CENTRE.clone().lerp(f.pos, 0.20 * (1 - smooth(0.1, 0.62, p)))
  // Low down, a flat sheet foreshortens into a letterbox, so the frame is filled on height
  // and the paper is allowed to run off the sides — the way a big drawing is photographed.
  // The constraint tightens as the camera rises, so the closing frame holds the whole plate.
  const marginX = 0.94 + 2.2 * (1 - smooth(0.30, 0.86, p))
  const distance = fitDistance(dir, target, fov, 0.94, marginX)

  camera.position.copy(target).addScaledVector(dir, distance)
  camera.up.set(0, 1, 0)
  // Aim a little below the sheet so its near edge, which perspective enlarges, does not drag
  // the plate to the bottom of the frame.
  camera.lookAt(target.clone().setY(target.y - SHEET * 0.055))
  camera.fov = fov
  camera.updateProjectionMatrix()
}

export function render(filmTime, total) {
  // Time-compress the flight, then hold on the finished plate for the last beat.
  const u = Math.min(1, filmTime / total)
  const drawTo = Math.min(1, u / 0.88)
  const f = frameAt(drawTo * DURATION)

  place(f, u)

  const shown = Math.max(2, Math.round(drawTo * (N - 1)))
  track.geometry.setDrawRange(0, shown * track.userData.perSegment)
  plot.geometry.setDrawRange(0, shown)
  estimate.geometry.setDrawRange(0, shown)

  aircraft.visible = true
  drop.visible = true
  {
    const fwd = bodyAxis(f.q, 1, 0, 0)
    const up = bodyAxis(f.q, 0, 0, -1)
    const right = bodyAxis(f.q, 0, 1, 0)
    // The real vehicle is 2.4 m across a 1.2 km sheet. The glyph is drawn at a roughly
    // constant angular size so its attitude stays readable; only its scale is a choice.
    const s = Math.max(3.0, Math.min(9.5, camera.position.distanceTo(f.pos) / 62))
    aircraft.matrixAutoUpdate = false
    aircraft.matrix.makeBasis(fwd, up, right)
      .scale(new THREE.Vector3(s, s, s))
      .setPosition(f.pos)

    const p = drop.geometry.attributes.position
    p.setXYZ(0, f.pos.x, f.pos.y, f.pos.z)
    p.setXYZ(1, f.pos.x, 1.2, f.pos.z)
    p.needsUpdate = true
    drop.geometry.computeBoundingSphere()
  }

  renderer.render(scene, camera)
  return f
}

export { DURATION, F }
