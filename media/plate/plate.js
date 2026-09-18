/*
 * Aether-6 — Plate 1: an engineering drawing the aircraft makes by flying.
 *
 * The circuit is printed on a drafting sheet. The aircraft flies above it at its logged
 * altitude, casts a shadow on the paper, and drops a vertical projection line to its plotted
 * position — the descriptive-geometry convention for relating a body in space to its plan view.
 * The track is laid down as it goes.
 *
 * The track's colour is ground speed on a single sequential ramp. That is why there are no
 * wind arrows: with a 6 m/s wind on a 26 m/s aeroplane, ground speed runs from 17 to 35 m/s
 * around the circuit while airspeed barely moves, so the wind draws itself as the gradient.
 *
 * Every number here is read from `flight.js`, a copy of the CSV the compiled simulator wrote.
 */
import * as THREE from './three.module.js'

export const F = window.FLIGHT
export const N = F.t.length
export const DURATION = F.t[N - 1]

const PAPER = '#eceee9'
const LINE = '#14191b'
const RULE = '#a9b2ae'
const GRID = '#ccd3ce'
const FAINT = '#7b8689'

/** Sequential ramp, pale to deep. Slow reads light, fast reads dark. */
const RAMP = ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b']
const GS_LO = Math.floor(F.meta.gs_min)
const GS_HI = Math.ceil(F.meta.gs_max)

export function rampAt(u) {
  const x = Math.max(0, Math.min(1, u)) * (RAMP.length - 1)
  const i = Math.min(RAMP.length - 2, Math.floor(x))
  return new THREE.Color(RAMP[i]).lerp(new THREE.Color(RAMP[i + 1]), x - i)
}
const speedU = (gs) => (gs - GS_LO) / (GS_HI - GS_LO)

// ---- sheet extent -----------------------------------------------------------------------------
const stride = 25
const ns = F.waypoints.map((w) => w.n).concat(F.n.filter((_, i) => i % stride === 0))
const es = F.waypoints.map((w) => w.e).concat(F.e.filter((_, i) => i % stride === 0))
const midN = (Math.min(...ns) + Math.max(...ns)) / 2
const midE = (Math.min(...es) + Math.max(...es)) / 2
export const SHEET = Math.max(Math.max(...ns) - Math.min(...ns),
                              Math.max(...es) - Math.min(...es)) * 1.62
const originN = midN - SHEET / 2
const originE = midE - SHEET / 2
export const CENTRE = new THREE.Vector3(midE, 0, -midN)

/** NED to the viewer's Y-up frame: east = +x, altitude = +y, north = −z (determinant +1). */
export const toScene = (n, e, alt, v = new THREE.Vector3()) => v.set(e, alt, -n)

/** Rotate a body-frame axis by the logged quaternion, then into scene axes. */
export function bodyAxis(q, bx, by, bz) {
  const ned = new THREE.Vector3(bx, by, bz).applyQuaternion(q)
  return new THREE.Vector3(ned.y, -ned.z, -ned.x)
}

// ---- the printed sheet --------------------------------------------------------------------------
const TEX = 2600
const tx = (e) => ((e - originE) / SHEET) * TEX
const ty = (n) => TEX - ((n - originN) / SHEET) * TEX

function drawSheet() {
  const c = document.createElement('canvas')
  c.width = TEX
  c.height = TEX
  const g = c.getContext('2d')
  const S = TEX / 1000
  const mono = (v, w = 400) => `${w} ${v * S}px "IBM Plex Mono", ui-monospace, monospace`
  const cond = (v, w = 600) => `${w} ${v * S}px "IBM Plex Sans Condensed", Arial, sans-serif`

  g.fillStyle = PAPER
  g.fillRect(0, 0, TEX, TEX)
  g.textBaseline = 'alphabetic'

  // grid: 50 m fine, 250 m heavy
  const firstOn = (v, step) => Math.ceil(v / step) * step
  for (const axis of ['e', 'n']) {
    for (let v = firstOn(axis === 'e' ? originE : originN, 50);
         v < (axis === 'e' ? originE : originN) + SHEET; v += 50) {
      const heavy = Math.abs(v % 250) < 1e-6
      g.strokeStyle = heavy ? RULE : GRID
      g.lineWidth = (heavy ? 1.0 : 0.55) * S
      g.beginPath()
      if (axis === 'e') { g.moveTo(tx(v), 0); g.lineTo(tx(v), TEX) }
      else { g.moveTo(0, ty(v)); g.lineTo(TEX, ty(v)) }
      g.stroke()
    }
  }

  const M = 44 * S
  g.strokeStyle = LINE
  g.lineWidth = 2.2 * S
  g.strokeRect(M, M, TEX - M * 2, TEX - M * 2)

  // coordinate figures in the margins
  g.fillStyle = FAINT
  g.font = mono(14)
  for (let e = firstOn(originE, 250); e < originE + SHEET; e += 250) {
    if (tx(e) < M + 40 * S || tx(e) > TEX - M - 40 * S) continue
    g.textAlign = 'center'
    g.fillText(`${e.toFixed(0)}`, tx(e), TEX - M + 24 * S)
  }
  for (let n = firstOn(originN, 250); n < originN + SHEET; n += 250) {
    if (ty(n) < M + 40 * S || ty(n) > TEX - M - 40 * S) continue
    g.textAlign = 'right'
    g.fillText(`${n.toFixed(0)}`, M - 10 * S, ty(n) + 5 * S)
  }
  g.textAlign = 'left'

  // the commanded circuit, as construction lines
  g.strokeStyle = LINE
  g.lineWidth = 1.5 * S
  g.setLineDash([13 * S, 9 * S])
  g.beginPath()
  F.waypoints.forEach((w, i) => (i ? g.lineTo(tx(w.e), ty(w.n)) : g.moveTo(tx(w.e), ty(w.n))))
  g.closePath()
  g.stroke()
  g.setLineDash([])

  // waypoint marks
  F.waypoints.forEach((w) => {
    const x = tx(w.e)
    const y = ty(w.n)
    g.strokeStyle = LINE
    g.lineWidth = 1.9 * S
    g.beginPath(); g.arc(x, y, 12 * S, 0, Math.PI * 2); g.stroke()
    g.beginPath()
    g.moveTo(x - 19 * S, y); g.lineTo(x + 19 * S, y)
    g.moveTo(x, y - 19 * S); g.lineTo(x, y + 19 * S)
    g.stroke()
    g.fillStyle = LINE
    g.font = mono(17, 500)
    g.fillText(`${w.i}`, x + 22 * S, y - 10 * S)
  })

  // north arrow
  const nx = TEX - M - 92 * S
  const ny = M + 116 * S
  g.strokeStyle = LINE
  g.lineWidth = 1.9 * S
  g.beginPath(); g.moveTo(nx, ny + 44 * S); g.lineTo(nx, ny - 34 * S); g.stroke()
  g.fillStyle = LINE
  g.beginPath()
  g.moveTo(nx, ny - 50 * S); g.lineTo(nx - 10 * S, ny - 24 * S); g.lineTo(nx + 10 * S, ny - 24 * S)
  g.closePath(); g.fill()
  g.font = cond(19, 600)
  g.textAlign = 'center'
  g.fillText('N', nx, ny + 66 * S)
  g.textAlign = 'left'

  // scale bar
  const bar = 500
  const bx = M + 34 * S
  const by = TEX - M - 42 * S
  const bw = (bar / SHEET) * TEX
  g.strokeStyle = LINE
  g.lineWidth = 2 * S
  g.beginPath()
  g.moveTo(bx, by); g.lineTo(bx + bw, by)
  for (const t of [0, 0.5, 1]) {
    g.moveTo(bx + bw * t, by - (t === 0.5 ? 4 : 6) * S)
    g.lineTo(bx + bw * t, by + (t === 0.5 ? 4 : 6) * S)
  }
  g.stroke()
  g.fillStyle = LINE
  g.font = mono(14)
  g.fillText('0', bx - 3 * S, by + 22 * S)
  g.fillText(`${bar} m`, bx + bw - 22 * S, by + 22 * S)

  // the ramp key: the only legend the plate needs
  const kx = M + 34 * S
  const ky = M + 74 * S
  const kw = 300 * S
  const kh = 13 * S
  for (let i = 0; i < 160; i += 1) {
    g.fillStyle = `#${rampAt(i / 159).getHexString()}`
    g.fillRect(kx + (kw * i) / 160, ky, kw / 160 + 1, kh)
  }
  g.strokeStyle = LINE
  g.lineWidth = 1.1 * S
  g.strokeRect(kx, ky, kw, kh)
  g.fillStyle = FAINT
  g.font = cond(13, 500)
  g.fillText('GROUND SPEED  [m/s]', kx, ky - 10 * S)
  g.fillStyle = LINE
  g.font = mono(14)
  g.fillText(`${GS_LO}`, kx - 2 * S, ky + kh + 20 * S)
  g.textAlign = 'right'
  g.fillText(`${GS_HI}`, kx + kw, ky + kh + 20 * S)
  g.textAlign = 'left'

  // printed subplot: cross-track error over the whole flight
  const sw = 330 * S
  const sh = 108 * S
  const sx = M + 34 * S
  const sy = TEX - M - 190 * S
  g.strokeStyle = LINE
  g.lineWidth = 1.4 * S
  g.beginPath()
  g.moveTo(sx, sy); g.lineTo(sx, sy + sh); g.lineTo(sx + sw, sy + sh)
  g.stroke()
  const LIM = 100
  const yOf = (v) => sy + sh / 2 - (Math.max(-LIM, Math.min(LIM, v)) / LIM) * (sh / 2)
  g.strokeStyle = RULE
  g.lineWidth = 0.9 * S
  g.beginPath(); g.moveTo(sx, yOf(0)); g.lineTo(sx + sw, yOf(0)); g.stroke()
  g.strokeStyle = '#3987e5'
  g.lineWidth = 1.5 * S
  g.beginPath()
  for (let i = 0; i < N; i += 2) {
    const x = sx + (i / (N - 1)) * sw
    const y = yOf(F.xtrack[i])
    if (i === 0) g.moveTo(x, y)
    else g.lineTo(x, y)
  }
  g.stroke()
  g.fillStyle = FAINT
  g.font = cond(13, 500)
  g.fillText('CROSS-TRACK ERROR  [m]', sx, sy - 9 * S)
  g.font = mono(12)
  g.fillText(`+${LIM}`, sx + sw + 6 * S, sy + 6 * S)
  g.fillText('0', sx + sw + 6 * S, yOf(0) + 4 * S)
  g.fillText(`−${LIM}`, sx + sw + 6 * S, sy + sh + 4 * S)

  // title block
  const tw = 410 * S
  const th = 142 * S
  const tbx = TEX - M - tw
  const tby = TEX - M - th
  g.fillStyle = PAPER
  g.fillRect(tbx, tby, tw, th)
  g.strokeStyle = LINE
  g.lineWidth = 2.2 * S
  g.strokeRect(tbx, tby, tw, th)
  g.lineWidth = 1 * S
  g.beginPath()
  g.moveTo(tbx, tby + 44 * S); g.lineTo(tbx + tw, tby + 44 * S)
  g.moveTo(tbx, tby + 93 * S); g.lineTo(tbx + tw, tby + 93 * S)
  g.moveTo(tbx + tw * 0.5, tby + 44 * S); g.lineTo(tbx + tw * 0.5, tby + th)
  g.stroke()

  g.fillStyle = LINE
  g.font = cond(21, 600)
  g.fillText('AETHER-6 · AUTONOMOUS CIRCUIT', tbx + 13 * S, tby + 30 * S)
  const field = (x, y, label, value) => {
    g.fillStyle = FAINT
    g.font = cond(11.5, 500)
    g.fillText(label, x, y)
    g.fillStyle = LINE
    g.font = mono(16, 500)
    g.fillText(value, x, y + 23 * S)
  }
  field(tbx + 13 * S, tby + 65 * S, 'CONTROL LAW', `${F.meta.controller} on EKF`)
  field(tbx + tw * 0.5 + 13 * S, tby + 65 * S, 'WIND', `${F.meta.wind.toFixed(1)} m/s`)
  field(tbx + 13 * S, tby + 114 * S, 'CROSS-TRACK RMS', `${F.meta.xtrack_rms.toFixed(1)} m`)
  field(tbx + tw * 0.5 + 13 * S, tby + 114 * S, 'NAV RMSE', `${F.meta.nav_rmse.toFixed(2)} m`)

  g.fillStyle = LINE
  g.font = cond(25, 600)
  g.fillText('PLATE 1', M + 34 * S, M + 40 * S)

  const tex = new THREE.CanvasTexture(c)
  tex.colorSpace = THREE.SRGBColorSpace
  tex.anisotropy = 8
  return tex
}

export function buildSheet() {
  const group = new THREE.Group()
  const paper = new THREE.Mesh(
    new THREE.PlaneGeometry(SHEET, SHEET),
    new THREE.MeshStandardMaterial({ map: drawSheet(), roughness: 0.95, metalness: 0 }))
  paper.rotation.x = -Math.PI / 2
  paper.position.copy(CENTRE)
  paper.receiveShadow = true
  group.add(paper)

  const slab = new THREE.Mesh(
    new THREE.BoxGeometry(SHEET, 3, SHEET),
    new THREE.MeshStandardMaterial({ color: 0xdcdfd9, roughness: 1 }))
  slab.position.set(CENTRE.x, -2, CENTRE.z)
  slab.castShadow = true
  slab.receiveShadow = true
  group.add(slab)

  const table = new THREE.Mesh(
    new THREE.PlaneGeometry(SHEET * 8, SHEET * 8),
    new THREE.MeshStandardMaterial({ color: 0x1d2124, roughness: 1 }))
  table.rotation.x = -Math.PI / 2
  table.position.set(CENTRE.x, -3.8, CENTRE.z)
  table.receiveShadow = true
  group.add(table)
  return group
}

// ---- the flown track, as a ribbon coloured by ground speed -------------------------------------
export function buildTrack() {
  const points = []
  for (let i = 0; i < N; i += 1) points.push(toScene(F.n[i], F.e[i], F.alt[i]))
  const curve = new THREE.CatmullRomCurve3(points)
  const TUBULAR = N - 1
  const RADIAL = 7
  const geo = new THREE.TubeGeometry(curve, TUBULAR, 2.6, RADIAL, false)

  // Colour each ring of the tube by the ground speed logged at that sample.
  const colours = new Float32Array(geo.attributes.position.count * 3)
  const c = new THREE.Color()
  for (let i = 0; i <= TUBULAR; i += 1) {
    const sample = Math.min(N - 1, Math.round((i / TUBULAR) * (N - 1)))
    c.copy(rampAt(speedU(F.gs[sample])))
    for (let j = 0; j <= RADIAL; j += 1) {
      const k = (i * (RADIAL + 1) + j) * 3
      colours[k] = c.r; colours[k + 1] = c.g; colours[k + 2] = c.b
    }
  }
  geo.setAttribute('color', new THREE.BufferAttribute(colours, 3))

  const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
    vertexColors: true, roughness: 0.55, metalness: 0.0,
  }))
  mesh.castShadow = false
  // Each tubular segment contributes RADIAL * 6 indices, so the draw range is the progress.
  mesh.userData.perSegment = RADIAL * 6
  mesh.userData.segments = TUBULAR
  return mesh
}

/** The plan-view track, laid on the paper as the pen would draw it. */
export function buildPlot() {
  const positions = new Float32Array(N * 3)
  const colours = new Float32Array(N * 3)
  const c = new THREE.Color()
  for (let i = 0; i < N; i += 1) {
    positions[i * 3] = F.e[i]
    positions[i * 3 + 1] = 0.9
    positions[i * 3 + 2] = -F.n[i]
    c.copy(rampAt(speedU(F.gs[i])))
    colours[i * 3] = c.r; colours[i * 3 + 1] = c.g; colours[i * 3 + 2] = c.b
  }
  const geo = new THREE.BufferGeometry()
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3))
  geo.setAttribute('color', new THREE.BufferAttribute(colours, 3))
  const line = new THREE.Line(geo, new THREE.LineBasicMaterial({ vertexColors: true }))
  line.frustumCulled = false
  return line
}

/** What the navigation filter believed the track was: one fine dashed line, in ink. */
export function buildEstimate() {
  const pts = []
  for (let i = 0; i < N; i += 1) pts.push(toScene(F.en[i], F.ee[i], F.ealt[i]))
  const geo = new THREE.BufferGeometry().setFromPoints(pts)
  const line = new THREE.Line(geo, new THREE.LineDashedMaterial({
    color: 0x14191b, dashSize: 22, gapSize: 13, transparent: true, opacity: 0.7,
  }))
  line.computeLineDistances()
  line.frustumCulled = false
  return line
}

/** A small, precise glyph: matte white over ink, no gloss, no emissive. */
export function buildAircraft() {
  const g = new THREE.Group()
  const shell = new THREE.MeshStandardMaterial({ color: 0xf2f4f1, roughness: 0.62, metalness: 0 })
  const dark = new THREE.MeshStandardMaterial({ color: 0x14191b, roughness: 0.7, metalness: 0 })

  const fuselage = new THREE.Mesh(new THREE.CapsuleGeometry(1.25, 8.2, 6, 18), shell)
  fuselage.rotation.z = Math.PI / 2
  g.add(fuselage)
  const nose = new THREE.Mesh(new THREE.ConeGeometry(1.25, 3.2, 18), dark)
  nose.rotation.z = -Math.PI / 2
  nose.position.x = 5.6
  g.add(nose)

  const planform = (root, tip, span) => {
    const s = new THREE.Shape()
    s.moveTo(root / 2, 0); s.lineTo(-root / 2, 0)
    s.lineTo(-tip / 2 - root * 0.12, span); s.lineTo(tip / 2 - root * 0.12, span)
    s.closePath()
    const geo = new THREE.ExtrudeGeometry(s, { depth: 0.3, bevelEnabled: false })
    geo.rotateX(Math.PI / 2)
    return geo
  }
  const wing = planform(3.4, 1.9, 9.0)
  for (const side of [1, -1]) {
    const w = new THREE.Mesh(wing, shell)
    w.scale.z = side
    w.position.set(-0.4, -0.3, 0)
    g.add(w)
  }
  const tail = planform(1.9, 1.1, 3.3)
  for (const side of [1, -1]) {
    const t = new THREE.Mesh(tail, dark)
    t.scale.z = side
    t.position.set(-5.2, -0.15, 0)
    g.add(t)
  }
  const finGeo = planform(1.9, 1.0, 2.8)
  finGeo.rotateX(-Math.PI / 2)
  const fin = new THREE.Mesh(finGeo, dark)
  fin.position.set(-5.2, 0.1, -0.15)
  g.add(fin)

  g.traverse((o) => { if (o.isMesh) o.castShadow = true })
  return g
}

export function frameAt(time) {
  const t = Math.max(0, Math.min(DURATION, time))
  let lo = 0
  let hi = N - 1
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1
    if (F.t[mid] <= t) lo = mid
    else hi = mid
  }
  const u = (t - F.t[lo]) / ((F.t[hi] - F.t[lo]) || 1)
  const l = (a) => a[lo] + (a[hi] - a[lo]) * u
  const q = new THREE.Quaternion(F.qx[lo], F.qy[lo], F.qz[lo], F.qw[lo])
    .slerp(new THREE.Quaternion(F.qx[hi], F.qy[hi], F.qz[hi], F.qw[hi]), u).normalize()
  return {
    t, q, index: lo,
    pos: toScene(l(F.n), l(F.e), l(F.alt)),
    est: toScene(l(F.en), l(F.ee), l(F.ealt)),
    gs: l(F.gs), alt: l(F.alt), xtrack: l(F.xtrack), wp: F.wp[lo],
  }
}
