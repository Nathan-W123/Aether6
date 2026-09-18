/*
 * Two standalone figures.
 *
 * No sheet furniture, no title block, no commentary — just the figure, its axes and the
 * labels the figure needs to be read. Same paper, ink and type as Plate 1 so they sit
 * together, but each stands on its own.
 */
const PAPER = '#eceee9'
const LINE = '#14191b'
const GRID = '#ccd3ce'
const RULE = '#a9b2ae'
const FAINT = '#6f7a7d'
const OXIDE = '#a8351b'
const BLUE = '#2a78d6'

const W = 2048
const S = W / 1000
const px = (v) => v * S
const mono = (v, w = 400) => `${w} ${v * S}px "IBM Plex Mono", ui-monospace, monospace`
const cond = (v, w = 600) => `${w} ${v * S}px "IBM Plex Sans Condensed", Arial, sans-serif`

function niceTicks(domain, count) {
  const span = domain[1] - domain[0]
  const rough = span / count
  const mag = 10 ** Math.floor(Math.log10(Math.abs(rough) || 1))
  const norm = rough / mag
  const step = (norm >= 5 ? 10 : norm >= 2 ? 5 : norm >= 1 ? 2 : 1) * mag
  const out = []
  for (let v = Math.ceil(domain[0] / step) * step; v <= domain[1] + step * 1e-6; v += step) {
    out.push(Math.abs(v) < step * 1e-6 ? 0 : v)
  }
  return out
}

function fmt(v, ticks) {
  const step = ticks.length > 1 ? Math.abs(ticks[1] - ticks[0]) : Math.abs(v) || 1
  const d = step >= 1 ? 0 : step >= 0.1 ? 1 : 2
  return v.toFixed(d)
}

/** Axes only: two rules, ticks outside, one label per axis. */
function axes(g, box, xDomain, yDomain, xLabel, yLabel, counts = [6, 6]) {
  const { x, y, w, h } = box
  const sx = (v) => x + ((v - xDomain[0]) / (xDomain[1] - xDomain[0])) * w
  const sy = (v) => y + h - ((v - yDomain[0]) / (yDomain[1] - yDomain[0])) * h
  const xt = niceTicks(xDomain, counts[0])
  const yt = niceTicks(yDomain, counts[1])

  g.strokeStyle = GRID
  g.lineWidth = px(0.8)
  for (const t of yt) { g.beginPath(); g.moveTo(x, sy(t)); g.lineTo(x + w, sy(t)); g.stroke() }
  for (const t of xt) { g.beginPath(); g.moveTo(sx(t), y); g.lineTo(sx(t), y + h); g.stroke() }

  g.fillStyle = FAINT
  g.font = mono(16)
  g.textAlign = 'right'
  g.textBaseline = 'alphabetic'
  for (const t of yt) g.fillText(fmt(t, yt), x - px(12), sy(t) + px(6))
  g.textAlign = 'center'
  for (const t of xt) g.fillText(fmt(t, xt), sx(t), y + h + px(32))

  g.strokeStyle = LINE
  g.lineWidth = px(1.8)
  g.beginPath()
  g.moveTo(x, y); g.lineTo(x, y + h); g.lineTo(x + w, y + h)
  g.stroke()

  g.fillStyle = FAINT
  g.font = cond(17, 500)
  g.fillText(xLabel, x + w / 2, y + h + px(66))
  g.save()
  g.translate(x - px(66), y + h / 2)
  g.rotate(-Math.PI / 2)
  g.fillText(yLabel, 0, 0)
  g.restore()
  g.textAlign = 'left'
  return { sx, sy, x, y, w, h }
}

// ---- Figure A: every dispersed ground track --------------------------------------------------
export function drawTracks(g) {
  const D = window.DISP
  g.fillStyle = PAPER
  g.fillRect(0, 0, W, W)

  let minN = Infinity; let maxN = -Infinity; let minE = Infinity; let maxE = -Infinity
  for (const tr of D.tracks) {
    for (const v of tr.n) { if (v < minN) minN = v; if (v > maxN) maxN = v }
    for (const v of tr.e) { if (v < minE) minE = v; if (v > maxE) maxE = v }
  }
  const size = Math.max(maxN - minN, maxE - minE) * 1.08
  const midN = (minN + maxN) / 2
  const midE = (minE + maxE) / 2

  const box = { x: px(130), y: px(105), w: px(745), h: px(745) }
  const p = axes(g, box,
    [midE - size / 2, midE + size / 2], [midN - size / 2, midN + size / 2],
    'EAST  [m]', 'NORTH  [m]', [5, 5])

  const draw = (tr, colour, width) => {
    g.strokeStyle = colour
    g.lineWidth = px(width)
    g.beginPath()
    for (let i = 0; i < tr.n.length; i += 1) {
      const X = p.sx(tr.e[i]); const Y = p.sy(tr.n[i])
      if (i === 0) g.moveTo(X, Y); else g.lineTo(X, Y)
    }
    g.stroke()
  }
  // The band the completed trials form is the message; the failures are picked out over it.
  for (const tr of D.tracks) if (tr.ok) draw(tr, 'rgba(42,120,214,0.30)', 1.0)
  for (const tr of D.tracks) {
    if (tr.ok) continue
    draw(tr, OXIDE, 1.9)
    const last = tr.n.length - 1
    g.beginPath(); g.arc(p.sx(tr.e[last]), p.sy(tr.n[last]), px(6), 0, Math.PI * 2)
    g.fillStyle = PAPER; g.fill()
    g.strokeStyle = OXIDE; g.lineWidth = px(2.2); g.stroke()
  }

  // Key, on the figure, naming the two things drawn.
  g.font = cond(17, 500)
  let kx = p.x
  const ky = p.y + p.h + px(110)
  for (const [colour, text, width] of [
    ['rgba(42,120,214,0.75)', `COMPLETED THE MISSION`, 2.6],
    [OXIDE, 'STOPPED BY A SAFETY LIMIT', 2.6],
  ]) {
    g.strokeStyle = colour
    g.lineWidth = px(width)
    g.beginPath(); g.moveTo(kx, ky); g.lineTo(kx + px(34), ky); g.stroke()
    g.fillStyle = FAINT
    g.fillText(text, kx + px(44), ky + px(6))
    kx += px(44) + g.measureText(text).width + px(52)
  }
}

// ---- Figure B: the modes of the linearised aircraft -------------------------------------------
export function drawModes(g) {
  const M = window.MODES
  g.fillStyle = PAPER
  g.fillRect(0, 0, W, W)

  const reals = M.map((m) => m.re)
  const ims = M.flatMap((m) => [m.im, -m.im])
  const rd = [Math.min(...reals) * 1.10, 2.4]
  const id = [Math.min(...ims) * 1.22, Math.max(...ims) * 1.22]

  const box = { x: px(130), y: px(105), w: px(745), h: px(745) }
  const p = axes(g, box, rd, id, 'REAL PART  [1/s]', 'IMAGINARY PART  [rad/s]', [6, 6])

  // Everything right of the imaginary axis grows rather than decays.
  g.fillStyle = 'rgba(168,53,27,0.07)'
  g.fillRect(p.sx(0), p.y, p.x + p.w - p.sx(0), p.h)
  g.strokeStyle = OXIDE
  g.lineWidth = px(1.6)
  g.beginPath(); g.moveTo(p.sx(0), p.y); g.lineTo(p.sx(0), p.y + p.h); g.stroke()

  // Offsets in figure units; the sign of dx is ignored — the side is chosen by position.
  const LABEL = {
    short_period: ['SHORT PERIOD', 28, -30],
    phugoid: ['PHUGOID', 30, -46],
    roll_subsidence: ['ROLL SUBSIDENCE', 26, -28],
    dutch_roll: ['DUTCH ROLL', 28, -30],
    spiral: ['SPIRAL', 30, 52],
  }

  for (const m of M) {
    const colour = m.stable ? LINE : OXIDE
    const points = m.im === 0 ? [0] : [m.im, -m.im]
    for (const im of points) {
      const X = p.sx(m.re)
      const Y = p.sy(im)
      g.strokeStyle = colour
      g.lineWidth = px(3)
      g.beginPath()
      g.moveTo(X - px(11), Y - px(11)); g.lineTo(X + px(11), Y + px(11))
      g.moveTo(X - px(11), Y + px(11)); g.lineTo(X + px(11), Y - px(11))
      g.stroke()
    }
    const [text, dx, dy] = LABEL[m.mode]
    const X = p.sx(m.re)
    const Y = p.sy(m.im === 0 ? 0 : m.im)
    // Anchor on the side with room: a label near the right edge is set right-aligned.
    const toLeft = X > p.x + p.w * 0.62
    const lx = toLeft ? X - px(Math.abs(dx)) : X + px(Math.abs(dx))
    g.textAlign = toLeft ? 'right' : 'left'
    g.fillStyle = colour
    g.font = cond(20, 600)
    g.fillText(text, lx, Y + px(dy))
    g.fillStyle = FAINT
    g.font = mono(15)
    const detail = m.im === 0
      ? `t½ ${Math.abs(0.693 / m.re).toFixed(2)} s`
      : `ωn ${m.wn.toFixed(2)}   ζ ${m.zeta.toFixed(3)}   T ${m.period.toFixed(2)} s`
    g.fillText(detail, lx, Y + px(dy) + px(25))
    g.textAlign = 'left'
  }

  // Say which half-plane is which — the one thing the axes alone do not carry.
  g.fillStyle = OXIDE
  g.font = cond(16, 500)
  g.textAlign = 'left'
  g.fillText('DIVERGENT', p.sx(0) + px(14), p.y + px(26))
  g.fillStyle = FAINT
  g.textAlign = 'right'
  g.fillText('CONVERGENT', p.sx(0) - px(14), p.y + px(26))
  g.textAlign = 'left'
}
