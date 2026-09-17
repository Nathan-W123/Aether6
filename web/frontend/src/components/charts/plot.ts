/** Scale, tick and path helpers shared by the SVG chart primitives. */

export interface Scale {
  (value: number): number
  domain: [number, number]
  range: [number, number]
}

export function linearScale(domain: [number, number], range: [number, number]): Scale {
  const [d0, d1] = domain
  const [r0, r1] = range
  const span = d1 - d0 || 1
  const scale = ((value: number) => r0 + ((value - d0) / span) * (r1 - r0)) as Scale
  scale.domain = domain
  scale.range = range
  return scale
}

/** Finite extent of several arrays, or `null` when there is nothing finite to plot. */
export function extent(arrays: (number[] | undefined)[]): [number, number] | null {
  let lo = Number.POSITIVE_INFINITY
  let hi = Number.NEGATIVE_INFINITY
  for (const array of arrays) {
    if (!array) continue
    for (const value of array) {
      if (!Number.isFinite(value)) continue
      if (value < lo) lo = value
      if (value > hi) hi = value
    }
  }
  return Number.isFinite(lo) && Number.isFinite(hi) ? [lo, hi] : null
}

/** Pad a domain by a fraction of its span, keeping a sensible width for a flat series. */
export function padDomain(
  domain: [number, number],
  fraction = 0.08,
  minimumSpan = 1e-6,
): [number, number] {
  const [lo, hi] = domain
  const span = hi - lo
  if (span < minimumSpan) {
    const pad = Math.max(Math.abs(lo) * 0.05, 0.5)
    return [lo - pad, hi + pad]
  }
  return [lo - span * fraction, hi + span * fraction]
}

/** Tick positions at 1/2/5 × 10^n, the usual readable set. */
export function niceTicks(domain: [number, number], target = 5): number[] {
  const [lo, hi] = domain
  const span = hi - lo
  if (!Number.isFinite(span) || span <= 0) return [lo]
  const rough = span / Math.max(1, target)
  const magnitude = 10 ** Math.floor(Math.log10(rough))
  const normalised = rough / magnitude
  const step = (normalised >= 5 ? 10 : normalised >= 2 ? 5 : normalised >= 1 ? 2 : 1) * magnitude
  const first = Math.ceil(lo / step) * step
  const ticks: number[] = []
  for (let value = first; value <= hi + step * 1e-6; value += step) {
    ticks.push(Math.abs(value) < step * 1e-6 ? 0 : value)
  }
  return ticks
}

/** Format a tick so a whole axis shares one sensible precision. */
export function formatTick(value: number, ticks: number[]): string {
  const step = ticks.length > 1 ? Math.abs(ticks[1] - ticks[0]) : Math.abs(value) || 1
  if (step >= 1000) return `${Math.round(value / 1000)}k`
  const decimals = step >= 10 ? 0 : step >= 1 ? 0 : step >= 0.1 ? 1 : step >= 0.01 ? 2 : 3
  return value.toFixed(decimals)
}

/** SVG path for a polyline, breaking the line wherever a sample is not finite. */
export function linePath(x: number[], y: number[], sx: Scale, sy: Scale): string {
  const parts: string[] = []
  let pen = false
  const n = Math.min(x.length, y.length)
  for (let i = 0; i < n; i += 1) {
    if (!Number.isFinite(x[i]) || !Number.isFinite(y[i])) {
      pen = false
      continue
    }
    const px = sx(x[i]).toFixed(1)
    const py = sy(y[i]).toFixed(1)
    parts.push(`${pen ? 'L' : 'M'}${px} ${py}`)
    pen = true
  }
  return parts.join('')
}

/** Closed SVG path for a band between two y series. */
export function bandPath(
  x: number[],
  lower: number[],
  upper: number[],
  sx: Scale,
  sy: Scale,
): string {
  const n = Math.min(x.length, lower.length, upper.length)
  if (n === 0) return ''
  const top: string[] = []
  const bottom: string[] = []
  for (let i = 0; i < n; i += 1) {
    if (!Number.isFinite(x[i]) || !Number.isFinite(upper[i]) || !Number.isFinite(lower[i])) continue
    top.push(`${sx(x[i]).toFixed(1)} ${sy(upper[i]).toFixed(1)}`)
    bottom.push(`${sx(x[i]).toFixed(1)} ${sy(lower[i]).toFixed(1)}`)
  }
  if (top.length === 0) return ''
  bottom.reverse()
  return `M${top.join('L')}L${bottom.join('L')}Z`
}

/** Index of the sample nearest to `value` in a monotonically increasing array. */
export function nearestIndex(values: number[], value: number): number {
  if (values.length === 0) return -1
  let lo = 0
  let hi = values.length - 1
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1
    if (values[mid] <= value) lo = mid
    else hi = mid
  }
  return Math.abs(values[lo] - value) <= Math.abs(values[hi] - value) ? lo : hi
}
