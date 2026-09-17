/**
 * Plotting colours.
 *
 * Every value is a CSS custom property rather than a literal, so a series keeps its identity
 * when the sheet switches between the light and dark rendering — the two sets are separately
 * stepped for their own surface, not one flipped into the other.
 *
 * The eight-hue order is fixed and never cycled. It was validated against both plotting
 * surfaces for the lightness band, the chroma floor, adjacent-pair separation under
 * deuteranopia, protanopia and tritanopia, and normal-vision separation. Three of the light
 * steps sit below 3:1 against the sheet, so every figure with two or more series carries a
 * legend naming each one: identity is never carried by colour alone.
 *
 * Forms that compare every pair at once rather than adjacent pairs — the dispersion scatter —
 * are capped at the first three slots, which are the ones that clear the all-pairs floors.
 */
export const SERIES = [
  'var(--series-1)', // blue
  'var(--series-2)', // orange
  'var(--series-3)', // aqua
  'var(--series-4)', // yellow
  'var(--series-5)', // magenta
  'var(--series-6)', // green
  'var(--series-7)', // violet
  'var(--series-8)', // red
] as const

/** The n-th plotting colour. Never wraps: a ninth series folds into a second figure. */
export function seriesColor(index: number): string {
  return SERIES[Math.min(index, SERIES.length - 1)]
}

/** Colours that carry a meaning rather than an identity. */
export const SEMANTIC = {
  truth: 'var(--series-1)',
  estimate: 'var(--series-2)',
  lqr: 'var(--series-1)',
  pid: 'var(--series-2)',
  /** Commands and references are drawn in ink, dashed — they are not a measured quantity. */
  reference: 'var(--ink-mute)',
  /** Reserved for a limit exceeded or a run cut short. Never used as a series. */
  limit: 'var(--oxide)',
  band: 'var(--series-1-band)',
  bandAlt: 'var(--series-2-band)',
  grid: 'var(--grid)',
  axis: 'var(--rule-strong)',
} as const
