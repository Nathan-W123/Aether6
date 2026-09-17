/**
 * Series colours.
 *
 * One fixed hue order, used in the same sequence in every chart, so a colour means the same
 * thing wherever it appears on the page. The set is checked for deuteranopia, protanopia and
 * tritanopia separation against the dark surface these charts sit on, and is never cycled:
 * a panel that would need a ninth series is split into two panels instead.
 */
export const SERIES = [
  '#3987e5', // blue
  '#d95926', // orange
  '#199e70', // green
  '#c98500', // amber
  '#d55181', // pink
  '#5cc8c0', // teal
  '#9085e9', // violet
  '#e66767', // coral
] as const

/** Semantic colours that carry meaning rather than identity. */
export const SEMANTIC = {
  truth: '#3987e5',
  estimate: '#d95926',
  command: '#8fa0ba',
  lqr: '#3987e5',
  pid: '#d95926',
  good: '#199e70',
  warn: '#c98500',
  bad: '#e66767',
  band: 'rgba(57, 135, 229, 0.22)',
  bandEstimate: 'rgba(217, 89, 38, 0.22)',
  grid: 'rgba(143, 160, 186, 0.16)',
  axis: 'rgba(143, 160, 186, 0.45)',
} as const

/** The n-th series colour, without wrapping past the validated set. */
export function seriesColor(index: number): string {
  return SERIES[Math.min(index, SERIES.length - 1)]
}
