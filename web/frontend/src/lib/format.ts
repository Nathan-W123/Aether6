/** Number formatting used across the tiles, tables and readouts. */

export function fixed(value: number | undefined, decimals = 2): string {
  if (value === undefined || !Number.isFinite(value)) return '—'
  return value.toFixed(decimals)
}

/** Compact scientific notation for very small or very large magnitudes. */
export function science(value: number | undefined, decimals = 2): string {
  if (value === undefined || !Number.isFinite(value)) return '—'
  if (value === 0) return '0'
  const magnitude = Math.abs(value)
  if (magnitude >= 1e-3 && magnitude < 1e5) return value.toFixed(decimals)
  const exponent = Math.floor(Math.log10(magnitude))
  const mantissa = value / 10 ** exponent
  return `${mantissa.toFixed(1)}e${exponent}`
}

export function seconds(value: number | undefined): string {
  if (value === undefined || !Number.isFinite(value)) return '—'
  return value < 10 ? `${value.toFixed(2)} s` : `${value.toFixed(1)} s`
}

/** "ground_contact" -> "Ground contact" */
export function humanise(text: string): string {
  const spaced = text.replace(/_/g, ' ').trim()
  return spaced.charAt(0).toUpperCase() + spaced.slice(1)
}
