import { useMemo, useState } from 'react'

import { useMeasure } from '../../lib/useMeasure'
import { SEMANTIC } from '../../lib/palette'
import {
  bandPath,
  extent,
  formatTick,
  linePath,
  linearScale,
  nearestIndex,
  niceTicks,
  padDomain,
} from './plot'

export interface LineSeries {
  label: string
  x: number[]
  y: number[]
  color: string
  /** Dashed lines are for references — commands, truth overlays — not for data. */
  dashed?: boolean
  width?: number
  /** Keep the series out of the legend (e.g. the mirror half of a ±σ pair). */
  hideFromLegend?: boolean
}

export interface ChartBand {
  label?: string
  x: number[]
  lower: number[]
  upper: number[]
  fill: string
}

interface Props {
  series: LineSeries[]
  bands?: ChartBand[]
  xLabel: string
  yLabel: string
  height?: number
  /** Draw a rule at y = 0; use where zero is the target, not merely inside the range. */
  zeroLine?: boolean
  yDomain?: [number, number]
  /** Rendered in the top-right of the plot area, e.g. an RMS value. */
  annotation?: string
}

const MARGIN = { top: 10, right: 12, bottom: 26, left: 52 }

/**
 * A multi-series line chart with one y-axis.
 *
 * Deliberately one axis only: a second scale on the right invites false comparisons between
 * quantities in different units. Where two units must appear together, they get two panels.
 */
export function LineChart({
  series,
  bands = [],
  xLabel,
  yLabel,
  height = 190,
  zeroLine = false,
  yDomain,
  annotation,
}: Props) {
  const [ref, { width }] = useMeasure<HTMLDivElement>()
  const [hoverX, setHoverX] = useState<number | null>(null)

  const plotWidth = Math.max(0, width - MARGIN.left - MARGIN.right)
  const plotHeight = Math.max(0, height - MARGIN.top - MARGIN.bottom)

  const model = useMemo(() => {
    const xs = extent(series.map((s) => s.x).concat(bands.map((b) => b.x)))
    const ysRaw = extent(
      series
        .map((s) => s.y)
        .concat(bands.flatMap((b) => [b.lower, b.upper])),
    )
    if (!xs || !ysRaw) return null
    const ys = yDomain ?? padDomain(zeroLine ? [Math.min(ysRaw[0], 0), Math.max(ysRaw[1], 0)] : ysRaw)
    return {
      sx: linearScale(xs, [0, plotWidth]),
      sy: linearScale(ys, [plotHeight, 0]),
      xTicks: niceTicks(xs, Math.max(2, Math.round(plotWidth / 90))),
      yTicks: niceTicks(ys, Math.max(2, Math.round(plotHeight / 42))),
    }
  }, [series, bands, plotWidth, plotHeight, zeroLine, yDomain])

  const hover = useMemo(() => {
    if (hoverX === null || !model || series.length === 0) return null
    const value = model.sx.domain[0] + (hoverX / (plotWidth || 1)) *
      (model.sx.domain[1] - model.sx.domain[0])
    const readouts = series
      .filter((s) => !s.hideFromLegend)
      .map((s) => {
        const index = nearestIndex(s.x, value)
        return { label: s.label, color: s.color, value: index >= 0 ? s.y[index] : NaN }
      })
    return { x: value, readouts }
  }, [hoverX, model, series, plotWidth])

  const legend = series.filter((s) => !s.hideFromLegend)

  return (
    <div>
      {legend.length > 1 && (
        <div className="legend" style={{ marginBottom: 6 }}>
          {legend.map((s) => {
            const readout = hover?.readouts.find((r) => r.label === s.label)
            return (
              <span className="legend-item" key={s.label}>
                <span
                  className={`legend-swatch${s.dashed ? ' dashed' : ''}`}
                  style={{ background: s.dashed ? undefined : s.color, color: s.color }}
                />
                {s.label}
                {readout && Number.isFinite(readout.value) && (
                  <span className="mono" style={{ color: 'var(--text)' }}>
                    {formatReadout(readout.value)}
                  </span>
                )}
              </span>
            )
          })}
        </div>
      )}
      <div ref={ref} style={{ width: '100%' }}>
        {width > 0 && model && (
          <svg
            width={width}
            height={height}
            role="img"
            aria-label={`${yLabel} against ${xLabel}`}
            onPointerMove={(event) => {
              const box = event.currentTarget.getBoundingClientRect()
              const x = event.clientX - box.left - MARGIN.left
              setHoverX(x >= 0 && x <= plotWidth ? x : null)
            }}
            onPointerLeave={() => setHoverX(null)}
          >
            <g transform={`translate(${MARGIN.left},${MARGIN.top})`}>
              {model.yTicks.map((tick) => (
                <g key={`y${tick}`} transform={`translate(0,${model.sy(tick).toFixed(1)})`}>
                  <line x2={plotWidth} stroke={SEMANTIC.grid} />
                  <text x={-8} dy="0.32em" textAnchor="end" fontSize={10.5} fill="var(--text-dim)"
                        className="mono">
                    {formatTick(tick, model.yTicks)}
                  </text>
                </g>
              ))}
              {model.xTicks.map((tick) => (
                <g key={`x${tick}`} transform={`translate(${model.sx(tick).toFixed(1)},0)`}>
                  <line y2={plotHeight} stroke={SEMANTIC.grid} />
                  <text y={plotHeight + 15} textAnchor="middle" fontSize={10.5}
                        fill="var(--text-dim)" className="mono">
                    {formatTick(tick, model.xTicks)}
                  </text>
                </g>
              ))}

              {zeroLine && model.sy.domain[0] <= 0 && model.sy.domain[1] >= 0 && (
                <line y1={model.sy(0)} y2={model.sy(0)} x2={plotWidth} stroke={SEMANTIC.axis}
                      strokeDasharray="3 3" />
              )}

              {bands.map((band, i) => (
                <path key={`band${i}`} d={bandPath(band.x, band.lower, band.upper, model.sx, model.sy)}
                      fill={band.fill} stroke="none" />
              ))}

              {series.map((s) => (
                <path
                  key={s.label}
                  d={linePath(s.x, s.y, model.sx, model.sy)}
                  fill="none"
                  stroke={s.color}
                  strokeWidth={s.width ?? 1.4}
                  strokeDasharray={s.dashed ? '4 3' : undefined}
                  strokeLinejoin="round"
                  strokeLinecap="round"
                />
              ))}

              {hover && (
                <line x1={model.sx(hover.x)} x2={model.sx(hover.x)} y2={plotHeight}
                      stroke={SEMANTIC.axis} strokeWidth={1} pointerEvents="none" />
              )}

              {annotation && (
                <text x={plotWidth} y={2} dy="0.8em" textAnchor="end" fontSize={11}
                      fill="var(--text-muted)" className="mono">
                  {annotation}
                </text>
              )}

              <line y1={plotHeight} y2={plotHeight} x2={plotWidth} stroke={SEMANTIC.axis} />
              <line y2={plotHeight} stroke={SEMANTIC.axis} />
            </g>
            <text x={MARGIN.left + plotWidth / 2} y={height - 1} textAnchor="middle" fontSize={10.5}
                  fill="var(--text-dim)">
              {xLabel}
            </text>
            <text transform={`translate(11,${MARGIN.top + plotHeight / 2}) rotate(-90)`}
                  textAnchor="middle" fontSize={10.5} fill="var(--text-dim)">
              {yLabel}
            </text>
          </svg>
        )}
        {width === 0 && <div className="skeleton" style={{ height }} />}
      </div>
    </div>
  )
}

function formatReadout(value: number): string {
  const magnitude = Math.abs(value)
  if (magnitude >= 100) return value.toFixed(0)
  if (magnitude >= 10) return value.toFixed(1)
  if (magnitude >= 0.1) return value.toFixed(2)
  return value.toFixed(3)
}
