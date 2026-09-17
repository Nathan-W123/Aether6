import { useMemo, useState } from 'react'

import { useMeasure } from '../../lib/useMeasure'
import { SEMANTIC } from '../../lib/palette'
import {
  bandPath, extent, formatTick, linePath, linearScale, nearestIndex, niceTicks, padDomain,
} from './plot'

export interface LineSeries {
  label: string
  x: number[]
  y: number[]
  color: string
  /** Dashed marks a reference — a command, a limit — not a measured quantity. */
  dashed?: boolean
  width?: number
  hideFromLegend?: boolean
}

export interface ChartBand {
  x: number[]
  lower: number[]
  upper: number[]
  fill: string
}

interface Props {
  series: LineSeries[]
  bands?: ChartBand[]
  /** Axis quantity and unit, e.g. "altitude" and "m". Printed separately, as on a plot. */
  y: string
  yUnit: string
  x?: string
  xUnit?: string
  height?: number
  /** A rule at zero, where zero is the target rather than merely inside the range. */
  zeroLine?: boolean
  yDomain?: [number, number]
  /** A horizontal limit line drawn in oxide, labelled. */
  limit?: { value: number; label: string }
}

const MARGIN = { top: 12, right: 10, bottom: 30, left: 54 }

/**
 * A time-history plot.
 *
 * One y-axis, always: a second scale on the right invites a comparison between quantities in
 * different units that the data does not support. Where two units must be read together they
 * get two figures. The hover crosshair reports every series at the sampled instant, so a
 * reader takes values off the plot rather than estimating them against the grid.
 */
export function LineChart({
  series, bands = [], y, yUnit, x = 'time', xUnit = 's', height = 176,
  zeroLine = false, yDomain, limit,
}: Props) {
  const [ref, { width }] = useMeasure<HTMLDivElement>()
  const [hoverX, setHoverX] = useState<number | null>(null)

  const plotWidth = Math.max(0, width - MARGIN.left - MARGIN.right)
  const plotHeight = Math.max(0, height - MARGIN.top - MARGIN.bottom)

  const model = useMemo(() => {
    const xs = extent(series.map((s) => s.x).concat(bands.map((b) => b.x)))
    const raw = extent(
      series.map((s) => s.y)
        .concat(bands.flatMap((b) => [b.lower, b.upper]))
        .concat(limit ? [[limit.value]] : []))
    if (!xs || !raw) return null
    const ys = yDomain
      ?? padDomain(zeroLine ? [Math.min(raw[0], 0), Math.max(raw[1], 0)] : raw)
    return {
      sx: linearScale(xs, [0, plotWidth]),
      sy: linearScale(ys, [plotHeight, 0]),
      xTicks: niceTicks(xs, Math.max(2, Math.round(plotWidth / 86))),
      yTicks: niceTicks(ys, Math.max(2, Math.round(plotHeight / 38))),
    }
  }, [series, bands, plotWidth, plotHeight, zeroLine, yDomain, limit])

  const readouts = useMemo(() => {
    if (hoverX === null || !model) return null
    const at = model.sx.domain[0]
      + (hoverX / (plotWidth || 1)) * (model.sx.domain[1] - model.sx.domain[0])
    return {
      at,
      values: series.filter((s) => !s.hideFromLegend).map((s) => {
        const i = nearestIndex(s.x, at)
        return { label: s.label, value: i >= 0 ? s.y[i] : NaN }
      }),
    }
  }, [hoverX, model, series, plotWidth])

  const legend = series.filter((s) => !s.hideFromLegend)

  return (
    <div>
      {legend.length > 1 && (
        <div className="legend">
          {legend.map((s) => {
            const readout = readouts?.values.find((r) => r.label === s.label)
            return (
              <span className="legend-item" key={s.label}>
                <span className="legend-key" style={{
                  borderTopColor: s.color,
                  borderTopStyle: s.dashed ? 'dashed' : 'solid',
                }} />
                {s.label}
                {readout && Number.isFinite(readout.value) && (
                  <span className="legend-value">{compact(readout.value)}</span>
                )}
              </span>
            )
          })}
        </div>
      )}

      <div ref={ref} style={{ width: '100%' }}>
        {width > 0 && model ? (
          <svg
            width={width} height={height} role="img"
            aria-label={`${y} in ${yUnit} against ${x} in ${xUnit}`}
            onPointerMove={(event) => {
              const box = event.currentTarget.getBoundingClientRect()
              const px = event.clientX - box.left - MARGIN.left
              setHoverX(px >= 0 && px <= plotWidth ? px : null)
            }}
            onPointerLeave={() => setHoverX(null)}
          >
            <g transform={`translate(${MARGIN.left},${MARGIN.top})`}>
              {model.yTicks.map((tick) => (
                <g key={`y${tick}`} transform={`translate(0,${model.sy(tick).toFixed(1)})`}>
                  <line x2={plotWidth} stroke={SEMANTIC.grid} strokeWidth={1} />
                  <line x1={-4} x2={0} stroke={SEMANTIC.axis} strokeWidth={1} />
                  <text x={-8} dy="0.32em" textAnchor="end" fontSize={10}
                        fill="var(--ink-mute)" fontFamily="var(--mono)">
                    {formatTick(tick, model.yTicks)}
                  </text>
                </g>
              ))}
              {model.xTicks.map((tick) => (
                <g key={`x${tick}`} transform={`translate(${model.sx(tick).toFixed(1)},0)`}>
                  <line y2={plotHeight} stroke={SEMANTIC.grid} strokeWidth={1} />
                  <line y1={plotHeight} y2={plotHeight + 4} stroke={SEMANTIC.axis} strokeWidth={1} />
                  <text y={plotHeight + 15} textAnchor="middle" fontSize={10}
                        fill="var(--ink-mute)" fontFamily="var(--mono)">
                    {formatTick(tick, model.xTicks)}
                  </text>
                </g>
              ))}

              {zeroLine && model.sy.domain[0] <= 0 && model.sy.domain[1] >= 0 && (
                <line y1={model.sy(0)} y2={model.sy(0)} x2={plotWidth}
                      stroke={SEMANTIC.axis} strokeWidth={1} />
              )}

              {bands.map((band, i) => (
                <path key={`b${i}`} fill={band.fill} stroke="none"
                      d={bandPath(band.x, band.lower, band.upper, model.sx, model.sy)} />
              ))}

              {series.map((s) => (
                <path
                  key={s.label}
                  d={linePath(s.x, s.y, model.sx, model.sy)}
                  fill="none"
                  stroke={s.color}
                  strokeWidth={s.width ?? 1.5}
                  strokeDasharray={s.dashed ? '5 3' : undefined}
                  strokeLinejoin="round"
                  strokeLinecap="round"
                />
              ))}

              {limit && (
                <g>
                  <line y1={model.sy(limit.value)} y2={model.sy(limit.value)} x2={plotWidth}
                        stroke="var(--oxide)" strokeWidth={1} strokeDasharray="6 3" />
                  <text x={plotWidth - 3} y={model.sy(limit.value) - 4} textAnchor="end"
                        fontSize={10} fill="var(--oxide)" fontFamily="var(--mono)">
                    {limit.label}
                  </text>
                </g>
              )}

              {readouts && (
                <line x1={model.sx(readouts.at)} x2={model.sx(readouts.at)} y2={plotHeight}
                      stroke="var(--ink)" strokeWidth={1} opacity={0.5} pointerEvents="none" />
              )}

              {/* Axes drawn last so marks never sit on top of the frame. */}
              <line y1={plotHeight} y2={plotHeight} x2={plotWidth}
                    stroke={SEMANTIC.axis} strokeWidth={1} />
              <line y2={plotHeight} stroke={SEMANTIC.axis} strokeWidth={1} />
            </g>

            <text x={MARGIN.left} y={height - 2} fontSize={10} fill="var(--ink-faint)"
                  fontFamily="var(--sans)" letterSpacing="0.08em">
              {x.toUpperCase()} [{xUnit}]
            </text>
            <text transform={`translate(11,${MARGIN.top + plotHeight / 2}) rotate(-90)`}
                  textAnchor="middle" fontSize={10} fill="var(--ink-faint)"
                  fontFamily="var(--sans)" letterSpacing="0.08em">
              {y.toUpperCase()} [{yUnit}]
            </text>
          </svg>
        ) : (
          <div className="placeholder" style={{ height }} />
        )}
      </div>
    </div>
  )
}

function compact(value: number): string {
  const m = Math.abs(value)
  if (m >= 100) return value.toFixed(0)
  if (m >= 10) return value.toFixed(1)
  if (m >= 0.1) return value.toFixed(2)
  return value.toFixed(3)
}
