import { useMemo } from 'react'

import { useMeasure } from '../../lib/useMeasure'
import { SEMANTIC } from '../../lib/palette'
import { formatTick, linearScale, niceTicks } from './plot'

interface Props {
  values: number[]
  x: string
  xUnit: string
  color: string
  height?: number
  bins?: number
  /** Percentile rules, labelled above the bars. */
  markers?: { value: number; label: string }[]
}

const MARGIN = { top: 18, right: 12, bottom: 32, left: 44 }

/** The distribution of one outcome metric across a campaign. */
export function Histogram({ values, x, xUnit, color, height = 210, bins = 24, markers = [] }: Props) {
  const [ref, { width }] = useMeasure<HTMLDivElement>()
  const plotWidth = Math.max(0, width - MARGIN.left - MARGIN.right)
  const plotHeight = Math.max(0, height - MARGIN.top - MARGIN.bottom)

  const model = useMemo(() => {
    const finite = values.filter((v) => Number.isFinite(v))
    if (finite.length === 0) return null
    const lo = Math.min(...finite)
    const hi = Math.max(...finite)
    const span = hi - lo || 1
    const counts = new Array(bins).fill(0)
    for (const value of finite) {
      counts[Math.min(bins - 1, Math.floor(((value - lo) / span) * bins))] += 1
    }
    const peak = Math.max(...counts)
    return {
      counts, lo, hi,
      sx: linearScale([lo, hi], [0, plotWidth]),
      sy: linearScale([0, peak], [plotHeight, 0]),
      xTicks: niceTicks([lo, hi], Math.max(2, Math.round(plotWidth / 80))),
      yTicks: niceTicks([0, peak], 4),
    }
  }, [values, bins, plotWidth, plotHeight])

  return (
    <div ref={ref} style={{ width: '100%' }}>
      {width > 0 && model ? (
        <svg width={width} height={height} role="img"
             aria-label={`distribution of ${x} in ${xUnit} across the campaign`}>
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

            {model.counts.map((count, i) => {
              const barWidth = plotWidth / model.counts.length
              return (
                <rect
                  key={i}
                  x={i * barWidth + 1}
                  y={model.sy(count)}
                  width={Math.max(0.5, barWidth - 2)}
                  height={Math.max(0, plotHeight - model.sy(count))}
                  fill={color}
                  fillOpacity={0.82}
                />
              )
            })}

            {markers.map((marker) => (
              <g key={marker.label}>
                <line x1={model.sx(marker.value)} x2={model.sx(marker.value)}
                      y1={-6} y2={plotHeight} stroke="var(--ink)" strokeWidth={1}
                      strokeDasharray="4 3" />
                <text x={model.sx(marker.value)} y={-9} textAnchor="middle" fontSize={10}
                      fill="var(--ink)" fontFamily="var(--mono)">
                  {marker.label}
                </text>
              </g>
            ))}

            {model.xTicks.map((tick) => (
              <g key={`x${tick}`} transform={`translate(${model.sx(tick).toFixed(1)},0)`}>
                <line y1={plotHeight} y2={plotHeight + 4} stroke={SEMANTIC.axis} strokeWidth={1} />
                <text y={plotHeight + 15} textAnchor="middle" fontSize={10}
                      fill="var(--ink-mute)" fontFamily="var(--mono)">
                  {formatTick(tick, model.xTicks)}
                </text>
              </g>
            ))}

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
            TRIALS
          </text>
        </svg>
      ) : (
        <div className="placeholder" style={{ height }} />
      )}
    </div>
  )
}
