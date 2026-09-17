import { useMemo } from 'react'

import { useMeasure } from '../../lib/useMeasure'
import { SEMANTIC } from '../../lib/palette'
import { formatTick, linearScale, niceTicks } from './plot'

interface Props {
  values: number[]
  xLabel: string
  color: string
  height?: number
  bins?: number
  /** Vertical rules to mark, e.g. the median and the 95th percentile. */
  markers?: { value: number; label: string }[]
}

const MARGIN = { top: 10, right: 12, bottom: 30, left: 44 }

/** Distribution of one Monte-Carlo outcome metric. */
export function Histogram({ values, xLabel, color, height = 200, bins = 22, markers = [] }: Props) {
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
      const index = Math.min(bins - 1, Math.floor(((value - lo) / span) * bins))
      counts[index] += 1
    }
    const peak = Math.max(...counts)
    return {
      counts,
      lo,
      hi,
      sx: linearScale([lo, hi], [0, plotWidth]),
      sy: linearScale([0, peak], [plotHeight, 0]),
      xTicks: niceTicks([lo, hi], Math.max(2, Math.round(plotWidth / 80))),
      yTicks: niceTicks([0, peak], 4),
    }
  }, [values, bins, plotWidth, plotHeight])

  return (
    <div ref={ref} style={{ width: '100%' }}>
      {width > 0 && model ? (
        <svg width={width} height={height} role="img" aria-label={`distribution of ${xLabel}`}>
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
            {model.counts.map((count, i) => {
              const barWidth = plotWidth / model.counts.length
              return (
                <rect
                  key={i}
                  x={i * barWidth + 0.5}
                  y={model.sy(count)}
                  width={Math.max(0.5, barWidth - 1)}
                  height={Math.max(0, plotHeight - model.sy(count))}
                  fill={color}
                  fillOpacity={0.75}
                />
              )
            })}
            {markers.map((marker) => (
              <g key={marker.label}>
                <line x1={model.sx(marker.value)} x2={model.sx(marker.value)} y2={plotHeight}
                      stroke="var(--text)" strokeWidth={1} strokeDasharray="4 3" />
                <text x={model.sx(marker.value)} y={-1} textAnchor="middle" fontSize={10}
                      fill="var(--text-muted)" className="mono">
                  {marker.label}
                </text>
              </g>
            ))}
            {model.xTicks.map((tick) => (
              <text key={`x${tick}`} x={model.sx(tick)} y={plotHeight + 15} textAnchor="middle"
                    fontSize={10.5} fill="var(--text-dim)" className="mono">
                {formatTick(tick, model.xTicks)}
              </text>
            ))}
            <line y1={plotHeight} y2={plotHeight} x2={plotWidth} stroke={SEMANTIC.axis} />
            <line y2={plotHeight} stroke={SEMANTIC.axis} />
          </g>
          <text x={MARGIN.left + plotWidth / 2} y={height - 3} textAnchor="middle" fontSize={10.5}
                fill="var(--text-dim)">
            {xLabel}
          </text>
          <text transform={`translate(11,${MARGIN.top + plotHeight / 2}) rotate(-90)`}
                textAnchor="middle" fontSize={10.5} fill="var(--text-dim)">
            trials
          </text>
        </svg>
      ) : (
        <div className="skeleton" style={{ height }} />
      )}
    </div>
  )
}
