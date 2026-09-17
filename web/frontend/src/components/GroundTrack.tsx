import { useMemo } from 'react'

import { useMeasure } from '../lib/useMeasure'
import { SEMANTIC } from '../lib/palette'
import { extent, formatTick, linearScale, niceTicks } from './charts/plot'
import type { Series, Waypoint } from '../api/types'

interface Props {
  series: Series
  waypoints: Waypoint[]
  showEstimate: boolean
  height?: number
}

const MARGIN = { top: 12, right: 14, bottom: 30, left: 54 }

/**
 * Plan view of the mission: north up, east right, equal scale on both axes.
 *
 * Equal scale matters here — a stretched aspect ratio would make the turns look tighter or
 * wider than they are, which is precisely the thing a reader judges from this panel.
 */
export function GroundTrack({ series, waypoints, showEstimate, height = 360 }: Props) {
  const [ref, { width }] = useMeasure<HTMLDivElement>()
  const plotWidth = Math.max(0, width - MARGIN.left - MARGIN.right)
  const plotHeight = Math.max(0, height - MARGIN.top - MARGIN.bottom)

  const model = useMemo(() => {
    const east = series.east ?? []
    const north = series.north ?? []
    if (east.length === 0) return null

    const ex = extent([east, waypoints.map((w) => w.east), series.est_east])
    const nx = extent([north, waypoints.map((w) => w.north), series.est_north])
    if (!ex || !nx) return null

    // One metres-per-pixel for both axes, centred on the data.
    const padding = 0.06
    const spanE = (ex[1] - ex[0]) * (1 + padding * 2) || 1
    const spanN = (nx[1] - nx[0]) * (1 + padding * 2) || 1
    const metresPerPixel = Math.max(spanE / (plotWidth || 1), spanN / (plotHeight || 1))
    const halfE = (metresPerPixel * plotWidth) / 2
    const halfN = (metresPerPixel * plotHeight) / 2
    const midE = (ex[0] + ex[1]) / 2
    const midN = (nx[0] + nx[1]) / 2

    const sx = linearScale([midE - halfE, midE + halfE], [0, plotWidth])
    const sy = linearScale([midN - halfN, midN + halfN], [plotHeight, 0])

    const path = (e: number[], n: number[]) => {
      const count = Math.min(e.length, n.length)
      const parts: string[] = []
      for (let i = 0; i < count; i += 1) {
        parts.push(`${i === 0 ? 'M' : 'L'}${sx(e[i]).toFixed(1)} ${sy(n[i]).toFixed(1)}`)
      }
      return parts.join('')
    }

    const legs = waypoints.length > 1
      ? `${waypoints
          .map((w, i) => `${i === 0 ? 'M' : 'L'}${sx(w.east).toFixed(1)} ${sy(w.north).toFixed(1)}`)
          .join('')}Z`
      : ''

    return {
      sx, sy, legs, metresPerPixel,
      flown: path(east, north),
      estimated: showEstimate && series.est_east && series.est_north
        ? path(series.est_east, series.est_north) : null,
      xTicks: niceTicks(sx.domain, Math.max(2, Math.round(plotWidth / 90))),
      yTicks: niceTicks(sy.domain, Math.max(2, Math.round(plotHeight / 60))),
    }
  }, [series, waypoints, plotWidth, plotHeight, showEstimate])

  return (
    <div>
      <div className="legend" style={{ marginBottom: 6 }}>
        <span className="legend-item">
          <span className="legend-swatch" style={{ background: SEMANTIC.truth }} /> flown
        </span>
        {showEstimate && (
          <span className="legend-item">
            <span className="legend-swatch dashed" style={{ color: SEMANTIC.estimate }} />
            EKF estimate
          </span>
        )}
        <span className="legend-item">
          <span className="legend-swatch dashed" style={{ color: '#4db6ac' }} /> commanded legs
        </span>
      </div>
      <div ref={ref} style={{ width: '100%' }}>
        {width > 0 && model ? (
          <svg width={width} height={height} role="img"
               aria-label="Ground track of the flown mission against the commanded waypoint legs">
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

              <path d={model.legs} fill="none" stroke="#4db6ac" strokeWidth={1.2}
                    strokeDasharray="6 5" opacity={0.8} />
              {model.estimated && (
                <path d={model.estimated} fill="none" stroke={SEMANTIC.estimate} strokeWidth={1.1}
                      strokeDasharray="4 3" opacity={0.85} />
              )}
              <path d={model.flown} fill="none" stroke={SEMANTIC.truth} strokeWidth={1.6}
                    strokeLinejoin="round" />

              {waypoints.map((waypoint) => (
                <g key={waypoint.index}
                   transform={`translate(${model.sx(waypoint.east).toFixed(1)},${model.sy(waypoint.north).toFixed(1)})`}>
                  <circle r={4.5} fill="#4db6ac" />
                  <text x={8} dy="0.32em" fontSize={10.5} fill="var(--text-muted)" className="mono">
                    {waypoint.index}
                  </text>
                </g>
              ))}

              <line y1={plotHeight} y2={plotHeight} x2={plotWidth} stroke={SEMANTIC.axis} />
              <line y2={plotHeight} stroke={SEMANTIC.axis} />
            </g>
            <text x={MARGIN.left + plotWidth / 2} y={height - 3} textAnchor="middle" fontSize={10.5}
                  fill="var(--text-dim)">
              east [m]
            </text>
            <text transform={`translate(11,${MARGIN.top + plotHeight / 2}) rotate(-90)`}
                  textAnchor="middle" fontSize={10.5} fill="var(--text-dim)">
              north [m]
            </text>
          </svg>
        ) : (
          <div className="skeleton" style={{ height }} />
        )}
      </div>
    </div>
  )
}
