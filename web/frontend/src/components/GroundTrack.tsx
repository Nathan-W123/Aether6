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

const MARGIN = { top: 14, right: 14, bottom: 32, left: 56 }

/**
 * Plan view of the mission: north up, east right, equal scale on both axes.
 *
 * Equal scale is the point — a stretched aspect would make the turns look tighter or wider
 * than they were, which is exactly what a reader judges from this figure. A scale bar states
 * the metres per division so distances can be taken off the plot directly.
 */
export function GroundTrack({ series, waypoints, showEstimate, height = 392 }: Props) {
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

    const padding = 0.07
    const spanE = (ex[1] - ex[0]) * (1 + padding * 2) || 1
    const spanN = (nx[1] - nx[0]) * (1 + padding * 2) || 1
    const metresPerPixel = Math.max(spanE / (plotWidth || 1), spanN / (plotHeight || 1))
    const midE = (ex[0] + ex[1]) / 2
    const midN = (nx[0] + nx[1]) / 2
    const halfE = (metresPerPixel * plotWidth) / 2
    const halfN = (metresPerPixel * plotHeight) / 2

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

    // A round number of metres, no wider than a quarter of the field.
    const target = metresPerPixel * plotWidth * 0.25
    const magnitude = 10 ** Math.floor(Math.log10(target))
    const normalised = target / magnitude
    const barMetres = (normalised >= 5 ? 5 : normalised >= 2 ? 2 : 1) * magnitude

    return {
      sx, sy, legs, barMetres,
      barPixels: barMetres / metresPerPixel,
      flown: path(east, north),
      estimated: showEstimate && series.est_east && series.est_north
        ? path(series.est_east, series.est_north) : null,
      xTicks: niceTicks(sx.domain, Math.max(2, Math.round(plotWidth / 90))),
      yTicks: niceTicks(sy.domain, Math.max(2, Math.round(plotHeight / 56))),
    }
  }, [series, waypoints, plotWidth, plotHeight, showEstimate])

  return (
    <div>
      <div className="legend">
        <span className="legend-item">
          <span className="legend-key" style={{ borderTopColor: 'var(--series-1)' }} /> flown
        </span>
        {showEstimate && (
          <span className="legend-item">
            <span className="legend-key" style={{
              borderTopColor: 'var(--series-2)', borderTopStyle: 'dashed' }} />
            navigation estimate
          </span>
        )}
        <span className="legend-item">
          <span className="legend-key" style={{
            borderTopColor: 'var(--ink-mute)', borderTopStyle: 'dashed' }} />
          commanded legs
        </span>
      </div>

      <div ref={ref} style={{ width: '100%' }}>
        {width > 0 && model ? (
          <svg width={width} height={height} role="img"
               aria-label="Plan view of the flown ground track against the commanded legs">
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

              <path d={model.legs} fill="none" stroke="var(--ink-mute)" strokeWidth={1}
                    strokeDasharray="7 5" />
              {model.estimated && (
                <path d={model.estimated} fill="none" stroke="var(--series-2)" strokeWidth={1.2}
                      strokeDasharray="5 3" />
              )}
              <path d={model.flown} fill="none" stroke="var(--series-1)" strokeWidth={1.7}
                    strokeLinejoin="round" />

              {waypoints.map((waypoint) => (
                <g key={waypoint.index}
                   transform={`translate(${model.sx(waypoint.east).toFixed(1)},${model.sy(waypoint.north).toFixed(1)})`}>
                  <circle r={3.5} fill="var(--field)" stroke="var(--ink)" strokeWidth={1.4} />
                  <text x={8} dy="0.32em" fontSize={10.5} fill="var(--ink)"
                        fontFamily="var(--mono)">
                    {waypoint.index}
                  </text>
                </g>
              ))}

              {/* Scale bar, as on a chart. */}
              <g transform={`translate(6,${plotHeight - 10})`}>
                <line x2={model.barPixels} stroke="var(--ink)" strokeWidth={1.5} />
                <line y1={-3} y2={3} stroke="var(--ink)" strokeWidth={1.5} />
                <line x1={model.barPixels} x2={model.barPixels} y1={-3} y2={3}
                      stroke="var(--ink)" strokeWidth={1.5} />
                <text x={model.barPixels / 2} y={-6} textAnchor="middle" fontSize={10}
                      fill="var(--ink)" fontFamily="var(--mono)">
                  {model.barMetres >= 1000
                    ? `${(model.barMetres / 1000).toFixed(1)} km`
                    : `${model.barMetres.toFixed(0)} m`}
                </text>
              </g>

              <line y1={plotHeight} y2={plotHeight} x2={plotWidth}
                    stroke={SEMANTIC.axis} strokeWidth={1} />
              <line y2={plotHeight} stroke={SEMANTIC.axis} strokeWidth={1} />
            </g>
            <text x={MARGIN.left} y={height - 2} fontSize={10} fill="var(--ink-faint)"
                  fontFamily="var(--sans)" letterSpacing="0.08em">EAST [m]</text>
            <text transform={`translate(11,${MARGIN.top + plotHeight / 2}) rotate(-90)`}
                  textAnchor="middle" fontSize={10} fill="var(--ink-faint)"
                  fontFamily="var(--sans)" letterSpacing="0.08em">NORTH [m]</text>
          </svg>
        ) : (
          <div className="placeholder" style={{ height }} />
        )}
      </div>
    </div>
  )
}
