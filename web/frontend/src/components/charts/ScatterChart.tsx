import { useMemo } from 'react'

import { useMeasure } from '../../lib/useMeasure'
import { SEMANTIC } from '../../lib/palette'
import { extent, formatTick, linearScale, niceTicks, padDomain } from './plot'

export interface ScatterPoint {
  x: number
  y: number
  /** Failed trials are drawn as open rings so an outcome is never inferred from colour alone. */
  failed?: boolean
  title?: string
}

interface Props {
  points: ScatterPoint[]
  xLabel: string
  yLabel: string
  color: string
  height?: number
  /** Least-squares trend line through the successful points. */
  trend?: boolean
}

const MARGIN = { top: 10, right: 12, bottom: 30, left: 52 }

/** Scatter of one dispersed input against one outcome, for the Monte-Carlo panel. */
export function ScatterChart({ points, xLabel, yLabel, color, height = 200, trend = false }: Props) {
  const [ref, { width }] = useMeasure<HTMLDivElement>()
  const plotWidth = Math.max(0, width - MARGIN.left - MARGIN.right)
  const plotHeight = Math.max(0, height - MARGIN.top - MARGIN.bottom)

  const model = useMemo(() => {
    const xs = extent([points.map((p) => p.x)])
    const ys = extent([points.map((p) => p.y)])
    if (!xs || !ys) return null
    const xd = padDomain(xs, 0.05)
    const yd = padDomain(ys, 0.08)
    const sx = linearScale(xd, [0, plotWidth])
    const sy = linearScale(yd, [plotHeight, 0])

    let fit: { x0: number; y0: number; x1: number; y1: number } | null = null
    if (trend) {
      const ok = points.filter((p) => !p.failed)
      const n = ok.length
      if (n >= 3) {
        const mx = ok.reduce((a, p) => a + p.x, 0) / n
        const my = ok.reduce((a, p) => a + p.y, 0) / n
        const sxx = ok.reduce((a, p) => a + (p.x - mx) ** 2, 0)
        const sxy = ok.reduce((a, p) => a + (p.x - mx) * (p.y - my), 0)
        if (sxx > 0) {
          const slope = sxy / sxx
          fit = {
            x0: xd[0], y0: my + slope * (xd[0] - mx),
            x1: xd[1], y1: my + slope * (xd[1] - mx),
          }
        }
      }
    }
    return {
      sx, sy, fit,
      xTicks: niceTicks(xd, Math.max(2, Math.round(plotWidth / 90))),
      yTicks: niceTicks(yd, Math.max(2, Math.round(plotHeight / 42))),
    }
  }, [points, plotWidth, plotHeight, trend])

  return (
    <div ref={ref} style={{ width: '100%' }}>
      {width > 0 && model ? (
        <svg width={width} height={height} role="img" aria-label={`${yLabel} against ${xLabel}`}>
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
                <text y={plotHeight + 15} textAnchor="middle" fontSize={10.5} fill="var(--text-dim)"
                      className="mono">
                  {formatTick(tick, model.xTicks)}
                </text>
              </g>
            ))}

            {model.fit && (
              <line
                x1={model.sx(model.fit.x0)} y1={model.sy(model.fit.y0)}
                x2={model.sx(model.fit.x1)} y2={model.sy(model.fit.y1)}
                stroke={SEMANTIC.axis} strokeWidth={1.2} strokeDasharray="5 4"
              />
            )}

            {points.map((point, i) => (
              <circle
                key={i}
                cx={model.sx(point.x)}
                cy={model.sy(point.y)}
                r={point.failed ? 4 : 2.6}
                fill={point.failed ? 'none' : color}
                fillOpacity={0.72}
                stroke={point.failed ? SEMANTIC.bad : 'none'}
                strokeWidth={point.failed ? 1.4 : 0}
              >
                {point.title && <title>{point.title}</title>}
              </circle>
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
            {yLabel}
          </text>
        </svg>
      ) : (
        <div className="skeleton" style={{ height }} />
      )}
    </div>
  )
}
