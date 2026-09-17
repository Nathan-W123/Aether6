import { useMemo, useState } from 'react'

import { useMeasure } from '../../lib/useMeasure'
import { SEMANTIC } from '../../lib/palette'
import { extent, formatTick, linearScale, niceTicks, padDomain } from './plot'

export interface ScatterPoint {
  x: number
  y: number
  /** Trials the safety monitor stopped: drawn as an open oxide ring, never colour alone. */
  failed?: boolean
  title?: string
}

interface Props {
  points: ScatterPoint[]
  x: string
  xUnit: string
  y: string
  yUnit: string
  color: string
  height?: number
  /** Least-squares fit through the completed trials, with its slope reported. */
  trend?: boolean
}

const MARGIN = { top: 12, right: 12, bottom: 32, left: 54 }

/** One dispersed input against one outcome, a mark per trial. */
export function ScatterChart({
  points, x, xUnit, y, yUnit, color, height = 210, trend = false,
}: Props) {
  const [ref, { width }] = useMeasure<HTMLDivElement>()
  const [hover, setHover] = useState<number | null>(null)
  const plotWidth = Math.max(0, width - MARGIN.left - MARGIN.right)
  const plotHeight = Math.max(0, height - MARGIN.top - MARGIN.bottom)

  const model = useMemo(() => {
    const xs = extent([points.map((p) => p.x)])
    const ys = extent([points.map((p) => p.y)])
    if (!xs || !ys) return null
    const xd = padDomain(xs, 0.05)
    const yd = padDomain(ys, 0.08)

    let fit: { x0: number; y0: number; x1: number; y1: number; slope: number } | null = null
    if (trend) {
      const ok = points.filter((p) => !p.failed)
      if (ok.length >= 3) {
        const mx = ok.reduce((a, p) => a + p.x, 0) / ok.length
        const my = ok.reduce((a, p) => a + p.y, 0) / ok.length
        const sxx = ok.reduce((a, p) => a + (p.x - mx) ** 2, 0)
        const sxy = ok.reduce((a, p) => a + (p.x - mx) * (p.y - my), 0)
        if (sxx > 0) {
          const slope = sxy / sxx
          fit = { x0: xd[0], y0: my + slope * (xd[0] - mx),
                  x1: xd[1], y1: my + slope * (xd[1] - mx), slope }
        }
      }
    }
    return {
      sx: linearScale(xd, [0, plotWidth]),
      sy: linearScale(yd, [plotHeight, 0]),
      fit,
      xTicks: niceTicks(xd, Math.max(2, Math.round(plotWidth / 86))),
      yTicks: niceTicks(yd, Math.max(2, Math.round(plotHeight / 38))),
    }
  }, [points, plotWidth, plotHeight, trend])

  const failures = points.filter((p) => p.failed).length

  return (
    <div>
      <div className="legend">
        <span className="legend-item">
          <svg width="11" height="11" aria-hidden="true">
            <circle cx="5.5" cy="5.5" r="3" fill={color} fillOpacity={0.8} />
          </svg>
          completed trial
        </span>
        {failures > 0 && (
          <span className="legend-item">
            <svg width="11" height="11" aria-hidden="true">
              <circle cx="5.5" cy="5.5" r="4" fill="none" stroke="var(--oxide)" strokeWidth="1.4" />
            </svg>
            stopped by a safety limit ({failures})
          </span>
        )}
        {model?.fit && (
          <span className="legend-item">
            <span className="legend-key" style={{
              borderTopColor: 'var(--ink-mute)', borderTopStyle: 'dashed' }} />
            fit, <span className="legend-value">
              {model.fit.slope >= 0 ? '+' : ''}{model.fit.slope.toFixed(2)}
            </span> {yUnit} per {xUnit}
          </span>
        )}
      </div>

      <div ref={ref} style={{ width: '100%' }}>
        {width > 0 && model ? (
          <svg width={width} height={height} role="img"
               aria-label={`${y} in ${yUnit} against ${x} in ${xUnit}, one mark per trial`}>
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
                  <line y1={plotHeight} y2={plotHeight + 4} stroke={SEMANTIC.axis} strokeWidth={1} />
                  <text y={plotHeight + 15} textAnchor="middle" fontSize={10}
                        fill="var(--ink-mute)" fontFamily="var(--mono)">
                    {formatTick(tick, model.xTicks)}
                  </text>
                </g>
              ))}

              {model.fit && (
                <line x1={model.sx(model.fit.x0)} y1={model.sy(model.fit.y0)}
                      x2={model.sx(model.fit.x1)} y2={model.sy(model.fit.y1)}
                      stroke="var(--ink-mute)" strokeWidth={1.2} strokeDasharray="6 4" />
              )}

              {points.map((point, i) => (
                <circle
                  key={i}
                  cx={model.sx(point.x)}
                  cy={model.sy(point.y)}
                  r={point.failed ? 4.5 : hover === i ? 5 : 3}
                  fill={point.failed ? 'none' : color}
                  fillOpacity={point.failed ? 1 : 0.8}
                  stroke={point.failed ? 'var(--oxide)' : hover === i ? 'var(--ink)' : 'none'}
                  strokeWidth={point.failed ? 1.4 : 1.5}
                  onPointerEnter={() => setHover(i)}
                  onPointerLeave={() => setHover(null)}
                >
                  {point.title && <title>{point.title}</title>}
                </circle>
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
