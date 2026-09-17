import { useMemo } from 'react'

import { useMeasure } from '../lib/useMeasure'
import { SEMANTIC, seriesColor } from '../lib/palette'
import { extent, formatTick, linearScale, niceTicks, padDomain } from './charts/plot'
import { fixed, humanise } from '../lib/format'
import type { Mode, PrecomputedModes } from '../api/types'

const MARGIN = { top: 12, right: 16, bottom: 30, left: 50 }

interface PolePlotProps {
  modes: Mode[]
  /** Index into the shared colour order, so a mode keeps its colour between the two plots. */
  colourOf: (mode: Mode) => string
  caption: string
  height?: number
}

/** Eigenvalues in the complex plane; the shaded half-plane is where a mode diverges. */
function PolePlot({ modes, colourOf, caption, height = 180 }: PolePlotProps) {
  const [ref, { width }] = useMeasure<HTMLDivElement>()
  const plotWidth = Math.max(0, width - MARGIN.left - MARGIN.right)
  const plotHeight = Math.max(0, height - MARGIN.top - MARGIN.bottom)

  const model = useMemo(() => {
    if (modes.length === 0) return null
    const reals = modes.map((m) => m.real)
    const imags = modes.flatMap((m) => [m.imag, -m.imag])
    // Always keep the imaginary axis in frame: the whole point is which side a pole is on.
    const rx = extent([reals.concat([0])])
    const ix = extent([imags.concat([0])])
    if (!rx || !ix) return null
    const rd = padDomain(rx, 0.16)
    const id = padDomain(ix, 0.22)
    return {
      sx: linearScale(rd, [0, plotWidth]),
      sy: linearScale(id, [plotHeight, 0]),
      xTicks: niceTicks(rd, Math.max(2, Math.round(plotWidth / 80))),
      yTicks: niceTicks(id, 4),
    }
  }, [modes, plotWidth, plotHeight])

  return (
    <div>
      <div ref={ref} style={{ width: '100%' }}>
        {width > 0 && model ? (
          <svg width={width} height={height} role="img" aria-label={caption}>
            <g transform={`translate(${MARGIN.left},${MARGIN.top})`}>
              {model.yTicks.map((tick) => (
                <g key={`y${tick}`} transform={`translate(0,${model.sy(tick).toFixed(1)})`}>
                  <line x2={plotWidth} stroke={SEMANTIC.grid} />
                  <text x={-8} dy="0.32em" textAnchor="end" fontSize={10} fill="var(--text-dim)"
                        className="mono">
                    {formatTick(tick, model.yTicks)}
                  </text>
                </g>
              ))}
              {model.xTicks.map((tick) => (
                <g key={`x${tick}`} transform={`translate(${model.sx(tick).toFixed(1)},0)`}>
                  <line y2={plotHeight} stroke={SEMANTIC.grid} />
                  <text y={plotHeight + 14} textAnchor="middle" fontSize={10} fill="var(--text-dim)"
                        className="mono">
                    {formatTick(tick, model.xTicks)}
                  </text>
                </g>
              ))}

              <rect x={model.sx(0)} width={Math.max(0, plotWidth - model.sx(0))} height={plotHeight}
                    fill="#e6676714" />
              <line x1={model.sx(0)} x2={model.sx(0)} y2={plotHeight} stroke={SEMANTIC.axis}
                    strokeWidth={1.2} />

              {modes.flatMap((mode) => {
                const colour = colourOf(mode)
                const points = mode.imag === 0 ? [0] : [mode.imag, -mode.imag]
                return points.map((imag, j) => (
                  <g key={`${mode.mode}${j}`}
                     transform={`translate(${model.sx(mode.real).toFixed(1)},${model.sy(imag).toFixed(1)})`}>
                    <line x1={-5.5} y1={-5.5} x2={5.5} y2={5.5} stroke={colour} strokeWidth={2.2} />
                    <line x1={-5.5} y1={5.5} x2={5.5} y2={-5.5} stroke={colour} strokeWidth={2.2} />
                    <title>
                      {`${humanise(mode.mode)}: ${mode.real.toFixed(3)} ${imag >= 0 ? '+' : '−'} ${Math.abs(imag).toFixed(3)}j`}
                    </title>
                  </g>
                ))
              })}

              <line y1={plotHeight} y2={plotHeight} x2={plotWidth} stroke={SEMANTIC.axis} />
            </g>
            <text x={MARGIN.left + plotWidth / 2} y={height - 2} textAnchor="middle" fontSize={10}
                  fill="var(--text-dim)">
              real part [1/s]
            </text>
            <text transform={`translate(10,${MARGIN.top + plotHeight / 2}) rotate(-90)`}
                  textAnchor="middle" fontSize={10} fill="var(--text-dim)">
              imag [rad/s]
            </text>
          </svg>
        ) : (
          <div className="skeleton" style={{ height }} />
        )}
      </div>
      <div style={{ fontSize: 11.5, color: 'var(--text-muted)', marginTop: 2 }}>{caption}</div>
    </div>
  )
}

/**
 * Eigenvalues of the numerically linearised model, as pole plots and a table.
 *
 * Two plots rather than one: the roll subsidence sits at −17 rad/s while the phugoid and
 * spiral are within 0.1 of the origin, so a single axis that shows the fast root makes the
 * slow ones a single unreadable blob — and whether the spiral is left or right of the
 * imaginary axis is exactly what a reader comes to this panel for.
 */
export function ModesPanel({ data }: { data: PrecomputedModes }) {
  const colourFor = useMemo(() => {
    const order = new Map(data.modes.map((mode, index) => [mode.mode, seriesColor(index)]))
    return (mode: Mode) => order.get(mode.mode) ?? seriesColor(0)
  }, [data.modes])

  const slow = data.modes.filter((mode) => Math.abs(mode.real) < 1 && Math.abs(mode.imag) < 1.5)
  const spiral = data.modes.find((mode) => mode.mode === 'spiral')

  return (
    <div className="panel">
      <div className="panel-head">
        <h2>Linearised dynamics</h2>
        <span className="badge">precomputed · reference trim</span>
      </div>
      <p className="panel-note">
        The 12×12 Jacobian is taken by central differences in the quaternion tangent space at
        the 25 m/s, 120 m trim point, then split into longitudinal and lateral-directional
        blocks. Every classical mode appears with the right character, including the slowly
        divergent spiral that real airframes of this class also have.
      </p>
      <div className="grid-2" style={{ alignItems: 'start' }}>
        <div style={{ display: 'grid', gap: 14 }}>
          <PolePlot modes={data.modes} colourOf={colourFor}
                    caption="All modes. The shaded half-plane is unstable." />
          {slow.length > 0 && (
            <PolePlot modes={slow} colourOf={colourFor} height={160}
                      caption="Detail near the origin: the phugoid and the spiral." />
          )}
        </div>

        <div>
          <table className="data">
            <thead>
              <tr>
                <th>mode</th>
                <th>ω<sub>n</sub> [rad/s]</th>
                <th>ζ</th>
                <th>period [s]</th>
                <th>t½ [s]</th>
              </tr>
            </thead>
            <tbody>
              {data.modes.map((mode) => (
                <tr key={mode.mode}>
                  <td>
                    <span style={{
                      display: 'inline-block', width: 9, height: 3, borderRadius: 2,
                      background: colourFor(mode), marginRight: 7, verticalAlign: 'middle',
                    }} />
                    {humanise(mode.mode)}
                    {!mode.stable && (
                      <span className="badge warn" style={{ marginLeft: 8 }}>divergent</span>
                    )}
                  </td>
                  <td>{fixed(mode.natural_frequency_rad_s, 3)}</td>
                  <td>{fixed(mode.damping_ratio, 3)}</td>
                  <td>{mode.period_s > 0 ? fixed(mode.period_s, 2) : '—'}</td>
                  <td>{mode.time_to_half_s !== 0 ? fixed(Math.abs(mode.time_to_half_s), 2) : '—'}</td>
                </tr>
              ))}
            </tbody>
          </table>
          {spiral && (
            <p className="panel-note" style={{ marginTop: 12, marginBottom: 0 }}>
              The spiral mode's real part is positive, so its "time to half" is really a time to
              double: {fixed(Math.abs(spiral.time_to_half_s), 1)} s. The autopilot's roll loop is
              what keeps it bounded in the closed-loop runs above.
            </p>
          )}
        </div>
      </div>
    </div>
  )
}
