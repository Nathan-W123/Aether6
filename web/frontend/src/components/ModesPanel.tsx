import { useMemo } from 'react'

import { DataTable } from './sheet/DataTable'
import { Figure } from './sheet/Figure'
import { useMeasure } from '../lib/useMeasure'
import { SEMANTIC, seriesColor } from '../lib/palette'
import { extent, formatTick, linearScale, niceTicks, padDomain } from './charts/plot'
import { fixed, humanise } from '../lib/format'
import type { Mode, PrecomputedModes } from '../api/types'

const MARGIN = { top: 14, right: 16, bottom: 32, left: 50 }

interface PolePlotProps {
  modes: Mode[]
  colourOf: (mode: Mode) => string
  height?: number
}

/** Eigenvalues in the complex plane; the shaded half-plane is where a mode diverges. */
function PolePlot({ modes, colourOf, height = 190 }: PolePlotProps) {
  const [ref, { width }] = useMeasure<HTMLDivElement>()
  const plotWidth = Math.max(0, width - MARGIN.left - MARGIN.right)
  const plotHeight = Math.max(0, height - MARGIN.top - MARGIN.bottom)

  const model = useMemo(() => {
    if (modes.length === 0) return null
    // Always keep the imaginary axis in frame: which side a pole sits on is the whole point.
    const rx = extent([modes.map((m) => m.real).concat([0])])
    const ix = extent([modes.flatMap((m) => [m.imag, -m.imag]).concat([0])])
    if (!rx || !ix) return null
    const rd = padDomain(rx, 0.18)
    const id = padDomain(ix, 0.24)
    return {
      sx: linearScale(rd, [0, plotWidth]),
      sy: linearScale(id, [plotHeight, 0]),
      xTicks: niceTicks(rd, Math.max(2, Math.round(plotWidth / 78))),
      yTicks: niceTicks(id, 4),
    }
  }, [modes, plotWidth, plotHeight])

  return (
    <div ref={ref} style={{ width: '100%' }}>
      {width > 0 && model ? (
        <svg width={width} height={height} role="img"
             aria-label="Eigenvalues of the linearised model in the complex plane">
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

            <rect x={model.sx(0)} width={Math.max(0, plotWidth - model.sx(0))} height={plotHeight}
                  fill="var(--oxide-wash)" />
            <line x1={model.sx(0)} x2={model.sx(0)} y2={plotHeight}
                  stroke="var(--oxide)" strokeWidth={1} />

            {modes.flatMap((mode) => {
              const colour = colourOf(mode)
              const points = mode.imag === 0 ? [0] : [mode.imag, -mode.imag]
              return points.map((imag, j) => (
                <g key={`${mode.mode}${j}`}
                   transform={`translate(${model.sx(mode.real).toFixed(1)},${model.sy(imag).toFixed(1)})`}>
                  <line x1={-5} y1={-5} x2={5} y2={5} stroke={colour} strokeWidth={2} />
                  <line x1={-5} y1={5} x2={5} y2={-5} stroke={colour} strokeWidth={2} />
                  <title>
                    {`${humanise(mode.mode)}: ${mode.real.toFixed(3)} ${imag >= 0 ? '+' : '−'} ${Math.abs(imag).toFixed(3)}j`}
                  </title>
                </g>
              ))
            })}

            <line y1={plotHeight} y2={plotHeight} x2={plotWidth}
                  stroke={SEMANTIC.axis} strokeWidth={1} />
          </g>
          <text x={MARGIN.left} y={height - 2} fontSize={10} fill="var(--ink-faint)"
                fontFamily="var(--sans)" letterSpacing="0.08em">RE(λ) [1/s]</text>
          <text transform={`translate(10,${MARGIN.top + plotHeight / 2}) rotate(-90)`}
                textAnchor="middle" fontSize={10} fill="var(--ink-faint)"
                fontFamily="var(--sans)" letterSpacing="0.08em">IM(λ) [rad/s]</text>
        </svg>
      ) : (
        <div className="placeholder" style={{ height }} />
      )}
    </div>
  )
}

/**
 * Eigenvalues of the numerically linearised model.
 *
 * Two plots rather than one: the roll subsidence sits at −17 rad/s while the phugoid and
 * spiral are within 0.1 of the origin, so a single axis that shows the fast root makes the
 * slow ones a single unreadable blob — and whether the spiral sits left or right of the
 * imaginary axis is exactly what a reader comes to this figure for.
 */
export function ModesPanel({ data }: { data: PrecomputedModes }) {
  const colourFor = useMemo(() => {
    const order = new Map(data.modes.map((mode, index) => [mode.mode, seriesColor(index)]))
    return (mode: Mode) => order.get(mode.mode) ?? seriesColor(0)
  }, [data.modes])

  const slow = data.modes.filter((mode) => Math.abs(mode.real) < 1 && Math.abs(mode.imag) < 1.5)
  const spiral = data.modes.find((mode) => mode.mode === 'spiral')

  return (
    <div className="figure-row">
      <div className="stack" style={{ gap: 24 }}>
        <Figure number="17" caption={<>All five modes. The shaded half-plane is unstable; the
          roll subsidence at −16.8 s⁻¹ sets the horizontal scale.</>}>
          <PolePlot modes={data.modes} colourOf={colourFor} />
        </Figure>
        {slow.length > 0 && (
          <Figure number="18" caption={<>The slow modes at scale. The phugoid sits just inside
            the stable half-plane; the spiral sits just outside it.</>}>
            <PolePlot modes={slow} colourOf={colourFor} height={172} />
          </Figure>
        )}
      </div>

      <div>
        <DataTable caption="Modal characteristics at the reference trim">
          <thead>
            <tr>
              <th>Mode</th>
              <th><span className="sym">ω</span><sub className="sym">n</sub> [rad/s]</th>
              <th><span className="sym">ζ</span></th>
              <th>Period [s]</th>
              <th><span className="sym">t</span>½ [s]</th>
            </tr>
          </thead>
          <tbody>
            {data.modes.map((mode) => (
              <tr key={mode.mode}>
                <td>
                  <span style={{
                    display: 'inline-block', width: 12, height: 0, marginRight: 8,
                    borderTop: `2px solid ${colourFor(mode)}`, verticalAlign: 'middle',
                  }} />
                  {humanise(mode.mode)}
                </td>
                <td>{fixed(mode.natural_frequency_rad_s, 3)}</td>
                <td className={mode.stable ? undefined : 'is-limit'}>
                  {fixed(mode.damping_ratio, 3)}
                </td>
                <td>{mode.period_s > 0 ? fixed(mode.period_s, 2) : '—'}</td>
                <td className={mode.stable ? undefined : 'is-limit'}>
                  {mode.time_to_half_s !== 0 ? fixed(Math.abs(mode.time_to_half_s), 2) : '—'}
                </td>
              </tr>
            ))}
          </tbody>
        </DataTable>
        {spiral && (
          <p className="section-note" style={{ marginTop: 12 }}>
            The spiral's real part is positive, so its damping ratio and time to half are set in
            oxide: the figure is really a time to <em>double</em>, {' '}
            {fixed(Math.abs(spiral.time_to_half_s), 1)} s. A mildly divergent spiral is normal
            for an airframe of this class — the autopilot's roll loop is what keeps it bounded
            in the closed-loop runs above.
          </p>
        )}
      </div>
    </div>
  )
}
