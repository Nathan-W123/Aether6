import { LineChart } from './charts/LineChart'
import { seriesColor } from '../lib/palette'
import type { Series } from '../api/types'

interface Props {
  series: Series
}

/**
 * Control inputs, after the first-order servo models and the position and rate limits.
 *
 * Surfaces and throttle are on separate panels because they are different units; the
 * deflection limits are drawn as a fixed domain so saturation is visible as the trace
 * flattening against the edge of the plot rather than as an unremarkable plateau.
 */
export function ControlsPanel({ series }: Props) {
  const t = series.t ?? []
  return (
    <div className="panel">
      <div className="panel-head"><h2>Control inputs</h2></div>
      <p className="panel-note">
        Actuator states, not commands: each surface is driven through a first-order servo with
        position and rate limits, so what you see here is what the aerodynamics received.
      </p>
      <div className="grid-2">
        <LineChart
          xLabel="time [s]" yLabel="surface deflection [deg]" height={195} zeroLine
          yDomain={[-26, 26]}
          series={[
            { label: 'elevator', x: t, y: series.elevator_deg ?? [], color: seriesColor(0) },
            { label: 'aileron', x: t, y: series.aileron_deg ?? [], color: seriesColor(1) },
            { label: 'rudder', x: t, y: series.rudder_deg ?? [], color: seriesColor(2) },
          ]}
        />
        <LineChart
          xLabel="time [s]" yLabel="throttle [0–1]" height={195}
          yDomain={[0, 1.02]}
          series={[
            { label: 'throttle', x: t, y: series.throttle ?? [], color: seriesColor(3) },
          ]}
        />
      </div>
    </div>
  )
}
