import { Figure } from './sheet/Figure'
import { LineChart } from './charts/LineChart'
import { seriesColor } from '../lib/palette'
import type { Series } from '../api/types'

/**
 * Actuator states — what the aerodynamics received, not what the autopilot asked for.
 *
 * Each surface is driven through a first-order servo with position and rate limits, so the
 * deflection limits are drawn as a fixed domain: saturation reads as the trace flattening
 * against the edge of the field rather than as an unremarkable plateau.
 */
export function ControlsPanel({ series }: { series: Series }) {
  const t = series.t ?? []
  return (
    <div className="figure-row">
      <Figure number="9" caption={<>Surface deflections, plotted against the full ±25° travel
        so saturation is visible as contact with the edge of the field.</>}>
        <LineChart
          y="deflection" yUnit="deg" zeroLine yDomain={[-26, 26]} height={196}
          series={[
            { label: 'elevator', x: t, y: series.elevator_deg ?? [], color: seriesColor(0) },
            { label: 'aileron', x: t, y: series.aileron_deg ?? [], color: seriesColor(1) },
            { label: 'rudder', x: t, y: series.rudder_deg ?? [], color: seriesColor(2) },
          ]}
        />
      </Figure>

      <Figure number="10" caption={<>Throttle, on its full 0–1 travel. The climbing legs sit
        near the top of the range; the descents idle.</>}>
        <LineChart
          y="throttle" yUnit="0–1" yDomain={[0, 1.02]} height={196}
          series={[{ label: 'throttle', x: t, y: series.throttle ?? [], color: seriesColor(3) }]}
        />
      </Figure>
    </div>
  )
}
