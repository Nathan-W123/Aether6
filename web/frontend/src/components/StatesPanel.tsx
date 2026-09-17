import { Figure } from './sheet/Figure'
import { LineChart } from './charts/LineChart'
import { SEMANTIC, seriesColor } from '../lib/palette'
import type { Series } from '../api/types'

/** Time histories of the vehicle's own states, from the 13-state integrator. */
export function StatesPanel({ series }: { series: Series }) {
  const t = series.t ?? []
  return (
    <div className="figure-row">
      <Figure number="3" caption={<>Altitude against the guidance command. The profile steps
        between legs; the tracking lag is the outer loop's filtered altitude command.</>}>
        <LineChart
          y="altitude" yUnit="m"
          series={[
            { label: 'flown', x: t, y: series.altitude ?? [], color: seriesColor(0) },
            { label: 'commanded', x: t, y: series.cmd_altitude ?? [],
              color: SEMANTIC.reference, dashed: true },
          ]}
        />
      </Figure>

      <Figure number="4" caption={<>Airspeed against command. The scatter is turbulence
        entering through the relative wind, not a control error.</>}>
        <LineChart
          y="airspeed" yUnit="m/s"
          series={[
            { label: 'flown', x: t, y: series.airspeed ?? [], color: seriesColor(0) },
            { label: 'commanded', x: t, y: series.cmd_airspeed ?? [],
              color: SEMANTIC.reference, dashed: true },
          ]}
        />
      </Figure>

      <Figure number="5" caption={<>Attitude. Each excursion in roll is a leg change; the
        bank command is what the guidance law asked for.</>}>
        <LineChart
          y="attitude" yUnit="deg" zeroLine
          series={[
            { label: 'roll', x: t, y: series.roll_deg ?? [], color: seriesColor(0) },
            { label: 'pitch', x: t, y: series.pitch_deg ?? [], color: seriesColor(1) },
            { label: 'roll command', x: t, y: series.roll_cmd_deg ?? [],
              color: SEMANTIC.reference, dashed: true },
          ]}
        />
      </Figure>

      <Figure number="6" caption={<>Body rates. The roll-rate spikes at leg changes decay at
        the roll-subsidence time constant measured in §7.</>}>
        <LineChart
          y="body rate" yUnit="deg/s" zeroLine
          series={[
            { label: 'p, roll', x: t, y: series.p_dps ?? [], color: seriesColor(0) },
            { label: 'q, pitch', x: t, y: series.q_dps ?? [], color: seriesColor(1) },
            { label: 'r, yaw', x: t, y: series.r_dps ?? [], color: seriesColor(2) },
          ]}
        />
      </Figure>

      <Figure number="7" caption={<>Aerodynamic angles. The envelope protection holds angle of
        attack below its 13.8° limit; sideslip stays small, so the turns are coordinated.</>}>
        <LineChart
          y="angle" yUnit="deg" zeroLine
          limit={{ value: 13.8, label: 'α limit' }}
          series={[
            { label: 'angle of attack α', x: t, y: series.alpha_deg ?? [], color: seriesColor(0) },
            { label: 'sideslip β', x: t, y: series.beta_deg ?? [], color: seriesColor(1) },
          ]}
        />
      </Figure>

      <Figure number="8" caption={<>Cross-track error against the active leg — the quantity the
        guidance law is minimising. RMS <b>{rms(series.cross_track)}</b>.</>}>
        <LineChart
          y="cross-track" yUnit="m" zeroLine
          series={[
            { label: 'cross-track', x: t, y: series.cross_track ?? [], color: seriesColor(0) },
          ]}
        />
      </Figure>
    </div>
  )
}

function rms(values: number[] | undefined): string {
  if (!values || values.length === 0) return '—'
  const mean = values.reduce((sum, v) => sum + v * v, 0) / values.length
  return `${Math.sqrt(mean).toFixed(2)} m`
}
