import { LineChart } from './charts/LineChart'
import { SEMANTIC, seriesColor } from '../lib/palette'
import type { Series } from '../api/types'

interface Props {
  series: Series
}

/** Aircraft states: what the vehicle actually did. */
export function StatesPanel({ series }: Props) {
  const t = series.t ?? []
  return (
    <div className="panel">
      <div className="panel-head"><h2>Aircraft states</h2></div>
      <p className="panel-note">
        Truth from the 13-state integrator. Dashed lines are the guidance commands the inner
        loops are tracking.
      </p>
      <div className="grid-2">
        <LineChart
          xLabel="time [s]" yLabel="altitude [m]" height={185}
          series={[
            { label: 'altitude', x: t, y: series.altitude ?? [], color: seriesColor(0) },
            { label: 'commanded', x: t, y: series.cmd_altitude ?? [], color: SEMANTIC.command,
              dashed: true },
          ]}
        />
        <LineChart
          xLabel="time [s]" yLabel="airspeed [m/s]" height={185}
          series={[
            { label: 'airspeed', x: t, y: series.airspeed ?? [], color: seriesColor(0) },
            { label: 'commanded', x: t, y: series.cmd_airspeed ?? [], color: SEMANTIC.command,
              dashed: true },
          ]}
        />
        <LineChart
          xLabel="time [s]" yLabel="attitude [deg]" height={185} zeroLine
          series={[
            { label: 'roll', x: t, y: series.roll_deg ?? [], color: seriesColor(0) },
            { label: 'pitch', x: t, y: series.pitch_deg ?? [], color: seriesColor(1) },
            { label: 'roll command', x: t, y: series.roll_cmd_deg ?? [], color: SEMANTIC.command,
              dashed: true },
          ]}
        />
        <LineChart
          xLabel="time [s]" yLabel="body rates [deg/s]" height={185} zeroLine
          series={[
            { label: 'p (roll)', x: t, y: series.p_dps ?? [], color: seriesColor(0) },
            { label: 'q (pitch)', x: t, y: series.q_dps ?? [], color: seriesColor(1) },
            { label: 'r (yaw)', x: t, y: series.r_dps ?? [], color: seriesColor(2) },
          ]}
        />
        <LineChart
          xLabel="time [s]" yLabel="aerodynamic angles [deg]" height={185} zeroLine
          series={[
            { label: 'angle of attack α', x: t, y: series.alpha_deg ?? [], color: seriesColor(0) },
            { label: 'sideslip β', x: t, y: series.beta_deg ?? [], color: seriesColor(1) },
          ]}
        />
        <LineChart
          xLabel="time [s]" yLabel="cross-track error [m]" height={185} zeroLine
          series={[
            { label: 'cross-track', x: t, y: series.cross_track ?? [], color: seriesColor(0) },
          ]}
          annotation={rms(series.cross_track)}
        />
      </div>
    </div>
  )
}

function rms(values: number[] | undefined): string | undefined {
  if (!values || values.length === 0) return undefined
  const mean = values.reduce((sum, v) => sum + v * v, 0) / values.length
  return `RMS ${Math.sqrt(mean).toFixed(2)} m`
}
