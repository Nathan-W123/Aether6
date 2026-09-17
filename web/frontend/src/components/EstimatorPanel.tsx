import { LineChart } from './charts/LineChart'
import { SEMANTIC, seriesColor } from '../lib/palette'
import { fixed } from '../lib/format'
import type { RunMetrics, Series } from '../api/types'

interface Props {
  series: Series
  metrics: RunMetrics
  inLoop: boolean
}

/**
 * Truth against the filter's estimate.
 *
 * Truth is held entirely separately from the estimator in the simulator, so these errors are
 * real errors rather than an artefact of the two sharing a state vector. Where the filter
 * publishes a standard deviation, it is drawn as a ±1σ band around zero: a well-tuned filter
 * keeps the error inside its own band roughly two thirds of the time.
 */
export function EstimatorPanel({ series, metrics, inLoop }: Props) {
  const t = series.t ?? []
  const negate = (values: number[] | undefined) => (values ?? []).map((v) => -v)

  return (
    <div className="panel">
      <div className="panel-head">
        <h2>Navigation filter</h2>
        <span className={`badge ${inLoop ? 'good' : ''}`}>
          {inLoop ? 'in the control loop' : 'monitoring only'}
        </span>
      </div>
      <p className="panel-note">
        18-state error-state EKF on 200 Hz IMU, 5 Hz GPS, 20 Hz barometer, 50 Hz magnetometer
        and 50 Hz airspeed.{' '}
        {metrics.metrics_window_s > 0
          ? `Position RMSE ${fixed(metrics.rmse_position_m, 2)} m, attitude ` +
            `${fixed(metrics.rmse_attitude_deg, 2)}° over ${metrics.metrics_window_s.toFixed(0)} s, and `
          : 'The run ended before the metric window opened, so there are no summary errors to ' +
            'quote; the traces below still show the whole run, and '}
        the horizontal wind and barometer bias are estimated rather than assumed.
      </p>
      <div className="grid-2">
        <LineChart
          xLabel="time [s]" yLabel="position error [m]" height={185} zeroLine
          bands={[{
            x: t, lower: negate(series.sig_north), upper: series.sig_north ?? [],
            fill: SEMANTIC.band,
          }]}
          series={[
            { label: 'north', x: t, y: series.err_north ?? [], color: seriesColor(0) },
            { label: 'east', x: t, y: series.err_east ?? [], color: seriesColor(1) },
            { label: 'down', x: t, y: series.err_down ?? [], color: seriesColor(2) },
          ]}
          annotation="shaded: filter ±1σ (north)"
        />
        <LineChart
          xLabel="time [s]" yLabel="attitude error [deg]" height={185} zeroLine
          bands={[{
            x: t, lower: negate(series.sig_yaw_deg), upper: series.sig_yaw_deg ?? [],
            fill: SEMANTIC.band,
          }]}
          series={[
            { label: 'roll', x: t, y: series.err_roll_deg ?? [], color: seriesColor(0) },
            { label: 'pitch', x: t, y: series.err_pitch_deg ?? [], color: seriesColor(1) },
            { label: 'yaw', x: t, y: series.err_yaw_deg ?? [], color: seriesColor(2) },
          ]}
          annotation="shaded: filter ±1σ (yaw)"
        />
        <LineChart
          xLabel="time [s]" yLabel="horizontal wind [m/s]" height={185} zeroLine
          series={[
            { label: 'north, truth', x: t, y: series.wind_n ?? [], color: seriesColor(0) },
            { label: 'north, estimated', x: t, y: series.est_wind_n ?? [], color: seriesColor(0),
              dashed: true },
            { label: 'east, truth', x: t, y: series.wind_e ?? [], color: seriesColor(1) },
            { label: 'east, estimated', x: t, y: series.est_wind_e ?? [], color: seriesColor(1),
              dashed: true },
          ]}
        />
        <LineChart
          xLabel="time [s]" yLabel="gyro bias error [deg/s]" height={185} zeroLine
          bands={[{
            x: t, lower: negate(series.sig_bg_x_dps), upper: series.sig_bg_x_dps ?? [],
            fill: SEMANTIC.band,
          }]}
          series={[
            { label: 'x', x: t, y: series.bg_err_x_dps ?? [], color: seriesColor(0) },
            { label: 'y', x: t, y: series.bg_err_y_dps ?? [], color: seriesColor(1) },
            { label: 'z', x: t, y: series.bg_err_z_dps ?? [], color: seriesColor(2) },
          ]}
          annotation="shaded: filter ±1σ (x)"
        />
      </div>
    </div>
  )
}
