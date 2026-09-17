import { Figure } from './sheet/Figure'
import { LineChart } from './charts/LineChart'
import { SEMANTIC, seriesColor } from '../lib/palette'
import type { Series } from '../api/types'

/**
 * Truth against the filter's estimate.
 *
 * Truth is held entirely separately from the estimator inside the simulator, so these are real
 * errors rather than an artefact of the two sharing a state vector. Where the filter publishes
 * a standard deviation it is drawn as a ±1σ band about zero: a well-tuned filter keeps its
 * error inside its own band roughly two thirds of the time, and that is checkable by eye.
 */
export function EstimatorPanel({ series }: { series: Series }) {
  const t = series.t ?? []
  const negate = (values: number[] | undefined) => (values ?? []).map((v) => -v)

  return (
    <div className="figure-row">
      <Figure number="11" caption={<>Position error by axis, with the filter's own ±1σ on north
        shaded. The error staying inside the band is the consistency check.</>}>
        <LineChart
          y="position error" yUnit="m" zeroLine
          bands={[{ x: t, lower: negate(series.sig_north), upper: series.sig_north ?? [],
                    fill: SEMANTIC.band }]}
          series={[
            { label: 'north', x: t, y: series.err_north ?? [], color: seriesColor(0) },
            { label: 'east', x: t, y: series.err_east ?? [], color: seriesColor(1) },
            { label: 'down', x: t, y: series.err_down ?? [], color: seriesColor(2) },
          ]}
        />
      </Figure>

      <Figure number="12" caption={<>Attitude error, with ±1σ on yaw shaded. Yaw converges
        once the magnetometer and the wind estimate agree.</>}>
        <LineChart
          y="attitude error" yUnit="deg" zeroLine
          bands={[{ x: t, lower: negate(series.sig_yaw_deg), upper: series.sig_yaw_deg ?? [],
                    fill: SEMANTIC.band }]}
          series={[
            { label: 'roll', x: t, y: series.err_roll_deg ?? [], color: seriesColor(0) },
            { label: 'pitch', x: t, y: series.err_pitch_deg ?? [], color: seriesColor(1) },
            { label: 'yaw', x: t, y: series.err_yaw_deg ?? [], color: seriesColor(2) },
          ]}
        />
      </Figure>

      <Figure number="13" caption={<>Horizontal wind: truth solid, estimate dashed. The filter
        is seeded from the wind triangle on its first airspeed sample, not from zero.</>}>
        <LineChart
          y="wind" yUnit="m/s" zeroLine
          series={[
            { label: 'north, truth', x: t, y: series.wind_n ?? [], color: seriesColor(0) },
            { label: 'north, estimated', x: t, y: series.est_wind_n ?? [],
              color: seriesColor(0), dashed: true },
            { label: 'east, truth', x: t, y: series.wind_e ?? [], color: seriesColor(1) },
            { label: 'east, estimated', x: t, y: series.est_wind_e ?? [],
              color: seriesColor(1), dashed: true },
          ]}
        />
      </Figure>

      <Figure number="14" caption={<>Gyro bias error with ±1σ on the x axis shaded. The biases
        are observable through the attitude update and converge within a minute.</>}>
        <LineChart
          y="gyro bias error" yUnit="deg/s" zeroLine
          bands={[{ x: t, lower: negate(series.sig_bg_x_dps), upper: series.sig_bg_x_dps ?? [],
                    fill: SEMANTIC.band }]}
          series={[
            { label: 'x', x: t, y: series.bg_err_x_dps ?? [], color: seriesColor(0) },
            { label: 'y', x: t, y: series.bg_err_y_dps ?? [], color: seriesColor(1) },
            { label: 'z', x: t, y: series.bg_err_z_dps ?? [], color: seriesColor(2) },
          ]}
        />
      </Figure>
    </div>
  )
}
