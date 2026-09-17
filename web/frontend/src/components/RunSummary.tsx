import type { ReactNode } from 'react'

import { fixed, humanise } from '../lib/format'
import type { RunMetrics, TrimPoint } from '../api/types'

interface Props {
  metrics: RunMetrics
  trim: TrimPoint
  feedback: 'estimate' | 'truth'
}

/**
 * The run's headline figures, set as a ruled results block.
 *
 * The tracking and estimator statistics only accumulate after a settling window, so a run the
 * safety monitor stopped before that has no samples and every RMS reads zero. Printing "0.00"
 * there would claim flawless tracking, so those cells read "—" and say why.
 */
export function RunSummary({ metrics, trim, feedback }: Props) {
  const measured = metrics.metrics_window_s > 0
  const noSamples = 'no samples: run ended first'

  return (
    <dl className="summary">
      <Cell
        label="Circuits flown"
        value={String(metrics.laps_completed)}
        unit={metrics.laps_completed === 1 ? 'lap' : 'laps'}
        foot={`${metrics.waypoints_reached} waypoint arrivals · ${humanise(metrics.termination_reason).toLowerCase()}`}
        tone={metrics.completed ? undefined : 'limit'}
      />
      <Cell
        label="Cross-track RMS"
        value={measured ? fixed(metrics.rms_cross_track_m, 1) : '—'}
        unit={measured ? 'm' : undefined}
        foot={measured ? `peak ${fixed(metrics.max_cross_track_m, 1)} m` : noSamples}
        tone={measured ? undefined : 'void'}
      />
      <Cell
        label="Altitude RMS"
        value={measured ? fixed(metrics.rms_altitude_error_m, 2) : '—'}
        unit={measured ? 'm' : undefined}
        foot={measured ? `peak ${fixed(metrics.max_altitude_error_m, 1)} m` : noSamples}
        tone={measured ? undefined : 'void'}
      />
      <Cell
        label="Navigation RMSE"
        value={measured ? fixed(metrics.rmse_position_m, 2) : '—'}
        unit={measured ? 'm' : undefined}
        foot={!measured ? noSamples
          : feedback === 'truth'
            ? 'filter runs, but not in the loop'
            : `attitude ${fixed(metrics.rmse_attitude_deg, 2)}°`}
        tone={measured ? undefined : 'void'}
      />
      <Cell
        label={<>Peak bank / <span className="sym">α</span></>}
        compound
        value={`${fixed(metrics.max_bank_deg, 0)}° / ${fixed(metrics.max_alpha_deg, 1)}°`}
        foot={metrics.max_alpha_deg > 13.5
          ? 'angle-of-attack protection engaged'
          : 'inside the flight envelope'}
        tone={metrics.max_alpha_deg > 13.5 ? 'limit' : undefined}
      />
      <Cell
        label="Trim residual"
        value={trim.residual_inf_norm.toExponential(0)}
        foot={`α ${fixed(trim.alpha_deg, 2)}° · throttle ${fixed(trim.throttle, 2)}`}
      />
    </dl>
  )
}

function Cell({ label, value, unit, foot, tone, compound }: {
  label: ReactNode
  value: string
  unit?: string
  foot?: string
  tone?: 'limit' | 'void'
  /** Two quantities in one cell: set smaller so the pair stays on one line. */
  compound?: boolean
}) {
  return (
    <div className={`summary-cell${tone ? ` is-${tone}` : ''}`}>
      <dt className="label">{label}</dt>
      <dd className={compound ? 'is-compound' : undefined}>
        {value}
        {unit && <span className="unit">{unit}</span>}
      </dd>
      {foot && <div className="foot">{foot}</div>}
    </div>
  )
}
