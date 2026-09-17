import { fixed, humanise } from '../lib/format'
import type { RunMetrics, TrimPoint } from '../api/types'

interface Props {
  metrics: RunMetrics
  trim: TrimPoint
  feedback: 'estimate' | 'truth'
}

/**
 * The six numbers that answer "did it work?" before the reader scrolls.
 *
 * When a run is cut short by the safety monitor before the simulator's settling window
 * opens, its RMS figures are all zero for want of samples. Showing "0.00 m" there would read
 * as perfect tracking, so those tiles show "—" and say why instead.
 */
export function StatTiles({ metrics, trim, feedback }: Props) {
  const outcome = metrics.completed
    ? { label: 'completed', tone: 'good' as const }
    : { label: humanise(metrics.termination_reason), tone: 'bad' as const }

  const measured = metrics.metrics_window_s > 0
  const window = measured
    ? `over ${metrics.metrics_window_s.toFixed(0)} s`
    : 'no samples before the run ended'

  return (
    <div className="tiles">
      <Tile
        label="Mission"
        value={`${metrics.laps_completed}`}
        unit={metrics.laps_completed === 1 ? 'lap' : 'laps'}
        sub={`${metrics.waypoints_reached} waypoint arrivals · ${outcome.label}`}
        tone={outcome.tone}
      />
      <Tile
        label="Cross-track RMS"
        value={measured ? fixed(metrics.rms_cross_track_m, 1) : '—'}
        unit={measured ? 'm' : undefined}
        sub={measured ? `peak ${fixed(metrics.max_cross_track_m, 1)} m` : window}
        tone={measured ? undefined : 'warn'}
      />
      <Tile
        label="Altitude RMS"
        value={measured ? fixed(metrics.rms_altitude_error_m, 2) : '—'}
        unit={measured ? 'm' : undefined}
        sub={measured ? `peak ${fixed(metrics.max_altitude_error_m, 1)} m` : window}
        tone={measured ? undefined : 'warn'}
      />
      <Tile
        label="Navigation RMSE"
        value={measured ? fixed(metrics.rmse_position_m, 2) : '—'}
        unit={measured ? 'm' : undefined}
        sub={!measured ? window
          : feedback === 'truth'
            ? 'filter running, not in the loop'
            : `attitude ${fixed(metrics.rmse_attitude_deg, 2)}°`}
        tone={measured ? undefined : 'warn'}
      />
      <Tile
        label="Peak bank / α"
        value={`${fixed(metrics.max_bank_deg, 0)}° / ${fixed(metrics.max_alpha_deg, 1)}°`}
        sub={metrics.max_alpha_deg > 13.5 ? 'α protection active' : 'inside the envelope'}
        tone={metrics.max_alpha_deg > 13.5 ? 'warn' : undefined}
      />
      <Tile
        label="Trim residual"
        value={trim.residual_inf_norm.toExponential(0)}
        sub={`α ${fixed(trim.alpha_deg, 2)}° · throttle ${fixed(trim.throttle, 2)}`}
      />
    </div>
  )
}

function Tile({ label, value, unit, sub, tone }: {
  label: string
  value: string
  unit?: string
  sub?: string
  tone?: 'good' | 'warn' | 'bad'
}) {
  const colour = tone === 'good' ? 'var(--good)'
    : tone === 'warn' ? 'var(--warn)'
    : tone === 'bad' ? 'var(--bad)' : 'var(--text)'
  return (
    <div className="tile">
      <div className="tile-label">{label}</div>
      <div className="tile-value" style={{ color: colour }}>
        {value}
        {unit && <span className="tile-unit">{unit}</span>}
      </div>
      {sub && <div className="tile-sub">{sub}</div>}
    </div>
  )
}
