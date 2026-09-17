import { humanise } from '../lib/format'
import type { RunMetrics } from '../api/types'

/** Plain-language explanation of why the safety monitor stopped a run. */
const EXPLANATIONS: Record<string, string> = {
  ground_contact: 'the vehicle reached the ground plane',
  airspeed_low: 'airspeed fell below the safety floor — the wing had stopped flying',
  airspeed_high: 'airspeed exceeded the safety ceiling, which follows a sustained dive',
  altitude_high: 'the vehicle climbed through the altitude ceiling',
  bank_exceeded: 'bank angle went past 80°, beyond any recoverable attitude for this autopilot',
  pitch_exceeded: 'pitch attitude went past 75°',
  diverged: 'the state stopped being finite',
}

/**
 * Shown when a run did not reach its requested duration.
 *
 * A departure is a legitimate result — the slider extremes are genuinely hard for a 25 m/s
 * aircraft — but it has to be unmistakable, because the charts below show a truncated flight
 * and the summary statistics may cover no samples at all.
 */
export function OutcomeBanner({ metrics, duration }: { metrics: RunMetrics; duration: number }) {
  if (metrics.completed) return null
  const why = EXPLANATIONS[metrics.termination_reason]
  return (
    <div
      role="status"
      className="panel"
      style={{ borderColor: '#c9850055', background: '#c985000f', padding: '12px 16px' }}
    >
      <strong style={{ color: 'var(--warn)', fontSize: 13.5 }}>
        Run stopped after {metrics.simulated_time_s.toFixed(0)} s of {duration.toFixed(0)}:{' '}
        {humanise(metrics.termination_reason).toLowerCase()}
      </strong>
      <div style={{ color: 'var(--text-muted)', fontSize: 12.5, marginTop: 3 }}>
        The safety monitor ended the run because {why ?? 'a safety limit was exceeded'}. The
        charts below cover the flight up to that point.
        {metrics.metrics_window_s <= 0 && ' It ended before the settling window the summary '
          + 'statistics are measured over, so those tiles have no samples to report.'}
      </div>
    </div>
  )
}
