import { humanise } from '../lib/format'
import type { RunMetrics } from '../api/types'

const EXPLANATIONS: Record<string, string> = {
  ground_contact: 'the vehicle reached the ground plane',
  airspeed_low: 'airspeed fell below the safety floor — the wing had stopped flying',
  airspeed_high: 'airspeed passed the safety ceiling, which follows a sustained dive',
  altitude_high: 'the vehicle climbed through the altitude ceiling',
  bank_exceeded: 'bank angle passed 80°, beyond a recoverable attitude for this autopilot',
  pitch_exceeded: 'pitch attitude passed 75°',
  diverged: 'the state stopped being finite',
}

/**
 * Printed when a run did not reach its requested duration.
 *
 * A departure is a real result — the extremes of the test matrix are genuinely hard for a
 * 25 m/s aircraft — but the figures below then cover a truncated flight, and the summary
 * statistics may cover no samples at all, so it has to be stated before either is read.
 */
export function OutcomeNotice({ metrics, duration }: { metrics: RunMetrics; duration: number }) {
  if (metrics.completed) return null
  return (
    <div className="notice" role="status">
      <div className="notice-title">
        Run stopped at {metrics.simulated_time_s.toFixed(0)} s of {duration.toFixed(0)} ·{' '}
        {humanise(metrics.termination_reason).toLowerCase()}
      </div>
      <p>
        The safety monitor ended the run because{' '}
        {EXPLANATIONS[metrics.termination_reason] ?? 'a safety limit was exceeded'}. Every
        figure below covers the flight up to that point.
        {metrics.metrics_window_s <= 0 && ' It ended before the settling window the summary '
          + 'statistics are measured over, so those cells have no samples to report.'}
      </p>
    </div>
  )
}
