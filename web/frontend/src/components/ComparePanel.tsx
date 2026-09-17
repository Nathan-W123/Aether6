import { useCallback, useState } from 'react'

import { LineChart } from './charts/LineChart'
import { api, ApiError } from '../api/client'
import { SEMANTIC } from '../lib/palette'
import { fixed } from '../lib/format'
import type { SimulationRequest, SimulationResponse } from '../api/types'

interface Props {
  request: SimulationRequest
  /** A run already in hand, so the matching controller costs nothing to show. */
  seeded?: SimulationResponse | null
}

const ROWS: { key: keyof SimulationResponse['metrics']; label: string; decimals: number;
  unit: string; lowerIsBetter: boolean }[] = [
  { key: 'rms_cross_track_m', label: 'Cross-track RMS', decimals: 2, unit: 'm', lowerIsBetter: true },
  { key: 'max_cross_track_m', label: 'Cross-track peak', decimals: 1, unit: 'm', lowerIsBetter: true },
  { key: 'rms_altitude_error_m', label: 'Altitude RMS', decimals: 2, unit: 'm', lowerIsBetter: true },
  { key: 'rms_airspeed_error_mps', label: 'Airspeed RMS', decimals: 2, unit: 'm/s', lowerIsBetter: true },
  { key: 'max_bank_deg', label: 'Peak bank', decimals: 1, unit: '°', lowerIsBetter: true },
  { key: 'max_alpha_deg', label: 'Peak α', decimals: 1, unit: '°', lowerIsBetter: true },
]

/**
 * The same mission flown by both control laws.
 *
 * Both runs use the identical seed, wind field and sensor suite, so the difference between
 * the traces is the difference between the designs and nothing else. Identical requests are
 * cached server-side, so re-opening this panel for a scenario already shown is free.
 */
export function ComparePanel({ request, seeded }: Props) {
  const [runs, setRuns] = useState<Record<string, SimulationResponse>>(() =>
    seeded ? { [seeded.request.controller]: seeded } : {})
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<string | null>(null)

  const compare = useCallback(async () => {
    setBusy(true)
    setError(null)
    try {
      // Sequential, not parallel: the service allows a small number of concurrent runs and
      // this panel should never be the thing that exhausts them.
      const next: Record<string, SimulationResponse> = {}
      for (const controller of ['lqr', 'pid'] as const) {
        next[controller] = await api.simulate({ ...request, controller })
      }
      setRuns(next)
    } catch (caught) {
      setError(caught instanceof ApiError ? caught.message : 'Comparison failed.')
    } finally {
      setBusy(false)
    }
  }, [request])

  const lqr = runs.lqr
  const pid = runs.pid
  const both = Boolean(lqr && pid)
  // A run the safety monitor cut short has no metric samples, so its column would be a row
  // of zeros that reads as flawless tracking. Say what happened instead.
  const cutShort = [lqr, pid].filter((run) => run && run.metrics.metrics_window_s <= 0)

  return (
    <div className="panel">
      <div className="panel-head">
        <h2>LQR against classical PID</h2>
        <button type="button" className="ghost-button" onClick={compare} disabled={busy}>
          {busy ? 'Running both…' : both ? 'Re-run comparison' : 'Run both controllers'}
        </button>
      </div>
      <p className="panel-note">
        Servo-LQR with integral augmentation, designed by solving the algebraic Riccati
        equation on the linearised model, against cascaded PID loops tuned by successive loop
        closure. Same seed, same wind, same sensors — at {fixed(request.wind_speed, 1)} m/s wind
        and {fixed(request.airspeed, 1)} m/s.
      </p>

      {error && <div className="status error" role="alert">{error}</div>}

      {both && cutShort.length > 0 && (
        <p className="status error" role="status" style={{ marginBottom: 10 }}>
          {cutShort
            .map((run) => `${run!.request.controller.toUpperCase()} stopped after `
              + `${run!.metrics.simulated_time_s.toFixed(0)} s `
              + `(${run!.metrics.termination_reason.replace(/_/g, ' ')})`)
            .join(' and ')}
          , before the metric window opened — the tracking figures below are not comparable.
        </p>
      )}

      {both ? (
        <>
          <div className="grid-2">
            <LineChart
              xLabel="time [s]" yLabel="cross-track error [m]" height={190} zeroLine
              series={[
                { label: 'LQR', x: lqr!.series.t, y: lqr!.series.cross_track, color: SEMANTIC.lqr },
                { label: 'PID', x: pid!.series.t, y: pid!.series.cross_track, color: SEMANTIC.pid },
              ]}
            />
            <LineChart
              xLabel="time [s]" yLabel="altitude error [m]" height={190} zeroLine
              series={[
                { label: 'LQR', x: lqr!.series.t, y: lqr!.series.altitude_error, color: SEMANTIC.lqr },
                { label: 'PID', x: pid!.series.t, y: pid!.series.altitude_error, color: SEMANTIC.pid },
              ]}
            />
          </div>
          <table className="data" style={{ marginTop: 14 }}>
            <thead>
              <tr>
                <th>metric</th>
                <th><span style={{ color: SEMANTIC.lqr }}>LQR</span></th>
                <th><span style={{ color: SEMANTIC.pid }}>PID</span></th>
                <th>difference</th>
              </tr>
            </thead>
            <tbody>
              {ROWS.map((row) => {
                const a = lqr!.metrics[row.key] as number
                const b = pid!.metrics[row.key] as number
                const delta = b === 0 ? 0 : ((a - b) / Math.abs(b)) * 100
                const better = row.lowerIsBetter ? a < b : a > b
                return (
                  <tr key={row.key}>
                    <td>{row.label}</td>
                    <td style={{ color: better ? 'var(--good)' : undefined }}>
                      {fixed(a, row.decimals)} {row.unit}
                    </td>
                    <td style={{ color: !better ? 'var(--good)' : undefined }}>
                      {fixed(b, row.decimals)} {row.unit}
                    </td>
                    <td style={{ color: 'var(--text-muted)' }}>
                      {delta >= 0 ? '+' : ''}{delta.toFixed(0)}%
                    </td>
                  </tr>
                )
              })}
            </tbody>
          </table>
        </>
      ) : (
        <p style={{ color: 'var(--text-muted)', fontSize: 13, margin: '8px 0 0' }}>
          Two {fixed(200, 0)}-second runs, roughly two seconds each on the server.
        </p>
      )}
    </div>
  )
}
