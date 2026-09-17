import { useCallback, useState } from 'react'

import { DataTable } from './sheet/DataTable'
import { Figure } from './sheet/Figure'
import { LineChart } from './charts/LineChart'
import { api, ApiError } from '../api/client'
import { SEMANTIC } from '../lib/palette'
import { fixed } from '../lib/format'
import type { SimulationRequest, SimulationResponse } from '../api/types'

interface Props {
  request: SimulationRequest
  /** A run already in hand, so the matching control law costs nothing to show. */
  seeded?: SimulationResponse | null
  children?: (running: boolean, run: () => void, ready: boolean) => void
}

const ROWS: {
  key: keyof SimulationResponse['metrics']
  label: string
  unit: string
  decimals: number
}[] = [
  { key: 'rms_cross_track_m', label: 'Cross-track RMS', unit: 'm', decimals: 2 },
  { key: 'max_cross_track_m', label: 'Cross-track peak', unit: 'm', decimals: 1 },
  { key: 'rms_altitude_error_m', label: 'Altitude RMS', unit: 'm', decimals: 2 },
  { key: 'rms_airspeed_error_mps', label: 'Airspeed RMS', unit: 'm/s', decimals: 2 },
  { key: 'max_bank_deg', label: 'Peak bank', unit: 'deg', decimals: 1 },
  { key: 'max_alpha_deg', label: 'Peak angle of attack', unit: 'deg', decimals: 1 },
]

/**
 * The same mission flown by both control laws.
 *
 * Identical seed, wind field and sensor suite, so the difference between the traces is the
 * difference between the designs and nothing else. Identical requests are cached by the
 * service, so re-running a case already shown costs nothing.
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
      setError(caught instanceof ApiError ? caught.message : 'The comparison failed.')
    } finally {
      setBusy(false)
    }
  }, [request])

  const lqr = runs.lqr
  const pid = runs.pid
  const both = Boolean(lqr && pid)
  const cutShort = [lqr, pid].filter((run) => run && run.metrics.metrics_window_s <= 0)

  if (!both) {
    return (
      <div>
        <button type="button" className="action-secondary" onClick={compare} disabled={busy}>
          {busy ? 'Flying both cases…' : 'Fly both control laws'}
        </button>
        <p className="section-note" style={{ marginTop: 12 }}>
          Two 200-second runs at the conditions in the run card, about two seconds each on the
          server. {error && <span style={{ color: 'var(--oxide)' }}>{error}</span>}
        </p>
      </div>
    )
  }

  return (
    <div>
      {cutShort.length > 0 && (
        <div className="notice" style={{ marginBottom: 20 }} role="status">
          <div className="notice-title">Incomparable</div>
          <p>
            {cutShort
              .map((run) => `${run!.request.controller.toUpperCase()} stopped after `
                + `${run!.metrics.simulated_time_s.toFixed(0)} s `
                + `(${run!.metrics.termination_reason.replace(/_/g, ' ')})`)
              .join(' and ')}
            , before the metric window opened. The figures below are not a like-for-like
            comparison at these conditions.
          </p>
        </div>
      )}

      <div className="figure-row">
        <Figure number="15" caption={<>Cross-track error under both control laws, same seed
          and same wind field.</>}>
          <LineChart
            y="cross-track" yUnit="m" zeroLine
            series={[
              { label: 'LQR', x: lqr!.series.t, y: lqr!.series.cross_track, color: SEMANTIC.lqr },
              { label: 'PID', x: pid!.series.t, y: pid!.series.cross_track, color: SEMANTIC.pid },
            ]}
          />
        </Figure>
        <Figure number="16" caption={<>Altitude error. The LQR's longitudinal loop holds
          altitude two to three times more tightly at every wind speed.</>}>
          <LineChart
            y="altitude error" yUnit="m" zeroLine
            series={[
              { label: 'LQR', x: lqr!.series.t, y: lqr!.series.altitude_error, color: SEMANTIC.lqr },
              { label: 'PID', x: pid!.series.t, y: pid!.series.altitude_error, color: SEMANTIC.pid },
            ]}
          />
        </Figure>
      </div>

      <div style={{ marginTop: 26, maxWidth: 680 }}>
        <DataTable caption="Tracking performance, both control laws">
          <thead>
            <tr>
              <th>Metric</th>
              <th>Unit</th>
              <th>LQR</th>
              <th>PID</th>
              <th>Δ</th>
            </tr>
          </thead>
          <tbody>
            {ROWS.map((row) => {
              const a = lqr!.metrics[row.key] as number
              const b = pid!.metrics[row.key] as number
              const delta = b === 0 ? 0 : ((a - b) / Math.abs(b)) * 100
              return (
                <tr key={row.key}>
                  <td>{row.label}</td>
                  <td style={{ color: 'var(--ink-faint)' }}>{row.unit}</td>
                  <td className={a < b ? 'is-best' : undefined}>{fixed(a, row.decimals)}</td>
                  <td className={b < a ? 'is-best' : undefined}>{fixed(b, row.decimals)}</td>
                  <td style={{ color: 'var(--ink-mute)' }}>
                    {delta >= 0 ? '+' : ''}{delta.toFixed(0)}%
                  </td>
                </tr>
              )
            })}
          </tbody>
        </DataTable>
        <p className="section-note" style={{ marginTop: 10 }}>
          Lower is better throughout; the tighter of the two is set in full ink. Δ is the LQR
          relative to the PID.
        </p>
        <button type="button" className="action-secondary" style={{ marginTop: 14 }}
                onClick={compare} disabled={busy}>
          {busy ? 'Flying both cases…' : 'Re-fly at the current conditions'}
        </button>
        {error && <p className="status is-error">{error}</p>}
      </div>
    </div>
  )
}
