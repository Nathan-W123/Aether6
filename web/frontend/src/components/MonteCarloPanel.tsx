import { useCallback, useMemo, useState } from 'react'

import { DataTable } from './sheet/DataTable'
import { Figure } from './sheet/Figure'
import { Histogram } from './charts/Histogram'
import { LineChart } from './charts/LineChart'
import { ScatterChart } from './charts/ScatterChart'
import { api, ApiError } from '../api/client'
import { SEMANTIC, seriesColor } from '../lib/palette'
import { fixed, humanise, seconds } from '../lib/format'
import type { Meta, MonteCarloResponse, PrecomputedMonteCarlo, SimulationRequest } from '../api/types'

interface Props {
  meta: Meta
  campaign: PrecomputedMonteCarlo
  request: SimulationRequest
}

const REPORTED = [
  'rms_cross_track', 'max_cross_track', 'rms_altitude_error',
  'estimator_position_rmse', 'estimator_attitude_rmse', 'max_bank',
]

/**
 * Robustness under dispersion.
 *
 * The published campaign — 256 trials over mass, inertia, aerodynamic, thrust, initial
 * condition, atmosphere and sensor dispersions — is bundled with the report because it takes
 * about 160 seconds on four threads. A visitor may launch a small live campaign to watch the
 * same machinery run, capped well below the published size.
 */
export function MonteCarloPanel({ meta, campaign, request }: Props) {
  const [trials, setTrials] = useState(meta.limits.mc_min_trials)
  const [live, setLive] = useState<MonteCarloResponse | null>(null)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<string | null>(null)

  const table = campaign.trials_table
  const envelope = campaign.envelope
  const crossTrack = useMemo(() => table.rms_cross_track_m ?? [], [table])
  const crossStat = campaign.statistics.find((s) => s.name === 'rms_cross_track')

  const scatter = useMemo(() => {
    const wind = table.wind_speed_mps ?? []
    const ok = table.ok ?? []
    return crossTrack.map((value, i) => ({
      x: wind[i] ?? 0,
      y: value,
      failed: (ok[i] ?? 1) < 0.5,
      title: `trial ${i}: ${fixed(wind[i], 1)} m/s wind, ${fixed(value, 1)} m RMS`,
    }))
  }, [table, crossTrack])

  const runLive = useCallback(async () => {
    setBusy(true)
    setError(null)
    try {
      setLive(await api.montecarlo({
        trials,
        controller: request.controller,
        wind_speed: request.wind_speed,
        airspeed: request.airspeed,
        seed: request.seed,
      }))
    } catch (caught) {
      setError(caught instanceof ApiError ? caught.message : 'The campaign failed.')
    } finally {
      setBusy(false)
    }
  }, [trials, request])

  const liveStat = live?.statistics.find((s) => s.name === 'rms_cross_track')

  return (
    <div>
      <div className="figure-row">
        <Figure number="19" caption={<>Distribution of per-trial cross-track RMS across the
          campaign. Median <b>{fixed(crossStat?.median, 1)} m</b>, 95th percentile{' '}
          <b>{fixed(crossStat?.p95, 1)} m</b>.</>}>
          <Histogram
            values={crossTrack}
            x="cross-track RMS per trial" xUnit="m"
            color={seriesColor(0)}
            markers={crossStat
              ? [{ value: crossStat.median, label: 'p50' }, { value: crossStat.p95, label: 'p95' }]
              : []}
          />
        </Figure>

        <Figure number="20" caption={<>Cross-track RMS against the wind speed each trial drew.
          One mark per trial; open rings are the trials a safety limit stopped.</>}>
          <ScatterChart
            points={scatter}
            x="dispersed wind speed" xUnit="m/s"
            y="cross-track RMS" yUnit="m"
            color={seriesColor(0)}
            trend
          />
        </Figure>
      </div>

      <div style={{ marginTop: 26 }}>
        <Figure number="21" caption={<>Cross-track error against time across every recorded
          trajectory: the median trial, the 5th-to-95th percentile band, and the single worst
          trial at each instant.</>}>
          <LineChart
            y="cross-track" yUnit="m" zeroLine height={210}
            bands={[{
              x: envelope.time_s ?? [],
              lower: envelope.cross_track_p05 ?? [],
              upper: envelope.cross_track_p95 ?? [],
              fill: SEMANTIC.band,
            }]}
            series={[
              { label: 'median trial', x: envelope.time_s ?? [],
                y: envelope.cross_track_p50 ?? [], color: seriesColor(0), width: 1.8 },
              { label: 'worst trial', x: envelope.time_s ?? [],
                y: envelope.cross_track_max ?? [], color: seriesColor(1), width: 1.2 },
            ]}
          />
        </Figure>
      </div>

      <div className="figure-row" style={{ marginTop: 30, alignItems: 'start' }}>
        <div>
          <DataTable caption={`Published campaign · ${campaign.trials} trials`}>
            <thead>
              <tr>
                <th>Metric</th><th>Unit</th><th>Mean</th><th>p50</th><th>p95</th><th>Max</th>
              </tr>
            </thead>
            <tbody>
              {campaign.statistics.filter((s) => REPORTED.includes(s.name)).map((s) => (
                <tr key={s.name}>
                  <td>{humanise(s.name)}</td>
                  <td style={{ color: 'var(--ink-faint)' }}>{s.unit}</td>
                  <td>{fixed(s.mean, 2)}</td>
                  <td>{fixed(s.median, 2)}</td>
                  <td>{fixed(s.p95, 2)}</td>
                  <td>{fixed(s.max, 2)}</td>
                </tr>
              ))}
            </tbody>
          </DataTable>
          {Object.keys(campaign.failure_reasons).length > 0 && (
            <p className="section-note" style={{ marginTop: 12 }}>
              {campaign.successes} of {campaign.trials} trials completed the mission, a{' '}
              {(campaign.failure_rate * 100).toFixed(1)}% failure rate:{' '}
              {Object.entries(campaign.failure_reasons)
                .map(([reason, count]) => `${count} × ${humanise(reason).toLowerCase()}`)
                .join(', ')}
              . These are outcomes of the dispersion, not simulation errors — the safety
              monitor stopped each trial and recorded why.
            </p>
          )}
        </div>

        <div>
          <p className="label" style={{ marginBottom: 10 }}>Fly a campaign now</p>
          <div className="field-row" style={{ borderTop: '1px solid var(--rule)', maxWidth: 300 }}>
            <div className="field-line">
              <label className="label" htmlFor="mc-trials">Trials</label>
              <span className="field-readout">{trials}</span>
            </div>
            <input
              id="mc-trials"
              type="range"
              min={meta.limits.mc_min_trials}
              max={meta.limits.mc_max_trials}
              step={1}
              value={trials}
              disabled={busy}
              onChange={(event) => setTrials(Number(event.target.value))}
            />
            <div className="scale-ends">
              <span>{meta.limits.mc_min_trials}</span>
              <span>{meta.limits.mc_max_trials}</span>
            </div>
          </div>
          <button type="button" className="action-secondary" style={{ marginTop: 14 }}
                  onClick={runLive} disabled={busy}>
            {busy ? `Flying ${trials} trials…` : `Fly ${trials} dispersed trials`}
          </button>
          <p className="section-note" style={{ marginTop: 12 }}>
            Capped at {meta.limits.mc_max_trials} trials of{' '}
            {meta.limits.mc_duration_s.toFixed(0)} s so a visitor cannot tie up the server. The
            published campaign above is what the project reports.
          </p>
          {error && <p className="status is-error">{error}</p>}
          {live && (
            <div style={{ marginTop: 6, maxWidth: 340 }}>
              <DataTable caption="This campaign">
                <tbody>
                  <tr><td>Completed</td><td>{live.successes} / {live.trials}</td></tr>
                  <tr><td>Cross-track RMS, median</td><td>{fixed(liveStat?.median, 2)} m</td></tr>
                  <tr><td>Cross-track RMS, worst</td><td>{fixed(liveStat?.max, 2)} m</td></tr>
                  <tr>
                    <td>Server time</td>
                    <td>{seconds(live.server_runtime_s)}{live.cached ? ' (cached)' : ''}</td>
                  </tr>
                </tbody>
              </DataTable>
            </div>
          )}
        </div>
      </div>
    </div>
  )
}
