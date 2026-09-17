import { useCallback, useMemo, useState } from 'react'

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

/**
 * Robustness under dispersion.
 *
 * The published campaign — 256 trials over mass, inertia, aerodynamic, thrust, initial
 * condition, atmosphere and sensor dispersions — is served precomputed, because it takes
 * about 160 seconds on four threads. Visitors may launch a small live campaign to watch the
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
  const statistic = (name: string) => campaign.statistics.find((s) => s.name === name)
  const crossStat = statistic('rms_cross_track')

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
    <div className="panel">
      <div className="panel-head">
        <h2>Monte-Carlo robustness</h2>
        <span className="badge">{campaign.trials} trials · precomputed</span>
      </div>
      <p className="panel-note">
        Mass and inertia ±8–12%, lift-curve and moment slopes ±12%, thrust ±8%, attitude and
        altitude offsets, wind and turbulence draws, and a sensor-quality multiplier — each
        trial reseeded deterministically from one master seed, so the whole campaign
        reproduces exactly. {campaign.successes} of {campaign.trials} trials completed the
        mission ({(campaign.failure_rate * 100).toFixed(1)}% failure rate) in{' '}
        {seconds(campaign.wall_clock_s)} across {campaign.threads} threads.
      </p>

      <div className="grid-2">
        <Histogram
          values={crossTrack}
          xLabel="cross-track RMS per trial [m]"
          color={seriesColor(0)}
          height={215}
          markers={crossStat ? [
            { value: crossStat.median, label: 'p50' },
            { value: crossStat.p95, label: 'p95' },
          ] : []}
        />
        <ScatterChart
          points={scatter}
          xLabel="dispersed wind speed [m/s]"
          yLabel="cross-track RMS [m]"
          color={seriesColor(0)}
          height={215}
          trend
        />
      </div>

      <div style={{ marginTop: 16 }}>
        <LineChart
          xLabel="time [s]" yLabel="cross-track error [m]" height={200} zeroLine
          bands={[{
            x: envelope.time_s ?? [],
            lower: envelope.cross_track_p05 ?? [],
            upper: envelope.cross_track_p95 ?? [],
            fill: SEMANTIC.band,
          }]}
          series={[
            { label: 'median across trials', x: envelope.time_s ?? [],
              y: envelope.cross_track_p50 ?? [], color: seriesColor(0), width: 1.8 },
            { label: 'worst trial', x: envelope.time_s ?? [], y: envelope.cross_track_max ?? [],
              color: seriesColor(1), width: 1 },
          ]}
          annotation="shaded: 5th–95th percentile"
        />
      </div>

      <div className="grid-2" style={{ marginTop: 16, alignItems: 'start' }}>
        <div>
          <h3 style={{ fontSize: 13, marginBottom: 8 }}>Published campaign</h3>
          <div className="table-scroll">
            <table className="data">
              <thead>
                <tr><th>metric</th><th>mean</th><th>p50</th><th>p95</th><th>max</th></tr>
              </thead>
              <tbody>
                {campaign.statistics
                  .filter((s) => ['rms_cross_track', 'max_cross_track', 'rms_altitude_error',
                    'estimator_position_rmse', 'estimator_attitude_rmse', 'max_bank']
                    .includes(s.name))
                  .map((s) => (
                    <tr key={s.name}>
                      <td>{humanise(s.name)} [{s.unit}]</td>
                      <td>{fixed(s.mean, 2)}</td>
                      <td>{fixed(s.median, 2)}</td>
                      <td>{fixed(s.p95, 2)}</td>
                      <td>{fixed(s.max, 2)}</td>
                    </tr>
                  ))}
              </tbody>
            </table>
          </div>
          {Object.keys(campaign.failure_reasons).length > 0 && (
            <p className="panel-note" style={{ marginTop: 10, marginBottom: 0 }}>
              Failures:{' '}
              {Object.entries(campaign.failure_reasons)
                .map(([reason, count]) => `${count} × ${humanise(reason).toLowerCase()}`)
                .join(', ')}
              . These are real outcomes of the dispersion, not simulation errors — the safety
              monitor stopped each trial and recorded why.
            </p>
          )}
        </div>

        <div>
          <h3 style={{ fontSize: 13, marginBottom: 8 }}>Run a small campaign now</h3>
          <div className="field" style={{ maxWidth: 260 }}>
            <label className="field-label" htmlFor="mc-trials">
              <span>Trials</span>
              <span className="field-value">{trials}</span>
            </label>
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
          </div>
          <button type="button" className="ghost-button" onClick={runLive} disabled={busy}>
            {busy ? `Running ${trials} trials…` : `Run ${trials} dispersed trials`}
          </button>
          <p className="panel-note" style={{ marginTop: 10 }}>
            Capped at {meta.limits.mc_max_trials} trials of{' '}
            {meta.limits.mc_duration_s.toFixed(0)} s so a visitor cannot tie up the server;
            the full campaign above is what the project actually reports.
          </p>
          {error && <div className="status error" role="alert">{error}</div>}
          {live && (
            <div className="table-scroll">
              <table className="data" style={{ marginTop: 6 }}>
                <tbody>
                  <tr>
                    <td>Completed</td>
                    <td>{live.successes} / {live.trials}</td>
                  </tr>
                  <tr>
                    <td>Cross-track RMS, median</td>
                    <td>{fixed(liveStat?.median, 2)} m</td>
                  </tr>
                  <tr>
                    <td>Cross-track RMS, worst</td>
                    <td>{fixed(liveStat?.max, 2)} m</td>
                  </tr>
                  <tr>
                    <td>Server time</td>
                    <td>{seconds(live.server_runtime_s)}{live.cached ? ' (cached)' : ''}</td>
                  </tr>
                </tbody>
              </table>
            </div>
          )}
        </div>
      </div>
    </div>
  )
}
