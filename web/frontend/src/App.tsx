import { useCallback, useEffect, useMemo, useRef, useState } from 'react'

import { ComparePanel } from './components/ComparePanel'
import { ControlPanel } from './components/ControlPanel'
import { ControlsPanel } from './components/ControlsPanel'
import { EstimatorPanel } from './components/EstimatorPanel'
import { GroundTrack } from './components/GroundTrack'
import { Header } from './components/Header'
import { ModesPanel } from './components/ModesPanel'
import { OutcomeBanner } from './components/OutcomeBanner'
import { MonteCarloPanel } from './components/MonteCarloPanel'
import { StatTiles } from './components/StatTiles'
import { StatesPanel } from './components/StatesPanel'
import { Trajectory3D } from './components/Trajectory3D'
import { TrimPanel } from './components/TrimPanel'
import { api, ApiError } from './api/client'
import { seconds } from './lib/format'
import type {
  Meta, PrecomputedModes, PrecomputedMonteCarlo, SimulationRequest, SimulationResponse,
} from './api/types'

const REPOSITORY = 'https://github.com/nathan-w123/aether6'

export default function App() {
  const [meta, setMeta] = useState<Meta | null>(null)
  const [campaign, setCampaign] = useState<PrecomputedMonteCarlo | null>(null)
  const [modes, setModes] = useState<PrecomputedModes | null>(null)
  const [bootError, setBootError] = useState<string | null>(null)

  const [request, setRequest] = useState<SimulationRequest | null>(null)
  const [activePreset, setActivePreset] = useState<string | null>(null)
  const [run, setRun] = useState<SimulationResponse | null>(null)
  const [running, setRunning] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const inFlight = useRef<AbortController | null>(null)

  // --- boot: fetch the metadata and the precomputed panels, then fly the default preset ---
  useEffect(() => {
    let cancelled = false
    ;(async () => {
      try {
        const [loadedMeta, loadedCampaign, loadedModes] = await Promise.all([
          api.meta(),
          api.precomputedMonteCarlo().catch(() => null),
          api.precomputedModes().catch(() => null),
        ])
        if (cancelled) return
        setMeta(loadedMeta)
        setCampaign(loadedCampaign)
        setModes(loadedModes)
        const preset = loadedMeta.presets.find((p) => p.id === 'nominal') ?? loadedMeta.presets[0]
        setRequest(preset.request)
        setActivePreset(preset.id)
      } catch (caught) {
        if (!cancelled) {
          setBootError(caught instanceof ApiError
            ? caught.message : 'Could not load the simulation service.')
        }
      }
    })()
    return () => { cancelled = true }
  }, [])

  const execute = useCallback(async (body: SimulationRequest) => {
    inFlight.current?.abort()
    const controller = new AbortController()
    inFlight.current = controller
    setRunning(true)
    setError(null)
    try {
      const response = await api.simulate(body, controller.signal)
      if (!controller.signal.aborted) setRun(response)
    } catch (caught) {
      if (controller.signal.aborted) return
      setError(caught instanceof ApiError ? caught.message : 'The simulation failed.')
    } finally {
      if (inFlight.current === controller) {
        inFlight.current = null
        setRunning(false)
      }
    }
  }, [])

  // The page is never empty: the default preset flies as soon as the metadata arrives.
  const booted = useRef(false)
  useEffect(() => {
    if (!request || booted.current) return
    booted.current = true
    void execute(request)
  }, [request, execute])

  const onPreset = useCallback((id: string) => {
    const preset = meta?.presets.find((p) => p.id === id)
    if (!preset) return
    setActivePreset(id)
    setRequest(preset.request)
    void execute(preset.request)
  }, [meta, execute])

  const onChange = useCallback((patch: Partial<SimulationRequest>) => {
    setActivePreset(null)
    setRequest((current) => (current ? { ...current, ...patch } : current))
  }, [])

  const statusLine = useMemo(() => {
    if (!run) return null
    const source = run.cached ? 'served from cache' : `${seconds(run.server_runtime_s)} on the server`
    return `${run.metrics.simulated_time_s.toFixed(0)} s simulated · ${source}`
  }, [run])

  if (bootError) {
    return (
      <div style={{ padding: 48, maxWidth: 640, margin: '0 auto' }}>
        <h1 style={{ fontSize: 20, marginBottom: 8 }}>Aether-6</h1>
        <p className="status error">{bootError}</p>
      </div>
    )
  }

  if (!meta || !request) {
    return (
      <div style={{ padding: 48, maxWidth: 1200, margin: '0 auto' }}>
        <div className="skeleton" style={{ height: 32, width: 220, marginBottom: 18 }} />
        <div className="skeleton" style={{ height: 380 }} />
      </div>
    )
  }

  const showEstimate = request.feedback === 'estimate'

  return (
    <>
      <Header version={meta.version} repositoryUrl={REPOSITORY} />
      <div className="app">
        <aside className="sidebar">
          <ControlPanel
            meta={meta}
            request={request}
            activePreset={activePreset}
            running={running}
            error={error}
            statusLine={statusLine}
            onPreset={onPreset}
            onChange={onChange}
            onRun={() => void execute(request)}
          />
        </aside>

        <main className="main">
          {run ? (
            <>
              <OutcomeBanner metrics={run.metrics} duration={meta.limits.duration_s} />
              <StatTiles metrics={run.metrics} trim={run.trim} feedback={request.feedback} />

              <div className="panel">
                <div className="panel-head">
                  <h2>Trajectory</h2>
                  <span className="badge">
                    {run.request.controller.toUpperCase()} ·{' '}
                    {run.request.feedback === 'truth' ? 'true states' : 'EKF estimate'} ·{' '}
                    {run.request.wind_speed.toFixed(1)} m/s wind
                  </span>
                </div>
                <p className="panel-note">
                  Drag to orbit, scroll to zoom. The glyph's attitude is the simulated
                  quaternion, so the bank into each turn and the pitch on each climb leg are
                  the vehicle's own.
                </p>
                <div className="grid-2">
                  <Trajectory3D
                    series={run.series}
                    waypoints={run.waypoints}
                    showEstimate={showEstimate}
                    height={420}
                  />
                  <GroundTrack
                    series={run.series}
                    waypoints={run.waypoints}
                    showEstimate={showEstimate}
                    height={420}
                  />
                </div>
              </div>

              <StatesPanel series={run.series} />
              <ControlsPanel series={run.series} />
              <EstimatorPanel series={run.series} metrics={run.metrics} inLoop={showEstimate} />

              <div className="grid-2" style={{ alignItems: 'start' }}>
                <TrimPanel
                  trim={run.trim}
                  request={run.request}
                  serverRuntime={run.server_runtime_s}
                  cached={run.cached}
                />
                <div className="panel">
                  <div className="panel-head"><h2>How this page works</h2></div>
                  <p className="panel-note" style={{ marginBottom: 10 }}>
                    Each run generates a scenario from the validated slider values, executes the
                    compiled <code>aether_sim</code> binary in a fresh temporary directory under a
                    wall-clock limit, parses its CSV and JSON output, deletes the directory and
                    returns the series you see. No text you type reaches a shell, a filename or a
                    configuration file.
                  </p>
                  <div className="table-scroll">
                    <table className="data">
                      <tbody>
                        <tr><td>Simulated duration</td><td>{meta.limits.duration_s.toFixed(0)} s</td></tr>
                        <tr><td>Integrator step</td><td>{(meta.limits.dt_s * 1000).toFixed(0)} ms (RK4)</td></tr>
                        <tr><td>Control rate</td><td>100 Hz</td></tr>
                        <tr><td>Samples returned</td><td>{meta.limits.series_points}</td></tr>
                        <tr><td>Concurrent runs allowed</td><td>{meta.limits.max_concurrency}</td></tr>
                        <tr><td>Run time limit</td><td>{meta.limits.simulate_timeout_s.toFixed(0)} s</td></tr>
                      </tbody>
                    </table>
                  </div>
                </div>
              </div>

              <ComparePanel request={request} seeded={run} />
            </>
          ) : (
            <div className="panel">
              <div className="skeleton" style={{ height: 420 }} />
            </div>
          )}

          {campaign?.available && (
            <MonteCarloPanel meta={meta} campaign={campaign} request={request} />
          )}
          {modes?.available && <ModesPanel data={modes} />}

          <footer style={{ color: 'var(--text-dim)', fontSize: 12, padding: '4px 2px 0' }}>
            Aether-6 uses a synthetic small-UAV parameter set, representative of the class but
            not identified from flight test. See{' '}
            <a href={`${REPOSITORY}/blob/main/docs/limitations.md`} target="_blank"
               rel="noreferrer noopener">docs/limitations.md</a>{' '}
            for what that does and does not support.
          </footer>
        </main>
      </div>
    </>
  )
}
