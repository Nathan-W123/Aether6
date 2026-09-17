import { useCallback, useEffect, useMemo, useRef, useState } from 'react'

import { ComparePanel } from './components/ComparePanel'
import { ControlsPanel } from './components/ControlsPanel'
import { EstimatorPanel } from './components/EstimatorPanel'
import { GroundTrack } from './components/GroundTrack'
import { Masthead } from './components/Masthead'
import { ModesPanel } from './components/ModesPanel'
import { MonteCarloPanel } from './components/MonteCarloPanel'
import { OutcomeNotice } from './components/OutcomeNotice'
import { RunCard } from './components/RunCard'
import { RunSummary } from './components/RunSummary'
import { StatesPanel } from './components/StatesPanel'
import { TitleBlock } from './components/TitleBlock'
import { Trajectory3D } from './components/Trajectory3D'
import { TrimPanel } from './components/TrimPanel'
import { Figure } from './components/sheet/Figure'
import { Section } from './components/sheet/Section'
import { api, ApiError } from './api/client'
import { fixed, seconds } from './lib/format'
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
            ? caught.message : 'Could not reach the simulation service.')
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

  // The sheet is never blank: the nominal case flies as soon as the metadata arrives.
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
    return `${run.metrics.simulated_time_s.toFixed(0)} s simulated · `
      + (run.cached ? 'served from cache' : `${seconds(run.server_runtime_s)} on the server`)
  }, [run])

  if (bootError) {
    return (
      <div className="sheet">
        <header className="masthead">
          <div className="masthead-rule" />
          <h1 className="masthead-title">Aether-6</h1>
        </header>
        <p className="status is-error" style={{ fontSize: 15 }}>{bootError}</p>
      </div>
    )
  }

  if (!meta || !request) {
    return (
      <div className="sheet">
        <header className="masthead">
          <div className="masthead-rule" />
          <h1 className="masthead-title">Aether-6<br />Flight Test</h1>
        </header>
        <div className="placeholder" style={{ height: 420, marginTop: 30 }} />
      </div>
    )
  }

  const showEstimate = request.feedback === 'estimate'

  return (
    <div className="sheet">
      <Masthead version={meta.version} repositoryUrl={REPOSITORY} run={run} />

      <div className="body-grid">
        <aside className="rail">
          <RunCard
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

        <main className="column">
          {run ? (
            <>
              <Section
                number="1"
                title="Run summary"
                note={<>The case in the run card, integrated by <code>aether_sim</code> at{' '}
                  {(1 / meta.limits.dt_s).toFixed(0)} Hz for {meta.limits.duration_s.toFixed(0)}{' '}
                  seconds of flight. The tracking and estimator statistics are measured over the{' '}
                  {run.metrics.metrics_window_s > 0
                    ? `${run.metrics.metrics_window_s.toFixed(0)} s`
                    : 'window'} that follows a settling period.</>}
              >
                <OutcomeNotice metrics={run.metrics} duration={meta.limits.duration_s} />
                <div style={{ marginTop: run.metrics.completed ? 0 : 20 }}>
                  <RunSummary metrics={run.metrics} trim={run.trim} feedback={request.feedback} />
                </div>
              </Section>

              <Section
                number="2"
                title="Trajectory"
                note={<>The flown path against the commanded circuit. The glyph's attitude is
                  the logged quaternion, so the bank into each turn and the pitch on each climb
                  leg are the vehicle's own. Drag to orbit, scroll to zoom.</>}
              >
                <div className="figure-row">
                  <Figure number="1" bare caption={<>Flight path in three dimensions, with the
                    ground projection beneath it and the six waypoints marked.</>}>
                    <Trajectory3D
                      series={run.series}
                      waypoints={run.waypoints}
                      showEstimate={showEstimate}
                      height={392}
                    />
                  </Figure>
                  <Figure number="2" bare caption={<>Plan view, equal scale on both axes.
                    Cross-track RMS <b>{fixed(run.metrics.rms_cross_track_m, 1)} m</b> against a
                    circuit {fixed(1.2, 1)} km across.</>}>
                    <GroundTrack
                      series={run.series}
                      waypoints={run.waypoints}
                      showEstimate={showEstimate}
                      height={392}
                    />
                  </Figure>
                </div>
              </Section>

              <Section
                number="3"
                title="Vehicle states"
                note="Truth from the integrator, not the filter's view of it. Dashed traces are
                      the guidance commands the inner loops are tracking."
              >
                <StatesPanel series={run.series} />
              </Section>

              <Section
                number="4"
                title="Control activity"
                note="Actuator states after the first-order servo models and the position and
                      rate limits, so these are the deflections the aerodynamics received."
              >
                <ControlsPanel series={run.series} />
              </Section>

              <Section
                number="5"
                title="Navigation"
                note={<>An 18-state error-state EKF on a 200 Hz IMU, 5 Hz GNSS, 20 Hz barometer,
                  50 Hz magnetometer and 50 Hz pitot, estimating position, velocity, attitude,
                  both IMU biases, the horizontal wind and the barometer bias.{' '}
                  {showEstimate
                    ? 'The control law is closed on this estimate, so its errors are inside the loop.'
                    : 'The filter is running but the control law is flying on true states, which separates estimation error from control error.'}</>}
              >
                <EstimatorPanel series={run.series} />
              </Section>

              <Section
                number="6"
                title="Trim and control law"
                note="A damped Levenberg–Marquardt solve for the state and actuator settings
                      that make every body-frame acceleration vanish, with one-sided penalties
                      keeping the answer inside the actuator box."
              >
                <TrimPanel trim={run.trim} request={run.request} />
                <div style={{ marginTop: 30 }}>
                  <p className="label" style={{ marginBottom: 12 }}>
                    Servo-LQR against classical PID
                  </p>
                  <ComparePanel request={request} seeded={run} />
                </div>
              </Section>
            </>
          ) : (
            <div className="placeholder" style={{ height: 480 }} />
          )}

          {modes?.available && (
            <Section
              number="7"
              title="Linearised dynamics"
              note="A 12×12 Jacobian taken by central differences in the quaternion tangent
                    space about the reference trim, split into longitudinal and
                    lateral-directional blocks and classified by eigenvalue."
              aside={<span className="label">Bundled · reference trim</span>}
            >
              <ModesPanel data={modes} />
            </Section>
          )}

          {campaign?.available && (
            <Section
              number="8"
              title="Robustness"
              note={<>Mass and inertia ±8–12%, lift-curve and moment slopes ±12%, thrust ±8%,
                attitude and altitude offsets, wind and turbulence draws and a sensor-quality
                multiplier. Each trial is reseeded deterministically from one master seed, so
                the campaign reproduces exactly.</>}
              aside={<span className="label">Bundled · {campaign.trials} trials</span>}
            >
              <MonteCarloPanel meta={meta} campaign={campaign} request={request} />
            </Section>
          )}

          <Section number="9" title="Reproduction">
            <TitleBlock meta={meta} run={run} repositoryUrl={REPOSITORY} />
          </Section>
        </main>
      </div>
    </div>
  )
}
