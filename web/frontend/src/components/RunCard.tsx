import type { Bound, ControllerId, FeedbackId, Meta, SimulationRequest } from '../api/types'

interface Props {
  meta: Meta
  request: SimulationRequest
  activePreset: string | null
  running: boolean
  error: string | null
  statusLine: string | null
  onPreset: (id: string) => void
  onChange: (patch: Partial<SimulationRequest>) => void
  onRun: () => void
}

type SliderKey = 'wind_speed' | 'target_altitude' | 'airspeed' | 'sensor_noise'

const SLIDERS: SliderKey[] = ['wind_speed', 'target_altitude', 'airspeed', 'sensor_noise']

function reading(key: SliderKey, value: number, bound: Bound) {
  if (key === 'sensor_noise') return { value: value.toFixed(2), unit: '× nominal' }
  return { value: value.toFixed(bound.step >= 1 ? 0 : 1), unit: bound.unit }
}

/**
 * The run card: the conditions this report was flown under.
 *
 * Six controls, not the ninety-odd values in the scenario file. Each carries its range printed
 * beneath the scale, the way a test card states the band a parameter was swept over, and the
 * bounds come from the service so a control can never set a value the simulator would refuse.
 */
export function RunCard({
  meta, request, activePreset, running, error, statusLine, onPreset, onChange, onRun,
}: Props) {
  return (
    <>
      <div className="card">
        <h2 className="card-title">Test matrix</h2>
        {meta.presets.map((preset, index) => (
          <button
            key={preset.id}
            type="button"
            className="preset"
            aria-pressed={activePreset === preset.id}
            disabled={running}
            onClick={() => onPreset(preset.id)}
          >
            <span className="preset-index">{String(index + 1).padStart(2, '0')}</span>
            <span>
              <span className="preset-name">{preset.label}</span>
              <span className="preset-detail">{preset.tagline}</span>
            </span>
          </button>
        ))}
      </div>

      <div className="card">
        <h2 className="card-title">Conditions</h2>

        {SLIDERS.map((key) => {
          const bound = meta.bounds[key]
          const read = reading(key, request[key], bound)
          return (
            <div className="field-row" key={key}>
              <div className="field-line">
                <label className="label" htmlFor={`slider-${key}`}>{bound.label}</label>
                <span className="field-readout">
                  {read.value}<span className="unit"> {read.unit}</span>
                </span>
              </div>
              <input
                id={`slider-${key}`}
                type="range"
                min={bound.min}
                max={bound.max}
                step={bound.step}
                value={request[key]}
                disabled={running}
                onChange={(event) =>
                  onChange({ [key]: Number(event.target.value) } as Partial<SimulationRequest>)}
              />
              <div className="scale-ends">
                <span>{bound.min.toFixed(bound.step >= 1 ? 0 : 2)}</span>
                <span>{bound.max.toFixed(bound.step >= 1 ? 0 : 2)}</span>
              </div>
            </div>
          )
        })}

        <div className="field-row">
          <div className="field-line">
            <span className="label">Control law</span>
          </div>
          <div className="switch">
            {meta.controllers.map((controller) => (
              <button
                key={controller.id}
                type="button"
                aria-pressed={request.controller === controller.id}
                disabled={running}
                title={controller.detail}
                onClick={() => onChange({ controller: controller.id as ControllerId })}
              >
                {controller.label}
              </button>
            ))}
          </div>
        </div>

        <div className="field-row">
          <div className="field-line">
            <span className="label">Controller input</span>
          </div>
          <div className="switch">
            {meta.feedback.map((feedback) => (
              <button
                key={feedback.id}
                type="button"
                aria-pressed={request.feedback === feedback.id}
                disabled={running}
                title={feedback.detail}
                onClick={() => onChange({ feedback: feedback.id as FeedbackId })}
              >
                {feedback.id === 'estimate' ? 'EKF' : 'Truth'}
              </button>
            ))}
          </div>
        </div>

        <div style={{ borderTop: '1px solid var(--rule)', paddingTop: 14, marginTop: 2 }}>
          <button type="button" className="action" onClick={onRun} disabled={running}>
            {running ? 'Integrating…' : 'Fly this case'}
          </button>
          <p className={`status${error ? ' is-error' : ''}`} role="status">
            {error ?? statusLine
              ?? `${meta.limits.duration_s.toFixed(0)} s of flight, integrated at `
                 + `${(1 / meta.limits.dt_s).toFixed(0)} Hz on the server.`}
          </p>
        </div>
      </div>
    </>
  )
}
