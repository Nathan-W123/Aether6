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

function formatValue(key: SliderKey, value: number, bound: Bound): string {
  if (key === 'sensor_noise') return `${value.toFixed(2)}× nominal`
  const decimals = bound.step >= 1 ? 0 : 1
  return `${value.toFixed(decimals)} ${bound.unit}`
}

/**
 * The whole input surface of the dashboard.
 *
 * Four sliders and two toggles, not the ninety-odd values in the scenario YAML. The bounds
 * come from `/api/meta`, so a control can never offer a value the service would reject.
 */
export function ControlPanel({
  meta, request, activePreset, running, error, statusLine, onPreset, onChange, onRun,
}: Props) {
  return (
    <div className="panel">
      <div className="panel-head">
        <h2>Scenario</h2>
        {running && <span className="badge warn">running</span>}
      </div>

      <div className="preset-list" style={{ marginBottom: 18 }}>
        {meta.presets.map((preset) => (
          <button
            key={preset.id}
            type="button"
            className="preset"
            aria-pressed={activePreset === preset.id}
            disabled={running}
            title={preset.detail}
            onClick={() => onPreset(preset.id)}
          >
            <strong>{preset.label}</strong>
            <span>{preset.tagline}</span>
          </button>
        ))}
      </div>

      {SLIDERS.map((key) => {
        const bound = meta.bounds[key]
        return (
          <div className="field" key={key}>
            <label className="field-label" htmlFor={`slider-${key}`}>
              <span>{bound.label}</span>
              <span className="field-value">{formatValue(key, request[key], bound)}</span>
            </label>
            <input
              id={`slider-${key}`}
              type="range"
              min={bound.min}
              max={bound.max}
              step={bound.step}
              value={request[key]}
              disabled={running}
              onChange={(event) => onChange({ [key]: Number(event.target.value) } as
                Partial<SimulationRequest>)}
            />
          </div>
        )
      })}

      <div className="field">
        <span className="field-label"><span>Control law</span></span>
        <div className="segmented">
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

      <div className="field">
        <span className="field-label"><span>Controller sees</span></span>
        <div className="segmented">
          {meta.feedback.map((feedback) => (
            <button
              key={feedback.id}
              type="button"
              aria-pressed={request.feedback === feedback.id}
              disabled={running}
              title={feedback.detail}
              onClick={() => onChange({ feedback: feedback.id as FeedbackId })}
            >
              {feedback.label}
            </button>
          ))}
        </div>
      </div>

      <button type="button" className="run-button" onClick={onRun} disabled={running}>
        {running ? 'Simulating…' : 'Run simulation'}
      </button>

      <div className={`status${error ? ' error' : ''}`} role="status">
        {error ?? statusLine ?? `${meta.limits.duration_s.toFixed(0)} s of flight, integrated at ` +
          `${(1 / meta.limits.dt_s).toFixed(0)} Hz`}
      </div>
    </div>
  )
}
