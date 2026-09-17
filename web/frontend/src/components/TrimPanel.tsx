import { fixed, science } from '../lib/format'
import type { SimulationRequest, TrimPoint } from '../api/types'

interface Props {
  trim: TrimPoint
  request: SimulationRequest
  serverRuntime: number
  cached: boolean
}

/**
 * The trim point the run started from.
 *
 * Worth its own panel because it is the step that makes everything after it meaningful: the
 * initial condition, the LQR design point and the linearisation all come from this solve,
 * and the residual says how well it converged.
 */
export function TrimPanel({ trim, request, serverRuntime, cached }: Props) {
  return (
    <div className="panel">
      <div className="panel-head">
        <h2>Trim solution</h2>
        <span className="badge">{cached ? 'cached result' : `${serverRuntime.toFixed(1)} s on the server`}</span>
      </div>
      <p className="panel-note">
        A damped Levenberg–Marquardt solve for the state and actuator settings that make every
        body-frame acceleration vanish at {fixed(request.airspeed, 1)} m/s and{' '}
        {fixed(request.target_altitude, 0)} m, with one-sided penalties keeping the answer
        inside the actuator box.
      </p>
      <table className="data">
        <tbody>
          <tr><td>Angle of attack α</td><td>{fixed(trim.alpha_deg, 3)}°</td></tr>
          <tr><td>Pitch attitude θ</td><td>{fixed(trim.theta_deg, 3)}°</td></tr>
          <tr><td>Elevator</td><td>{fixed(trim.elevator_deg, 3)}°</td></tr>
          <tr><td>Throttle</td><td>{fixed(trim.throttle, 4)}</td></tr>
          <tr>
            <td>Residual ‖f(x,u)‖<sub>∞</sub></td>
            <td style={{ color: trim.residual_inf_norm < 1e-8 ? 'var(--good)' : 'var(--warn)' }}>
              {science(trim.residual_inf_norm)}
            </td>
          </tr>
        </tbody>
      </table>
    </div>
  )
}
