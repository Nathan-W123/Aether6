import { DataTable } from './sheet/DataTable'
import { fixed, science } from '../lib/format'
import type { SimulationRequest, TrimPoint } from '../api/types'

/**
 * The equilibrium the run started from.
 *
 * Worth reporting on its own because it is the step that makes everything after it mean
 * something: the initial condition, the LQR design point and the linearisation in §7 all come
 * out of this solve, and the residual states how far from equilibrium the answer actually is.
 */
export function TrimPanel({ trim, request }: { trim: TrimPoint; request: SimulationRequest }) {
  return (
    <div style={{ maxWidth: 480 }}>
      <DataTable caption={`Trim at ${fixed(request.airspeed, 1)} m/s, `
        + `${fixed(request.target_altitude, 0)} m, wings level`}>
        <tbody>
          <tr><td>Angle of attack α</td><td>{fixed(trim.alpha_deg, 3)}</td><td
              style={{ color: 'var(--ink-faint)', width: '4em' }}>deg</td></tr>
          <tr><td>Pitch attitude θ</td><td>{fixed(trim.theta_deg, 3)}</td><td
              style={{ color: 'var(--ink-faint)' }}>deg</td></tr>
          <tr><td>Elevator</td><td>{fixed(trim.elevator_deg, 3)}</td><td
              style={{ color: 'var(--ink-faint)' }}>deg</td></tr>
          <tr><td>Throttle</td><td>{fixed(trim.throttle, 4)}</td><td
              style={{ color: 'var(--ink-faint)' }}>—</td></tr>
          <tr>
            <td>Residual ‖f(x,u)‖<sub>∞</sub></td>
            <td className={trim.residual_inf_norm < 1e-8 ? undefined : 'is-limit'}>
              {science(trim.residual_inf_norm)}
            </td>
            <td style={{ color: 'var(--ink-faint)' }}>—</td>
          </tr>
        </tbody>
      </DataTable>
    </div>
  )
}
