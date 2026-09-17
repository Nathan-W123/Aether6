import type { SimulationResponse } from '../api/types'

interface Props {
  version: string
  repositoryUrl: string
  run: SimulationResponse | null
}

/**
 * The cover of the report.
 *
 * The identification strip carries what a test report's cover carries — the article under
 * test, the condition it was flown at and the seed that makes it reproducible — rather than
 * a row of marketing statistics.
 */
export function Masthead({ version, repositoryUrl, run }: Props) {
  const request = run?.request
  return (
    <header className="masthead">
      <div className="masthead-rule" />

      <div style={{ display: 'flex', alignItems: 'flex-start', justifyContent: 'space-between',
                    gap: 28, flexWrap: 'wrap' }}>
        <div style={{ flex: '1 1 460px', minWidth: 0 }}>
          <p className="label" style={{ marginTop: 0, marginBottom: 10 }}>
            Six-degree-of-freedom flight dynamics · report {version}
          </p>
          <h1 className="masthead-title">
            Aether-6<br />Flight Test
          </h1>
          <p className="masthead-standfirst">
            A nonlinear 6-DOF fixed-wing simulator with a full aerodynamic and propulsion
            model, ISA atmosphere and Dryden turbulence, numerical trim and linearisation,
            LQR and PID autopilots, vector-field waypoint guidance and an 18-state error-state
            navigation filter on simulated multi-rate avionics. <em>Every figure below is
            integrated on request by the compiled C++ binaries</em> — nothing on this page is
            a stored picture.
          </p>
        </div>

        <a href={repositoryUrl} target="_blank" rel="noreferrer noopener"
           className="action-secondary" style={{ textDecoration: 'none', flex: 'none' }}>
          Source ↗
        </a>
      </div>

      <dl className="masthead-meta">
        <div>
          <dt className="label">Article</dt>
          <dd>13.5 kg UAV</dd>
        </div>
        <div>
          <dt className="label">Control law</dt>
          <dd>{request ? request.controller.toUpperCase() : '—'}</dd>
        </div>
        <div>
          <dt className="label">Feedback</dt>
          <dd>{request ? (request.feedback === 'truth' ? 'True states' : 'EKF') : '—'}</dd>
        </div>
        <div>
          <dt className="label">Wind</dt>
          <dd>{request ? `${request.wind_speed.toFixed(1)} m/s` : '—'}</dd>
        </div>
        <div>
          <dt className="label">Airspeed</dt>
          <dd>{request ? `${request.airspeed.toFixed(1)} m/s` : '—'}</dd>
        </div>
        <div>
          <dt className="label">Seed</dt>
          <dd>{request ? request.seed : '—'}</dd>
        </div>
      </dl>
    </header>
  )
}
