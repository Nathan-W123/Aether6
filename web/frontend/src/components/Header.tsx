interface Props {
  version: string
  repositoryUrl: string
}

const FACTS = [
  { value: '6-DOF', label: 'rigid-body dynamics' },
  { value: 'C++17', label: 'core, Eigen only' },
  { value: '18-state', label: 'error-state EKF' },
  { value: '103', label: 'unit tests' },
]

/** The first thing a visitor reads: what this is, in one line, plus the four facts. */
export function Header({ version, repositoryUrl }: Props) {
  return (
    <header style={{
      maxWidth: 1680, margin: '0 auto', padding: '28px 24px 4px',
      display: 'flex', flexWrap: 'wrap', gap: 20, alignItems: 'flex-end',
      justifyContent: 'space-between',
    }}>
      <div style={{ minWidth: 0 }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: 10 }}>
          <svg width="26" height="26" viewBox="0 0 32 32" aria-hidden="true">
            <rect width="32" height="32" rx="7" fill="#17202e" stroke="#26344a" />
            <path d="M6 22 L16 7 L26 22 L16 17.5 Z" fill="#4db6ac" />
          </svg>
          <h1 style={{ fontSize: 22, letterSpacing: '-0.02em' }}>Aether-6</h1>
          <span className="badge">v{version}</span>
        </div>
        <p style={{ margin: '7px 0 0', color: 'var(--text-muted)', fontSize: 14, maxWidth: '68ch' }}>
          A six-degree-of-freedom fixed-wing flight simulator: nonlinear dynamics, a full
          aerodynamic and propulsion model, turbulence, trim and linearisation, LQR and PID
          autopilots, waypoint guidance and an error-state EKF on simulated sensors. Every
          figure on this page is computed by the compiled C++ binaries, on request.
        </p>
      </div>

      <div style={{ display: 'flex', gap: 22, alignItems: 'flex-end' }}>
        {FACTS.map((fact) => (
          <div key={fact.label}>
            <div className="mono" style={{ fontSize: 17, color: 'var(--text)' }}>{fact.value}</div>
            <div style={{ fontSize: 11, color: 'var(--text-dim)' }}>{fact.label}</div>
          </div>
        ))}
        <a href={repositoryUrl} target="_blank" rel="noreferrer noopener" className="ghost-button"
           style={{ textDecoration: 'none' }}>
          Source ↗
        </a>
      </div>
    </header>
  )
}
