import type { Meta, SimulationResponse } from '../api/types'

/**
 * The title block that closes the sheet.
 *
 * An engineering drawing records how it was produced in its own corner: the tool, the
 * settings, the revision. This one records what it would take to reproduce the figures above.
 */
export function TitleBlock({ meta, run, repositoryUrl }: {
  meta: Meta
  run: SimulationResponse | null
  repositoryUrl: string
}) {
  return (
    <>
      <p className="label" style={{ marginBottom: 8 }}>Reproduction</p>
      <dl className="title-block">
        <div>
          <dt className="label">Integrator</dt>
          <dd>RK4, {(meta.limits.dt_s * 1000).toFixed(0)} ms</dd>
        </div>
        <div>
          <dt className="label">Control rate</dt>
          <dd>100 Hz</dd>
        </div>
        <div>
          <dt className="label">Duration</dt>
          <dd>{meta.limits.duration_s.toFixed(0)} s</dd>
        </div>
        <div>
          <dt className="label">Seed</dt>
          <dd>{run ? run.request.seed : '—'}</dd>
        </div>
        <div>
          <dt className="label">Server time</dt>
          <dd>{run ? `${run.server_runtime_s.toFixed(2)} s` : '—'}</dd>
        </div>
        <div>
          <dt className="label">Samples</dt>
          <dd>{meta.limits.series_points}</dd>
        </div>
      </dl>
      <p className="colophon">
        The airframe uses a synthetic parameter set derived from classical tail-volume and
        thin-aerofoil relations for a plausible 13.5 kg UAV, not one identified from flight
        test — the dynamics are representative of the class rather than of any particular
        aircraft. See{' '}
        <a href={`${repositoryUrl}/blob/main/docs/limitations.md`} target="_blank"
           rel="noreferrer noopener">docs/limitations.md</a>{' '}
        for what that does and does not support, and{' '}
        <a href={`${repositoryUrl}/blob/main/docs/dashboard.md`} target="_blank"
           rel="noreferrer noopener">docs/dashboard.md</a>{' '}
        for how this page executes the simulator.
      </p>
    </>
  )
}
