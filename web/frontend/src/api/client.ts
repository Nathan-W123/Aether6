/** Thin typed wrapper over the dashboard API. */

import type {
  Meta,
  MonteCarloRequest,
  MonteCarloResponse,
  PrecomputedModes,
  PrecomputedMonteCarlo,
  SimulationRequest,
  SimulationResponse,
} from './types'

/** An API error carrying the server's own explanation, ready to show in the UI. */
export class ApiError extends Error {
  readonly status: number
  constructor(status: number, message: string) {
    super(message)
    this.status = status
    this.name = 'ApiError'
  }
}

async function readError(response: Response): Promise<string> {
  try {
    const body = await response.json()
    if (typeof body?.detail === 'string') return body.detail
    if (Array.isArray(body?.detail) && body.detail.length > 0) {
      const first = body.detail[0]
      return `${first.field ?? 'request'}: ${first.message ?? 'invalid value'}`
    }
  } catch {
    /* fall through to the generic message below */
  }
  if (response.status === 503) return 'The server is busy running another simulation.'
  if (response.status === 504) return 'The simulation exceeded its time limit.'
  return `Request failed (${response.status})`
}

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  let response: Response
  try {
    response = await fetch(path, init)
  } catch {
    throw new ApiError(0, 'Could not reach the simulation service.')
  }
  if (!response.ok) throw new ApiError(response.status, await readError(response))
  return (await response.json()) as T
}

function post<T>(path: string, body: unknown, signal?: AbortSignal): Promise<T> {
  return request<T>(path, {
    method: 'POST',
    headers: { 'content-type': 'application/json' },
    body: JSON.stringify(body),
    signal,
  })
}

export const api = {
  meta: () => request<Meta>('/api/meta'),
  simulate: (body: SimulationRequest, signal?: AbortSignal) =>
    post<SimulationResponse>('/api/simulate', body, signal),
  montecarlo: (body: MonteCarloRequest, signal?: AbortSignal) =>
    post<MonteCarloResponse>('/api/montecarlo', body, signal),
  precomputedMonteCarlo: () => request<PrecomputedMonteCarlo>('/api/precomputed/montecarlo'),
  precomputedModes: () => request<PrecomputedModes>('/api/precomputed/modes'),
}
