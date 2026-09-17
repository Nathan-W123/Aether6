import { useEffect, useMemo, useRef, useState } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'

import type { Series, Waypoint } from '../api/types'

/**
 * NED (north-east-down) to the viewer's Y-up frame.
 *
 * East becomes +x, altitude becomes +y and north becomes -z. The determinant is +1, so this
 * is a proper rotation and the rendered attitude is the simulated attitude, not its mirror.
 */
function toScene(north: number, east: number, altitude: number, target = new THREE.Vector3()) {
  return target.set(east, altitude, -north)
}

/** Rotate a body-frame direction into NED with the Hamilton quaternion q_nb, then into scene axes. */
function bodyAxisToScene(q: THREE.Quaternion, bx: number, by: number, bz: number) {
  const ned = new THREE.Vector3(bx, by, bz).applyQuaternion(q)
  // ned = (north, east, down) -> scene = (east, -down, -north)
  return new THREE.Vector3(ned.y, -ned.z, -ned.x)
}

/** Resolve one of the sheet's CSS tokens to a Three.js colour. */
function token(name: string, fallback: string): THREE.Color {
  const value = getComputedStyle(document.documentElement).getPropertyValue(name).trim()
  return new THREE.Color(value || fallback)
}

/** A small delta-wing glyph: nose along +x, canopy along +y, right wing along +z. */
function buildAircraft(): THREE.Group {
  const group = new THREE.Group()
  const body = new THREE.MeshStandardMaterial({
    color: token('--ink', '#14191b'), roughness: 0.55, metalness: 0.05,
  })
  const accent = new THREE.MeshStandardMaterial({
    color: token('--oxide', '#a8351b'), roughness: 0.6, metalness: 0.0,
  })

  const fuselage = new THREE.Mesh(new THREE.CapsuleGeometry(1.5, 8, 4, 12), body)
  fuselage.rotation.z = Math.PI / 2
  group.add(fuselage)

  const nose = new THREE.Mesh(new THREE.ConeGeometry(1.5, 3.4, 12), body)
  nose.rotation.z = -Math.PI / 2
  nose.position.x = 5.6
  group.add(nose)

  const wing = new THREE.Mesh(new THREE.BoxGeometry(3.0, 0.45, 18), accent)
  wing.position.x = -0.4
  group.add(wing)

  const tail = new THREE.Mesh(new THREE.BoxGeometry(1.7, 0.4, 7.0), accent)
  tail.position.x = -5.2
  group.add(tail)

  const fin = new THREE.Mesh(new THREE.BoxGeometry(1.7, 3.2, 0.4), accent)
  fin.position.set(-5.2, 1.6, 0)
  group.add(fin)

  return group
}

interface Props {
  series: Series
  waypoints: Waypoint[]
  /** Overlay the filter's estimated path, to show navigation error in space. */
  showEstimate: boolean
  height?: number
}

const SPEEDS = [1, 4, 16] as const

export function Trajectory3D({ series, waypoints, showEstimate, height = 420 }: Props) {
  const mountRef = useRef<HTMLDivElement>(null)
  const sceneRef = useRef<{
    renderer: THREE.WebGLRenderer
    scene: THREE.Scene
    camera: THREE.PerspectiveCamera
    controls: OrbitControls
    aircraft: THREE.Group
    glyphScale: { value: number }
    dynamic: THREE.Group
    trail: THREE.Line
    dispose: () => void
  } | null>(null)

  const [playing, setPlaying] = useState(true)
  const [speed, setSpeed] = useState<(typeof SPEEDS)[number]>(4)
  const [frame, setFrame] = useState(0)
  const frameRef = useRef(0)

  const track = useMemo(() => {
    const t = series.t ?? []
    const north = series.north ?? []
    const east = series.east ?? []
    const altitude = series.altitude ?? []
    const n = Math.min(t.length, north.length, east.length, altitude.length)
    const quaternions: THREE.Quaternion[] = []
    for (let i = 0; i < n; i += 1) {
      // THREE.Quaternion is (x, y, z, w); the log stores (w, x, y, z).
      quaternions.push(new THREE.Quaternion(
        series.qx?.[i] ?? 0, series.qy?.[i] ?? 0, series.qz?.[i] ?? 0, series.qw?.[i] ?? 1,
      ).normalize())
    }
    const points = Array.from({ length: n }, (_, i) => toScene(north[i], east[i], altitude[i]))
    const estimate = showEstimate && series.est_north && series.est_east && series.est_altitude
      ? Array.from({ length: n }, (_, i) =>
          toScene(series.est_north[i], series.est_east[i], series.est_altitude[i]))
      : null
    return { n, t, points, quaternions, estimate }
  }, [series, showEstimate])

  // --- one-time scene construction -------------------------------------------------
  useEffect(() => {
    const mount = mountRef.current
    if (!mount) return

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false })
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    renderer.setSize(mount.clientWidth, mount.clientHeight)
    mount.appendChild(renderer.domElement)

    const scene = new THREE.Scene()
    const field = token('--field', '#f7f8f5')
    scene.background = field
    scene.fog = new THREE.Fog(field, 1100, 3800)

    const camera = new THREE.PerspectiveCamera(
      42, mount.clientWidth / Math.max(1, mount.clientHeight), 1, 12000)
    camera.position.set(700, 620, 900)

    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true
    controls.dampingFactor = 0.08
    controls.maxPolarAngle = Math.PI / 2 - 0.02
    controls.minDistance = 60
    controls.maxDistance = 6000

    scene.add(new THREE.HemisphereLight(0xffffff, 0x9aa5a3, 2.1))
    const sun = new THREE.DirectionalLight(0xffffff, 1.5)
    sun.position.set(500, 950, 420)
    scene.add(sun)

    // The ground is plotting paper: a ruled grid, no shading.
    const grid = new THREE.GridHelper(4000, 40,
      token('--rule-strong', '#8d9a96'), token('--grid', '#d7ded9'))
    ;(grid.material as THREE.Material).transparent = true
    ;(grid.material as THREE.Material).opacity = 0.9
    scene.add(grid)

    const dynamic = new THREE.Group()
    scene.add(dynamic)

    const aircraft = buildAircraft()
    scene.add(aircraft)

    const trail = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color: token('--ink', '#14191b') }),
    )
    scene.add(trail)

    let raf = 0
    const render = () => {
      controls.update()
      renderer.render(scene, camera)
      raf = requestAnimationFrame(render)
    }
    render()

    const resize = new ResizeObserver(() => {
      const w = mount.clientWidth
      const h = mount.clientHeight
      if (w === 0 || h === 0) return
      renderer.setSize(w, h)
      camera.aspect = w / h
      camera.updateProjectionMatrix()
    })
    resize.observe(mount)

    const dispose = () => {
      cancelAnimationFrame(raf)
      resize.disconnect()
      controls.dispose()
      renderer.dispose()
      scene.traverse((object) => {
        const mesh = object as THREE.Mesh
        mesh.geometry?.dispose()
        const material = mesh.material as THREE.Material | THREE.Material[] | undefined
        if (Array.isArray(material)) material.forEach((m) => m.dispose())
        else material?.dispose()
      })
      mount.removeChild(renderer.domElement)
    }

    sceneRef.current = {
      renderer, scene, camera, controls, aircraft, glyphScale: { value: 6 }, dynamic, trail, dispose,
    }
    return () => {
      dispose()
      sceneRef.current = null
    }
  }, [])

  // --- rebuild the path, waypoints and camera framing whenever a new run arrives -----
  useEffect(() => {
    const context = sceneRef.current
    if (!context || track.n === 0) return
    const { scene, dynamic, camera, controls } = context

    dynamic.clear()

    const path = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(track.points),
      new THREE.LineBasicMaterial({ color: token('--series-1', '#2a78d6') }),
    )
    dynamic.add(path)

    // Ground shadow: the same track flattened onto the terrain, which is what makes the
    // altitude profile legible from any camera angle.
    const shadow = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(
        track.points.map((p) => new THREE.Vector3(p.x, 0.5, p.z))),
      new THREE.LineBasicMaterial({
        color: token('--ink-faint', '#7b8689'), transparent: true, opacity: 0.55 }),
    )
    dynamic.add(shadow)

    if (track.estimate) {
      const estimated = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(track.estimate),
        new THREE.LineDashedMaterial({
          color: token('--series-2', '#eb6834'), dashSize: 26, gapSize: 16,
        }),
      )
      estimated.computeLineDistances()
      dynamic.add(estimated)
    }

    const markerGeometry = new THREE.SphereGeometry(7, 16, 12)
    const markerMaterial = new THREE.MeshStandardMaterial({
      color: token('--ink', '#14191b'), roughness: 0.6,
    })
    for (const waypoint of waypoints) {
      const position = toScene(waypoint.north, waypoint.east, waypoint.altitude)
      const marker = new THREE.Mesh(markerGeometry, markerMaterial)
      marker.position.copy(position)
      dynamic.add(marker)
      const drop = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([
          position.clone(), new THREE.Vector3(position.x, 0, position.z),
        ]),
        new THREE.LineBasicMaterial({
          color: token('--ink-mute', '#4e585b'), transparent: true, opacity: 0.45 }),
      )
      dynamic.add(drop)
    }

    // Frame the whole circuit once per run, then leave the camera to the viewer.
    const box = new THREE.Box3().setFromPoints(track.points)
    const centre = box.getCenter(new THREE.Vector3())
    const radius = Math.max(120, box.getSize(new THREE.Vector3()).length() * 0.5)
    // The real vehicle is ~2 m across a ~1 km circuit and would be a single pixel. Draw it
    // at a fixed fraction of the framed extent instead, so the attitude stays legible.
    context.glyphScale.value = radius / 90
    controls.target.copy(centre)
    camera.position.set(centre.x + radius * 0.95, centre.y + radius * 0.75, centre.z + radius * 1.15)
    camera.updateProjectionMatrix()
    controls.update()

    frameRef.current = 0
    setFrame(0)
    return () => {
      dynamic.clear()
      markerGeometry.dispose()
      markerMaterial.dispose()
      void scene
    }
  }, [track, waypoints])

  // --- animation ---------------------------------------------------------------------
  useEffect(() => {
    if (!playing || track.n < 2) return
    let raf = 0
    let last = performance.now()
    const step = (now: number) => {
      const dt = (now - last) / 1000
      last = now
      const simulatedSpan = (track.t[track.n - 1] ?? 1) - (track.t[0] ?? 0)
      const framesPerSecond = (track.n / Math.max(1, simulatedSpan)) * speed
      frameRef.current = (frameRef.current + dt * framesPerSecond) % track.n
      setFrame(frameRef.current)
      raf = requestAnimationFrame(step)
    }
    raf = requestAnimationFrame(step)
    return () => cancelAnimationFrame(raf)
  }, [playing, speed, track])

  // --- pose the aircraft and draw its trail at the current frame -----------------------
  useEffect(() => {
    const context = sceneRef.current
    if (!context || track.n === 0) return
    const index = Math.min(track.n - 1, Math.max(0, Math.round(frame)))
    const position = track.points[index]
    const quaternion = track.quaternions[index]

    const forward = bodyAxisToScene(quaternion, 1, 0, 0)
    const up = bodyAxisToScene(quaternion, 0, 0, -1)
    const right = bodyAxisToScene(quaternion, 0, 1, 0)
    const scale = context.glyphScale.value
    context.aircraft.matrixAutoUpdate = false
    context.aircraft.matrix
      .makeBasis(forward, up, right)
      .scale(new THREE.Vector3(scale, scale, scale))
      .setPosition(position)

    const tailStart = Math.max(0, index - Math.round(track.n * 0.12))
    context.trail.geometry.dispose()
    context.trail.geometry = new THREE.BufferGeometry().setFromPoints(
      track.points.slice(tailStart, index + 1))
  }, [frame, track])

  const currentTime = track.n > 0
    ? track.t[Math.min(track.n - 1, Math.max(0, Math.round(frame)))]
    : 0

  return (
    <div>
      <div
        ref={mountRef}
        style={{
          width: '100%', height, background: 'var(--field)',
          border: '1px solid var(--rule)', cursor: 'grab',
        }}
      />
      <div className="transport">
        <button
          type="button"
          className="action-secondary"
          onClick={() => setPlaying((value) => !value)}
          aria-label={playing ? 'Pause the animation' : 'Play the animation'}
          style={{ minWidth: 76, flex: 'none' }}
        >
          {playing ? 'Pause' : 'Play'}
        </button>
        <input
          type="range"
          min={0}
          max={Math.max(0, track.n - 1)}
          step={1}
          value={Math.round(frame)}
          onChange={(event) => {
            const value = Number(event.target.value)
            frameRef.current = value
            setFrame(value)
          }}
          aria-label="Scrub through the trajectory"
        />
        <span className="clock">t = {currentTime.toFixed(0)} s</span>
        <div className="switch" style={{ width: 136, flex: 'none' }}>
          {SPEEDS.map((value) => (
            <button key={value} type="button" aria-pressed={speed === value}
                    onClick={() => setSpeed(value)}>
              {value}×
            </button>
          ))}
        </div>
      </div>
      <div className="legend" style={{ marginTop: 9, marginBottom: 0 }}>
        <span className="legend-item">
          <span className="legend-key" style={{ borderTopColor: 'var(--series-1)' }} />
          flown path
        </span>
        {showEstimate && (
          <span className="legend-item">
            <span className="legend-key" style={{
              borderTopColor: 'var(--series-2)', borderTopStyle: 'dashed' }} />
            navigation estimate
          </span>
        )}
        <span className="legend-item">
          <span className="legend-key" style={{ borderTopColor: 'var(--ink)' }} />
          waypoints
        </span>
        <span className="legend-item">
          <span className="legend-key" style={{ borderTopColor: 'var(--ink-faint)' }} />
          ground projection
        </span>
      </div>
    </div>
  )
}
