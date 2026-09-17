import { useCallback, useLayoutEffect, useRef, useState } from 'react'

/** Track an element's content-box width, so SVG charts can render at real pixel size. */
export function useMeasure<T extends HTMLElement>(): [
  (node: T | null) => void,
  { width: number; height: number },
] {
  const [size, setSize] = useState({ width: 0, height: 0 })
  const observer = useRef<ResizeObserver | null>(null)

  const ref = useCallback((node: T | null) => {
    observer.current?.disconnect()
    if (!node) return
    observer.current = new ResizeObserver((entries) => {
      const box = entries[0]?.contentRect
      if (box) setSize({ width: Math.round(box.width), height: Math.round(box.height) })
    })
    observer.current.observe(node)
    setSize({ width: Math.round(node.clientWidth), height: Math.round(node.clientHeight) })
  }, [])

  useLayoutEffect(() => () => observer.current?.disconnect(), [])
  return [ref, size]
}
