import type { ReactNode } from 'react'

interface Props {
  /** Figure number, continuous through the sheet. */
  number: string
  /** What the figure shows, and the one number worth reading off it. */
  caption: ReactNode
  children: ReactNode
  /** Figures that draw their own field (the 3D view) opt out of the ruled plotting field. */
  bare?: boolean
}

/** A numbered, captioned figure drawn on its own plotting field. */
export function Figure({ number, caption, children, bare = false }: Props) {
  return (
    <figure>
      <div className={bare ? undefined : 'figure-field'}>{children}</div>
      <figcaption>
        <b>Fig.&nbsp;{number}</b> — {caption}
      </figcaption>
    </figure>
  )
}
