import type { ReactNode } from 'react'

interface Props {
  /** Section number, as printed in the left margin. Sections are read in order. */
  number: string
  title: string
  /** One or two sentences on what this section establishes. */
  note?: ReactNode
  /** Right-aligned controls belonging to this section, such as a re-run button. */
  aside?: ReactNode
  children: ReactNode
}

/**
 * A numbered report section.
 *
 * Sections are divided by a hairline rule and space rather than drawn as boxes, so the page
 * reads as one continuous sheet. The numbers are not decoration: the sections build on one
 * another, and a reader referring to "§4" means something.
 */
export function Section({ number, title, note, aside, children }: Props) {
  return (
    <section className="section">
      <div className="section-head">
        <span className="section-number">§{number}</span>
        <div style={{ display: 'flex', alignItems: 'baseline', justifyContent: 'space-between',
                      gap: 16, flexWrap: 'wrap' }}>
          <h2 className="section-title">{title}</h2>
          {aside}
        </div>
        {note && <p className="section-note">{note}</p>}
      </div>
      <div className="section-body">{children}</div>
    </section>
  )
}
