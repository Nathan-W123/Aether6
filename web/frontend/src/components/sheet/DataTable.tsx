import type { ReactNode } from 'react'

/** A ruled table: horizontal rules only, scrolling on its own axis when the sheet is narrow. */
export function DataTable({ caption, children }: { caption?: string; children: ReactNode }) {
  return (
    <div className="table-scroll">
      <table className="data">
        {caption && <caption>{caption}</caption>}
        {children}
      </table>
    </div>
  )
}
