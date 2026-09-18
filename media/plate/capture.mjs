/*
 * Capture the plate animation frame by frame.
 *
 * Frames are stepped by an explicit film time rather than by wall clock, so the output is
 * deterministic and does not depend on how fast the software renderer happens to run.
 */
import { chromium } from 'playwright-core'
import fs from 'node:fs'

const FPS = 30
const WORK = process.env.AETHER_PLATE_WORK
if (!WORK) throw new Error('set AETHER_PLATE_WORK')
const OUT = `${WORK}/frames`
fs.rmSync(OUT, { recursive: true, force: true })
fs.mkdirSync(OUT, { recursive: true })

const browser = await chromium.launch({
  executablePath: '/opt/pw-browsers/chromium-1194/chrome-linux/chrome',
  args: ['--no-sandbox', '--use-gl=swiftshader', '--enable-unsafe-swiftshader',
         '--force-device-scale-factor=1', '--hide-scrollbars'],
})
const page = await browser.newPage({ viewport: { width: 1080, height: 1080 } })
const PORT = process.env.AETHER_PLATE_PORT ?? '8811'
await page.goto(`http://127.0.0.1:${PORT}/film.html`, { waitUntil: 'load', timeout: 120000 })
await page.waitForFunction(() => window.READY === true, { timeout: 120000 })

const total = await page.evaluate(() => window.FILM.total)
const frames = Math.round(total * FPS)
console.log(`capturing ${frames} frames (${total}s @ ${FPS}fps)`)
const stage = page.locator('#stage')
const t0 = Date.now()
for (let i = 0; i < frames; i += 1) {
  await page.evaluate((t) => window.renderFrame(t), i / FPS)
  await stage.screenshot({ path: `${OUT}/f${String(i).padStart(5, '0')}.png` })
  if (i % 90 === 0 && i > 0) {
    const r = i / ((Date.now() - t0) / 1000)
    console.log(`  ${i}/${frames}  ${r.toFixed(1)} fps  eta ${((frames - i) / r).toFixed(0)}s`)
  }
}
console.log(`captured in ${((Date.now() - t0) / 1000).toFixed(0)}s`)
await browser.close()
