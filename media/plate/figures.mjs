/*
 * Shoot the two standalone figures at 2048².
 *
 * They carry no plate furniture: each is the figure, its axes and the labels it needs to be
 * read, so either can stand alone next to the animation.
 */
import { chromium } from 'playwright-core'

const PORT = process.env.AETHER_PLATE_PORT ?? '8811'
const OUT = process.env.AETHER_PLATE_OUT
if (!OUT) throw new Error('set AETHER_PLATE_OUT')

const browser = await chromium.launch({
  executablePath: process.env.CHROMIUM ?? '/opt/pw-browsers/chromium-1194/chrome-linux/chrome',
  args: ['--no-sandbox', '--force-device-scale-factor=1', '--hide-scrollbars'],
})
const page = await browser.newPage({ viewport: { width: 2048, height: 2048 } })
await page.goto(`http://127.0.0.1:${PORT}/figures.html`, { waitUntil: 'load', timeout: 90000 })
await page.waitForFunction(() => window.READY === true, { timeout: 90000 })

for (const [which, file] of [
  ['tracks', 'aether6-dispersed-tracks.png'],
  ['modes', 'aether6-modes.png'],
]) {
  await page.evaluate((w) => window.draw(w), which)
  await page.waitForTimeout(250)
  await page.locator('#fig').screenshot({ path: `${OUT}/${file}` })
  console.log(`  ${which} -> ${file}`)
}
await browser.close()
