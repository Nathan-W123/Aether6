/*
 * Shoot the plate as stills, at the film's own viewpoints.
 *
 * The plan view is the legible one — square to the sheet, the whole plate in frame. The
 * three-quarter is the same drawing seen from the angle the film passes through.
 */
import { chromium } from 'playwright-core'

const PORT = process.env.AETHER_PLATE_PORT ?? '8811'
const OUT = process.env.AETHER_PLATE_OUT
if (!OUT) throw new Error('set AETHER_PLATE_OUT')

const browser = await chromium.launch({
  executablePath: process.env.CHROMIUM ?? '/opt/pw-browsers/chromium-1194/chrome-linux/chrome',
  args: ['--no-sandbox', '--use-gl=swiftshader', '--enable-unsafe-swiftshader',
         '--force-device-scale-factor=1', '--hide-scrollbars'],
})
const page = await browser.newPage({ viewport: { width: 2048, height: 2048 } })
await page.goto(`http://127.0.0.1:${PORT}/still.html`, { waitUntil: 'load', timeout: 120000 })
await page.waitForFunction(() => window.READY === true, { timeout: 120000 })

const total = await page.evaluate(() => window.FILM.total)
for (const [name, at] of [['plan', total - 0.01], ['quarter', total * 0.61]]) {
  const info = await page.evaluate((t) => window.renderFrame(t), at)
  await page.locator('#stage').screenshot({ path: `${OUT}/aether6-plate1-${name}.png` })
  console.log(`  ${name}: flight t = ${info.t.toFixed(0)} s`)
}
await browser.close()
