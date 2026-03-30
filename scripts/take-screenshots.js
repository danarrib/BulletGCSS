/**
 * take-screenshots.js
 *
 * Automates screenshot capture for Bullet GCSS documentation.
 * Runs a local HTTP server to serve the UI, then uses Playwright
 * to open the page and capture screenshots into ../docs/screenshots/.
 *
 * Usage:
 *   node take-screenshots.js
 *
 * Dependencies: playwright, serve-handler (npm install in this directory)
 *
 * ── Mock data ────────────────────────────────────────────────────────────────
 * Each screenshot definition can specify a log file (relative to ../tools/)
 * and an optional line limit. Lines are fed through window.gcssInject(),
 * which runs the exact same parser path as a real MQTT message.
 *
 * Example:
 *   logFile:   'testflight1.txt',  // file in tools/
 *   logLines:  300,                // replay first 300 lines (omit for all)
 */

'use strict';

const http         = require('http');
const path         = require('path');
const fs           = require('fs');
const handler      = require('serve-handler');
const { chromium } = require('playwright');

// ─── Configuration ────────────────────────────────────────────────────────────

const UI_DIR     = path.resolve(__dirname, '../UI');
const TOOLS_DIR  = path.resolve(__dirname, '../tools');
const OUTPUT_DIR = path.resolve(__dirname, '../docs/screenshots');
const PORT       = 8765;
const BASE_URL   = `http://localhost:${PORT}`;

// ─── Screenshot definitions ───────────────────────────────────────────────────

const screenshots = [
  {
    file:    'readme_userinterface01.png',
    url:     `${BASE_URL}/basicui.html`,
    width:   1920,
    height:  1080,
    dpr:     2,
    logFile:  'testflight3.txt',
    logLines: 162,
    zoom:     14,
    waitMs:   3000,   // wait for map tiles, fonts, and layout to settle
  },
];

// ─── Helpers ──────────────────────────────────────────────────────────────────

function startServer() {
  return new Promise((resolve) => {
    const server = http.createServer((req, res) =>
      handler(req, res, { public: UI_DIR })
    );
    server.listen(PORT, () => {
      console.log(`[server] Serving ${UI_DIR} on ${BASE_URL}`);
      resolve(server);
    });
  });
}

/**
 * Read a log file from tools/ and inject each line into the UI via
 * window.gcssInject(), which runs the same parser path as a real MQTT message.
 *
 * @param {import('playwright').Page} page
 * @param {string} logFile  - filename inside tools/
 * @param {number} [logLines] - how many lines to replay (default: all)
 */
async function injectLogFile(page, logFile, logLines) {
  const filePath = path.join(TOOLS_DIR, logFile);
  const lines = fs.readFileSync(filePath, 'utf8')
    .split('\n')
    .filter(l => l.includes('|'));

  const batch = logLines != null ? lines.slice(0, logLines) : lines;
  console.log(`[inject] ${logFile} — feeding ${batch.length} lines`);

  // Extract payload (strip "timestamp|" prefix) and call gcssInject for each
  const payloads = batch.map(l => l.slice(l.indexOf('|') + 1));
  await page.evaluate((payloads) => {
    for (const payload of payloads) {
      window.gcssInject(payload);
    }
  }, payloads);

  // Build flight path from gla/glo fields — the timerOneSecond that normally
  // populates currentFlightWaypoints doesn't fire during synchronous injection,
  // so we reconstruct it here from the GPS coordinates in each log line.
  // Only fields that changed are included in each message, so we carry forward
  // the last known value for whichever coordinate wasn't in that line.
  const flightPath = [];
  let lastLat = null;
  let lastLon = null;
  for (const payload of payloads) {
    const glaMatch = payload.match(/(?:^|,)gla:(-?\d+)/);
    const gloMatch = payload.match(/(?:^|,)glo:(-?\d+)/);
    if (glaMatch) lastLat = parseInt(glaMatch[1]) / 10000000;
    if (gloMatch) lastLon = parseInt(gloMatch[1]) / 10000000;
    if ((glaMatch || gloMatch) && lastLat !== null && lastLon !== null) {
      flightPath.push({ wpLatitude: lastLat, wpLongitude: lastLon });
    }
  }
  if (flightPath.length > 0) {
    console.log(`[inject] built flight path with ${flightPath.length} points`);
    await page.evaluate((path) => {
      window._gcssData.currentFlightWaypoints = path;
    }, flightPath);
  }
}

async function capture(page, def) {
  const outPath = path.join(OUTPUT_DIR, def.file);

  console.log(`[capture] ${def.file} — navigating to ${def.url}`);
  await page.goto(def.url, { waitUntil: 'networkidle' });

  if (def.logFile) {
    await injectLogFile(page, def.logFile, def.logLines);
  }

  if (def.waitMs) {
    console.log(`[capture] Waiting ${def.waitMs} ms for render…`);
    await page.waitForTimeout(def.waitMs);
  }

  // Set zoom level if specified, then center on aircraft and wait for tiles to load
  if (def.zoom != null) {
    await page.evaluate((z) => window._gcssMap.setZoom(z), def.zoom);
  }
  await page.click('.maplibregl-ctrl-center');
  await page.waitForTimeout(3000);

  await page.screenshot({ path: outPath, fullPage: false });
  console.log(`[capture] Saved → ${outPath}`);
}

// ─── Main ──────────────────────────────────────────────────────────────────────

(async () => {
  fs.mkdirSync(OUTPUT_DIR, { recursive: true });

  const server  = await startServer();
  const browser = await chromium.launch();

  try {
    for (const def of screenshots) {
      const context = await browser.newContext({
        viewport:          { width: def.width, height: def.height },
        deviceScaleFactor: def.dpr ?? 1,
      });
      // Set an invalid MQTT broker before any page script runs so the app
      // never attempts a real connection during screenshot capture.
      await context.addInitScript(() => {
        localStorage.setItem('mqttHost', 'screenshot.invalid');
      });
      const page = await context.newPage();
      await capture(page, def);
      await context.close();
    }
  } finally {
    await browser.close();
    server.close();
    console.log('[done]');
  }
})();
