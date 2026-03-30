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
 * Dependencies: playwright, serve-handler, sharp (npm install in this directory)
 *
 * ── Mock data ────────────────────────────────────────────────────────────────
 * Each screenshot definition can specify a log file (relative to ../tools/)
 * and an optional line limit. Lines are fed through window.gcssInject(),
 * which runs the exact same parser path as a real MQTT message.
 *
 * ── Compositions ─────────────────────────────────────────────────────────────
 * A composition captures multiple frames in sequence on the same page and
 * stitches them side-by-side. Each frame can optionally highlight a UI element
 * by drawing a rounded rectangle over it.
 *
 *   composition: 'output.png',
 *   gap: 40,               // px gap between frames in the final image
 *   frames: [
 *     { actions: async (page) => { ... }, highlight: '#cssSelector' },
 *   ]
 */

'use strict';

const http         = require('http');
const path         = require('path');
const fs           = require('fs');
const handler      = require('serve-handler');
const { chromium } = require('playwright');
const sharp        = require('sharp');

// ─── Configuration ────────────────────────────────────────────────────────────

const UI_DIR     = path.resolve(__dirname, '../UI');
const TOOLS_DIR  = path.resolve(__dirname, '../tools');
const OUTPUT_DIR = path.resolve(__dirname, '../docs/screenshots');
const PORT       = 8765;
const BASE_URL   = `http://localhost:${PORT}`;

// Highlight style
const HIGHLIGHT_COLOR  = '#FF4400';
const HIGHLIGHT_STROKE = 6;   // px at 1× — scaled by DPR automatically
const HIGHLIGHT_PAD    = 10;  // extra space around the element bounding box

// ─── Screenshot definitions ───────────────────────────────────────────────────

// Shared setup for the main UI screenshots
const MAIN_UI = {
  url:      `${BASE_URL}/basicui.html`,
  width:    1920,
  height:   1080,
  dpr:      2,
  logFile:  'testflight3.txt',
  logLines: 478,
  zoom:     14,
  waitMs:   3000,
};

const screenshots = [
  {
    ...MAIN_UI,
    file: 'readme_userinterface01.png',
  },
  {
    ...MAIN_UI,
    file:         'ui_statuslights.png',
    clipSelector: '.statusIconBar',
  },
  {
    ...MAIN_UI,
    file:         'ui_infopanel.png',
    clipSelector: '#dataview',
  },
  {
    ...MAIN_UI,
    file:         'ui_map.png',
    clipSelector: '#mapview',
  },
  {
    ...MAIN_UI,
    file:         'ui_efis.png',
    clipSelector: '#hudview',
  },
  {
    composition: 'mqtt_settings.png',
    url:         `${BASE_URL}/basicui.html`,
    width:       1080,
    height:      1920,
    dpr:         2,
    gap:         40,
    outputWidth: 1920,
    logFile:     'testflight3.txt',
    logLines:    478,
    zoom:        14,
    waitMs:      3000,
    frames: [
      {
        label:     'step1-main',
        actions:   async () => {},
        highlight: '#gearIcon',
      },
      {
        label:     'step2-menu',
        actions:   async (page) => {
          await page.click('#gearIcon');
          await page.waitForTimeout(500);
        },
        highlight: '#navSettingsMenu',
      },
      {
        label:     'step3-settings',
        actions:   async (page) => {
          await page.click('#navSettingsMenu');
          await page.waitForTimeout(500);
        },
        highlight: '#navBrokerSettings',
      },
      {
        label:   'step4-broker',
        actions: async (page) => {
          await page.click('#navBrokerSettings');
          await page.waitForTimeout(500);
          await page.fill('#brokerTopic', 'bulletgcss/telem/your_callsign');
          await page.waitForTimeout(500);
        },
      },
    ],
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
 */
async function injectLogFile(page, logFile, logLines) {
  const filePath = path.join(TOOLS_DIR, logFile);
  const lines = fs.readFileSync(filePath, 'utf8')
    .split('\n')
    .filter(l => l.includes('|'));

  const batch = logLines != null ? lines.slice(0, logLines) : lines;
  console.log(`[inject] ${logFile} — feeding ${batch.length} lines`);

  const payloads = batch.map(l => l.slice(l.indexOf('|') + 1));
  await page.evaluate((payloads) => {
    for (const payload of payloads) {
      window.gcssInject(payload);
    }
  }, payloads);

  // Reconstruct flight path from gla/glo fields (timerOneSecond doesn't fire
  // during synchronous injection). Carry forward last known value per field.
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

/**
 * Overlay a highlight rectangle on a screenshot buffer around a page element.
 * @param {Buffer} buf         - PNG screenshot buffer
 * @param {object} box         - Playwright boundingBox() result (CSS pixels)
 * @param {number} dpr         - deviceScaleFactor used when capturing
 * @returns {Promise<Buffer>}  - new PNG buffer with the highlight drawn
 */
async function applyHighlight(buf, box, dpr) {
  const meta = await sharp(buf).metadata();

  const pad    = HIGHLIGHT_PAD * dpr;
  const stroke = HIGHLIGHT_STROKE * dpr;
  const radius = 12 * dpr;

  const x = Math.max(0, box.x * dpr - pad);
  const y = Math.max(0, box.y * dpr - pad);
  const w = box.width  * dpr + pad * 2;
  const h = box.height * dpr + pad * 2;

  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${meta.width}" height="${meta.height}">
    <rect x="${x}" y="${y}" width="${w}" height="${h}"
          rx="${radius}" ry="${radius}"
          fill="none"
          stroke="${HIGHLIGHT_COLOR}"
          stroke-width="${stroke}"
          opacity="0.9"/>
  </svg>`;

  return await sharp(buf)
    .composite([{ input: Buffer.from(svg), top: 0, left: 0 }])
    .png()
    .toBuffer();
}

/** Set up the page: inject log data, wait, zoom, center map. */
async function setupPage(page, def) {
  console.log(`[setup] navigating to ${def.url}`);
  await page.goto(def.url, { waitUntil: 'networkidle' });

  if (def.logFile) {
    await injectLogFile(page, def.logFile, def.logLines);
  }

  if (def.waitMs) {
    console.log(`[setup] waiting ${def.waitMs} ms for render…`);
    await page.waitForTimeout(def.waitMs);
  }

  if (def.zoom != null) {
    await page.evaluate((z) => window._gcssMap.setZoom(z), def.zoom);
  }
  await page.click('.maplibregl-ctrl-center');
  await page.waitForTimeout(3000);
}

/** Capture a single screenshot to a file, optionally clipped to an element. */
async function capture(page, def) {
  const outPath = path.join(OUTPUT_DIR, def.file);
  await setupPage(page, def);

  let clip;
  if (def.clipSelector) {
    const box = await page.locator(def.clipSelector).boundingBox();
    if (!box) throw new Error(`clipSelector not found: ${def.clipSelector}`);
    clip = box;
  }

  await page.screenshot({ path: outPath, fullPage: false, ...(clip ? { clip } : {}) });
  console.log(`[capture] Saved → ${outPath}`);
}

/** Capture a multi-frame composition and stitch frames side-by-side. */
async function captureComposition(page, def) {
  const outPath = path.join(OUTPUT_DIR, def.composition);
  const dpr     = def.dpr ?? 1;
  const gap     = (def.gap ?? 0) * dpr;  // gap scaled to actual pixel space

  await setupPage(page, def);

  // Capture each frame, run its actions, then apply optional highlight
  const frameBuffers = [];
  for (const frame of def.frames) {
    if (frame.actions) await frame.actions(page);
    console.log(`[composition] capturing frame: ${frame.label}`);

    let buf = await page.screenshot({ fullPage: false });

    if (frame.highlight) {
      const box = await page.locator(frame.highlight).boundingBox();
      if (box) {
        buf = await applyHighlight(buf, box, dpr);
        console.log(`[composition] highlighted ${frame.highlight}`);
      } else {
        console.warn(`[composition] element not found for highlight: ${frame.highlight}`);
      }
    }

    frameBuffers.push(buf);
  }

  // Stitch frames side-by-side with configurable gap between them
  const meta    = await sharp(frameBuffers[0]).metadata();
  const fWidth  = meta.width;
  const fHeight = meta.height;
  const n       = frameBuffers.length;
  const total   = fWidth * n + gap * (n - 1);

  console.log(`[composition] stitching ${n} frames (${fWidth}×${fHeight}) with ${gap}px gap → ${total}×${fHeight}`);

  const composites = frameBuffers.map((buf, i) => ({
    input: buf,
    left:  Math.round(i * (fWidth + gap)),
    top:   0,
  }));

  // Composite first into a full-size buffer, then resize — Sharp applies
  // resize before composite internally, so they must be separate steps.
  const fullBuf = await sharp({
    create: {
      width:      total,
      height:     fHeight,
      channels:   4,
      background: { r: 18, g: 18, b: 18, alpha: 1 },  // dark background for gaps
    },
  })
    .composite(composites)
    .png()
    .toBuffer();

  const out = sharp(fullBuf);
  if (def.outputWidth) {
    const targetHeight = Math.round(fHeight * (def.outputWidth / total));
    out.resize(def.outputWidth, targetHeight);
  }
  await out.png().toFile(outPath);

  console.log(`[composition] Saved → ${outPath}`);
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
      // Subscribe to a silent topic so the broker connects normally (no
      // "not connected" message) but no real telemetry can arrive.
      await context.addInitScript(() => {
        localStorage.setItem('mqttTopic', 'bulletgcss/telem/screenshot_invalid');
      });
      const page = await context.newPage();

      if (def.composition) {
        await captureComposition(page, def);
      } else {
        await capture(page, def);
      }

      await context.close();
    }
  } finally {
    await browser.close();
    server.close();
    console.log('[done]');
  }
})();
