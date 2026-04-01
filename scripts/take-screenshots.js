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

// ─── Mission planner mock data ────────────────────────────────────────────────

const PLANNER_MISSION = {
  name: 'Test mission 1',
  savedAt: '2026-03-31',
  waypoints: [
    { lat: -23.62038567823072,  lon: -46.658252483313674, altM: 50,  action: 1, speedMs: 0, loiterSec: 10  },
    { lat: -23.63141733978425,  lon: -46.650709742796124, altM: 100, action: 1, speedMs: 0, loiterSec: 10  },
    { lat: -23.6392004966537,   lon: -46.65999383407163,  altM: 150, action: 1, speedMs: 0, loiterSec: 10  },
    { lat: -23.631417338499958, lon: -46.67214356794332,  altM: 150, action: 3, speedMs: 0, loiterSec: 120 },
    { lat: -23.62157903999895,  lon: -46.67905245096887,  altM: 100, action: 1, speedMs: 0, loiterSec: 10  },
    { lat: -23.613135574843,    lon: -46.6709877768518,   altM: 100, action: 4, speedMs: 0, loiterSec: 10  },
  ],
};

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

  // ── Mission Planner screenshots ────────────────────────────────────────────

  {
    url:        `${BASE_URL}/basicui.html`,
    width:      1080,
    height:     1920,
    dpr:        2,
    logFile:    'testflight3.txt',
    logLines:   478,
    waitMs:     3000,
    skipCenter: true,
    file:       'ui_missionplanner_overview.png',
    setup:      async (page) => {
      await openAndLoadMission(page, PLANNER_MISSION);
    },
  },

  {
    url:        `${BASE_URL}/basicui.html`,
    width:      1080,
    height:     1920,
    dpr:        2,
    logFile:    'testflight3.txt',
    logLines:   478,
    waitMs:     3000,
    skipCenter: true,
    file:       'ui_missionplanner_modal.png',
    setup:      async (page) => {
      await openAndLoadMission(page, PLANNER_MISSION);
      // Open the modal for the middle WP (index 2 = WP 3, a 150 m Waypoint)
      await page.locator('.wp-marker').nth(2).click();
      await page.waitForTimeout(500);
    },
  },

  {
    socialMedia: '../docs/screenshots/github-social-media.png',
    url:         `${BASE_URL}/basicui.html`,
    logFile:     'testflight3.txt',
    logLines:    478,
    waitMs:      3000,
  },

  {
    composition: 'mp_workflow.png',
    url:         `${BASE_URL}/basicui.html`,
    width:       1080,
    height:      1920,
    dpr:         2,
    gap:         40,
    outputWidth: 1920,
    logFile:     'testflight3.txt',
    logLines:    478,
    waitMs:      3000,
    skipCenter:  true,
    frames: [
      {
        label:   'step1-nav',
        actions: async (page) => {
          await page.click('#gearIcon');
          await page.waitForTimeout(500);
        },
        highlight: '#missionplannerlink',
      },
      {
        label:   'step2-planner-empty',
        actions: async (page) => {
          await page.click('#missionplannerlink');
          await page.waitForSelector('#mpMap canvas', { timeout: 10000 });
          await page.waitForTimeout(2000);
        },
      },
      {
        label:   'step3-planner-mission',
        actions: async (page) => {
          await page.evaluate((mission) => {
            localStorage.setItem('gcss_saved_missions', JSON.stringify([mission]));
          }, PLANNER_MISSION);
          await page.click('#mpBtLoad');
          await page.waitForTimeout(500);
          await page.locator('#mpSavedMissionsList .btRcCmd').first().click();
          await page.waitForTimeout(3000);
        },
        highlight: '#mpBtUpload',
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

/**
 * Generate the GitHub social media preview image (1280×640).
 * Captures portrait (440×956 CSS @ dpr 3 = 1320×2868 physical) and landscape
 * (956×440 CSS @ dpr 3 = 2868×1320 physical) screenshots, places them inside
 * phone frame mockups, and composites them onto a white canvas with text.
 */
async function generateSocialMedia(browser, def) {
  const FRAME_FILE = path.resolve(__dirname, 'phone_frame_2k.png');
  const OUTPUT     = path.resolve(__dirname, def.socialMedia);
  const CANVAS_W = 1280, CANVAS_H = 640;

  // ── Capture portrait screenshot (440×956 CSS @ dpr 3 = 1320×2868 physical) ─
  console.log('[social] capturing portrait screenshot…');
  const portCtx = await browser.newContext({
    viewport: { width: 440, height: 956 },
    deviceScaleFactor: 1,
  });
  await portCtx.addInitScript(() => {
    localStorage.setItem('mqttTopic', 'bulletgcss/telem/screenshot_invalid');
  });
  const portPage = await portCtx.newPage();
  await setupPage(portPage, { ...def, skipCenter: false, zoom: 14 });
  const portShotBuf = await portPage.screenshot({ fullPage: false });
  await portCtx.close();

  // ── Capture landscape screenshot (956×440 CSS @ dpr 3 = 2868×1320 physical) ─
  console.log('[social] capturing landscape screenshot…');
  const landCtx = await browser.newContext({
    viewport: { width: 956, height: 440 },
    deviceScaleFactor: 1,
  });
  await landCtx.addInitScript(() => {
    localStorage.setItem('mqttTopic', 'bulletgcss/telem/screenshot_invalid');
  });
  const landPage = await landCtx.newPage();
  await setupPage(landPage, { ...def, skipCenter: false, zoom: 14 });
  const landShotBuf = await landPage.screenshot({ fullPage: false });
  await landCtx.close();

  // ── Resize screenshots ─────────────────────────────────────────────────────
  const portShotResized = await sharp(portShotBuf)
    .resize(256, 559)
    .toBuffer();

  const landShotResized = await sharp(landShotBuf)
    .resize(559, 256)
    .toBuffer();

  // ── Resize frames ──────────────────────────────────────────────────────────
  const portFrameResized = await sharp(FRAME_FILE)
    .resize(278, 579)
    .toBuffer();

  const landFrameResized = await sharp(FRAME_FILE)
    .rotate(-90)   // counter-clockwise
    .resize(579, 278)
    .toBuffer();

  // ── Text SVG (centered on x=836) ──────────────────────────────────────────
  const textSvg = `<svg xmlns="http://www.w3.org/2000/svg" width="${CANVAS_W}" height="${CANVAS_H}">
    <text x="836" y="456"
          font-family="Ubuntu, Arial, sans-serif"
          font-size="82" font-weight="bold" fill="#1a1a1a"
          text-anchor="middle">Bullet GCSS</text>
    <text x="836" y="513"
          font-family="Ubuntu, Arial, sans-serif"
          font-size="40" fill="#1a1a1a"
          text-anchor="middle">
      <tspan x="836" dy="0">A high caliber ground control station system</tspan>
      <tspan x="836" dy="1.4em">designed for the 21st century lifestyle</tspan>
    </text>
  </svg>`;

  // ── Composite onto 1280×640 white canvas ──────────────────────────────────
  // Layer order (bottom to top): land shot, port shot, land frame, port frame, text
  await sharp({
    create: { width: CANVAS_W, height: CANVAS_H, channels: 4, background: { r: 255, g: 255, b: 255, alpha: 1 } },
  })
    .composite([
      { input: landShotResized,      left: 554, top: 98 },
      { input: portShotResized,      left: 149, top: 53 },
      { input: landFrameResized,     left: 547, top: 90 },
      { input: portFrameResized,     left: 137, top: 42 },
      { input: Buffer.from(textSvg), left: 0,   top: 0  },
    ])
    .png()
    .toFile(OUTPUT);

  console.log(`[social] Saved → ${OUTPUT}`);
}

/**
 * Open the Mission Planner and load a saved mission from localStorage.
 * The mission is written to localStorage just before clicking Load,
 * since getSavedMissions() reads it lazily on panel open.
 */
async function openAndLoadMission(page, mission) {
  await page.click('#gearIcon');
  await page.waitForTimeout(500);
  await page.click('#missionplannerlink');
  await page.waitForSelector('#mpMap canvas', { timeout: 10000 });
  await page.waitForTimeout(2000);
  await page.evaluate((m) => {
    localStorage.setItem('gcss_saved_missions', JSON.stringify([m]));
  }, mission);
  await page.click('#mpBtLoad');
  await page.waitForTimeout(500);
  await page.locator('#mpSavedMissionsList .btRcCmd').first().click();
  await page.waitForTimeout(3000);
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

  if (!def.skipCenter) {
    if (def.zoom != null) {
      await page.evaluate((z) => window._gcssMap.setZoom(z), def.zoom);
    }
    await page.click('.maplibregl-ctrl-center');
    await page.waitForTimeout(3000);
  }
}

/** Capture a single screenshot to a file, optionally clipped to an element. */
async function capture(page, def) {
  const outPath = path.join(OUTPUT_DIR, def.file);
  await setupPage(page, def);

  if (def.setup) {
    await def.setup(page);
  }

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

  // Optional CLI filter: node take-screenshots.js <pattern>
  // Matches against file name, composition name, or socialMedia path (case-insensitive).
  const filter = process.argv[2] ? process.argv[2].toLowerCase() : null;
  const queue = filter
    ? screenshots.filter(def => {
        const key = (def.file || def.composition || def.socialMedia || '').toLowerCase();
        return key.includes(filter);
      })
    : screenshots;

  if (filter && queue.length === 0) {
    console.error(`[error] No screenshots matched filter: "${filter}"`);
    process.exit(1);
  }
  if (filter) {
    console.log(`[filter] Running ${queue.length} screenshot(s) matching "${filter}"`);
  }

  const server  = await startServer();
  const browser = await chromium.launch();

  try {
    for (const def of queue) {
      // Social media generation manages its own browser contexts internally
      if (def.socialMedia) {
        await generateSocialMedia(browser, def);
        continue;
      }

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
