const { test } = require('node:test');
const assert = require('node:assert/strict');
const { spawn } = require('node:child_process');
const { chromium } = require('playwright');
const fs = require('node:fs');
const net = require('node:net');
const path = require('node:path');

const projectRoot = path.resolve(__dirname, '../..');
const serverBinary = path.join(projectRoot, 'build/robot_simulator_server');
const baseUrl = 'http://127.0.0.1:8080';
const wait = ms => new Promise(resolve => setTimeout(resolve, ms));

async function assertPortFree() {
  await new Promise((resolve, reject) => {
    const probe = net.connect({ host: '127.0.0.1', port: 8080 });
    probe.once('connect', () => {
      probe.destroy();
      reject(new Error('Port 8080 already has a listening server'));
    });
    probe.once('error', error => {
      if (error.code === 'ECONNREFUSED') resolve();
      else reject(error);
    });
  });
}

async function startBackend() {
  if (!fs.existsSync(serverBinary)) {
    throw new Error('Build the C++ server first: cmake -S . -B build -DBUILD_TESTING=ON && cmake --build build --parallel');
  }
  await assertPortFree();
  const child = spawn(serverBinary, [], { cwd: projectRoot, stdio: ['ignore', 'ignore', 'pipe'] });
  let stderr = '';
  child.stderr.on('data', chunk => { stderr += chunk.toString(); });
  for (let attempt = 0; attempt < 100; attempt++) {
    if (child.exitCode !== null) break;
    try {
      const response = await fetch(`${baseUrl}/api/health`, { signal: AbortSignal.timeout(500) });
      if (response.ok && child.exitCode === null) return child;
    } catch { /* Wait until the owned server is ready. */ }
    await wait(100);
  }
  await stopBackend(child);
  throw new Error(`The C++ server did not start: ${stderr || 'no error output'}`);
}

async function stopBackend(child) {
  if (child.exitCode !== null) return;
  await new Promise(resolve => {
    const timeout = setTimeout(() => child.kill('SIGKILL'), 3000);
    child.once('exit', () => { clearTimeout(timeout); resolve(); });
    child.kill('SIGTERM');
  });
}

async function newPage(browser) {
  const page = await browser.newPage({ viewport: { width: 1280, height: 900 } });
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  await page.goto(baseUrl);
  await page.waitForFunction(() => !document.querySelector('#initialize').disabled);
  return { page, errors };
}

async function initialize(page) {
  await page.click('#initialize');
  await page.waitForFunction(() => !document.querySelector('#step').disabled);
  assert.equal(await page.locator('#metric-step').innerText(), '0');
}

test('smoke tests against the real C++ HTTP server', { timeout: 60000 }, async t => {
  const server = await startBackend();
  let browser;
  try {
    browser = await chromium.launch({ headless: true });

    await t.test('Initialize, Step and Reset reflect the server snapshot', async () => {
      const { page, errors } = await newPage(browser);
      try {
        await initialize(page);
        assert.equal(await page.locator('#metric-position').innerText(), '-8.00, 0.00');
        await page.click('#step');
        await page.waitForFunction(() => document.querySelector('#metric-step').textContent === '1');
        const snapshot = await (await fetch(`${baseUrl}/api/simulation/state`)).json();
        assert.equal(await page.locator('#metric-position').innerText(),
          `${snapshot.robot_pose.x.toFixed(2)}, ${snapshot.robot_pose.y.toFixed(2)}`);
        assert.notEqual(snapshot.robot_pose.x, -8);
        assert.equal(await page.evaluate(() => state.trajectory.length), 2);
        await page.click('#initialize');
        await page.waitForFunction(() => document.querySelector('#metric-step').textContent === '0');
        assert.equal(await page.evaluate(() => state.trajectory.length), 1);
        await page.click('#step');
        await page.waitForFunction(() => document.querySelector('#metric-step').textContent === '1');
        await page.click('#reset');
        await page.waitForFunction(() => document.querySelector('#metric-step').textContent === '0');
        assert.equal(await page.locator('#metric-position').innerText(), '-8.00, 0.00');
        assert.equal(await page.evaluate(() => state.trajectory.length), 1);
        assert.deepEqual(errors, []);
      } finally { await page.close(); }
    });

    await t.test('invalid scene shows an error and preserves the active session', async () => {
      const { page, errors } = await newPage(browser);
      try {
        await page.locator('[data-path="robot.pose.x"]').fill('100');
        await page.click('#initialize');
        await page.waitForFunction(() => document.querySelector('#message').textContent.includes('Inicjalizacja nie została potwierdzona'));
        assert.equal(await page.locator('#message').evaluate(node => node.classList.contains('error')), true);
        const snapshot = await (await fetch(`${baseUrl}/api/simulation/state`)).json();
        assert.equal(snapshot.robot_pose.x, -8);
        assert.deepEqual(errors, []);
      } finally { await page.close(); }
    });

    await t.test('Start, Pause and Reset serialize requests and clear the displayed run', async () => {
      const { page, errors } = await newPage(browser);
      let activeSteps = 0, maxActiveSteps = 0, stepRequests = 0;
      page.on('request', request => {
        if (request.url().endsWith('/api/simulation/step')) {
          activeSteps++;
          stepRequests++;
          maxActiveSteps = Math.max(maxActiveSteps, activeSteps);
        }
      });
      const complete = request => {
        if (request.url().endsWith('/api/simulation/step')) activeSteps--;
      };
      page.on('requestfinished', complete);
      page.on('requestfailed', complete);
      try {
        await initialize(page);
        await page.selectOption('#pace', '100');
        await page.click('#start');
        assert.equal(await page.locator('#reset').isDisabled(), true);
        await page.waitForFunction(() => Number(document.querySelector('#metric-step').textContent) >= 3);
        await page.click('#pause');
        await page.waitForFunction(() => !document.querySelector('#reset').disabled);
        const pausedCount = stepRequests;
        await wait(250);
        assert.equal(stepRequests, pausedCount);
        assert.equal(maxActiveSteps, 1);
        await page.click('#reset');
        await page.waitForFunction(() => document.querySelector('#metric-step').textContent === '0');
        await wait(250);
        assert.equal(await page.locator('#metric-step').innerText(), '0');
        assert.equal(await page.evaluate(() => state.trajectory.length), 1);
        assert.equal(activeSteps, 0);
        assert.deepEqual(errors, []);
      } finally { await page.close(); }
    });
  } finally {
    if (browser) await browser.close();
    await stopBackend(server);
  }
});
