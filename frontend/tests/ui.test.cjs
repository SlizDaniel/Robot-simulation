const { test } = require('node:test');
const assert = require('node:assert/strict');
const { chromium } = require('playwright');
const http = require('node:http');
const fs = require('node:fs/promises');
const path = require('node:path');
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));

// Real browser interactions with a deterministic implementation of the existing HTTP contract.
// Network failures are injected without modifying or relying on a running C++ server.
test('editor, playback, synchronization and responsive canvas', { timeout: 60000 }, async () => {
  const root = path.resolve(__dirname, '..');
  const server = http.createServer(async (req, res) => {
    const file = req.url === '/' ? 'index.html' : req.url.slice(1);
    if (!['index.html', 'app.js', 'styles.css'].includes(file)) { res.writeHead(404).end(); return; }
    res.setHeader('Content-Type', file.endsWith('.js') ? 'text/javascript' : file.endsWith('.css') ? 'text/css' : 'text/html');
    res.end(await fs.readFile(path.join(root, file)));
  });
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  let browser;
  try {
    browser = await chromium.launch({ headless: true });
    const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    let config = null, snapshot = null, failConfig = false, hangStep = false;
    let shiftSnapshotAfterInit = true, mismatchConfigOnce = false;
    let inFlight = 0, maxInFlight = 0, stepRequests = 0;
    const sensors = ['front_left', 'front', 'front_right'].map((role, id) => ({
      id, role, relative_pose: { x: 0, y: 0, theta: (1 - id) * Math.PI / 4 },
      min_distance: 0, max_distance: 20, field_of_view: Math.PI / 2,
    }));
    function resetSnapshot() {
      snapshot = { robot_pose: { ...config.robot.pose }, robot_velocity: { ...config.robot.velocity },
        simulation_step: 0, simulation_time: 0, stopped: false, update: null };
    }
    await page.route('**/api/**', async route => {
      const endpoint = new URL(route.request().url()).pathname;
      const reply = (body, status = 200) => route.fulfill({ status, contentType: 'application/json', body: JSON.stringify(body) });
      if (endpoint.endsWith('/health')) return reply({ status: 'ok', backend: 'cpp' });
      if (endpoint.endsWith('/init')) {
        assert.equal(route.request().method(), 'POST');
        config = route.request().postDataJSON();
        assert.equal(config.sensors, undefined);
        resetSnapshot(); return reply(snapshot);
      }
      if (endpoint.endsWith('/config') && failConfig) return route.abort();
      if (!config) return reply({ error: 'Simulation is not initialized' }, 409);
      if (endpoint.endsWith('/config')) {
        if (mismatchConfigOnce) {
          mismatchConfigOnce = false;
          return reply({ ...config, environment: { ...config.environment, width: config.environment.width + 1 }, sensors });
        }
        if (shiftSnapshotAfterInit) {
          snapshot.robot_pose.x += 1;
          shiftSnapshotAfterInit = false;
        }
        return reply({ ...config, sensors });
      }
      if (endpoint.endsWith('/state')) return reply(snapshot);
      if (endpoint.endsWith('/reset')) { resetSnapshot(); return reply(snapshot); }
      if (endpoint.endsWith('/step')) {
        stepRequests++;
        if (hangStep) return;
        inFlight++; maxInFlight = Math.max(maxInFlight, inFlight);
        await delay(60);
        snapshot.simulation_step++;
        snapshot.simulation_time = snapshot.simulation_step * config.simulation.dt;
        snapshot.robot_pose.x += 0.1;
        snapshot.update = { step_accepted: true, stop_reason: 'None', raycast_results: sensors.map(s => ({
          sensor_id: s.id, object_detected: true, distance_to_object: 3, detection_type: 'OBJECTDETECTED',
        })) };
        inFlight--; return reply(snapshot);
      }
      throw new Error(`Unexpected endpoint ${endpoint}`);
    });
    const url = `http://127.0.0.1:${server.address().port}`;
    await page.goto(url);
    await page.waitForFunction(() => !document.querySelector('#initialize').disabled);
    await page.click('#initialize');
    await page.waitForFunction(() => !document.querySelector('#step').disabled);
    assert.equal(await page.locator('#metric-position').innerText(), '-7.00, 0.00');
    await page.uncheck('#show-sensors');
    const coordinates = await page.evaluate(() => {
      const center = worldToScreen({ x: 0, y: 0 });
      const upper = worldToScreen({ x: 0, y: 2 });
      const robot = worldToScreen(state.snapshot.robot_pose);
      const obstacle = worldToScreen(state.active.obstacles[0].pose);
      const context = document.querySelector('#scene-canvas').getContext('2d');
      const ratio = devicePixelRatio;
      const colorAt = point => Array.from(context.getImageData(point.x * ratio, point.y * ratio, 1, 1).data).slice(0, 3);
      return { center, upper, roundTrip: screenToWorld(robot.x, robot.y),
        robotColor: colorAt(robot), obstacleColor: colorAt(obstacle), view: state.view };
    });
    assert.equal(coordinates.center.x, coordinates.view.centerX);
    assert.equal(coordinates.center.y, coordinates.view.centerY);
    assert.ok(coordinates.upper.y < coordinates.center.y);
    assert.ok(Math.abs(coordinates.roundTrip.x + 7) < 1e-9);
    assert.deepEqual(coordinates.robotColor, [226, 166, 94]);
    assert.deepEqual(coordinates.obstacleColor, [191, 112, 104]);
    await page.check('#show-sensors');
    await page.click('#step');
    await page.waitForFunction(() => document.querySelector('#metric-step').textContent === '1');
    await page.selectOption('#pace', '100');
    await page.click('#start');
    await page.waitForFunction(() => Number(document.querySelector('#metric-step').textContent) >= 3);
    await page.click('#pause');
    await page.waitForFunction(() => !document.querySelector('#reset').disabled);
    const paused = stepRequests; await delay(300); assert.equal(stepRequests, paused);
    assert.equal(maxInFlight, 1);
    await page.click('#reset');
    await page.waitForFunction(() => document.querySelector('#metric-step').textContent === '0');
    const x = page.locator('[data-path="robot.pose.x"]');
    await x.fill('');
    assert.equal(await page.locator('#add-obstacle').isDisabled(), true);
    assert.equal(await page.locator('#reset').isDisabled(), false);
    await page.click('#reset');
    assert.equal(await x.inputValue(), '');
    await page.click('#undo'); assert.equal(Number(await x.inputValue()), config.robot.pose.x);
    await x.fill('-7');
    await page.click('#discard'); assert.equal(Number(await x.inputValue()), config.robot.pose.x);
    await page.click('#add-obstacle');
    assert.equal(await page.locator('.obstacle-card').count(), 2);
    await page.locator('[data-duplicate="1"]').click();
    assert.equal(await page.locator('.obstacle-card').count(), 3);
    await page.click('#undo'); assert.equal(await page.locator('.obstacle-card').count(), 2);
    const downloadEvent = page.waitForEvent('download');
    await page.click('#export-scene');
    const download = await downloadEvent;
    const exported = JSON.parse(await fs.readFile(await download.path(), 'utf8'));
    assert.equal(exported.obstacles.length, 2); assert.equal(exported.sensors, undefined);
    exported.robot.pose.x = -6;
    await page.locator('#scene-file').setInputFiles({ name: 'scene.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify(exported)) });
    await page.waitForFunction(() => document.querySelector('[data-path="robot.pose.x"]').value === '-6');
    await page.locator('#scene-file').setInputFiles({ name: 'bad.json', mimeType: 'application/json', buffer: Buffer.from('{}') });
    await page.waitForFunction(() => document.querySelector('#message').textContent.includes('Import odrzucony'));
    assert.equal(await x.inputValue(), '-6');
    mismatchConfigOnce = true;
    await page.click('#initialize');
    await page.waitForFunction(() => document.querySelector('#run-state').textContent === 'Wymaga synchronizacji');
    await page.click('#reconnect');
    await page.waitForFunction(() => !document.querySelector('#step').disabled);
    failConfig = true;
    await page.click('#initialize');
    await page.waitForFunction(() => document.querySelector('#run-state').textContent === 'Wymaga synchronizacji');
    assert.equal(await page.locator('#step').isDisabled(), true);
    failConfig = false;
    await page.click('#reconnect');
    await page.waitForFunction(() => !document.querySelector('#step').disabled);
    hangStep = true;
    const beforeTimeout = stepRequests;
    await page.click('#step');
    await page.waitForFunction(() => document.querySelector('#message').textContent.includes('8 s'), null, { timeout: 12000 });
    assert.equal(stepRequests, beforeTimeout + 1);
    hangStep = false;
    await page.click('#reconnect');
    await page.waitForFunction(() => !document.querySelector('#step').disabled);
    await page.click('#zoom-in'); await page.click('#fit-scene');
    await page.uncheck('#show-sensors'); await page.check('#show-sensors');
    await page.locator('[data-path="robot.pose.theta"]').fill('0.3');
    const box = await page.locator('#scene-canvas').boundingBox();
    const scale = Math.min((box.width - 76) / 30, (box.height - 76) / 12);
    await page.mouse.move(box.x + box.width / 2 - 6 * scale, box.y + box.height / 2);
    await page.mouse.down();
    await page.mouse.move(box.x + box.width / 2 + 15 * scale, box.y + box.height / 2, { steps: 4 });
    await page.mouse.up();
    const boundaryX = Number(await x.inputValue());
    assert.ok(boundaryX + (2 * Math.cos(0.3) + 1.5 * Math.sin(0.3)) / 2 <= 15 + 1e-12);
    assert.ok(boundaryX > 13);
    await page.click('#discard');
    await page.screenshot({ path: '/tmp/robot-frontend-desktop.png', fullPage: true });
    await page.setViewportSize({ width: 390, height: 844 });
    await page.screenshot({ path: '/tmp/robot-frontend-mobile.png', fullPage: true });
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true);
    assert.equal(await page.evaluate(() => document.querySelector('.stage-column').getBoundingClientRect().top < document.querySelector('.editor').getBoundingClientRect().top), true);
    assert.deepEqual(errors, []);
  } finally {
    if (browser) await browser.close();
    await new Promise(resolve => server.close(resolve));
  }
});
