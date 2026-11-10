"use strict";

// Mirrors scenarios/defaul.json for a new, uninitialized browser session.
const defaultScene = {
  environment: { length: 30, width: 12 },
  robot: {
    pose: { x: -8, y: 0, theta: 0 },
    size: { length: 2, width: 1.5 },
    velocity: { linear_velocity: 2, angular_velocity: 0 },
  },
  obstacles: [{ pose: { x: 8, y: 0, theta: 0 }, size: { length: 2, width: 4 } }],
  simulation: { dt: 0.5, ray_count: 3 },
  controller: { safe_distance: 5, critical_distance: 2, velocity_gain_critical: 0.2 },
};

const el = (id) => document.getElementById(id);
const canvas = el("scene-canvas");
const ctx = canvas.getContext("2d");
const form = el("scene-form");
const state = {
  draft: structuredClone(defaultScene),
  active: null,
  sensors: [],
  snapshot: null,
  trajectory: [],
  connected: false,
  dirty: true,
  invalid: false,
  busy: false,
  running: false,
  timer: null,
  drag: null,
  view: null,
  needsSync: false,
  history: [],
  selected: null,
  zoom: 1,
  pan: { x: 0, y: 0 },
  animation: null,
  frame: null,
};

function sceneFromConfig(config) {
  return {
    environment: { length: config.environment.length, width: config.environment.width },
    robot: {
      pose: { x: config.robot.pose.x, y: config.robot.pose.y, theta: config.robot.pose.theta },
      size: { length: config.robot.size.length, width: config.robot.size.width },
      velocity: {
        linear_velocity: config.robot.velocity.linear_velocity,
        angular_velocity: config.robot.velocity.angular_velocity,
      },
    },
    obstacles: config.obstacles.map((obstacle) => ({
      pose: { x: obstacle.pose.x, y: obstacle.pose.y, theta: obstacle.pose.theta },
      size: { length: obstacle.size.length, width: obstacle.size.width },
    })),
    simulation: { dt: config.simulation.dt, ray_count: config.simulation.ray_count },
    controller: {
      safe_distance: config.controller.safe_distance,
      critical_distance: config.controller.critical_distance,
      velocity_gain_critical: config.controller.velocity_gain_critical,
    },
  };
}

function updateDirty() {
  state.dirty = !state.active || JSON.stringify(state.draft) !== JSON.stringify(state.active);
}

function getPath(object, path) {
  return path.split(".").reduce((value, key) => value[key], object);
}

function setPath(object, path, value) {
  const keys = path.split(".");
  let parent = object;
  for (const key of keys.slice(0, -1)) parent = parent[key];
  parent[keys.at(-1)] = value;
}

function format(value, digits = 2) {
  return Number.isFinite(value) ? Number(value).toFixed(digits) : "—";
}

function setMessage(message, error = false) {
  const target = el("message");
  target.textContent = message;
  target.classList.toggle("error", error);
}

function setConnection(connected) {
  state.connected = connected;
  const target = el("connection");
  target.className = `connection ${connected ? "online" : "offline"}`;
  target.lastChild.textContent = connected ? "Backend połączony" : "Brak połączenia";
  renderControls();
}

async function request(path, options = {}) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), 8000);
  try {
    const response = await fetch(path, { cache: "no-store", ...options, signal: controller.signal });
    setConnection(true);
    const body = await response.json();
    if (!response.ok) {
      const error = new Error(body?.error || `HTTP ${response.status}`);
      error.status = response.status;
      throw error;
    }
    if (!body || typeof body !== "object") throw new Error("Nieprawidłowa odpowiedź JSON.");
    return body;
  } catch (error) {
    if (!error.status) {
      setConnection(false);
      throw new Error(error.name === "AbortError" ?
        "Serwer nie odpowiedział w ciągu 8 s. Zsynchronizuj sesję przed kolejną operacją." :
        "Nie udało się odczytać odpowiedzi serwera. Sprawdź połączenie i zsynchronizuj sesję.");
    }
    throw error;
  } finally {
    clearTimeout(timer);
  }
}

function rememberEdit() {
  state.history.push(structuredClone(state.draft));
  if (state.history.length > 50) state.history.shift();
}

function selectObject(selection, scroll = false) {
  state.selected = selection;
  document.querySelectorAll(".obstacle-card").forEach((card, index) => {
    card.classList.toggle("selected", selection?.type === "obstacle" && selection.index === index);
  });
  const input = form.querySelector(`[data-path="${selection?.type === "robot" ? "robot.pose.x" :
    `obstacles.${selection?.index}.pose.x`}"]`);
  if (scroll && input) {
    const section = input.closest("details");
    if (section) section.open = true;
    input.scrollIntoView({ block: "nearest", behavior: "smooth" });
  }
  renderCanvas();
}

function acceptSnapshot(snapshot, animate = false) {
  const previous = state.snapshot?.robot_pose;
  state.snapshot = snapshot;
  state.animation = animate && previous && el("smooth-motion").checked &&
    !window.matchMedia("(prefers-reduced-motion: reduce)").matches ?
    { from: previous, to: snapshot.robot_pose, start: performance.now(), duration: 100 } : null;
}

function displayedPose() {
  const animation = state.animation;
  if (!animation) return state.snapshot?.robot_pose;
  const t = Math.min(1, (performance.now() - animation.start) / animation.duration);
  if (t >= 1) { state.animation = null; return animation.to; }
  return Object.fromEntries(["x", "y", "theta"].map((key) =>
    [key, animation.from[key] + (animation.to[key] - animation.from[key]) * t]));
}

function renderObstacleEditors() {
  const list = el("obstacle-list");
  if (!state.draft.obstacles.length) {
    list.innerHTML = '<p class="obstacle-empty">Brak przeszkód w scenie</p>';
    return;
  }
  list.innerHTML = state.draft.obstacles.map((obstacle, index) => `
    <div class="obstacle-card">
      <div class="obstacle-card-head"><button class="select-obstacle" type="button" data-select="${index}">Przeszkoda ${index + 1}</button><button class="duplicate-obstacle" type="button" data-duplicate="${index}" aria-label="Duplikuj przeszkodę ${index + 1}">Kopiuj</button><button class="remove-obstacle" type="button" data-remove="${index}" aria-label="Usuń przeszkodę ${index + 1}">Usuń</button></div>
      <div class="fields three">
        <label>X <input type="number" data-path="obstacles.${index}.pose.x" value="${obstacle.pose.x}" step="any" required></label>
        <label>Y <input type="number" data-path="obstacles.${index}.pose.y" value="${obstacle.pose.y}" step="any" required></label>
        <label>θ · rad <input type="number" data-path="obstacles.${index}.pose.theta" value="${obstacle.pose.theta}" step="any" required></label>
      </div>
      <div class="fields two spaced">
        <label>Długość <input type="number" data-path="obstacles.${index}.size.length" value="${obstacle.size.length}" min="0" step="any" required></label>
        <label>Szerokość <input type="number" data-path="obstacles.${index}.size.width" value="${obstacle.size.width}" min="0" step="any" required></label>
      </div>
    </div>`).join("");
}

function fillForm() {
  renderObstacleEditors();
  form.querySelectorAll("[data-path]").forEach((input) => {
    input.value = getPath(state.draft, input.dataset.path);
  });
  state.invalid = false;
  el("editor-error").hidden = true;
  selectObject(state.selected);
  renderAll();
}

function validateScene(scene) {
  if (scene.environment.length <= 0 || scene.environment.width <= 0 ||
      scene.robot.size.length <= 0 || scene.robot.size.width <= 0 ||
      scene.obstacles.some((obstacle) => obstacle.size.length <= 0 || obstacle.size.width <= 0)) {
    return "Wymiary środowiska i obiektów muszą być dodatnie.";
  }
  if (scene.simulation.dt <= 0) return "Krok czasu dt musi być dodatni.";
  if (!Number.isInteger(scene.simulation.ray_count) || scene.simulation.ray_count < 1 ||
      scene.simulation.ray_count % 2 === 0) return "Liczba promieni musi być dodatnia i nieparzysta.";
  if (Math.abs(scene.robot.velocity.angular_velocity) > 2 * Math.PI) {
    return "Prędkość kątowa robota musi być w zakresie −2π…2π.";
  }
  if (scene.controller.safe_distance < 0 || scene.controller.critical_distance < 0 ||
      scene.controller.velocity_gain_critical < 0) return "Parametry kontrolera nie mogą być ujemne.";
  if (scene.controller.safe_distance < scene.controller.critical_distance) {
    return "Strefa bezpieczna musi być nie mniejsza od krytycznej.";
  }
  if (scene.controller.velocity_gain_critical > 0.5) {
    return "Wzmocnienie krytyczne musi być w zakresie 0…0,5.";
  }
  return null;
}

function readDraftFromForm(showError = false) {
  const next = structuredClone(state.draft);
  let invalidInput = null;
  form.querySelectorAll("[data-path]").forEach((input) => {
    if (!input.checkValidity() || !Number.isFinite(input.valueAsNumber)) {
      invalidInput ||= input;
    } else {
      setPath(next, input.dataset.path, input.valueAsNumber);
    }
  });
  const error = invalidInput ? "Uzupełnij poprawnie wszystkie pola liczbowe." : validateScene(next);
  state.invalid = Boolean(error);
  el("editor-error").hidden = !error;
  el("editor-error").textContent = error || "";
  if (error) {
    if (showError && invalidInput) invalidInput.focus();
    renderControls();
    return false;
  }
  state.draft = next;
  updateDirty();
  renderAll();
  return true;
}

function renderControls() {
  const active = Boolean(state.active);
  const ready = !state.needsSync && state.connected && active && !state.dirty && !state.invalid && !state.busy && !state.snapshot?.stopped;
  el("initialize").disabled = state.needsSync || !state.connected || state.busy || state.running;
  el("start").disabled = !ready || state.running;
  el("step").disabled = !ready || state.running;
  el("pause").disabled = !state.running;
  el("reset").disabled = state.needsSync || !state.connected || !active || state.busy || state.running;
  form.querySelectorAll("input, button").forEach((input) => { input.disabled = state.busy || state.running; });
  el("add-obstacle").disabled = state.busy || state.running || state.invalid;
  form.querySelectorAll(".remove-obstacle, .duplicate-obstacle").forEach((button) => {
    button.disabled = state.busy || state.running || state.invalid;
  });
  for (const id of ["import-scene", "export-scene", "discard", "undo", "reconnect"]) {
    el(id).disabled = state.busy || state.running;
  }
  el("export-scene").disabled ||= state.invalid;
  el("undo").disabled ||= !state.history.length;
  el("discard").disabled ||= !state.dirty && !state.invalid;
  el("telemetry-context").textContent = state.needsSync ? "Stan niepotwierdzony — zsynchronizuj sesję." :
    state.dirty || state.invalid ? "Dane poprzedniej aktywnej sesji. Plansza pokazuje projekt." : "Dane aktywnej sesji";
  el("draft-badge").textContent = !active ? "Scena domyślna" : state.dirty ? "Niezapisane zmiany" : "Scena aktywna";
  el("draft-badge").classList.toggle("dirty", state.dirty && active);
  el("view-mode").textContent = state.dirty ? "Podgląd projektu" : "Aktywna symulacja";
  el("view-mode").classList.toggle("dirty", state.dirty);
  const runState = el("run-state");
  runState.textContent = state.needsSync ? "Wymaga synchronizacji" : !active ? "Nie zainicjowano" : state.running ? "W ruchu" :
    state.snapshot?.stopped ? "Zatrzymano" : state.dirty ? "Projekt zmieniony" : "Gotowa";
  runState.className = `run-state ${state.running ? "running" : state.snapshot?.stopped ? "stopped" : ""}`;
}

function renderTelemetry() {
  const snapshot = state.snapshot;
  el("metric-step").textContent = snapshot ? snapshot.simulation_step : "—";
  el("metric-time").textContent = snapshot ? format(snapshot.simulation_time) : "—";
  el("metric-position").textContent = snapshot ? `${format(snapshot.robot_pose.x)}, ${format(snapshot.robot_pose.y)}` : "—";
  el("metric-heading").textContent = snapshot ? `${format(snapshot.robot_pose.theta)} rad` : "—";
  el("metric-linear").textContent = snapshot ? `${format(snapshot.robot_velocity.linear_velocity)} j.d./j.t.` : "—";
  el("metric-angular").textContent = snapshot ? `${format(snapshot.robot_velocity.angular_velocity)} rad/j.t.` : "—";

  const labels = { front_left: "Lewy przedni", front: "Przedni", front_right: "Prawy przedni" };
  const results = new Map((snapshot?.update?.raycast_results || []).map((item) => [item.sensor_id, item]));
  el("sensor-list").innerHTML = state.sensors.length ? state.sensors.map((sensor) => {
    const result = results.get(sensor.id);
    const value = !result ? "—" : result.object_detected && Number.isFinite(result.distance_to_object) ?
      format(result.distance_to_object) : result.detection_type === "OBJECTOUTOFRANGE" ? "poza zakresem" : "brak echa";
    return `<div class="sensor-item"><span class="sensor-icon" aria-hidden="true">⌁</span><span class="sensor-name">${labels[sensor.role] || sensor.role}<small>ID ${sensor.id} · θ ${format(sensor.relative_pose.theta)} rad</small></span><span class="sensor-distance">${value}</span></div>`;
  }).join("") : '<p class="muted">Dane pojawią się po inicjalizacji.</p>';

  const reason = snapshot?.update?.stop_reason;
  const messages = { None: snapshot?.update?.step_accepted ? "Krok zaakceptowany." : "Brak kroku.",
    OutOfEnvironment: "Ruch odrzucony: robot opuściłby środowisko.",
    ObstacleCollision: "Ruch odrzucony: kolizja z przeszkodą." };
  el("stop-reason").textContent = reason ? (messages[reason] || reason) : "Brak pomiaru.";
  el("stop-reason").classList.toggle("alert", Boolean(snapshot?.stopped));
}

function stopPlayback(message) {
  state.running = false;
  clearTimeout(state.timer);
  state.timer = null;
  if (message) setMessage(message);
  renderControls();
}

async function initialize() {
  if (state.busy || state.running || !readDraftFromForm(true)) return;
  state.busy = true;
  renderControls();
  state.needsSync = true;
  try {
    const requestedScene = structuredClone(state.draft);
    await request("/api/simulation/init", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(requestedScene),
    });
    const config = await request("/api/simulation/config");
    const snapshot = await request("/api/simulation/state");
    const activeScene = sceneFromConfig(config);
    if (JSON.stringify(activeScene) !== JSON.stringify(requestedScene)) {
      throw new Error("Aktywna konfiguracja zmieniła się podczas inicjalizacji. Zsynchronizuj sesję.");
    }
    state.active = activeScene;
    state.draft = structuredClone(state.active);
    state.sensors = config.sensors || [];
    acceptSnapshot(snapshot);
    state.needsSync = false;
    state.history = [];
    state.selected = null;
    state.trajectory = [snapshot.robot_pose];
    updateDirty();
    fillForm();
    setMessage("Scena zainicjowana. Użyj Start lub Step.");
  } catch (error) {
    if (error.status === 400) state.needsSync = false;
    setMessage(`Inicjalizacja nie została potwierdzona: ${error.message}`, true);
  } finally {
    state.busy = false;
    renderAll();
  }
}

async function performStep() {
  if (state.busy || !state.active || state.snapshot?.stopped) return false;
  state.busy = true;
  renderControls();
  state.needsSync = true;
  try {
    const snapshot = await request("/api/simulation/step", { method: "POST" });
    acceptSnapshot(snapshot, true);
    state.needsSync = false;
    if (snapshot.update?.step_accepted) {
      state.trajectory.push(snapshot.robot_pose);
      if (state.trajectory.length > 5000) state.trajectory.shift();
    }
    if (snapshot.stopped) {
      stopPlayback("Symulacja zatrzymana. Zobacz przyczynę w telemetrii lub użyj Reset.");
      return false;
    }
    setMessage(`Wykonano krok ${snapshot.simulation_step}.`);
    return true;
  } catch (error) {
    stopPlayback(`Nie udało się wykonać kroku: ${error.message}`);
    el("message").classList.add("error");
    return false;
  } finally {
    state.busy = false;
    renderAll();
  }
}

async function runLoop() {
  if (!state.running) return;
  const keepGoing = await performStep();
  if (state.running && keepGoing) state.timer = setTimeout(runLoop, Number(el("pace").value));
}

async function reset() {
  if (state.busy || state.running || !state.active || state.needsSync) return;
  state.busy = true;
  renderControls();
  state.needsSync = true;
  try {
    acceptSnapshot(await request("/api/simulation/reset", { method: "POST" }));
    state.needsSync = false;
    state.trajectory = [state.snapshot.robot_pose];
    setMessage(state.dirty || state.invalid ?
      "Aktywna sesja została zresetowana; niezastosowany projekt pozostał w edytorze." :
      "Sesja przywrócona do początku aktywnej sceny.");
  } catch (error) {
    setMessage(`Nie udało się zresetować: ${error.message}`, true);
  } finally {
    state.busy = false;
    renderAll();
  }
}

async function synchronize() {
  if (state.busy || state.running) return;
  state.busy = true;
  state.needsSync = true;
  renderControls();
  const keepDraft = state.dirty || state.invalid;
  try {
    await request("/api/health");
    let config;
    try { config = await request("/api/simulation/config"); }
    catch (error) { if (error.status !== 409) throw error; }
    if (config) {
      const snapshot = await request("/api/simulation/state");
      state.active = sceneFromConfig(config);
      state.sensors = config.sensors || [];
      acceptSnapshot(snapshot);
      state.trajectory = [snapshot.robot_pose];
      if (!keepDraft) { state.draft = structuredClone(state.active); fillForm(); }
      updateDirty();
      setMessage("Sesja zsynchronizowana. Ślad zaczyna się od bieżącej pozycji. Projekt edytora został zachowany.");
    } else {
      state.active = null;
      state.sensors = [];
      state.snapshot = null;
      state.animation = null;
      state.trajectory = [];
      updateDirty();
      setMessage("Serwer nie ma aktywnej sesji. Kliknij Initialize, aby zastosować projekt.");
    }
    state.needsSync = false;
  } catch (error) { setMessage(error.message, true); }
  finally { state.busy = false; renderAll(); }
}

async function bootstrap() {
  fillForm();
  // Initial load adopts the server scene; later synchronization preserves edits.
  state.dirty = false;
  await synchronize();
  if (!state.active) { updateDirty(); renderAll(); }
}

function worldToScreen(point) {
  const { centerX, centerY, scale } = state.view;
  return { x: centerX + point.x * scale, y: centerY - point.y * scale };
}

function screenToWorld(x, y) {
  const { centerX, centerY, scale } = state.view;
  return { x: (x - centerX) / scale, y: (centerY - y) / scale };
}

function gridStep(scale) {
  const target = 38 / scale;
  const power = 10 ** Math.floor(Math.log10(target));
  return [1, 2, 5, 10].find((step) => step * power >= target) * power;
}

function drawGrid(scene, left, top, width, height) {
  ctx.save();
  ctx.beginPath();
  ctx.rect(left, top, width, height);
  ctx.clip();
  ctx.fillStyle = "#173841";
  ctx.fillRect(left, top, width, height);
  const step = gridStep(state.view.scale);
  ctx.strokeStyle = el("show-grid").checked ? "rgba(156, 211, 207, .10)" : "transparent";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = Math.ceil(-scene.environment.length / 2 / step) * step;
       x <= scene.environment.length / 2; x += step) {
    const px = worldToScreen({ x, y: 0 }).x;
    ctx.moveTo(px, top); ctx.lineTo(px, top + height);
  }
  for (let y = Math.ceil(-scene.environment.width / 2 / step) * step;
       y <= scene.environment.width / 2; y += step) {
    const py = worldToScreen({ x: 0, y }).y;
    ctx.moveTo(left, py); ctx.lineTo(left + width, py);
  }
  ctx.stroke();
  ctx.strokeStyle = "rgba(160, 224, 218, .28)";
  ctx.beginPath();
  const axisX = worldToScreen({ x: 0, y: 0 }).x;
  const axisY = worldToScreen({ x: 0, y: 0 }).y;
  ctx.moveTo(axisX, top); ctx.lineTo(axisX, top + height);
  ctx.moveTo(left, axisY); ctx.lineTo(left + width, axisY);
  ctx.stroke();
  ctx.restore();
  ctx.strokeStyle = "#5d9293";
  ctx.lineWidth = 2;
  ctx.strokeRect(left, top, width, height);
  ctx.fillStyle = "#80aead";
  ctx.font = "11px ui-monospace, monospace";
  ctx.fillText(`X ${format(scene.environment.length, 1)}`, left + 8, top + height - 10);
  ctx.fillText(`Y ${format(scene.environment.width, 1)}`, left + 8, top + 17);
}

function drawRectangle(pose, size, fill, outline, label, selected = false) {
  const point = worldToScreen(pose);
  const length = size.length * state.view.scale;
  const width = size.width * state.view.scale;
  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.rotate(-pose.theta);
  ctx.fillStyle = fill;
  ctx.strokeStyle = selected ? "#fff0ad" : outline;
  ctx.lineWidth = selected ? 3 : 1.5;
  ctx.fillRect(-length / 2, -width / 2, length, width);
  ctx.strokeRect(-length / 2, -width / 2, length, width);
  if (label === "robot") {
    ctx.fillStyle = "#fff7da";
    ctx.beginPath();
    ctx.moveTo(length / 2 - 5, 0);
    ctx.lineTo(Math.max(0, length / 2 - 17), -Math.min(9, width / 3));
    ctx.lineTo(Math.max(0, length / 2 - 17), Math.min(9, width / 3));
    ctx.closePath();
    ctx.fill();
  }
  ctx.restore();
  ctx.fillStyle = "#d0e5df";
  ctx.font = "700 10px system-ui, sans-serif";
  ctx.fillText(label === "robot" ? "ROBOT" : label, point.x + 9, point.y - Math.max(12, width / 2 + 7));
}

function drawTrajectory() {
  if (state.trajectory.length < 2) return;
  ctx.save();
  ctx.strokeStyle = "#f7db9c";
  ctx.lineWidth = 2;
  ctx.lineJoin = "round";
  ctx.beginPath();
  state.trajectory.forEach((pose, index) => {
    const point = worldToScreen(pose);
    if (index === 0) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawSensors(robotPose) {
  if (!state.sensors.length) return;
  ctx.save();
  const scene = state.active;
  const topLeft = worldToScreen({ x: -scene.environment.length / 2, y: scene.environment.width / 2 });
  ctx.beginPath();
  ctx.rect(topLeft.x, topLeft.y, scene.environment.length * state.view.scale,
    scene.environment.width * state.view.scale);
  ctx.clip();
  for (const sensor of state.sensors) {
    const relative = sensor.relative_pose;
    const cos = Math.cos(robotPose.theta);
    const sin = Math.sin(robotPose.theta);
    const origin = {
      x: robotPose.x + relative.x * cos - relative.y * sin,
      y: robotPose.y + relative.x * sin + relative.y * cos,
    };
    const point = worldToScreen(origin);
    const angle = robotPose.theta + relative.theta;
    // The API has no hit direction: show geometry only, never an invented hit point.
    ctx.fillStyle = "rgba(106, 219, 208, .045)";
    ctx.strokeStyle = "rgba(135, 232, 217, .28)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(point.x, point.y, sensor.max_distance * state.view.scale,
      -angle - sensor.field_of_view / 2, -angle + sensor.field_of_view / 2);
    ctx.arc(point.x, point.y, sensor.min_distance * state.view.scale,
      -angle + sensor.field_of_view / 2, -angle - sensor.field_of_view / 2, true);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
}

function renderCanvas() {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (!width || !height) return;
  const ratio = window.devicePixelRatio || 1;
  const pixelWidth = Math.round(width * ratio);
  const pixelHeight = Math.round(height * ratio);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#102932";
  ctx.fillRect(0, 0, width, height);

  const scene = state.dirty ? state.draft : state.active;
  if (!scene) return;
  const scale = Math.min((width - 76) / scene.environment.length,
    (height - 76) / scene.environment.width) * state.zoom;
  state.view = { centerX: width / 2 + state.pan.x, centerY: height / 2 + state.pan.y, scale };
  const envWidth = scene.environment.length * scale;
  const envHeight = scene.environment.width * scale;
  drawGrid(scene, state.view.centerX - envWidth / 2, state.view.centerY - envHeight / 2, envWidth, envHeight);
  if (!state.dirty && el("show-trail").checked) drawTrajectory();

  scene.obstacles.forEach((obstacle, index) => {
    drawRectangle(obstacle.pose, obstacle.size, "#bf7068", "#f3aaa0", `P${index + 1}`,
      state.selected?.type === "obstacle" && state.selected.index === index);
  });
  const robotPose = !state.dirty && state.snapshot ? displayedPose() : scene.robot.pose;
  if (!state.dirty && !state.animation && el("show-sensors").checked) drawSensors(robotPose);
  drawRectangle(robotPose, scene.robot.size, state.snapshot?.stopped && !state.dirty ? "#d37261" : "#e2a65e",
    "#ffe0a3", "robot", state.selected?.type === "robot");
  if (state.animation && !state.dirty) {
    cancelAnimationFrame(state.frame);
    state.frame = requestAnimationFrame(renderCanvas);
  }
}

function renderAll() {
  renderControls();
  renderTelemetry();
  renderCanvas();
}

function insideRectangle(point, pose, size) {
  const dx = point.x - pose.x;
  const dy = point.y - pose.y;
  const cos = Math.cos(pose.theta);
  const sin = Math.sin(pose.theta);
  const localX = dx * cos + dy * sin;
  const localY = -dx * sin + dy * cos;
  return Math.abs(localX) <= size.length / 2 && Math.abs(localY) <= size.width / 2;
}

function clampPosition(point, size, theta) {
  const env = state.draft.environment;
  const cos = Math.abs(Math.cos(theta));
  const sin = Math.abs(Math.sin(theta));
  const halfX = (cos * size.length + sin * size.width) / 2;
  const halfY = (sin * size.length + cos * size.width) / 2;
  const maxX = Math.max(0, env.length / 2 - halfX);
  const maxY = Math.max(0, env.width / 2 - halfY);
  return { x: Math.max(-maxX, Math.min(maxX, point.x)),
    y: Math.max(-maxY, Math.min(maxY, point.y)) };
}

function pointerPoint(event) {
  const box = canvas.getBoundingClientRect();
  return screenToWorld(event.clientX - box.left, event.clientY - box.top);
}

canvas.addEventListener("pointerdown", (event) => {
  if (state.busy || state.running || !state.view || state.invalid) return;
  const point = pointerPoint(event);
  const scene = state.dirty ? state.draft : state.active;
  const robotPose = !state.dirty && state.snapshot ? displayedPose() : scene.robot.pose;
  state.animation = null;
  let target = null;
  for (let index = scene.obstacles.length - 1; index >= 0; index--) {
    if (insideRectangle(point, scene.obstacles[index].pose, scene.obstacles[index].size)) {
      target = { type: "obstacle", index, pose: scene.obstacles[index].pose };
      break;
    }
  }
  if (!target && insideRectangle(point, robotPose, scene.robot.size)) {
    target = { type: "robot", pose: robotPose };
  }
  if (!target) {
    state.drag = { type: "pan", startX: event.clientX, startY: event.clientY, pan: { ...state.pan } };
    canvas.setPointerCapture(event.pointerId);
    return;
  }
  selectObject({ type: target.type, index: target.index }, true);
  state.drag = { ...target, offsetX: point.x - target.pose.x, offsetY: point.y - target.pose.y };
  canvas.setPointerCapture(event.pointerId);
  canvas.classList.add("dragging");
  event.preventDefault();
  renderCanvas();
});

canvas.addEventListener("pointermove", (event) => {
  if (!state.drag) return;
  const drag = state.drag;
  if (drag.type === "pan") {
    state.pan = { x: drag.pan.x + event.clientX - drag.startX, y: drag.pan.y + event.clientY - drag.startY };
    renderCanvas();
    return;
  }
  if (!drag.saved) { rememberEdit(); drag.saved = true; }
  const point = pointerPoint(event);
  const object = drag.type === "robot" ? state.draft.robot : state.draft.obstacles[drag.index];
  if (drag.type === "robot") object.pose.theta = drag.pose.theta;
  const next = clampPosition({ x: point.x - drag.offsetX, y: point.y - drag.offsetY },
    object.size, object.pose.theta);
  object.pose.x = next.x;
  object.pose.y = next.y;
  for (const coordinate of ["x", "y", "theta"]) {
    const path = drag.type === "robot" ? `robot.pose.${coordinate}` :
      `obstacles.${drag.index}.pose.${coordinate}`;
    form.querySelector(`[data-path="${path}"]`).value = object.pose[coordinate];
  }
  updateDirty();
  renderAll();
});

function endDrag(event) {
  if (!state.drag) return;
  state.drag = null;
  canvas.classList.remove("dragging");
  if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
  renderCanvas();
}
canvas.addEventListener("pointerup", endDrag);
canvas.addEventListener("pointercancel", endDrag);

let editingRemembered = false;
form.addEventListener("focusin", () => { editingRemembered = false; });
form.addEventListener("input", () => {
  if (!editingRemembered) { rememberEdit(); editingRemembered = true; }
  if (readDraftFromForm() && state.dirty && state.active) {
    setMessage("Projekt został zmieniony. Kliknij Initialize, aby zastosować go na serwerze.");
  }
});
form.addEventListener("submit", (event) => event.preventDefault());
el("obstacle-list").addEventListener("click", (event) => {
  const selection = event.target.closest("[data-select]");
  if (selection) { selectObject({ type: "obstacle", index: Number(selection.dataset.select) }); return; }
  const duplicate = event.target.closest("[data-duplicate]");
  if (duplicate && !state.busy && !state.running && readDraftFromForm(true)) {
    rememberEdit();
    const copy = structuredClone(state.draft.obstacles[Number(duplicate.dataset.duplicate)]);
    copy.pose.x += copy.size.length + 0.2;
    state.draft.obstacles.push(copy);
    state.selected = { type: "obstacle", index: state.draft.obstacles.length - 1 };
    updateDirty(); fillForm(); return;
  }
  const button = event.target.closest("[data-remove]");
  if (!button || state.running || state.busy || !readDraftFromForm(true)) return;
  rememberEdit();
  state.selected = null;
  state.draft.obstacles.splice(Number(button.dataset.remove), 1);
  updateDirty();
  fillForm();
});
el("add-obstacle").addEventListener("click", () => {
  if (state.running || state.busy || !readDraftFromForm(true)) return;
  rememberEdit();
  const env = state.draft.environment;
  state.draft.obstacles.push({ pose: { x: Math.min(3, env.length / 4),
    y: Math.min(2, env.width / 4), theta: 0 }, size: { length: 2, width: 2 } });
  updateDirty();
  fillForm();
});
el("initialize").addEventListener("click", initialize);
el("start").addEventListener("click", () => {
  if (state.running || state.busy || state.dirty || !state.active || state.snapshot?.stopped) return;
  state.running = true;
  setMessage("Symulacja działa. Kolejne kroki są wysyłane sekwencyjnie.");
  renderControls();
  runLoop();
});
el("pause").addEventListener("click", () => stopPlayback("Symulacja wstrzymana."));
el("step").addEventListener("click", () => { if (!state.running && !state.dirty) performStep(); });
el("reset").addEventListener("click", reset);


el("reconnect").addEventListener("click", synchronize);
el("undo").addEventListener("click", () => {
  if (state.busy || state.running || !state.history.length) return;
  state.draft = state.history.pop();
  state.selected = null;
  updateDirty(); fillForm();
  setMessage("Cofnięto edycję projektu. Aktywna sesja nie została zmieniona.");
});
el("discard").addEventListener("click", () => {
  if (state.busy || state.running) return;
  rememberEdit();
  state.draft = structuredClone(state.active || defaultScene);
  state.selected = null;
  updateDirty(); fillForm();
  setMessage("Przywrócono konfigurację projektu.");
});
el("import-scene").addEventListener("click", () => el("scene-file").click());
el("scene-file").addEventListener("change", async (event) => {
  const file = event.target.files[0];
  event.target.value = "";
  if (!file || state.busy || state.running) return;
  state.busy = true;
  renderControls();
  try {
    if (file.size > 1024 * 1024) throw new Error("Plik sceny może mieć maksymalnie 1 MB.");
    const scene = sceneFromConfig(JSON.parse(await file.text()));
    function finiteNumbers(value) {
      return typeof value === "number" ? Number.isFinite(value) :
        value !== null && typeof value === "object" && Object.values(value).every(finiteNumbers);
    }
    if (!finiteNumbers(scene)) throw new Error("Wszystkie parametry sceny muszą być skończonymi liczbami.");
    const error = validateScene(scene);
    if (error) throw new Error(error);
    rememberEdit();
    state.draft = scene;
    state.selected = null;
    updateDirty(); fillForm();
    setMessage("Wczytano projekt. Initialize zastosuje go na serwerze.");
  } catch (error) {
    setMessage(`Import odrzucony: ${error.message}. Poprzedni projekt został zachowany.`, true);
  } finally { state.busy = false; renderAll(); }
});
el("export-scene").addEventListener("click", () => {
  if (!readDraftFromForm(true)) return;
  const url = URL.createObjectURL(new Blob([JSON.stringify(state.draft, null, 2)], { type: "application/json" }));
  const link = document.createElement("a");
  link.href = url; link.download = "scene.json"; link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});
function changeZoom(factor) {
  state.zoom = Math.max(0.5, Math.min(6, state.zoom * factor));
  renderCanvas();
}
el("zoom-in").addEventListener("click", () => changeZoom(1.25));
el("zoom-out").addEventListener("click", () => changeZoom(0.8));
el("fit-scene").addEventListener("click", () => {
  state.zoom = 1; state.pan = { x: 0, y: 0 }; renderCanvas();
});
canvas.addEventListener("wheel", (event) => {
  event.preventDefault(); changeZoom(event.deltaY < 0 ? 1.1 : 1 / 1.1);
}, { passive: false });
for (const id of ["show-grid", "show-sensors", "show-trail", "smooth-motion"]) {
  el(id).addEventListener("change", () => {
    if (id === "smooth-motion") state.animation = null;
    renderCanvas();
  });
}
// Collapsible sections retain their form fields and validation behavior.
form.querySelectorAll("fieldset").forEach((fieldset, index) => {
  const details = document.createElement("details");
  details.className = "editor-section";
  details.open = index < 2 || index === 4;
  const summary = document.createElement("summary");
  summary.textContent = fieldset.querySelector("legend").textContent;
  fieldset.before(details);
  details.append(summary, fieldset);
});
new ResizeObserver(renderCanvas).observe(canvas);
bootstrap();
