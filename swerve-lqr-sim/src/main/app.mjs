import { createController, createMatchRecorder, fieldToRobot, inverseKinematics, loadConfiguration, lqrStep, processAnalogInput, simulateModule } from "./subsystems/swerve/physics.mjs";
import { readDriverInput } from "./commands/driveCommand.mjs";

const canvas = document.querySelector("#field");
const ctx = canvas.getContext("2d");
const pressed = new Set();
const field = { width: 8.21, height: 4.11 };
let config;
let controller;
let state;
let modules;
let accumulator = 0;
let previous = performance.now();
let matchTime = 0;
let recorder = createMatchRecorder();

function resetPose() {
  state = { x: field.width / 2, y: field.height / 2, heading: 0, vx: 0, vy: 0, omega: 0,
    effort: { forceX: 0, forceY: 0, torque: 0 } };
  matchTime = 0; recorder.clear();
  if (config) modules = config.modules.map((module) => ({ ...module, angle: 0, speed: 0, roll: 0,
    drive: { appliedVoltage: 0, currentAmps: 0 }, steer: { appliedVoltage: 0, currentAmps: 0 } }));
}

function referenceFromKeys() {
  return processAnalogInput(readDriverInput(navigator.getGamepads(), pressed), state.heading, config);
}

function step() {
  const dt = config.plant.periodSeconds;
  const reference = referenceFromKeys();
  const next = lqrStep(state, reference, config, controller);
  Object.assign(state, next);
  state.x = (state.x + state.vx * dt + field.width) % field.width;
  state.y = (state.y + state.vy * dt + field.height) % field.height;
  state.heading += state.omega * dt;

  const body = fieldToRobot(state.vx, state.vy, state.heading);
  const targets = inverseKinematics({ ...body, omega: state.omega }, config.modules,
    config.limits.maxSpeedMetersPerSecond);
  modules = modules.map((module, index) => simulateModule(module, targets[index], config));
  matchTime += dt; recorder.record(matchTime, state, modules);
  updateTelemetry(reference);
}

function updateTelemetry(reference) {
  const set = (id, value) => { document.querySelector(id).textContent = value; };
  set("#vx", state.vx.toFixed(2)); set("#vxRef", `/ ${reference.vx.toFixed(2)} m/s`);
  set("#vy", state.vy.toFixed(2)); set("#vyRef", `/ ${reference.vy.toFixed(2)} m/s`);
  set("#omega", state.omega.toFixed(2)); set("#omegaRef", `/ ${reference.omega.toFixed(2)} rad/s`);
  const effort = Math.max(Math.abs(state.effort.forceX) / config.limits.maxForceNewtons,
    Math.abs(state.effort.forceY) / config.limits.maxForceNewtons,
    Math.abs(state.effort.torque) / config.limits.maxTorqueNewtonMeters);
  document.querySelector("#effort").style.width = `${Math.min(100, effort * 100)}%`;
  document.querySelector("#recording").textContent = `${recorder.samples.length} samples`;
  document.querySelector("#moduleTelemetry").innerHTML = modules.map((module) =>
    `<tr><th>${module.name}</th><td>${module.drive.appliedVoltage.toFixed(1)} V</td><td>${module.steer.appliedVoltage.toFixed(1)} V</td><td>${module.drive.currentAmps.toFixed(0)} A</td></tr>`).join("");
}

function resize() {
  const rect = canvas.getBoundingClientRect();
  const ratio = Math.min(2, window.devicePixelRatio || 1);
  canvas.width = Math.round(rect.width * ratio);
  canvas.height = Math.round(rect.height * ratio);
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
}

function drawGrid(width, height, scale, originX, originY) {
  ctx.fillStyle = "#0b1210"; ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "#18231f"; ctx.lineWidth = 1;
  for (let x = 0; x <= field.width; x += .5) {
    const px = originX + x * scale; ctx.beginPath(); ctx.moveTo(px, originY); ctx.lineTo(px, originY + field.height * scale); ctx.stroke();
  }
  for (let y = 0; y <= field.height; y += .5) {
    const py = originY + (field.height - y) * scale; ctx.beginPath(); ctx.moveTo(originX, py); ctx.lineTo(originX + field.width * scale, py); ctx.stroke();
  }
  ctx.strokeStyle = "#2a3b35"; ctx.lineWidth = 2; ctx.strokeRect(originX, originY, field.width * scale, field.height * scale);
  ctx.setLineDash([5, 7]); ctx.strokeStyle = "#26332f"; ctx.beginPath(); ctx.moveTo(originX + field.width * scale / 2, originY); ctx.lineTo(originX + field.width * scale / 2, originY + field.height * scale); ctx.stroke(); ctx.setLineDash([]);
}

function drawRobot(scale, originX, originY) {
  const px = originX + state.x * scale;
  const py = originY + (field.height - state.y) * scale;
  const robotScale = scale;
  ctx.save(); ctx.translate(px, py); ctx.rotate(-state.heading);
  const length = config.chassis.wheelbaseMeters * robotScale;
  const width = config.chassis.trackwidthMeters * robotScale;
  ctx.shadowColor = "#43e7d155"; ctx.shadowBlur = 24;
  ctx.fillStyle = "#14241f"; ctx.strokeStyle = "#43e7d1"; ctx.lineWidth = 2;
  ctx.fillRect(-length / 2, -width / 2, length, width); ctx.strokeRect(-length / 2, -width / 2, length, width);
  ctx.shadowBlur = 0;
  ctx.fillStyle = "#43e7d1"; ctx.beginPath(); ctx.moveTo(length / 2 + 10, 0); ctx.lineTo(length / 2 - 4, -7); ctx.lineTo(length / 2 - 4, 7); ctx.closePath(); ctx.fill();
  ctx.fillStyle = "#1b2d27"; ctx.beginPath(); ctx.arc(0, 0, config.chassis.centerDiskRadiusMeters * robotScale, 0, Math.PI * 2); ctx.fill(); ctx.strokeStyle = "#385149"; ctx.stroke();

  modules.forEach((module) => {
    const x = module.location.xMeters * robotScale;
    const y = -module.location.yMeters * robotScale;
    ctx.save(); ctx.translate(x, y); ctx.rotate(-module.angle);
    const wheelLength = Math.max(22, .28 * robotScale); const wheelWidth = Math.max(8, .1 * robotScale);
    ctx.fillStyle = "#070a09"; ctx.strokeStyle = "#a8b5b0"; ctx.lineWidth = 1.5;
    ctx.fillRect(-wheelLength / 2, -wheelWidth / 2, wheelLength, wheelWidth); ctx.strokeRect(-wheelLength / 2, -wheelWidth / 2, wheelLength, wheelWidth);
    ctx.strokeStyle = "#43e7d1"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(wheelLength / 2 + Math.min(28, module.speed * 7), 0); ctx.stroke();
    ctx.fillStyle = "#43e7d1"; ctx.beginPath(); ctx.moveTo(wheelLength / 2 + Math.min(28, module.speed * 7), 0); ctx.lineTo(wheelLength / 2 + Math.min(28, module.speed * 7) - 6, -4); ctx.lineTo(wheelLength / 2 + Math.min(28, module.speed * 7) - 6, 4); ctx.fill();
    ctx.strokeStyle = "#66736e"; ctx.lineWidth = 1; const stripe = ((module.roll % 8) - 4); ctx.beginPath(); ctx.moveTo(stripe, -wheelWidth / 2); ctx.lineTo(stripe, wheelWidth / 2); ctx.stroke();
    ctx.restore();
  });
  ctx.restore();
}

function render() {
  const width = canvas.clientWidth; const height = canvas.clientHeight;
  const margin = 42; const scale = Math.min((width - margin * 2) / field.width, (height - margin * 2) / field.height);
  const originX = (width - field.width * scale) / 2; const originY = (height - field.height * scale) / 2;
  drawGrid(width, height, scale, originX, originY); drawRobot(scale, originX, originY);
  if (recorder.samples.length > 1) {
    ctx.strokeStyle = "#f6b44a"; ctx.lineWidth = 1.5; ctx.beginPath();
    recorder.samples.forEach((sample, index) => { const x = originX + sample.pose.x * scale; const y = originY + (field.height - sample.pose.y) * scale; if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); }); ctx.stroke();
  }
}

function frame(now) {
  accumulator += Math.min(.1, (now - previous) / 1000); previous = now;
  while (accumulator >= config.plant.periodSeconds) { step(); accumulator -= config.plant.periodSeconds; }
  render(); requestAnimationFrame(frame);
}

function setKey(event, down) {
  const controlled = ["KeyW", "KeyA", "KeyS", "KeyD", "ArrowLeft", "ArrowRight", "Space", "KeyR"];
  if (!controlled.includes(event.code)) return;
  event.preventDefault();
  if (event.code === "Space" && down) { pressed.clear(); state.vx = 0; state.vy = 0; state.omega = 0; }
  else if (event.code === "KeyR" && down) resetPose();
  else if (down) pressed.add(event.code); else pressed.delete(event.code);
  document.querySelectorAll("kbd[data-key]").forEach((key) => key.classList.toggle("active", pressed.has(key.dataset.key)));
}

async function start() {
  config = await loadConfiguration("./src/main/deploy/swerve"); controller = createController(config); resetPose(); resize();
  const pounds = (kg) => kg * 2.2046226218;
  document.querySelector("#moduleMass").textContent = `4 × ${pounds(config.chassis.moduleMassKg).toFixed(1)} lb`;
  document.querySelector("#centerMass").textContent = `${pounds(controller.model.centerMassKg).toFixed(1)} lb`;
  document.querySelector("#inertia").textContent = `${controller.model.yawInertiaKgM2.toFixed(2)} kg·m²`;
  document.querySelector("#gains").textContent = `${controller.linearGain.toFixed(0)} N·s/m · ${controller.angularGain.toFixed(0)} N·m·s/rad`;
  window.addEventListener("resize", resize); window.addEventListener("keydown", (event) => setKey(event, true)); window.addEventListener("keyup", (event) => setKey(event, false));
  window.addEventListener("blur", () => pressed.clear()); document.querySelector("#reset").addEventListener("click", resetPose); canvas.addEventListener("pointerdown", () => canvas.focus());
  document.querySelector("#export").addEventListener("click", () => {
    const blob = new Blob([recorder.toJSON()], { type: "application/json" }); const url = URL.createObjectURL(blob);
    const link = Object.assign(document.createElement("a"), { href: url, download: "swerve-match.json" }); link.click(); URL.revokeObjectURL(url);
  });
  requestAnimationFrame(frame);
}

start().catch((error) => { document.body.innerHTML = `<pre class="fatal">${error.message}</pre>`; console.error(error); });
