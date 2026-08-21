import { IntakeMode, IntakeSimulation } from "./intake.mjs";
import { IntakeTelemetry } from "./telemetry.mjs";

const config = await fetch("./config/intake.json").then((response) => response.json());
const simulation = new IntakeSimulation(config);
const telemetry = new IntakeTelemetry(config);
const buttons = [...document.querySelectorAll("button[data-mode]")];
const position = document.querySelector("#position");
const target = document.querySelector("#target");
const velocity = document.querySelector("#velocity");
const roller = document.querySelector("#roller");
const deployVoltage = document.querySelector("#deploy-voltage");
const rollerVoltage = document.querySelector("#roller-voltage");
const carriage = document.querySelector("#carriage");
const rollers = document.querySelector("#rollers");
const status = document.querySelector("#status");
const sampleCount = document.querySelector("#sample-count");
const limitFlag = document.querySelector("#limit-flag");
const downloadCsv = document.querySelector("#download-csv");
const downloadJson = document.querySelector("#download-json");

function download(content, filename, mimeType) {
  const blob = new Blob([content], { type: mimeType });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob); link.download = filename; link.click();
  URL.revokeObjectURL(link.href);
}

function setMode(mode) {
  simulation.setMode(mode);
  buttons.forEach((button) => button.classList.toggle("active", button.dataset.mode === mode));
  status.textContent = mode.replaceAll("-", " ");
}

buttons.forEach((button) => button.addEventListener("click", () => setMode(button.dataset.mode)));
downloadCsv.addEventListener("click", () => download(telemetry.toCsv(), "intake-telemetry.csv", "text/csv"));
downloadJson.addEventListener("click", () => download(telemetry.toJson(), "intake-telemetry.json", "application/json"));
setMode(IntakeMode.RETRACTED_NOT_SPINNING);

let previousTime = performance.now();
let accumulator = 0;
function animate(now) {
  accumulator += Math.min((now - previousTime) / 1000, 0.1);
  previousTime = now;
  let sample = simulation.lastOutput;
  while (accumulator >= config.loopPeriodSeconds) {
    sample = simulation.step();
    const logged = telemetry.record(sample, telemetry.samples.length * config.loopPeriodSeconds);
    accumulator -= config.loopPeriodSeconds;
    sampleCount.textContent = `${telemetry.samples.length} samples`;
    limitFlag.textContent = logged["Intake/Safety/VoltageLimited"] ? "LIMIT ACTIVE" : "WITHIN LIMITS";
    limitFlag.classList.toggle("warning", logged["Intake/Safety/VoltageLimited"]);
  }
  if (sample) {
    const extensionPercent = sample.state[0] / config.mechanism.maximumExtensionMeters * 100;
    position.textContent = `${sample.state[0].toFixed(3)} m`;
    target.textContent = `${sample.reference[0].toFixed(3)} m`;
    velocity.textContent = `${sample.state[1].toFixed(2)} m/s`;
    roller.textContent = `${sample.state[2].toFixed(1)} rps`;
    deployVoltage.textContent = `${sample.deploymentVoltage.toFixed(1)} V`;
    rollerVoltage.textContent = `${sample.rollerVoltage.toFixed(1)} V`;
    carriage.style.left = `${extensionPercent}%`;
    rollers.style.animationDuration = `${Math.max(0.08, 2 / Math.max(1, Math.abs(sample.state[2])))}s`;
    rollers.classList.toggle("stopped", Math.abs(sample.state[2]) < 0.5);
  }
  requestAnimationFrame(animate);
}
requestAnimationFrame(animate);
