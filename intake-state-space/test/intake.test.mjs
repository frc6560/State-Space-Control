import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import {
  createIntakeModel,
  IntakeController,
  IntakeMode,
  IntakeSimulation,
} from "../src/intake.mjs";

const config = JSON.parse(await readFile(new URL("../config/intake.json", import.meta.url)));

function runFor(simulation, seconds) {
  const samples = [];
  for (let index = 0; index < seconds / config.loopPeriodSeconds; index += 1) samples.push(simulation.step());
  return samples;
}

test("publishes explicit three-state, two-input state-space matrices", () => {
  const model = createIntakeModel(config);
  assert.deepEqual(model.A.map((row) => row.length), [3, 3, 3]);
  assert.deepEqual(model.B.map((row) => row.length), [2, 2, 2]);
  assert.deepEqual(model.C, [[1, 0, 0], [0, 1, 0], [0, 0, 1]]);
  assert.deepEqual(model.D, [[0, 0], [0, 0], [0, 0]]);
});

test("LQR gains are finite and use both deployment states", () => {
  const controller = new IntakeController(config);
  assert.ok(controller.deployGain.every(Number.isFinite));
  assert.ok(controller.deployGain[0] > 0);
  assert.ok(controller.deployGain[1] > 0);
  assert.ok(Number.isFinite(controller.rollerGain));
  assert.ok(controller.rollerGain > 0);
});

test("extended and spinning reaches both references with voltage limits", () => {
  const simulation = new IntakeSimulation(config);
  simulation.setMode(IntakeMode.EXTENDED_SPINNING);
  const samples = runFor(simulation, 3);
  const final = samples.at(-1);
  assert.ok(Math.abs(final.state[0] - config.mechanism.maximumExtensionMeters) < 0.012);
  assert.ok(Math.abs(final.state[2] - config.roller.targetSpeedRotationsPerSecond) < 2.0);
  assert.ok(samples.every((sample) => Math.abs(sample.deploymentVoltage) <= 12 && Math.abs(sample.rollerVoltage) <= 12));
});

test("extended and not spinning holds extension while stopping the roller", () => {
  const simulation = new IntakeSimulation(config);
  simulation.setMode(IntakeMode.EXTENDED_SPINNING);
  runFor(simulation, 2);
  simulation.setMode(IntakeMode.EXTENDED_NOT_SPINNING);
  const samples = runFor(simulation, 2);
  const final = samples.at(-1);
  assert.ok(Math.abs(final.state[0] - config.mechanism.maximumExtensionMeters) < 0.012);
  assert.ok(Math.abs(final.state[2]) < 1.0);
});

test("retracted and not spinning returns to the hard-safe state", () => {
  const simulation = new IntakeSimulation(config);
  simulation.setMode(IntakeMode.EXTENDED_SPINNING);
  runFor(simulation, 2);
  simulation.setMode(IntakeMode.RETRACTED_NOT_SPINNING);
  const samples = runFor(simulation, 3);
  const final = samples.at(-1);
  assert.ok(final.state[0] < 0.012);
  assert.ok(Math.abs(final.state[2]) < 1.0);
  assert.ok(samples.every((sample) => sample.state[0] >= 0 && sample.state[0] <= config.mechanism.maximumExtensionMeters));
});

test("shooting mode oscillates the rack forward and backward without rollers", () => {
  const simulation = new IntakeSimulation(config);
  simulation.setMode(IntakeMode.EXTENDED_NOT_SPINNING);
  runFor(simulation, 2);
  simulation.setMode(IntakeMode.SHOOTING_OSCILLATION);
  const samples = runFor(simulation, 3);
  const positions = samples.slice(50).map((sample) => sample.state[0]);
  const velocities = samples.slice(50).map((sample) => sample.state[1]);
  assert.ok(Math.max(...positions) - Math.min(...positions) > 0.07);
  assert.ok(velocities.some((velocity) => velocity > 0.05));
  assert.ok(velocities.some((velocity) => velocity < -0.05));
  assert.ok(Math.max(...samples.map((sample) => Math.abs(sample.state[2]))) < 0.1);
});

test("unknown modes fail safe instead of silently commanding hardware", () => {
  const controller = new IntakeController(config);
  assert.throws(() => controller.setMode("banana"), /Unknown intake mode/);
});
