import test from "node:test";
import assert from "node:assert/strict";
import { applyCircularDeadband, characterize, createController, fieldToRobot, inverseKinematics, lqrStep, processAnalogInput, updateSteering, wrapAngle } from "../src/physics.mjs";
import config from "../config/swervedrive.json" with { type: "json" };
import frontLeft from "../config/modules/frontleft.json" with { type: "json" };
import frontRight from "../config/modules/frontright.json" with { type: "json" };
import backLeft from "../config/modules/backleft.json" with { type: "json" };
import backRight from "../config/modules/backright.json" with { type: "json" };

config.modules = [frontLeft, frontRight, backLeft, backRight];

test("mass model preserves 140 lb total and expected yaw inertia", () => {
  const model = characterize(config);
  assert.ok(Math.abs(model.centerMassKg + 4 * config.chassis.moduleMassKg - 63.5029) < 1e-9);
  assert.ok(Math.abs(model.yawInertiaKgM2 - 7.09) < 0.02);
});

test("LQR closes velocity error and respects effort saturation", () => {
  const controller = createController(config);
  let state = { vx:0, vy:0,omega:0 };
  const reference = { vx:3,vy:-1,omega:1.5 };
  for (let i = 0; i < 250; i += 1) state = lqrStep(state, reference, config, controller);
  assert.ok(Math.abs(state.vx - reference.vx) < 0.02);
  assert.ok(Math.abs(state.vy - reference.vy) < 0.02);
  assert.ok(Math.abs(state.omega - reference.omega) < 0.02);
  assert.ok(Math.abs(state.effort.forceX) <= config.limits.maxForceNewtons);
  assert.ok(Math.abs(state.effort.torque) <= config.limits.maxTorqueNewtonMeters);
});

test("inverse kinematics gives equal wheel speeds in translation", () => {
  const states = inverseKinematics({ vx:2,vy:0,omega:0 }, config.modules, 4.5);
  states.forEach((state) => { assert.equal(state.speed, 2); assert.equal(state.angle, 0); });
});

test("pure rotation produces tangent module states", () => {
  const states = inverseKinematics({ vx:0,vy:0,omega:1 }, config.modules, 4.5);
  const expectedSpeed = Math.hypot(.381, .381);
  states.forEach((state) => assert.ok(Math.abs(state.speed - expectedSpeed) < 1e-12));
  assert.ok(Math.abs(wrapAngle(states[0].angle - 3 * Math.PI / 4)) < 1e-12);
});

test("wheel speeds desaturate proportionally", () => {
  const states = inverseKinematics({ vx:9,vy:0,omega:0 }, config.modules, 4.5);
  states.forEach((state) => assert.equal(state.speed, 4.5));
});

test("field-to-robot transform and steering rate limit are consistent", () => {
  const body = fieldToRobot(1, 0, Math.PI / 2);
  assert.ok(Math.abs(body.vx) < 1e-12); assert.ok(Math.abs(body.vy + 1) < 1e-12);
  assert.ok(Math.abs(updateSteering(0, Math.PI, 1, .02, 1)) <= .020000001);
});

test("analog heading vector preserves an arbitrary 35.7 degree target", () => {
  const targetRadians = 35.7 * Math.PI / 180;
  const reference = processAnalogInput({
    translationX: 0.357,
    translationY: 0.42,
    headingX: Math.cos(targetRadians),
    headingY: Math.sin(targetRadians)
  }, 0, config);
  assert.ok(Math.abs(reference.targetHeading - targetRadians) < 1e-12);
  assert.ok(reference.omega > 0);
  assert.ok(reference.vx > 0 && reference.vy > 0);
});

test("analog deadband retains direction and scales magnitude continuously", () => {
  assert.deepEqual(applyCircularDeadband(.02, -.03, .08), { x:0, y:0 });
  const output = applyCircularDeadband(.35, .7, .08);
  assert.ok(Math.abs(output.y / output.x - 2) < 1e-12);
  assert.ok(Math.hypot(output.x, output.y) > 0 && Math.hypot(output.x, output.y) < 1);
});
