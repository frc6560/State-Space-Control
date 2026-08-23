const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

export function wrapAngle(radians) {
  return Math.atan2(Math.sin(radians), Math.cos(radians));
}

export function characterize(config) {
  const chassis = config.chassis;
  const moduleCount = config.modules.length;
  const centerMassKg = chassis.totalMassKg - moduleCount * chassis.moduleMassKg;
  if (centerMassKg <= 0) throw new Error("Corner module masses exceed total robot mass");

  const halfLength = chassis.wheelbaseMeters / 2;
  const halfWidth = chassis.trackwidthMeters / 2;
  const moduleRadiusSquared = halfLength ** 2 + halfWidth ** 2;
  const cornerInertia = moduleCount * chassis.moduleMassKg * moduleRadiusSquared;
  const diskInertia = 0.5 * centerMassKg * chassis.centerDiskRadiusMeters ** 2;

  return {
    centerMassKg,
    cornerInertiaKgM2: cornerInertia,
    diskInertiaKgM2: diskInertia,
    yawInertiaKgM2: cornerInertia + diskInertia
  };
}

export function discretizeDampedVelocity(mass, drag, period) {
  if (mass <= 0 || period <= 0 || drag < 0) throw new Error("Invalid plant parameters");
  if (drag === 0) return { a: 1, b: period / mass };
  const a = Math.exp((-drag / mass) * period);
  return { a, b: (1 - a) / drag };
}

export function solveScalarLqr(a, b, q, r) {
  if (q <= 0 || r <= 0) throw new Error("LQR costs must be positive");
  let p = q;
  for (let i = 0; i < 10000; i += 1) {
    const next = q + a * a * p - ((a * b * p) ** 2) / (r + b * b * p);
    if (Math.abs(next - p) < 1e-12) {
      p = next;
      break;
    }
    p = next;
  }
  return (b * p * a) / (r + b * b * p);
}

export function createController(config) {
  const model = characterize(config);
  const { periodSeconds: dt, linearDragNewtonSecondsPerMeter: linearDrag,
    angularDragNewtonMeterSecondsPerRadian: angularDrag } = config.plant;
  const linearPlant = discretizeDampedVelocity(config.chassis.totalMassKg, linearDrag, dt);
  const angularPlant = discretizeDampedVelocity(model.yawInertiaKgM2, angularDrag, dt);
  return {
    model,
    linearPlant,
    angularPlant,
    linearGain: solveScalarLqr(linearPlant.a, linearPlant.b,
      config.lqr.velocityStateCost, config.lqr.velocityEffortCost),
    angularGain: solveScalarLqr(angularPlant.a, angularPlant.b,
      config.lqr.omegaStateCost, config.lqr.omegaEffortCost)
  };
}

export function lqrStep(state, reference, config, controller) {
  const limits = config.limits;
  const forceX = clamp(
    config.plant.linearDragNewtonSecondsPerMeter * reference.vx
      + controller.linearGain * (reference.vx - state.vx),
    -limits.maxForceNewtons,
    limits.maxForceNewtons
  );
  const forceY = clamp(
    config.plant.linearDragNewtonSecondsPerMeter * reference.vy
      + controller.linearGain * (reference.vy - state.vy),
    -limits.maxForceNewtons,
    limits.maxForceNewtons
  );
  const torque = clamp(
    config.plant.angularDragNewtonMeterSecondsPerRadian * reference.omega
      + controller.angularGain * (reference.omega - state.omega),
    -limits.maxTorqueNewtonMeters,
    limits.maxTorqueNewtonMeters
  );

  return {
    vx: controller.linearPlant.a * state.vx + controller.linearPlant.b * forceX,
    vy: controller.linearPlant.a * state.vy + controller.linearPlant.b * forceY,
    omega: controller.angularPlant.a * state.omega + controller.angularPlant.b * torque,
    effort: { forceX, forceY, torque }
  };
}

export function fieldToRobot(vx, vy, heading) {
  const cos = Math.cos(heading);
  const sin = Math.sin(heading);
  return { vx: cos * vx + sin * vy, vy: -sin * vx + cos * vy };
}

export function applyCircularDeadband(x, y, deadband) {
  const magnitude = Math.hypot(x, y);
  if (magnitude <= deadband) return { x: 0, y: 0 };
  const scaledMagnitude = (Math.min(1, magnitude) - deadband) / (1 - deadband);
  return { x: (x / magnitude) * scaledMagnitude, y: (y / magnitude) * scaledMagnitude };
}

/**
 * Convert normalized, continuous driver inputs into field-relative chassis references.
 * A future controller adapter only needs to supply axes in [-1, 1]. When headingX/Y
 * are present, their polar angle is treated as the desired absolute field heading,
 * matching YAGSL's heading-vector style of control. Otherwise omega is an angular
 * velocity axis, which is how the six keyboard keys feed this same path.
 */
export function processAnalogInput(input, currentHeading, config) {
  const translation = applyCircularDeadband(
    clamp(input.translationX ?? 0, -1, 1),
    clamp(input.translationY ?? 0, -1, 1),
    config.input.translationDeadband
  );
  let omega = 0;
  let targetHeading = null;
  const headingX = clamp(input.headingX ?? 0, -1, 1);
  const headingY = clamp(input.headingY ?? 0, -1, 1);

  if (Math.hypot(headingX, headingY) > config.input.headingVectorDeadband) {
    targetHeading = Math.atan2(headingY, headingX);
    omega = clamp(
      config.input.headingProportionalGain * wrapAngle(targetHeading - currentHeading),
      -config.limits.keyboardOmegaRadiansPerSecond,
      config.limits.keyboardOmegaRadiansPerSecond
    );
  } else {
    const rawOmega = clamp(input.omega ?? 0, -1, 1);
    const magnitude = Math.abs(rawOmega);
    const scaledOmega = magnitude <= config.input.rotationDeadband
      ? 0
      : Math.sign(rawOmega) * (magnitude - config.input.rotationDeadband) / (1 - config.input.rotationDeadband);
    omega = scaledOmega * config.limits.keyboardOmegaRadiansPerSecond;
  }

  return {
    vx: translation.x * config.limits.keyboardSpeedMetersPerSecond,
    vy: translation.y * config.limits.keyboardSpeedMetersPerSecond,
    omega,
    targetHeading
  };
}

export function inverseKinematics(chassisSpeeds, modules, maxSpeed) {
  const states = modules.map((module) => {
    const wheelVx = chassisSpeeds.vx - chassisSpeeds.omega * module.location.yMeters;
    const wheelVy = chassisSpeeds.vy + chassisSpeeds.omega * module.location.xMeters;
    return {
      name: module.name,
      speed: Math.hypot(wheelVx, wheelVy),
      angle: Math.atan2(wheelVy, wheelVx),
      location: module.location
    };
  });
  const fastest = Math.max(0, ...states.map((state) => state.speed));
  const scale = fastest > maxSpeed ? maxSpeed / fastest : 1;
  return states.map((state) => ({ ...state, speed: state.speed * scale }));
}

export function updateSteering(currentAngle, targetAngle, maxRate, dt, speed) {
  if (speed < 0.015) return currentAngle;
  const error = wrapAngle(targetAngle - currentAngle);
  return wrapAngle(currentAngle + clamp(error, -maxRate * dt, maxRate * dt));
}

/**
 * Predict a motor's next output before it is applied to the simulated drivetrain.
 * This deliberately sits between a voltage command and the plant, mirroring the
 * measurement/actuation boundary that a real AdvantageKit IO implementation logs.
 */
export function predictMotorOutput({ velocity = 0 }, voltage, freeSpeed, timeConstant, resistance, dt, nominalVoltage = 12) {
  const appliedVoltage = clamp(voltage, -nominalVoltage, nominalVoltage);
  const targetVelocity = (appliedVoltage / nominalVoltage) * freeSpeed;
  const nextVelocity = velocity + (targetVelocity - velocity) * (1 - Math.exp(-dt / timeConstant));
  const backEmfVoltage = freeSpeed === 0 ? 0 : (velocity / freeSpeed) * nominalVoltage;
  const currentAmps = Math.max(0, Math.abs(appliedVoltage - backEmfVoltage) / resistance);
  return { appliedVoltage, velocity: nextVelocity, currentAmps, backEmfVoltage };
}

export function simulateModule(module, target, config) {
  const dt = config.plant.periodSeconds;
  const electrical = config.electrical;
  const speedError = target.speed - (module.driveVelocity ?? 0);
  const driveVoltage = clamp(
    (target.speed / config.limits.maxSpeedMetersPerSecond) * electrical.nominalVoltage + speedError * 1.4,
    -electrical.nominalVoltage, electrical.nominalVoltage
  );
  const angleError = wrapAngle(target.angle - module.angle);
  const steerVoltage = clamp(angleError * 4.2, -electrical.nominalVoltage, electrical.nominalVoltage);
  const drive = predictMotorOutput(module.drive ?? { velocity: module.driveVelocity ?? 0 }, driveVoltage,
    config.limits.maxSpeedMetersPerSecond, electrical.driveTimeConstantSeconds,
    electrical.driveResistanceOhms, dt, electrical.nominalVoltage);
  const steer = predictMotorOutput(module.steer ?? { velocity: module.steerVelocity ?? 0 }, steerVoltage,
    electrical.maxSteerRateRadiansPerSecond, electrical.steerTimeConstantSeconds,
    electrical.steerResistanceOhms, dt, electrical.nominalVoltage);
  return {
    ...module,
    targetSpeed: target.speed,
    targetAngle: target.angle,
    speed: drive.velocity,
    driveVelocity: drive.velocity,
    steerVelocity: steer.velocity,
    angle: wrapAngle(module.angle + steer.velocity * dt),
    drive,
    steer,
    roll: (module.roll ?? 0) + drive.velocity * dt * 7
  };
}

export function createMatchRecorder() {
  const samples = [];
  return {
    record(timestampSeconds, pose, modules) {
      samples.push({ timestampSeconds, pose: { x: pose.x, y: pose.y, heading: pose.heading },
        moduleVoltages: modules.map(({ name, drive, steer }) => ({ name, driveVolts: drive?.appliedVoltage ?? 0,
          steerVolts: steer?.appliedVoltage ?? 0 })) });
    },
    clear() { samples.length = 0; },
    toJSON() { return JSON.stringify({ format: "advantagekit-swerve-match-v1", samples }, null, 2); },
    get samples() { return samples; }
  };
}

export async function loadConfiguration(baseUrl = "./config") {
  const driveResponse = await fetch(`${baseUrl}/swervedrive.json`);
  if (!driveResponse.ok) throw new Error(`Unable to load swervedrive.json (${driveResponse.status})`);
  const config = await driveResponse.json();
  config.modules = await Promise.all(config.modules.map(async (path) => {
    const response = await fetch(`${baseUrl}/${path}`);
    if (!response.ok) throw new Error(`Unable to load ${path} (${response.status})`);
    return response.json();
  }));
  return config;
}
