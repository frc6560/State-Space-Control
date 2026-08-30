export const IntakeMode = Object.freeze({
  RETRACTED_NOT_SPINNING: "retracted-not-spinning",
  EXTENDED_NOT_SPINNING: "extended-not-spinning",
  EXTENDED_SPINNING: "extended-spinning",
  SHOOTING_OSCILLATION: "shooting-oscillation",
});

const clamp = (value, minimum, maximum) => Math.min(maximum, Math.max(minimum, value));

function transpose2(matrix) {
  return [
    [matrix[0][0], matrix[1][0]],
    [matrix[0][1], matrix[1][1]],
  ];
}

function multiply2(left, right) {
  return [
    [left[0][0] * right[0][0] + left[0][1] * right[1][0], left[0][0] * right[0][1] + left[0][1] * right[1][1]],
    [left[1][0] * right[0][0] + left[1][1] * right[1][0], left[1][0] * right[0][1] + left[1][1] * right[1][1]],
  ];
}

function add2(left, right) {
  return [
    [left[0][0] + right[0][0], left[0][1] + right[0][1]],
    [left[1][0] + right[1][0], left[1][1] + right[1][1]],
  ];
}

function subtract2(left, right) {
  return [
    [left[0][0] - right[0][0], left[0][1] - right[0][1]],
    [left[1][0] - right[1][0], left[1][1] - right[1][1]],
  ];
}

export function solveDiscreteLqr2x1(A, B, qPosition, qVelocity, rEffort) {
  const Q = [[qPosition, 0], [0, qVelocity]];
  let P = [[...Q[0]], [...Q[1]]];
  const AT = transpose2(A);

  for (let iteration = 0; iteration < 10000; iteration += 1) {
    const PA = multiply2(P, A);
    const ATPA = multiply2(AT, PA);
    const PB = [P[0][0] * B[0] + P[0][1] * B[1], P[1][0] * B[0] + P[1][1] * B[1]];
    const denominator = rEffort + B[0] * PB[0] + B[1] * PB[1];
    const BTPA = [B[0] * PA[0][0] + B[1] * PA[1][0], B[0] * PA[0][1] + B[1] * PA[1][1]];
    const ATPB = [AT[0][0] * PB[0] + AT[0][1] * PB[1], AT[1][0] * PB[0] + AT[1][1] * PB[1]];
    const correction = [
      [ATPB[0] * BTPA[0] / denominator, ATPB[0] * BTPA[1] / denominator],
      [ATPB[1] * BTPA[0] / denominator, ATPB[1] * BTPA[1] / denominator],
    ];
    const nextP = add2(Q, subtract2(ATPA, correction));
    const difference = Math.max(
      Math.abs(nextP[0][0] - P[0][0]), Math.abs(nextP[0][1] - P[0][1]),
      Math.abs(nextP[1][0] - P[1][0]), Math.abs(nextP[1][1] - P[1][1]),
    );
    P = nextP;
    if (difference < 1e-12) break;
  }

  const PA = multiply2(P, A);
  const PB = [P[0][0] * B[0] + P[0][1] * B[1], P[1][0] * B[0] + P[1][1] * B[1]];
  const denominator = rEffort + B[0] * PB[0] + B[1] * PB[1];
  return [
    (B[0] * PA[0][0] + B[1] * PA[1][0]) / denominator,
    (B[0] * PA[0][1] + B[1] * PA[1][1]) / denominator,
  ];
}

export function solveScalarDiscreteLqr(a, b, q, r) {
  let p = q;
  for (let iteration = 0; iteration < 10000; iteration += 1) {
    const nextP = q + a * a * p - (a * b * p) ** 2 / (r + b * b * p);
    if (Math.abs(nextP - p) < 1e-12) {
      p = nextP;
      break;
    }
    p = nextP;
  }
  return (b * p * a) / (r + b * b * p);
}

export function createIntakeModel(config) {
  const dt = config.loopPeriodSeconds;
  const mechanism = config.mechanism;
  const roller = config.roller;
  const accelerationPerVolt = mechanism.estimatedForcePerVoltNewtons / mechanism.estimatedMovingMassKg;
  const velocityDecay = mechanism.estimatedViscousDampingNewtonSecondsPerMeter / mechanism.estimatedMovingMassKg;
  const deployA = [[1, dt], [0, 1 - velocityDecay * dt]];
  const deployB = [0.5 * accelerationPerVolt * dt * dt, accelerationPerVolt * dt];
  const rollerA = Math.exp(-dt / roller.estimatedTimeConstantSeconds);
  const rollerB = (1 - rollerA) * roller.estimatedFreeSpeedRotationsPerSecondAt12Volts / 12;

  return {
    deployA,
    deployB,
    rollerA,
    rollerB,
    A: [
      [deployA[0][0], deployA[0][1], 0],
      [deployA[1][0], deployA[1][1], 0],
      [0, 0, rollerA],
    ],
    B: [
      [deployB[0], 0],
      [deployB[1], 0],
      [0, rollerB],
    ],
    C: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    D: [[0, 0], [0, 0], [0, 0]],
  };
}

export class IntakeController {
  constructor(config) {
    this.config = config;
    this.model = createIntakeModel(config);
    const controller = config.controller;
    const [positionTolerance, velocityTolerance] = controller.deploymentStateTolerance;
    this.deployGain = solveDiscreteLqr2x1(
      this.model.deployA,
      this.model.deployB,
      1 / positionTolerance ** 2,
      1 / velocityTolerance ** 2,
      1 / controller.deploymentControlEffortToleranceVolts ** 2,
    );
    this.rollerGain = solveScalarDiscreteLqr(
      this.model.rollerA,
      this.model.rollerB,
      1 / controller.rollerSpeedToleranceRotationsPerSecond ** 2,
      1 / controller.rollerControlEffortToleranceVolts ** 2,
    );
    this.mode = IntakeMode.RETRACTED_NOT_SPINNING;
    this.elapsedSeconds = 0;
  }

  setMode(mode) {
    if (!Object.values(IntakeMode).includes(mode)) throw new Error(`Unknown intake mode: ${mode}`);
    this.mode = mode;
    this.elapsedSeconds = 0;
  }

  getReference() {
    const config = this.config;
    if (this.mode === IntakeMode.RETRACTED_NOT_SPINNING) return [config.mechanism.minimumExtensionMeters, 0, 0];
    if (this.mode === IntakeMode.EXTENDED_NOT_SPINNING) return [config.mechanism.maximumExtensionMeters, 0, 0];
    if (this.mode === IntakeMode.EXTENDED_SPINNING) {
      return [config.mechanism.maximumExtensionMeters, 0, config.roller.targetSpeedRotationsPerSecond];
    }
    const oscillation = config.shootingOscillation;
    const phase = 2 * Math.PI * oscillation.frequencyHz * this.elapsedSeconds;
    const position = oscillation.centerExtensionMeters + oscillation.amplitudeMeters * Math.sin(phase);
    const velocity = 2 * Math.PI * oscillation.frequencyHz * oscillation.amplitudeMeters * Math.cos(phase);
    const rollerSpeed = oscillation.rollerRunsDuringOscillation ? config.roller.targetSpeedRotationsPerSecond : 0;
    return [position, velocity, rollerSpeed];
  }

  update(measuredState) {
    const reference = this.getReference();
    const maximumVoltage = this.config.controller.maximumVoltage;
    const deploymentVoltage = clamp(
      this.deployGain[0] * (reference[0] - measuredState[0]) + this.deployGain[1] * (reference[1] - measuredState[1]),
      -maximumVoltage,
      maximumVoltage,
    );
    const rollerFeedforward = reference[2] * (1 - this.model.rollerA) / this.model.rollerB;
    const rollerVoltage = clamp(
      rollerFeedforward + this.rollerGain * (reference[2] - measuredState[2]),
      -maximumVoltage,
      maximumVoltage,
    );
    this.elapsedSeconds += this.config.loopPeriodSeconds;
    return { reference, deploymentVoltage, rollerVoltage, mode: this.mode };
  }
}

export class IntakePlant {
  constructor(config) {
    this.config = config;
    this.model = createIntakeModel(config);
    this.state = [config.mechanism.minimumExtensionMeters, 0, 0];
  }

  reset(state = [this.config.mechanism.minimumExtensionMeters, 0, 0]) {
    this.state = [...state];
  }

  update(deploymentVoltage, rollerVoltage) {
    const maximumVoltage = this.config.controller.maximumVoltage;
    let deployInput = clamp(deploymentVoltage, -maximumVoltage, maximumVoltage);
    const rollerInput = clamp(rollerVoltage, -maximumVoltage, maximumVoltage);
    const [position, velocity, rollerSpeed] = this.state;
    const minimum = this.config.mechanism.minimumExtensionMeters;
    const maximum = this.config.mechanism.maximumExtensionMeters;
    if ((position <= minimum && deployInput < 0) || (position >= maximum && deployInput > 0)) deployInput = 0;

    let nextPosition = this.model.deployA[0][0] * position + this.model.deployA[0][1] * velocity + this.model.deployB[0] * deployInput;
    let nextVelocity = this.model.deployA[1][0] * position + this.model.deployA[1][1] * velocity + this.model.deployB[1] * deployInput;
    if (nextPosition <= minimum) {
      nextPosition = minimum;
      nextVelocity = Math.max(0, nextVelocity);
    } else if (nextPosition >= maximum) {
      nextPosition = maximum;
      nextVelocity = Math.min(0, nextVelocity);
    }
    const nextRollerSpeed = this.model.rollerA * rollerSpeed + this.model.rollerB * rollerInput;
    this.state = [nextPosition, nextVelocity, nextRollerSpeed];
    return [...this.state];
  }
}

export class IntakeSimulation {
  constructor(config) {
    this.controller = new IntakeController(config);
    this.plant = new IntakePlant(config);
    this.lastOutput = null;
  }

  setMode(mode) {
    this.controller.setMode(mode);
  }

  step() {
    const output = this.controller.update(this.plant.state);
    const state = this.plant.update(output.deploymentVoltage, output.rollerVoltage);
    // Keep the cached output in the same shape returned by step(). The browser
    // animation loop reads lastOutput between fixed-rate physics ticks.
    this.lastOutput = { ...output, state };
    return this.lastOutput;
  }
}
