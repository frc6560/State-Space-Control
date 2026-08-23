/** Stable signal names shared by the simulator and AdvantageKit robot IO. */
export const telemetryKeys = Object.freeze({
  referenceVx: "Drive/LQR/ReferenceVxMetersPerSec", referenceVy: "Drive/LQR/ReferenceVyMetersPerSec",
  referenceOmega: "Drive/LQR/ReferenceOmegaRadPerSec", forceX: "Drive/LQR/ControlForceXNewtons",
  forceY: "Drive/LQR/ControlForceYNewtons", torque: "Drive/LQR/ControlTorqueNewtonMeters", pose: "Drive/Odometry/Pose"
});
export function moduleTelemetryKey(moduleName, signal) { return `Drive/Modules/${moduleName}/${signal}`; }
