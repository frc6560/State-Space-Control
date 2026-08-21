const CSV_FIELDS = [
  "timestampSeconds", "Intake/Mode", "Intake/State/ExtensionMeters", "Intake/State/VelocityMetersPerSecond",
  "Intake/State/RollerSpeedRps", "Intake/Reference/ExtensionMeters", "Intake/Reference/VelocityMetersPerSecond",
  "Intake/Reference/RollerSpeedRps", "Intake/Error/ExtensionMeters", "Intake/Error/VelocityMetersPerSecond",
  "Intake/Error/RollerSpeedRps", "Intake/Controller/X44Voltage", "Intake/Controller/X60Voltage",
  "Intake/Safety/AtMinimumExtension", "Intake/Safety/AtMaximumExtension", "Intake/Safety/VoltageLimited",
];

const csvEscape = (value) => {
  const text = String(value ?? "");
  return /[",\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
};

export const INTAKE_TELEMETRY_FIELDS = Object.freeze(CSV_FIELDS);

export function buildTelemetrySample(config, simulationSample, timestampSeconds) {
  const [extension, velocity, rollerSpeed] = simulationSample.state;
  const [targetExtension, targetVelocity, targetRollerSpeed] = simulationSample.reference;
  const maximumVoltage = config.controller.maximumVoltage;
  return {
    timestampSeconds,
    "Intake/Mode": simulationSample.mode,
    "Intake/State/ExtensionMeters": extension,
    "Intake/State/VelocityMetersPerSecond": velocity,
    "Intake/State/RollerSpeedRps": rollerSpeed,
    "Intake/Reference/ExtensionMeters": targetExtension,
    "Intake/Reference/VelocityMetersPerSecond": targetVelocity,
    "Intake/Reference/RollerSpeedRps": targetRollerSpeed,
    "Intake/Error/ExtensionMeters": targetExtension - extension,
    "Intake/Error/VelocityMetersPerSecond": targetVelocity - velocity,
    "Intake/Error/RollerSpeedRps": targetRollerSpeed - rollerSpeed,
    "Intake/Controller/X44Voltage": simulationSample.deploymentVoltage,
    "Intake/Controller/X60Voltage": simulationSample.rollerVoltage,
    "Intake/Safety/AtMinimumExtension": extension <= config.mechanism.minimumExtensionMeters + 1e-9,
    "Intake/Safety/AtMaximumExtension": extension >= config.mechanism.maximumExtensionMeters - 1e-9,
    "Intake/Safety/VoltageLimited": Math.abs(simulationSample.deploymentVoltage) >= maximumVoltage - 1e-9
      || Math.abs(simulationSample.rollerVoltage) >= maximumVoltage - 1e-9,
  };
}

export class IntakeTelemetry {
  constructor(config, maximumSamples = 10000) {
    this.config = config;
    this.maximumSamples = maximumSamples;
    this.samples = [];
  }

  record(simulationSample, timestampSeconds) {
    const sample = buildTelemetrySample(this.config, simulationSample, timestampSeconds);
    this.samples.push(sample);
    if (this.samples.length > this.maximumSamples) this.samples.shift();
    return sample;
  }

  latest() {
    return this.samples.at(-1) ?? null;
  }

  clear() {
    this.samples.length = 0;
  }

  toCsv() {
    return [
      CSV_FIELDS.join(","),
      ...this.samples.map((sample) => CSV_FIELDS.map((field) => csvEscape(sample[field])).join(",")),
    ].join("\n") + "\n";
  }

  toJson() {
    return JSON.stringify({
      format: "frc6560-intake-telemetry-v1",
      source: "State-Space-Control/intake-state-space",
      fields: CSV_FIELDS,
      samples: this.samples,
    }, null, 2);
  }
}
