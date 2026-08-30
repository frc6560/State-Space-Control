# Intake command-ready integration

Branch: agent/intake-command-ready

## Controls

Recommended driver-Xbox mapping:

| Control | Mode |
|---|---|
| A | Retracted, roller stopped |
| B | Extended, roller stopped |
| Right trigger | Extended and roller spinning |
| Y | 0.10 m peak-to-peak extension oscillation, roller stopped |
| Back | Retracted and stopped |

## Hardware source

The extension-bearing intake implementation in frc6560/Robot-Code-2026 identifies:

- Extension motor: CAN ID 15, integrated TalonFX encoder.
- Spin motor: CAN ID 16.
- CAN bus: rio.
- Existing spin target: 2,500 RPM; this branch caps it at 1,250 RPM.
- The later ventura-ready branch instead exposes roller IDs 24 and 25, so those are not used for this extension implementation.

## Encoder and soft-limit assumptions

- Extension position is motor rotations converted through a 64:14 reduction and a 1.751 inch pinion.
- The encoder must be zeroed while the mechanism is physically fully retracted using zeroExtensionEncoder().
- Maximum extension reference and forward soft limit: 0.50 m.
- Reverse soft limit: 0.0 m / 0.0 motor rotations.
- Runtime control also blocks motion beyond either limit.
- Oscillation center: 0.45 m.
- Oscillation amplitude: 0.05 m, giving 0.10 m peak-to-peak travel.
- Every motor output is bounded to plus/minus 6 V.
- No autonomous encoder zeroing is performed because no confirmed retract limit switch was selected for this branch.
- Verify encoder sign, zero, gear ratio, hard stops, and soft-limit threshold on blocks before enabling motor power.

## Integration

Place the Java files under the robot project's frc.robot.intake package. Instantiate:

IntakeStateSpaceSubsystem intake =
    new IntakeStateSpaceSubsystem(new TalonFXRollerIO());

Before enabling the subsystem, physically retract the intake and call:

((TalonFXRollerIO) io).zeroExtensionEncoder();

Bind the Xbox controls with IntakeXboxBindings.configure(controller, intake).
