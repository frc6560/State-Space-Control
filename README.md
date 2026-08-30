# State-Space-Control

Interactive state-space control experiments for FRC mechanisms.

## Projects in this branch

- [Swerve-drive LQR simulation](swerve-lqr-sim/README.md) — browser simulation with keyboard driving, chassis velocity control, and four-module inverse kinematics.
- [Intake state-space simulation](intake-state-space/README.md) — browser model of the intake mechanism.
- [Robot intake integration](robot-intake/CONTROLS_AND_ASSUMPTIONS.md) — Java files intended to be copied into the WPILib robot project.

## Intake controller: intended functionality

The intake has two controlled mechanisms:

1. An extension motor moves the intake between 0.0 m and 0.50 m.
2. A spin motor runs the intake roller.

The controller uses the extension motor's integrated TalonFX encoder to estimate extension position. It applies state-feedback control to extension position/velocity and roller velocity, with conservative output limits while the mechanism is being validated.

## Controller bindings

These bindings are implemented in robot-intake/IntakeXboxBindings.java.

| Control | While pressed | When released |
|---|---|---|
| B | Extend and spin the roller at half speed | Remain extended; stop the roller |
| A | Retract completely and stop | Remain retracted |
| Right trigger | Oscillate the extension while held; roller stopped | Return to extended; remain stopped |
| Back | Retract completely and stop | Remain retracted |

The right trigger oscillation is centered at 0.45 m with an amplitude of 0.05 m, giving 0.10 m peak-to-peak motion.

## Hardware mapping

The CAN IDs were taken from the extension-bearing intake implementations in frc6560/Robot-Code-2026:

| Device | CAN ID | Function |
|---|---:|---|
| TalonFX | 15 | Extension motor and integrated encoder |
| TalonFX | 16 | Intake spin motor |

CAN bus: rio.

The later ventura-ready branch exposes CAN IDs 24 and 25 as a different two-roller arrangement. Those IDs are not used by this extension-control implementation.

## Limits and assumptions

- Maximum extension reference: 0.50 m.
- Minimum extension reference: 0.0 m.
- Forward and reverse Phoenix software soft limits are enabled.
- Runtime feedback also blocks commands that would move beyond either limit.
- All motor voltage commands are bounded to plus or minus 6 V.
- The existing 2,500 RPM spin target is reduced to 1,250 RPM maximum.
- Extension distance assumes a 64:14 motor-to-pinion reduction and a 1.751 inch pinion.
- The integrated encoder must be zeroed while the mechanism is physically fully retracted.
- No automatic homing is performed because the selected implementation does not rely on a confirmed retract sensor.
- Soft limits are software protection, not a substitute for mechanical hard stops.

## Setup and integration

The Java files are a command-ready integration layer, not automatically inserted into Robot-Code-2026. Copy the files under the robot project's frc.robot.intake package, instantiate:

```java
IntakeStateSpaceSubsystem intake =
    new IntakeStateSpaceSubsystem(new TalonFXRollerIO());
```

After physically retracting the mechanism, zero the integrated encoder using zeroExtensionEncoder() before enabling motion. Then bind the controller:

```java
IntakeXboxBindings.configure(driverXbox, intake);
```

Before applying full robot power, verify motor inversion, encoder sign, encoder zero, gear ratio, soft-limit direction, current limits, and physical hard stops with the robot supported safely.

## Safety note

This branch intentionally runs at half speed and half voltage. It has not been characterized on the physical mechanism. The first hardware test should be performed with the robot secured, with a person ready to disable the robot, and with the extension direction verified at low command authority.
