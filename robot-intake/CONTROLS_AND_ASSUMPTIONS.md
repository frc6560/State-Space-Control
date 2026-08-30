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
| Back | Stop outputs immediately |

For the existing button board in ventura-ready, button 3 remains the current roller-intake trigger and button 6 remains the existing intake-reset trigger. The new Xbox mapping is separate so it can be reviewed and added to RobotContainer without changing competition bindings accidentally.

## Hardware source

Values are from frc6560/Robot-Code-2026, branch ventura-ready:

- Left intake roller: CAN ID 24, not inverted.
- Right intake roller: CAN ID 25, inverted.
- CAN bus: rio.
- Existing roller target: 2,500 RPM; this branch caps it at 1,250 RPM.
- No deployment motor CAN ID or extension sensor is present in that branch.

## Assumptions and safety limits

- Maximum extension reference: 0.50 m.
- Oscillation center: 0.45 m.
- Oscillation amplitude: 0.05 m, giving 0.10 m peak-to-peak travel.
- Every motor output is bounded to plus/minus 6 V.
- Deployment feedback is inactive until a real deployment motor and position measurement are connected.
- The roller adapter uses existing 24/25 motors only.
- This is integration-ready code, not a claim that the physical intake can extend: the competition branch exposes no extension actuator.
- Do not deploy until wiring, motor direction, hard stops, sensor zero, and current limits are checked on blocks.

## Integration

Place the Java files under the robot project's frc.robot.intake package. Instantiate:

IntakeStateSpaceSubsystem intake =
    new IntakeStateSpaceSubsystem(new TalonFXRollerIO());

Bind the Xbox controls to intake.setMode(...). The current robot Intake class can remain untouched until the team decides whether this subsystem should replace or wrap it.
