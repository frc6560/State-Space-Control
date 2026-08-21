# Intake state-space control

This directory implements the intake Kevin described in Slack: a rack-and-pinion deployer driven by one Kraken X44, rollers driven by one Kraken X60 (with room for a second motor later), three normal modes, and a forward/backward oscillation mode used while shooting.

## Run and test

```text
cd intake-state-space
npm test
npm start
```

Open <http://localhost:8081>. The simulation has no external packages.

## Required behavior

The controller exposes these states:

- `RETRACTED_NOT_SPINNING`: retract the rack to its minimum travel and stop the roller.
- `EXTENDED_NOT_SPINNING`: hold full extension and stop the roller.
- `EXTENDED_SPINNING`: hold full extension and regulate the roller to its intake speed.
- `SHOOTING_OSCILLATION`: command a sinusoidal rack position around a configurable extended position, moving it forward and backward while leaving the roller stopped by default.

An unknown mode is rejected instead of producing a motor command. Both outputs are limited to ±12 V. The plant enforces the rack's hard minimum and maximum travel.

## State-space design

The explicit state is

```text
x = [rack position (m), rack velocity (m/s), roller speed (rotations/s)]
u = [X44 voltage, X60 voltage]
y = x
```

`createIntakeModel(...)` produces discrete `A`, `B`, `C`, and `D` matrices for a 20 ms loop. The rack uses a mass, voltage-to-force, and viscous-damping model. The roller uses a first-order speed model. `IntakeController` computes a two-state discrete LQR for rack position/velocity and a scalar discrete LQR for roller speed; model-based roller feedforward supplies the steady-state voltage so feedback does not need to tolerate a permanent speed error. The shooting reference includes both sinusoidal position and its analytic velocity, so the LQR tracks a physically consistent moving state.

The core is independent from the browser and from any vendor motor API. Robot code can call `update([measuredPosition, measuredVelocity, measuredRollerSpeed])` every 20 ms and send the returned voltages to the X44 and X60. This makes it state-space-compatible even if the final robot implementation later replaces LQR with characterized feedforward plus feedback.

## Integration boundary and assumptions

The team has not supplied CAN IDs, rack travel, pinion size/gearing, moving mass, current limits, encoder conversion factors, or measured motor response. `config/intake.json` therefore contains visible teaching estimates, not hardware-validated constants. Before enabling real motors:

1. Assign CAN IDs and verify inversion one motor at a time at low voltage.
2. Measure rack travel and configure encoder-to-meter conversion plus real limit switches.
3. Run WPILib SysId separately for the X44 rack and X60 roller; replace the estimated plant coefficients.
4. Add supply/stator current limits and verify the hard-stop behavior with the robot disabled between tests.
5. Confirm with mechanical whether shooting oscillation means rack motion, as implemented, or roller direction reversal; both interpretations are easy to express as a changing state reference.

The existing browser simulation is therefore a complete, testable control implementation for the stated behavior, but it is not authorization to run uncharacterized hardware.
