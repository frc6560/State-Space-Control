# State-Space-Control

Minimal WPILib 2026 example of a Talon FX-powered arm using LQR state feedback.

The arm has three commandable positions: stowed (20 degrees), intake (55 degrees), and score
(100 degrees). In simulation, use Xbox controller buttons A, B, and X respectively. Run
`gradlew.bat simulateJava`, then open the generated `.wpilog` in AdvantageScope. The important
channels are under `StateSpaceSuperstructure/Arm`.

Simulation also opens a separate `State-Space Arm Simulation` window showing the physical arm
motion. With that window focused, press `1` for stowed, `2` for intake, or `3` for score. The
original Mechanism2d view remains on the dashboard for use in Glass.

`ArmSubsystem` lives in `src/main/java/frc/robot/subsystems/stateSpaceSuperstructure` so the
control code, hardware interface, simulation, and logging are kept together for this example.

The same folder also contains an IO-based `turret` example based on the 2026 season bot's Talon
FX/CANcoder conversions and cable-safe range. It replaces Motion Magic/PID with continuous,
unwrapped LQR state feedback. In the turret simulation window, press `1`/`2`/`3` for the
forward/left/right presets; hold Left/Right Arrow for continuous counterclockwise/clockwise
manual rotation; or hold WASD to move the red target block. While WASD is active, the turret
continuously points at that block. The controller bindings are Y (forward), left bumper (left),
and right bumper (right).

## Turret system identification

The dashboard autonomous chooser includes `Turret SysId (full sequence)`. This is deliberately not
the default. Before selecting it, put the real robot on blocks, start the turret near forward, clear
the full turret travel, verify that positive voltage increases the reported turret angle, and keep
an operator ready to disable.
The routine applies a 0.5 V/s quasistatic ramp and a 4 V dynamic step in both directions. Each test
has an 8 second timeout and stops 20 degrees before the configured cable limits.

After the four tests finish, open the AdvantageKit `.wpilog` in AdvantageScope and export a WPILOG
using `AdvantageKit Cycles` timestamps. Load that export in the WPILib SysId analyzer as a Simple
mechanism with radians, using the turret SysId state, applied-voltage, angle, and velocity channels.
Copy the fitted radians-based `kV` and `kA` into
`CHARACTERIZED_KV_VOLTS_PER_RADIAN_PER_SECOND` and
`CHARACTERIZED_KA_VOLTS_PER_RADIAN_PER_SECOND_SQUARED` in `Constants.Turret`. On the next build and
deploy, the code builds the position/velocity plant and LQR from those measured constants instead
of the estimated mass and ring inertia. `kS` is useful for a separate friction feedforward but does
not change the linear LQR plant.
