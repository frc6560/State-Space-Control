# Swerve-drive LQR simulation

This is a small, interactive browser model of a square FRC swerve drivetrain. It follows the useful shape of the team's YAGSL projects without importing robot hardware libraries: a drive-level JSON file names four module JSON files, the simulation turns field-relative driver intent into chassis speeds, and inverse kinematics turns the chassis state into one speed and steering angle per module.

## Run it

Requires Node.js 18 or newer. No packages need to be installed.

```text
cd swerve-lqr-sim
npm start
```

Open <http://localhost:8080>. Click the field once if the browser does not immediately capture keys.

Controls:

- `W` / `S`: forward / backward on the field
- `A` / `D`: left / right on the field
- `Left Arrow` / `Right Arrow`: counterclockwise / clockwise rotation
- `Space`: stop and reset the velocity controller
- `R`: return the robot to the center

The key display shows commanded keys. The velocity cards show the LQR reference and the simulated state. Cyan wheel arrows show the individual module speed and steering direction.

### Analog/YAGSL-style input path

The keyboard is only an input adapter: its six driving keys produce normalized values and pass through the same continuous input processor intended for a future controller. `processAnalogInput(...)` accepts decimal translation axes, an analog angular-velocity axis, or a two-axis heading vector.

For heading-vector control, the desired field heading is calculated with `atan2(headingY, headingX)`. For example, a stick held at 35.7° produces a 35.7° chassis heading target with no directional rounding. A proportional heading loop turns that continuous heading error into the yaw-rate reference tracked by the rotational LQR. Circular translation deadband preserves the original analog direction and rescales only its magnitude.

No browser Gamepad API mapping is included yet; adding a controller later only requires mapping its axes into this existing processor.

## Model assumptions

All parameters live in `config/`, so the model is easy to characterize or retune without changing the controller code.

| Quantity | Value | Assumption |
| --- | ---: | --- |
| Total robot mass | 140 lb / 63.50 kg | User-specified competition mass |
| Chassis wheelbase and trackwidth | 30 in / 0.762 m | Square, representative FRC footprint |
| Module mass | 8.5 lb / 3.86 kg each | Conservative top of AndyMark's published 6.5–8.5 lb configured Swerve & Steer range; used as the requested corner point-mass estimate |
| Center-body mass | 106 lb / 48.08 kg | Total mass less four modules |
| Center-body shape | Uniform disk, radius 0.33 m | Approximation for structure, battery, electronics, and mechanisms above the chassis |
| Calculated yaw inertia | 7.09 kg m² | Four corner point masses plus the centered disk |
| Max chassis speed | 4.5 m/s | Representative FRC swerve limit |
| Max yaw rate | 2.5 rad/s | Friendly keyboard limit for visualization |

The theoretical yaw moment of inertia is

```text
I = 4 m_module [(L/2)^2 + (W/2)^2] + 1/2 m_center r_disk^2
  = 7.09 kg m^2
```

This is a theoretical teaching model, not a hardware characterization. Module mass varies with selected drive motor, steering motor, encoder, fasteners, and wheel. The centered disk is deliberately simple and does not capture a real robot's mechanism placement or center-of-gravity height.

## Controller and kinematics

The simulated chassis plant is a damped planar rigid body:

```text
m v_dot = F - c_v v
I omega_dot = tau - c_omega omega
```

Each axis is discretized at 20 ms. A scalar infinite-horizon discrete LQR gain is solved by Riccati iteration at startup. Feedforward cancels the modeled drag at the requested velocity, while LQR feedback regulates field-relative `vx`, `vy`, and `omega`. Force and torque are saturated to make acceleration visibly finite.

For module position `(x_i, y_i)` in robot coordinates, inverse kinematics uses

```text
wheel_vx = chassis_vx - omega * y_i
wheel_vy = chassis_vy + omega * x_i
speed    = hypot(wheel_vx, wheel_vy)
angle    = atan2(wheel_vy, wheel_vx)
```

All wheel speeds are desaturated together if any exceeds the configured maximum. Steering angle has a rate limit, so the wheels visibly rotate into place instead of teleporting.

## YAGSL-inspired organization

- `config/swervedrive.json`: chassis, dynamics, input limits, and four module file names
- `config/modules/*.json`: module name and corner location
- `src/physics.mjs`: configuration, characterization, LQR, plant, and inverse kinematics
- `src/app.mjs`: keyboard input, animation loop, and visualization
- `test/physics.test.mjs`: characterization, LQR, kinematics, desaturation, and steering tests

The team references use `SwerveParser`, `SwerveSubsystem`, `SwerveInputStream`, field-oriented commands, and `src/main/deploy/swerve/...` JSON. This simulation preserves those conceptual boundaries but intentionally omits CAN devices, odometry sensors, PathPlanner, vision, and autonomous behavior.
