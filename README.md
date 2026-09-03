# State-Space Control

This repository contains FRC examples for controlling and coordinating a robot with several moving
mechanisms. The intended robot has a swerve drivetrain, arm, turret, flywheel, intake, and an
AprilTag camera. Each mechanism controls its own motion, while a higher-level coordinator starts
independent actions in parallel and waits for measured stability before continuing.

The design separates three jobs:

1. **State estimation** determines where the robot and its mechanisms are now.
2. **Low-level control** moves each mechanism toward a requested position or velocity.
3. **Task coordination** decides which mechanisms may move together and when the next stage is safe.

State-space control handles the physical motion. It does not replace the command scheduler or the
autonomous state machine.

## Intended operation

A shooting sequence should work as follows:

1. The drivetrain begins moving toward the desired field pose.
2. The arm moves to its shooting angle, the turret aims, and the flywheel spins up simultaneously.
3. Every subsystem continuously reports whether it is at its goal.
4. The coordinator checks arm error, turret error, flywheel speed, drivetrain pose, and robot motion.
5. All conditions must remain valid for 0.15 seconds before `readyToFire()` becomes true.
6. A feeder may fire only while `readyToFire()` is true.
7. A timeout prevents a failed mechanism or sensor from blocking an autonomous routine forever.

This uses measured readiness instead of commands such as "wait 1.5 seconds." The same sequence can
therefore handle different starting positions, battery voltages, loads, and disturbances.

## Control architecture

Each mechanism owns its sensors, model, controller, goal, and safety limits:

| Mechanism | State used by the controller | Requested goal |
|---|---|---|
| Arm | angle and angular velocity | arm angle |
| Turret | continuous angle and angular velocity | safe field-relative angle |
| Flywheel | angular velocity | shooting speed |
| Intake extension | position and velocity | extended, retracted, or oscillating |
| Swerve drivetrain | field pose and chassis velocity | path or target pose |

The arm, turret, and flywheel examples use Linear Quadratic Regulator feedback. LQR chooses motor
voltage from the difference between the measured state and requested state. The arm also includes
gravity feedforward. This structure can later use observers when a state cannot be measured reliably.

`SuperstructureCoordinator` does not calculate motor voltage. It sends goals to the subsystems,
runs independent preparation commands in parallel, and combines their `atGoal()` results with a
drivetrain-stability condition. This keeps mechanism physics separate from task sequencing.

## AprilTag pose estimation

The intended drivetrain pose estimator combines:

- swerve wheel odometry for short-term translation;
- a gyro for heading and angular velocity; and
- timestamped Limelight AprilTag poses for drift correction.

Vision results must be inserted with the timestamp of image capture, not the time the NetworkTables
packet arrives. `TimestampedVisionMeasurement` represents this input. `VisionMeasurementGate`
rejects high-ambiguity, distant, fast-rotation, and implausibly discontinuous measurements. It also
produces distance- and tag-count-dependent uncertainty so weak measurements influence the estimator
less than strong multi-tag measurements.

The drivetrain should use it as follows after substituting its real estimator:

```java
if (VisionMeasurementGate.shouldAccept(vision, poseEstimator.getEstimatedPosition(), gyroRate)) {
  poseEstimator.addVisionMeasurement(
      vision.pose(),
      vision.timestampSeconds(),
      VisionMeasurementGate.standardDeviations(vision));
}
```

The repository deliberately does not hard-code a Limelight name, robot-to-camera transform, module
locations, or drivetrain gearing. Those values must be measured on the real robot; invented values
would produce incorrect localization.

## Repository contents

- `src/main/java/frc/robot/subsystems/stateSpaceSuperstructure` — WPILib arm, turret, and flywheel
  LQR examples, simulation, hardware IO, and logging.
- `src/main/java/frc/robot/coordination` — parallel preparation and debounced readiness logic.
- `src/main/java/frc/robot/vision` — timestamped vision data and AprilTag measurement validation.
- `swerve-lqr-sim` — interactive browser simulation of a square FRC swerve chassis.
- `intake-state-space` — interactive intake-extension state-space simulation.
- `robot-intake` — command-ready intake reference code and controller behavior.

The browser simulations are separate Node projects. The Java project is the WPILib robot example.

## WPILib simulation controls

Run `./gradlew simulateJava`. On Windows, use `gradlew.bat simulateJava`.

| Control | Action |
|---|---|
| A | Arm stowed position |
| B | Arm intake position |
| X | Arm score position |
| Y | Turret forward |
| Left bumper | Turret left |
| Right bumper | Turret right |
| Start | Prepare arm, turret, and flywheel in parallel; then wait for stability |

The arm and turret also open keyboard-controlled simulation windows. AdvantageKit publishes the
controller states, goals, voltages, and coordinator readiness values for AdvantageScope.

## Browser simulations

For either browser simulator, enter its directory and run:

```text
npm install
npm test
npm start
```

See `swerve-lqr-sim/README.md` and `intake-state-space/README.md` for their controls.

## Before deploying to a real robot

This branch is an architecture and simulation reference, not a drop-in season robot program. Before
deployment:

1. Replace estimated masses, inertias, gear ratios, CAN IDs, encoder offsets, and soft limits.
2. Run SysId on each physical mechanism and update the plant models.
3. Insert the real drivetrain and its `SwerveDrivePoseEstimator`.
4. Configure the measured robot-to-camera transform and parse Limelight latency correctly.
5. Replace the coordinator's temporary `() -> true` drivetrain-stability supplier with checks on
   pose error, translational speed, and angular speed.
6. Add collision constraints between the arm, turret, intake, and frame perimeter.
7. Connect the firing action to a feeder only after verifying `readyToFire()`.
8. Test every limit at low voltage with the robot supported and an operator ready to disable.

The goal is a robot whose mechanisms prepare simultaneously, whose global pose is corrected by
AprilTags, and whose sequential actions advance because the physical system is actually ready—not
because a fixed timer expired.
