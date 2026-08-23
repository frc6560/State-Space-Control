# AdvantageKit integration contract

The browser lab is hardware independent, but its IO boundary matches an AdvantageKit swerve subsystem: the controller produces desired module states, a motor-output predictor produces the values that would be measured after a voltage command, and every applied voltage is recorded with the robot pose.

In the robot project, retain the existing vendor/YAGSL module IO and log these outputs from the periodic drive loop:

| AdvantageKit key | Value |
| --- | --- |
| `Drive/LQR/ReferenceVxMetersPerSec` | field-relative translation reference |
| `Drive/LQR/ReferenceVyMetersPerSec` | field-relative translation reference |
| `Drive/LQR/ReferenceOmegaRadPerSec` | rotation reference from the controller right stick |
| `Drive/LQR/ControlForceXNewtons` | translational LQR output |
| `Drive/LQR/ControlForceYNewtons` | translational LQR output |
| `Drive/LQR/ControlTorqueNewtonMeters` | rotational LQR output using `Constants` mass/inertia |
| `Drive/Modules/<name>/DriveAppliedVolts` | voltage sent to the drive motor |
| `Drive/Modules/<name>/SteerAppliedVolts` | voltage sent to the steering motor |
| `Drive/Modules/<name>/DriveCurrentAmps` | motor/output-stage current estimate or hardware measurement |
| `Drive/Odometry/Pose` | `Pose2d` recorded every scheduler cycle |

For real hardware, replace `predictMotorOutput()` with the motor controller's measured velocity/current and continue calling `Logger.processInputs(...)` on each module's IO inputs. For simulation, keep `predictMotorOutput()` ahead of the simulated motor/plant; this makes every simulated voltage command observable, deterministic, and replayable.

The exported `swerve-match.json` is a portable 20 ms record of `Drive/Odometry/Pose` and per-module applied voltages. It can be copied into a test fixture or compared against an AdvantageKit `.wpilog` conversion without requiring browser state.
