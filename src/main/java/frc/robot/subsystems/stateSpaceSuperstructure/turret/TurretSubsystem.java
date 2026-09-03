package frc.robot.subsystems.stateSpaceSuperstructure.turret;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Continuous/unwrapped LQR turret controller. It keeps the season bot's safe cable range while
 * treating [position, velocity] as one state and accepting a position/velocity setpoint.
 */
public class TurretSubsystem extends SubsystemBase {
  private enum ControlMode { PRESET, MANUAL, TARGET_TRACKING, SYSID }
  private final TurretIO io;
  private final TurretIO.Inputs inputs = new TurretIO.Inputs();
  private final LinearSystem<N2, N1, N2> plant = createPlant();
  private final LinearQuadraticRegulator<N2, N1, N2> lqr = new LinearQuadraticRegulator<>(
      plant,
      VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(12.0)),
      VecBuilder.fill(12.0), Constants.Turret.LOOP_PERIOD_SECONDS);

  private volatile double goalRadians;
  private volatile double goalVelocityRadiansPerSecond;
  private volatile ControlMode controlMode = ControlMode.PRESET;
  private final TurretSimulationWindow simulationWindow;
  private final SysIdRoutine sysIdRoutine;
  private boolean sysIdActive;
  private double sysIdRequestedVolts;

  public TurretSubsystem(TurretIO io) {
    this.io = io;
    simulationWindow = RobotBase.isSimulation()
        ? new TurretSimulationWindow(this::selectPreset) : null;
    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Constants.Turret.SYSID_RAMP_VOLTS_PER_SECOND).per(Second),
            Volts.of(Constants.Turret.SYSID_STEP_VOLTS),
            Seconds.of(Constants.Turret.SYSID_TIMEOUT_SECONDS),
            state -> Logger.recordOutput(
                "StateSpaceSuperstructure/Turret/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            voltage -> {
              sysIdRequestedVolts = voltage.in(Volts);
              io.setVoltage(limitSysIdVoltage(sysIdRequestedVolts));
            },
            null,
            this,
            "Turret"));
  }

  /**
   * Sets a wrapped field-relative target and converts it into the closest legal unwrapped angle.
   * This is what makes the controller continuous across +/-180 degrees without twisting wires.
   */
  public void setFieldAngleDegrees(double wrappedAngleDegrees) {
    controlMode = ControlMode.PRESET;
    if (simulationWindow != null) {
      simulationWindow.setPresetMode();
    }
    setContinuousSetpointDegrees(selectSafeContinuousGoal(wrappedAngleDegrees), 0.0);
  }

  /** Same safe setpoint selection, plus a velocity reference for tracking a moving target. */
  public void setFieldAngleWithVelocity(double wrappedAngleDegrees, double velocityDegreesPerSecond) {
    setContinuousSetpointDegrees(selectSafeContinuousGoal(wrappedAngleDegrees), velocityDegreesPerSecond);
  }

  /** Direct unwrapped angle setter for testing or motion-planned superstructure commands. */
  public void setContinuousSetpointDegrees(double angleDegrees, double velocityDegreesPerSecond) {
    goalRadians = Units.degreesToRadians(MathUtil.clamp(angleDegrees,
        Constants.Turret.LOWER_SOFT_LIMIT_DEGREES, Constants.Turret.UPPER_SOFT_LIMIT_DEGREES));
    goalVelocityRadiansPerSecond = Units.degreesToRadians(velocityDegreesPerSecond);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (simulationWindow != null) {
      TurretSimulationWindow.ControlInput control = simulationWindow.getControlInput();
      if (control.mode() == TurretSimulationWindow.Mode.MANUAL) {
        controlMode = ControlMode.MANUAL;
        setContinuousSetpointDegrees(
            Units.radiansToDegrees(goalRadians)
                + control.manualRateDegreesPerSecond() * Constants.Turret.LOOP_PERIOD_SECONDS,
            0.0);
        controlMode = ControlMode.MANUAL;
      } else if (control.mode() == TurretSimulationWindow.Mode.TARGET_TRACKING) {
        controlMode = ControlMode.TARGET_TRACKING;
        setContinuousSetpointDegrees(
            selectSafeContinuousGoal(Units.radiansToDegrees(
                Math.atan2(control.targetY(), control.targetX()))), 0.0);
        simulationWindow.setTurretAngleRadians(inputs.angleRadians);
      }
      simulationWindow.setTurretAngleRadians(inputs.angleRadians);
      simulationWindow.setGoalAngleRadians(goalRadians);
      simulationWindow.setControlMode(controlMode.name());
    }
    double lqrVolts = 0.0;
    double commandedVolts;
    if (sysIdActive) {
      commandedVolts = limitSysIdVoltage(sysIdRequestedVolts);
    } else {
      lqrVolts = lqr.calculate(
          VecBuilder.fill(inputs.angleRadians, inputs.velocityRadiansPerSecond),
          VecBuilder.fill(goalRadians, goalVelocityRadiansPerSecond)).get(0, 0);
      commandedVolts = MathUtil.clamp(lqrVolts, -RobotController.getBatteryVoltage(),
          RobotController.getBatteryVoltage());
    }
    io.setVoltage(commandedVolts);

    Logger.recordOutput("StateSpaceSuperstructure/Turret/AngleDegrees", Units.radiansToDegrees(inputs.angleRadians));
    Logger.recordOutput("StateSpaceSuperstructure/Turret/VelocityDegreesPerSecond",
        Units.radiansToDegrees(inputs.velocityRadiansPerSecond));
    Logger.recordOutput("StateSpaceSuperstructure/Turret/GoalDegrees", Units.radiansToDegrees(goalRadians));
    Logger.recordOutput("StateSpaceSuperstructure/Turret/GoalVelocityDegreesPerSecond",
        Units.radiansToDegrees(goalVelocityRadiansPerSecond));
    Logger.recordOutput("StateSpaceSuperstructure/Turret/LQRVolts", lqrVolts);
    Logger.recordOutput("StateSpaceSuperstructure/Turret/CommandedVolts", commandedVolts);
    Logger.recordOutput("StateSpaceSuperstructure/Turret/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("StateSpaceSuperstructure/Turret/SysIdActive", sysIdActive);
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  public Command forwardCommand() {
    return Commands.runOnce(() -> setFieldAngleDegrees(Constants.Turret.FORWARD_DEGREES), this);
  }

  public Command leftCommand() {
    return Commands.runOnce(() -> setFieldAngleDegrees(Constants.Turret.LEFT_DEGREES), this);
  }

  public Command rightCommand() {
    return Commands.runOnce(() -> setFieldAngleDegrees(Constants.Turret.RIGHT_DEGREES), this);
  }

  /** True only when position and velocity are both inside the firing tolerances. */
  public boolean atGoal() {
    return Math.abs(inputs.angleRadians - goalRadians)
            <= Constants.Coordination.TURRET_ANGLE_TOLERANCE_RAD
        && Math.abs(inputs.velocityRadiansPerSecond - goalVelocityRadiansPerSecond)
            <= Constants.Coordination.TURRET_VELOCITY_TOLERANCE_RAD_PER_SECOND;
  }

  /**
   * Runs all four SysId tests as a deliberately selected autonomous routine. The command stops each
   * test before the physical software limits and holds position briefly before reversing.
   */
  public Command sysIdSequenceCommand() {
    return Commands.sequence(
        guardedSysIdCommand(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward), true),
        Commands.waitSeconds(1.0),
        guardedSysIdCommand(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse), false),
        Commands.waitSeconds(1.0),
        guardedSysIdCommand(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward), true),
        Commands.waitSeconds(1.0),
        guardedSysIdCommand(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse), false))
        .withName("Turret complete SysId sequence");
  }

  /** Selects one of the keyboard presets: 1 = forward, 2 = left, 3 = right. */
  public void selectPreset(int preset) {
    switch (preset) {
      case 1 -> setFieldAngleDegrees(Constants.Turret.FORWARD_DEGREES);
      case 2 -> setFieldAngleDegrees(Constants.Turret.LEFT_DEGREES);
      case 3 -> setFieldAngleDegrees(Constants.Turret.RIGHT_DEGREES);
      default -> { }
    }
  }

  private double selectSafeContinuousGoal(double targetDegrees) {
    double wrapped = MathUtil.inputModulus(targetDegrees, -180.0, 180.0);
    double currentDegrees = Units.radiansToDegrees(inputs.angleRadians);
    double bestGoal = 0.0;
    double bestScore = Double.POSITIVE_INFINITY;
    boolean protectWires = currentDegrees <= Constants.Turret.LOWER_SOFT_LIMIT_DEGREES
        || currentDegrees >= Constants.Turret.UPPER_SOFT_LIMIT_DEGREES;

    for (int turns = -1; turns <= 1; turns++) {
      double candidate = wrapped + 360.0 * turns;
      if (candidate < Constants.Turret.LOWER_SOFT_LIMIT_DEGREES
          || candidate > Constants.Turret.UPPER_SOFT_LIMIT_DEGREES) {
        continue;
      }
      double score = protectWires ? Math.abs(candidate) : Math.abs(candidate - currentDegrees);
      if (score < bestScore) {
        bestScore = score;
        bestGoal = candidate;
      }
    }
    return MathUtil.clamp(bestGoal, Constants.Turret.LOWER_SOFT_LIMIT_DEGREES,
        Constants.Turret.UPPER_SOFT_LIMIT_DEGREES);
  }

  private Command guardedSysIdCommand(Command test, boolean forward) {
    return Commands.runOnce(() -> {
          sysIdActive = true;
          sysIdRequestedVolts = 0.0;
          controlMode = ControlMode.SYSID;
        }, this)
        .andThen(test.until(() -> forward ? atSysIdUpperLimit() : atSysIdLowerLimit()))
        .finallyDo(() -> {
          sysIdActive = false;
          sysIdRequestedVolts = 0.0;
          io.setVoltage(0.0);
          goalRadians = inputs.angleRadians;
          goalVelocityRadiansPerSecond = 0.0;
          controlMode = ControlMode.PRESET;
        });
  }

  private double limitSysIdVoltage(double requestedVolts) {
    double batteryLimit = RobotController.getBatteryVoltage();
    double volts = MathUtil.clamp(requestedVolts, -batteryLimit, batteryLimit);
    if ((volts > 0.0 && atSysIdUpperLimit()) || (volts < 0.0 && atSysIdLowerLimit())) {
      return 0.0;
    }
    return volts;
  }

  private boolean atSysIdUpperLimit() {
    return Units.radiansToDegrees(inputs.angleRadians)
        >= Constants.Turret.UPPER_SOFT_LIMIT_DEGREES
            - Constants.Turret.SYSID_LIMIT_MARGIN_DEGREES;
  }

  private boolean atSysIdLowerLimit() {
    return Units.radiansToDegrees(inputs.angleRadians)
        <= Constants.Turret.LOWER_SOFT_LIMIT_DEGREES
            + Constants.Turret.SYSID_LIMIT_MARGIN_DEGREES;
  }

  private static LinearSystem<N2, N1, N2> createPlant() {
    double kV = Constants.Turret.CHARACTERIZED_KV_VOLTS_PER_RADIAN_PER_SECOND;
    double kA = Constants.Turret.CHARACTERIZED_KA_VOLTS_PER_RADIAN_PER_SECOND_SQUARED;
    if (Double.isFinite(kV) && kV > 0.0 && Double.isFinite(kA) && kA > 0.0) {
      return LinearSystemId.identifyPositionSystem(kV, kA);
    }
    return LinearSystemId.createSingleJointedArmSystem(
        DCMotor.getKrakenX60(1), Constants.Turret.MOMENT_OF_INERTIA_KG_METERS_SQUARED,
        Constants.Turret.MOTOR_GEAR_RATIO);
  }
}
