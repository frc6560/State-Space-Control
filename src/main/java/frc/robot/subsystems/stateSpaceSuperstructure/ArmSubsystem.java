package frc.robot.subsystems.stateSpaceSuperstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Two-state LQR arm control: x = [arm angle (rad), arm velocity (rad/s)].
 *
 * <p>The Talon FX's integrated encoder measures angle. A Pigeon 2 mounted on the arm supplies
 * angular velocity. LQR computes the feedback voltage; the cosine term is gravity feedforward.
 * This class intentionally contains the controller, real hardware interface, simulation, and logs
 * so it is easy to study as one small state-space example.
 */
public class ArmSubsystem extends SubsystemBase {
  public enum TargetPosition {
    STOWED(Constants.Arm.STOWED_ANGLE_RAD),
    INTAKE(Constants.Arm.INTAKE_ANGLE_RAD),
    SCORE(Constants.Arm.SCORE_ANGLE_RAD);

    final double angleRad;

    TargetPosition(double angleRad) {
      this.angleRad = angleRad;
    }
  }

  // Hardware is intentionally not constructed in desktop simulation. The plant below supplies
  // sensor values there, avoiding a dependency on CTRE's hardware simulator for this lesson.
  private final TalonFX motor;
  private final Pigeon2 armGyro;
  private final LinearSystem<N2, N1, N2> plant = LinearSystemId.createSingleJointedArmSystem(
      DCMotor.getFalcon500(1), Constants.Arm.MOMENT_OF_INERTIA_KG_METERS_SQUARED,
      Constants.Arm.GEAR_RATIO);
  private final LinearQuadraticRegulator<N2, N1, N2> lqr = new LinearQuadraticRegulator<>(
      plant,
      VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)),
      VecBuilder.fill(12.0),
      Constants.Arm.LOOP_PERIOD_SECONDS);

  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
      DCMotor.getFalcon500(1), Constants.Arm.GEAR_RATIO,
      Constants.Arm.MOMENT_OF_INERTIA_KG_METERS_SQUARED, Constants.Arm.ARM_LENGTH_METERS,
      Constants.Arm.MIN_ANGLE_RAD, Constants.Arm.MAX_ANGLE_RAD, true,
      Constants.Arm.STOWED_ANGLE_RAD);
  private final MechanismLigament2d simulatedArm;
  private final ArmSimulationWindow simulationWindow;

  private volatile TargetPosition target = TargetPosition.STOWED;
  private double commandedVolts;

  public ArmSubsystem() {
    if (RobotBase.isReal()) {
      motor = new TalonFX(Constants.Arm.MOTOR_CAN_ID);
      armGyro = new Pigeon2(Constants.Arm.GYRO_CAN_ID);
      var config = new TalonFXConfiguration();
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      motor.getConfigurator().apply(config);
    } else {
      motor = null;
      armGyro = null;
    }

    var mechanism = new Mechanism2d(3.0, 3.0);
    MechanismRoot2d pivot = mechanism.getRoot("ArmPivot", 1.5, 0.5);
    simulatedArm = pivot.append(new MechanismLigament2d("Arm", Constants.Arm.ARM_LENGTH_METERS * 2.0,
        Units.radiansToDegrees(Constants.Arm.STOWED_ANGLE_RAD)));
    SmartDashboard.putData("StateSpaceSuperstructure/Arm Mechanism", mechanism);
    simulationWindow = RobotBase.isSimulation() ? new ArmSimulationWindow(this::setTarget) : null;
  }

  @Override
  public void periodic() {
    // This is the LQR equation u = -K(x - r), with direct sensor measurements for x.
    Matrix<N2, N1> measuredState = VecBuilder.fill(getAngleRad(), getVelocityRadPerSec());
    Matrix<N2, N1> referenceState = VecBuilder.fill(target.angleRad, 0.0);
    double lqrVolts = lqr.calculate(measuredState, referenceState).get(0, 0);
    double gravityVolts = Constants.Arm.kG_VOLTS * Math.cos(getAngleRad());
    commandedVolts = clamp(lqrVolts + gravityVolts, -12.0, 12.0);
    if (motor != null) {
      motor.setVoltage(commandedVolts);
    }

    Logger.recordOutput("StateSpaceSuperstructure/Arm/TargetDegrees",
        Units.radiansToDegrees(target.angleRad));
    Logger.recordOutput("StateSpaceSuperstructure/Arm/AngleDegrees",
        Units.radiansToDegrees(getAngleRad()));
    Logger.recordOutput("StateSpaceSuperstructure/Arm/VelocityDegreesPerSecond",
        Units.radiansToDegrees(getVelocityRadPerSec()));
    Logger.recordOutput("StateSpaceSuperstructure/Arm/LQRVolts", lqrVolts);
    Logger.recordOutput("StateSpaceSuperstructure/Arm/GravityVolts", gravityVolts);
    Logger.recordOutput("StateSpaceSuperstructure/Arm/AppliedVolts", commandedVolts);
  }

  @Override
  public void simulationPeriodic() {
    armSim.setInputVoltage(commandedVolts);
    armSim.update(Constants.Arm.LOOP_PERIOD_SECONDS);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    simulatedArm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    simulationWindow.setAngleRadians(armSim.getAngleRads());
  }

  public void setTarget(TargetPosition target) {
    this.target = target;
  }

  public Command stowCommand() {
    return Commands.runOnce(() -> setTarget(TargetPosition.STOWED), this);
  }

  public Command intakeCommand() {
    return Commands.runOnce(() -> setTarget(TargetPosition.INTAKE), this);
  }

  public Command scoreCommand() {
    return Commands.runOnce(() -> setTarget(TargetPosition.SCORE), this);
  }

  /** Talon FX integrated sensor is in rotor rotations; convert it to arm radians. */
  public double getAngleRad() {
    if (RobotBase.isSimulation()) {
      return armSim.getAngleRads();
    }
    return motor.getPosition().getValueAsDouble() * 2.0 * Math.PI / Constants.Arm.GEAR_RATIO;
  }

  /** Pigeon pitch-rate assumes its pitch axis is aligned with the arm pivot axis. */
  public double getVelocityRadPerSec() {
    if (RobotBase.isSimulation()) {
      return armSim.getVelocityRadPerSec();
    }
    return Units.degreesToRadians(armGyro.getAngularVelocityYWorld().getValueAsDouble());
  }

  private static double clamp(double value, double minimum, double maximum) {
    return Math.max(minimum, Math.min(value, maximum));
  }
}
