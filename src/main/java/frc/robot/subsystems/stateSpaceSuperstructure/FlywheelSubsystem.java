package frc.robot.subsystems.stateSpaceSuperstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/** One-state LQR flywheel controller whose state is angular velocity in radians per second. */
public class FlywheelSubsystem extends SubsystemBase {
  private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
      DCMotor.getKrakenX60(1), Constants.Flywheel.MOMENT_OF_INERTIA_KG_METERS_SQUARED,
      Constants.Flywheel.GEAR_RATIO);
  private final LinearQuadraticRegulator<N1, N1, N1> lqr = new LinearQuadraticRegulator<>(
      plant, VecBuilder.fill(Constants.Flywheel.SPEED_TOLERANCE_RAD_PER_SECOND),
      VecBuilder.fill(12.0), Constants.Flywheel.LOOP_PERIOD_SECONDS);
  private final FlywheelSim simulation = new FlywheelSim(
      DCMotor.getKrakenX60(1), Constants.Flywheel.GEAR_RATIO,
      Constants.Flywheel.MOMENT_OF_INERTIA_KG_METERS_SQUARED);
  private final TalonFX motor;

  private double goalRadPerSecond = Constants.Flywheel.IDLE_RAD_PER_SECOND;
  private double commandedVolts;

  public FlywheelSubsystem() {
    if (RobotBase.isReal()) {
      motor = new TalonFX(Constants.Flywheel.MOTOR_CAN_ID);
      var config = new TalonFXConfiguration();
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      motor.getConfigurator().apply(config);
    } else {
      motor = null;
    }
  }

  @Override
  public void periodic() {
    commandedVolts = lqr.calculate(
        VecBuilder.fill(getVelocityRadPerSecond()), VecBuilder.fill(goalRadPerSecond)).get(0, 0);
    commandedVolts = Math.max(-RobotController.getBatteryVoltage(),
        Math.min(commandedVolts, RobotController.getBatteryVoltage()));
    if (motor != null) {
      motor.setVoltage(commandedVolts);
    }

    Logger.recordOutput("StateSpaceSuperstructure/Flywheel/VelocityRadPerSecond",
        getVelocityRadPerSecond());
    Logger.recordOutput("StateSpaceSuperstructure/Flywheel/GoalRadPerSecond", goalRadPerSecond);
    Logger.recordOutput("StateSpaceSuperstructure/Flywheel/AppliedVolts", commandedVolts);
    Logger.recordOutput("StateSpaceSuperstructure/Flywheel/AtGoal", atGoal());
  }

  @Override
  public void simulationPeriodic() {
    simulation.setInputVoltage(commandedVolts);
    simulation.update(Constants.Flywheel.LOOP_PERIOD_SECONDS);
  }

  public void setGoalRadPerSecond(double goalRadPerSecond) {
    this.goalRadPerSecond = Math.max(0.0, goalRadPerSecond);
  }

  public boolean atGoal() {
    return Math.abs(getVelocityRadPerSecond() - goalRadPerSecond)
        <= Constants.Flywheel.SPEED_TOLERANCE_RAD_PER_SECOND;
  }

  public Command spinUpCommand() {
    return Commands.runOnce(
        () -> setGoalRadPerSecond(Constants.Flywheel.SHOOT_RAD_PER_SECOND), this);
  }

  public Command stopCommand() {
    return Commands.runOnce(
        () -> setGoalRadPerSecond(Constants.Flywheel.IDLE_RAD_PER_SECOND), this);
  }

  private double getVelocityRadPerSecond() {
    if (RobotBase.isSimulation()) {
      return simulation.getAngularVelocityRadPerSec();
    }
    // Talon velocity is rotor rotations/s. Divide by the motor-to-wheel ratio and convert to rad/s.
    return motor.getVelocity().getValueAsDouble() * 2.0 * Math.PI / Constants.Flywheel.GEAR_RATIO;
  }
}
