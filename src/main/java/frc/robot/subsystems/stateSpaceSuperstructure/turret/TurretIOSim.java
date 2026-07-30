package frc.robot.subsystems.stateSpaceSuperstructure.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/** WPILib physics model; unlike the 2026 version, it contains no PID controller. */
public class TurretIOSim implements TurretIO {
  private final SingleJointedArmSim sim = new SingleJointedArmSim(
      DCMotor.getKrakenX60(1), Constants.Turret.MOTOR_GEAR_RATIO,
      Constants.Turret.MOMENT_OF_INERTIA_KG_METERS_SQUARED, Constants.Turret.LENGTH_METERS,
      Units.degreesToRadians(Constants.Turret.LOWER_SOFT_LIMIT_DEGREES),
      Units.degreesToRadians(Constants.Turret.UPPER_SOFT_LIMIT_DEGREES), false, 0.0);
  private double appliedVolts;

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.angleRadians = sim.getAngleRads();
    inputs.velocityRadiansPerSecond = sim.getVelocityRadPerSec();
    inputs.motorPositionRotations = inputs.angleRadians / (2.0 * Math.PI) * Constants.Turret.MOTOR_GEAR_RATIO;
    inputs.motorVelocityRotationsPerSecond = inputs.velocityRadiansPerSecond / (2.0 * Math.PI)
        * Constants.Turret.MOTOR_GEAR_RATIO;
    inputs.absoluteEncoderRotations = inputs.angleRadians / (2.0 * Math.PI)
        * Constants.Turret.ENCODER_GEAR_RATIO;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void simulationPeriodic() {
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.Turret.LOOP_PERIOD_SECONDS);
  }
}
