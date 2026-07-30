package frc.robot.subsystems.stateSpaceSuperstructure.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Talon FX/CANcoder implementation retaining the 2026 season bot's conversion conventions. */
public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor = new TalonFX(Constants.Turret.MOTOR_CAN_ID, Constants.Turret.CAN_BUS);
  private final CANcoder absoluteEncoder = new CANcoder(
      Constants.Turret.ABSOLUTE_ENCODER_CAN_ID, Constants.Turret.CAN_BUS);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public TurretIOTalonFX() {
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = Constants.Turret.ABSOLUTE_ENCODER_OFFSET_ROTATIONS;
    absoluteEncoder.getConfigurator().apply(encoderConfig);

    var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.Turret.LOWER_SOFT_LIMIT_DEGREES / 360.0 * Constants.Turret.MOTOR_GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.Turret.UPPER_SOFT_LIMIT_DEGREES / 360.0 * Constants.Turret.MOTOR_GEAR_RATIO;
    motor.getConfigurator().apply(motorConfig);

    // The CANcoder establishes an absolute zero; the Talon integrates from that known position.
    double turretRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble()
        / Constants.Turret.ENCODER_GEAR_RATIO;
    motor.setPosition(turretRotations * Constants.Turret.MOTOR_GEAR_RATIO);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    double motorRotations = motor.getPosition().getValueAsDouble();
    double motorRps = motor.getVelocity().getValueAsDouble();
    inputs.motorPositionRotations = motorRotations;
    inputs.motorVelocityRotationsPerSecond = motorRps;
    inputs.absoluteEncoderRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    inputs.angleRadians = motorRotations / Constants.Turret.MOTOR_GEAR_RATIO * 2.0 * Math.PI;
    inputs.velocityRadiansPerSecond = motorRps / Constants.Turret.MOTOR_GEAR_RATIO * 2.0 * Math.PI;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
