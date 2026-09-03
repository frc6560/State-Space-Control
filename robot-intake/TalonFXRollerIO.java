package frc.robot.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public final class TalonFXRollerIO implements IntakeStateSpaceIO {
  public static final int EXTENSION_MOTOR_CAN_ID = 15;
  public static final int SPIN_MOTOR_CAN_ID = 16;
  public static final String CAN_BUS = "rio";

  // V3/intake-2 identify 15 as the extension motor and 16 as the intake spin motor.
  // The integrated TalonFX encoder is the extension position measurement.
  private static final double PINION_DIAMETER_INCHES = 1.751;
  private static final double EXTENSION_GEAR_RATIO = 64.0 / 14.0;
  private static final double METERS_PER_MOTOR_ROTATION =
      (Math.PI * PINION_DIAMETER_INCHES * 0.0254 / EXTENSION_GEAR_RATIO);
  private static final double MAX_EXTENSION_MOTOR_ROTATIONS =
      IntakeStateSpaceController.MAX_EXTENSION_METERS / METERS_PER_MOTOR_ROTATION;

  private final TalonFX extensionMotor = new TalonFX(EXTENSION_MOTOR_CAN_ID, CAN_BUS);
  private final TalonFX spinMotor = new TalonFX(SPIN_MOTOR_CAN_ID, CAN_BUS);
  private final StatusSignal<AngularVelocity> extensionVelocity = extensionMotor.getVelocity();
  private final StatusSignal<AngularVelocity> spinVelocity = spinMotor.getVelocity();
  private final StatusSignal<Voltage> extensionVoltage = extensionMotor.getMotorVoltage();
  private final StatusSignal<Voltage> spinVoltage = spinMotor.getMotorVoltage();

  public TalonFXRollerIO() {
    configureExtensionMotor();
    configureSpinMotor();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, extensionVelocity, spinVelocity, extensionVoltage, spinVoltage);
    extensionMotor.optimizeBusUtilization();
    spinMotor.optimizeBusUtilization();
  }

  private void configureExtensionMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_EXTENSION_MOTOR_ROTATIONS;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    extensionMotor.getConfigurator().apply(config);
  }

  private void configureSpinMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    spinMotor.getConfigurator().apply(config);
  }

  @Override
  public double getExtensionMeters() {
    BaseStatusSignal.refreshAll(extensionVelocity);
    return extensionMotor.getPosition().getValueAsDouble() * METERS_PER_MOTOR_ROTATION;
  }

  @Override
  public double getExtensionVelocityMetersPerSecond() {
    return extensionVelocity.getValueAsDouble() * METERS_PER_MOTOR_ROTATION;
  }

  @Override
  public double getRollerVelocityRps() {
    BaseStatusSignal.refreshAll(spinVelocity);
    return spinVelocity.getValueAsDouble();
  }

  @Override
  public void setDeploymentVoltage(double volts) {
    extensionMotor.setVoltage(Math.max(-6.0, Math.min(6.0, volts)));
  }

  @Override
  public void setRollerVoltage(double volts) {
    spinMotor.setVoltage(Math.max(-6.0, Math.min(6.0, volts)));
  }

  /** Call only after physically placing the intake at its fully retracted position. */
  public void zeroExtensionEncoder() {
    extensionMotor.setPosition(0.0);
  }

  @Override
  public void stop() {
    extensionMotor.stopMotor();
    spinMotor.stopMotor();
  }
}
