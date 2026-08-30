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
  public static final int LEFT_ROLLER_CAN_ID = 24;
  public static final int RIGHT_ROLLER_CAN_ID = 25;
  public static final String CAN_BUS = "rio";

  private final TalonFX left = new TalonFX(LEFT_ROLLER_CAN_ID, CAN_BUS);
  private final TalonFX right = new TalonFX(RIGHT_ROLLER_CAN_ID, CAN_BUS);
  private final StatusSignal<AngularVelocity> leftVelocity = left.getVelocity();
  private final StatusSignal<AngularVelocity> rightVelocity = right.getVelocity();
  private final StatusSignal<Voltage> leftVoltage = left.getMotorVoltage();
  private final StatusSignal<Voltage> rightVoltage = right.getMotorVoltage();

  public TalonFXRollerIO() {
    configure(left, false);
    configure(right, true);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leftVelocity, rightVelocity, leftVoltage, rightVoltage);
    left.optimizeBusUtilization();
    right.optimizeBusUtilization();
  }

  private static void configure(TalonFX motor, boolean inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = inverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(config);
  }

  @Override
  public double getRollerVelocityRps() {
    BaseStatusSignal.refreshAll(leftVelocity, rightVelocity);
    return 0.5 * (leftVelocity.getValueAsDouble() + rightVelocity.getValueAsDouble());
  }

  @Override
  public void setRollerVoltage(double volts) {
    double bounded = Math.max(-6.0, Math.min(6.0, volts));
    left.setVoltage(bounded);
    right.setVoltage(bounded);
  }

  @Override
  public void stop() {
    left.stopMotor();
    right.stopMotor();
  }
}
