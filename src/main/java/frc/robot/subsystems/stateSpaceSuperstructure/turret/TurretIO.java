package frc.robot.subsystems.stateSpaceSuperstructure.turret;

/** Hardware boundary modeled after the 2026 season turret's IO layer. */
public interface TurretIO {
  class Inputs {
    public double motorPositionRotations;
    public double motorVelocityRotationsPerSecond;
    public double absoluteEncoderRotations;
    public double angleRadians;
    public double velocityRadiansPerSecond;
    public double appliedVolts;
    public double currentAmps;
  }

  default void updateInputs(Inputs inputs) {}
  default void setVoltage(double volts) {}
  default void simulationPeriodic() {}
}
