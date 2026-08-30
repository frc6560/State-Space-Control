package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class IntakeStateSpaceSubsystem extends SubsystemBase {
  private final IntakeStateSpaceIO io;
  private final IntakeStateSpaceController controller = new IntakeStateSpaceController();
  private IntakeStateSpaceController.Mode requestedMode =
      IntakeStateSpaceController.Mode.RETRACTED;
  private IntakeStateSpaceController.Output lastOutput;

  public IntakeStateSpaceSubsystem(IntakeStateSpaceIO io) { this.io = io; }

  public void setMode(IntakeStateSpaceController.Mode mode) {
    requestedMode = mode;
    controller.setMode(mode);
  }

  public IntakeStateSpaceController.Mode getMode() { return requestedMode; }
  public IntakeStateSpaceController.Output getLastOutput() { return lastOutput; }

  @Override
  public void periodic() {
    lastOutput = controller.calculate(
        io.getExtensionMeters(), io.getExtensionVelocityMetersPerSecond(),
        io.getRollerVelocityRps(), 0.02);
    io.setDeploymentVoltage(lastOutput.deploymentVolts());
    io.setRollerVoltage(lastOutput.rollerVolts());
  }

  public void stop() { io.stop(); }
}
