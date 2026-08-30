package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Optional bindings for RobotContainer. These use a separate Xbox controller
 * mapping so the current button-board bindings can be reviewed independently.
 */
public final class IntakeXboxBindings {
  private IntakeXboxBindings() {}

  public static void configure(CommandXboxController controller,
                                IntakeStateSpaceSubsystem intake) {
    controller.a().onTrue(Commands.runOnce(
        () -> intake.setMode(IntakeStateSpaceController.Mode.RETRACTED), intake));
    controller.b().onTrue(Commands.runOnce(
        () -> intake.setMode(IntakeStateSpaceController.Mode.EXTENDED), intake));
    controller.y().onTrue(Commands.runOnce(
        () -> intake.setMode(IntakeStateSpaceController.Mode.OSCILLATING), intake));
    controller.rightTrigger().onTrue(Commands.runOnce(
        () -> intake.setMode(IntakeStateSpaceController.Mode.EXTENDED_SPINNING), intake));
    controller.rightTrigger().onFalse(Commands.runOnce(
        () -> intake.setMode(IntakeStateSpaceController.Mode.EXTENDED), intake));
    controller.back().onTrue(Commands.runOnce(
        () -> intake.setMode(IntakeStateSpaceController.Mode.RETRACTED), intake));
  }
}
