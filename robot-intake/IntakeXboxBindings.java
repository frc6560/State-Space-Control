package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class IntakeXboxBindings {
  private IntakeXboxBindings() {}

  public static void configure(CommandXboxController controller,
                                IntakeStateSpaceSubsystem intake) {
    // Hold B: extend and spin. Release B: stay extended, stop spinning.
    controller.b().onTrue(Commands.runOnce(
        () -> intake.setMode(IntakeStateSpaceController.Mode.EXTENDED_SPINNING), intake));
    controller.b().onFalse(Commands.runOnce(
        () -> intake.setMode(IntakeStateSpaceController.Mode.EXTENDED), intake));

    // Press A: retract completely and stop.
    controller.a().onTrue(Commands.runOnce(
        () -> intake.setMode(IntakeStateSpaceController.Mode.RETRACTED), intake));

    // Hold right trigger: oscillate. Release: stay extended, stop spinning.
    controller.rightTrigger().onTrue(Commands.runOnce(
        () -> intake.setMode(IntakeStateSpaceController.Mode.OSCILLATING), intake));
    controller.rightTrigger().onFalse(Commands.runOnce(
        () -> intake.setMode(IntakeStateSpaceController.Mode.EXTENDED), intake));

    // Back is an additional immediate safe retract.
    controller.back().onTrue(Commands.runOnce(
        () -> intake.setMode(IntakeStateSpaceController.Mode.RETRACTED), intake));
  }
}
