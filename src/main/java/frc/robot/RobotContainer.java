package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.stateSpaceSuperstructure.ArmSubsystem;
import frc.robot.subsystems.stateSpaceSuperstructure.turret.TurretIO;
import frc.robot.subsystems.stateSpaceSuperstructure.turret.TurretIOTalonFX;
import frc.robot.subsystems.stateSpaceSuperstructure.turret.TurretIOSim;
import frc.robot.subsystems.stateSpaceSuperstructure.turret.TurretSubsystem;

/** Operator bindings for the three arm presets. */
public class RobotContainer {
  private final ArmSubsystem arm = new ArmSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem(
      RobotBase.isReal() ? new TurretIOTalonFX() : new TurretIOSim());
  @SuppressWarnings("unused")
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    controller.a().onTrue(arm.stowCommand());
    controller.b().onTrue(arm.intakeCommand());
    controller.x().onTrue(arm.scoreCommand());
    controller.y().onTrue(turret.forwardCommand());
    controller.leftBumper().onTrue(turret.leftCommand());
    controller.rightBumper().onTrue(turret.rightCommand());
  }
}
