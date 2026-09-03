package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.stateSpaceSuperstructure.ArmSubsystem;
import frc.robot.subsystems.stateSpaceSuperstructure.FlywheelSubsystem;
import frc.robot.subsystems.stateSpaceSuperstructure.turret.TurretIOTalonFX;
import frc.robot.subsystems.stateSpaceSuperstructure.turret.TurretIOSim;
import frc.robot.subsystems.stateSpaceSuperstructure.turret.TurretSubsystem;
import frc.robot.coordination.SuperstructureCoordinator;

/** Operator bindings for the three arm presets. */
public class RobotContainer {
  private final ArmSubsystem arm = new ArmSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem(
      RobotBase.isReal() ? new TurretIOTalonFX() : new TurretIOSim());
  private final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  // Replace this temporary supplier with the drivetrain's measured pose/velocity readiness test.
  private final SuperstructureCoordinator coordinator =
      new SuperstructureCoordinator(arm, turret, flywheel, () -> true);
  @SuppressWarnings("unused")
  private final CommandXboxController controller = new CommandXboxController(0);
  private final SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public RobotContainer() {
    controller.a().onTrue(arm.stowCommand());
    controller.b().onTrue(arm.intakeCommand());
    controller.x().onTrue(arm.scoreCommand());
    controller.y().onTrue(turret.forwardCommand());
    controller.leftBumper().onTrue(turret.leftCommand());
    controller.rightBumper().onTrue(turret.rightCommand());
    controller.start().onTrue(
        coordinator.prepareToShootCommand(Constants.Turret.FORWARD_DEGREES)
            .andThen(coordinator.waitUntilReadyCommand()));

    autonomousChooser.setDefaultOption("Do nothing", Commands.none());
    autonomousChooser.addOption("Turret SysId (full sequence)", turret.sysIdSequenceCommand());
    SmartDashboard.putData("Autonomous", autonomousChooser);
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}
