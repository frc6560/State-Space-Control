package frc.robot.coordination;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.stateSpaceSuperstructure.ArmSubsystem;
import frc.robot.subsystems.stateSpaceSuperstructure.FlywheelSubsystem;
import frc.robot.subsystems.stateSpaceSuperstructure.turret.TurretSubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinates mechanism goals without owning their low-level motor control.
 *
 * <p>The arm, turret, and flywheel prepare in parallel. A later firing mechanism should be allowed
 * to feed only after every mechanism and the drivetrain have remained stable for the debounce
 * period. This avoids fragile fixed delays.
 */
public class SuperstructureCoordinator extends SubsystemBase {
  private final ArmSubsystem arm;
  private final TurretSubsystem turret;
  private final FlywheelSubsystem flywheel;
  private final BooleanSupplier drivetrainStable;
  private final Debouncer readyDebouncer = new Debouncer(
      Constants.Coordination.READY_DEBOUNCE_SECONDS, Debouncer.DebounceType.kRising);
  private boolean readyToFire;

  public SuperstructureCoordinator(
      ArmSubsystem arm,
      TurretSubsystem turret,
      FlywheelSubsystem flywheel,
      BooleanSupplier drivetrainStable) {
    this.arm = arm;
    this.turret = turret;
    this.flywheel = flywheel;
    this.drivetrainStable = drivetrainStable;
  }

  @Override
  public void periodic() {
    boolean instantReady = arm.atGoal() && turret.atGoal() && flywheel.atGoal()
        && drivetrainStable.getAsBoolean();
    readyToFire = readyDebouncer.calculate(instantReady);

    Logger.recordOutput("StateSpaceSuperstructure/Coordinator/ArmReady", arm.atGoal());
    Logger.recordOutput("StateSpaceSuperstructure/Coordinator/TurretReady", turret.atGoal());
    Logger.recordOutput("StateSpaceSuperstructure/Coordinator/FlywheelReady", flywheel.atGoal());
    Logger.recordOutput("StateSpaceSuperstructure/Coordinator/DrivetrainStable",
        drivetrainStable.getAsBoolean());
    Logger.recordOutput("StateSpaceSuperstructure/Coordinator/ReadyToFire", readyToFire);
  }

  public boolean readyToFire() {
    return readyToFire;
  }

  /** Starts all independent preparation work during the same scheduler cycle. */
  public Command prepareToShootCommand(double fieldTurretAngleDegrees) {
    return Commands.parallel(
        arm.scoreCommand(),
        Commands.runOnce(() -> turret.setFieldAngleDegrees(fieldTurretAngleDegrees), turret),
        flywheel.spinUpCommand());
  }

  /**
   * Waits for measured stability. The timeout prevents a failed sensor or mechanism from blocking
   * an autonomous routine forever; callers should check readyToFire before operating a feeder.
   */
  public Command waitUntilReadyCommand() {
    return Commands.waitUntil(this::readyToFire)
        .withTimeout(Constants.Coordination.PREPARE_TIMEOUT_SECONDS);
  }
}
