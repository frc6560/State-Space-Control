package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Robot entry point. Keep initialization in {@link Robot}, not here. */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
