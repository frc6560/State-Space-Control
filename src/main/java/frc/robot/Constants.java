package frc.robot;

import edu.wpi.first.math.util.Units;

/** Hardware IDs and physical measurements for the demonstration arm. */
public final class Constants {
  private Constants() {}

  public static final class Arm {
    private Arm() {}

    public static final int MOTOR_CAN_ID = 18; // Matches Summer2024's hood motor ID.
    public static final int GYRO_CAN_ID = 19;  // Pigeon 2 mounted to the arm.
    public static final double LOOP_PERIOD_SECONDS = 0.020;

    // Replace these four measured values before using this on a real robot.
    public static final double GEAR_RATIO = 100.0; // Motor rotations / arm rotation.
    public static final double ARM_LENGTH_METERS = Units.inchesToMeters(18.0);
    public static final double ARM_MASS_KG = 4.0;
    public static final double MOMENT_OF_INERTIA_KG_METERS_SQUARED = 1.15;

    public static final double MIN_ANGLE_RAD = Units.degreesToRadians(10.0);
    public static final double MAX_ANGLE_RAD = Units.degreesToRadians(115.0);
    public static final double STOWED_ANGLE_RAD = Units.degreesToRadians(20.0);
    public static final double INTAKE_ANGLE_RAD = Units.degreesToRadians(55.0);
    public static final double SCORE_ANGLE_RAD = Units.degreesToRadians(100.0);

    // Holding-voltage estimate. Tune on the real arm after the mechanism is characterized.
    public static final double kG_VOLTS = 1.15;
  }

  /** Values copied from the 2026 season turret where available; tune the model before real use. */
  public static final class Turret {
    private Turret() {}

    public static final int MOTOR_CAN_ID = 17;
    public static final int ABSOLUTE_ENCODER_CAN_ID = 18;
    public static final String CAN_BUS = "rio";
    public static final double MOTOR_GEAR_RATIO = 254.0 / 28.0;
    public static final double ENCODER_GEAR_RATIO = 127.0 / 168.0;
    public static final double ABSOLUTE_ENCODER_OFFSET_ROTATIONS = 0.377;

    // First-pass model: treat the approximately 20 lb turret as a thin ring.
    public static final double RING_RADIUS_METERS = 0.30;
    public static final double MASS_KG = Units.lbsToKilograms(20.0);
    public static final double MOMENT_OF_INERTIA_KG_METERS_SQUARED =
        MASS_KG * RING_RADIUS_METERS * RING_RADIUS_METERS;
    public static final double LOOP_PERIOD_SECONDS = 0.020;

    // Replace these with the radians-based kV and kA produced by the turret SysId run. Until both
    // are positive, the controller falls back to the ring-based plant model above.
    public static final double CHARACTERIZED_KV_VOLTS_PER_RADIAN_PER_SECOND = Double.NaN;
    public static final double CHARACTERIZED_KA_VOLTS_PER_RADIAN_PER_SECOND_SQUARED = Double.NaN;

    // Conservative characterization settings for the selectable autonomous SysId sequence.
    public static final double SYSID_RAMP_VOLTS_PER_SECOND = 0.5;
    public static final double SYSID_STEP_VOLTS = 4.0;
    public static final double SYSID_TIMEOUT_SECONDS = 8.0;
    public static final double SYSID_LIMIT_MARGIN_DEGREES = 20.0;

    // These are continuous, unwrapped mechanical angles—not wrapped headings.
    public static final double LOWER_SOFT_LIMIT_DEGREES = -170.0;
    public static final double UPPER_SOFT_LIMIT_DEGREES = 270.0;
    public static final double FORWARD_DEGREES = 0.0;
    public static final double LEFT_DEGREES = 90.0;
    public static final double RIGHT_DEGREES = -90.0;
  }
}
