package frc.robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;

/** Rejects implausible AprilTag fixes and assigns uncertainty to accepted measurements. */
public final class VisionMeasurementGate {
  private VisionMeasurementGate() {}

  public static boolean shouldAccept(
      TimestampedVisionMeasurement measurement,
      Pose2d estimatedPose,
      double robotAngularVelocityRadPerSecond) {
    if (measurement == null || measurement.tagCount() < 1) {
      return false;
    }
    if (Math.abs(robotAngularVelocityRadPerSecond) > 12.0) {
      return false;
    }
    if (measurement.tagCount() == 1 && measurement.ambiguity() > 0.20) {
      return false;
    }
    if (measurement.averageTagDistanceMeters() > 6.0) {
      return false;
    }

    double translationJumpMeters = measurement.pose().getTranslation()
        .getDistance(estimatedPose.getTranslation());
    return translationJumpMeters < (measurement.tagCount() >= 2 ? 3.0 : 1.5);
  }

  /** Larger distance and single-tag observations receive less influence in the pose estimator. */
  public static Matrix<N3, edu.wpi.first.math.numbers.N1> standardDeviations(
      TimestampedVisionMeasurement measurement) {
    double distanceScale = Math.max(1.0,
        measurement.averageTagDistanceMeters() * measurement.averageTagDistanceMeters());
    double tagScale = measurement.tagCount() >= 2 ? 0.5 : 1.0;
    double xyStdDevMeters = 0.15 * distanceScale * tagScale;
    double thetaStdDevRadians = measurement.tagCount() >= 2 ? 0.35 * distanceScale : 1.0e6;
    return VecBuilder.fill(xyStdDevMeters, xyStdDevMeters, thetaStdDevRadians);
  }
}
