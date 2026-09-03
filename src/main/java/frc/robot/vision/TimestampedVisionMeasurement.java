package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

/** A Limelight field-pose result after camera latency has been converted to an FPGA timestamp. */
public record TimestampedVisionMeasurement(
    Pose2d pose,
    double timestampSeconds,
    int tagCount,
    double averageTagDistanceMeters,
    double ambiguity) {}
