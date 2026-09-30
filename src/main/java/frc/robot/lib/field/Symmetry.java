package frc.robot.lib.field;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * How the red alliance's half of the field relates to the blue alliance's. Fields have been one of
 * two kinds, and which one changes by season, so each season's field model declares it. WPILib's
 * tag layouts show 2022, 2025, and 2026 rotated, and 2023 and 2024 mirrored.
 *
 * <p>Each applies to a whole pose, position and heading together, and applying it twice gives back
 * what it started with.
 */
public enum Symmetry {
  /** Red's half is blue's reflected across the center line, x = length / 2: 2023 and 2024. */
  MIRRORED,

  /** Red's half is blue's turned half a turn about the center of the field: 2022, 2025, 2026. */
  ROTATED;

  /** Where a point on one alliance's half is on the other's. */
  public Translation2d apply(Translation2d point, double length, double width) {
    return switch (this) {
      case MIRRORED -> new Translation2d(length - point.getX(), point.getY());
      case ROTATED -> new Translation2d(length - point.getX(), width - point.getY());
    };
  }

  /** Which way a heading on one alliance's half points on the other's. */
  public Rotation2d apply(Rotation2d heading) {
    return switch (this) {
      case MIRRORED -> Rotation2d.PI.minus(heading);
      case ROTATED -> heading.plus(Rotation2d.PI);
    };
  }

  /** Where a pose on one alliance's half is on the other's. */
  public Pose2d apply(Pose2d pose, double length, double width) {
    return new Pose2d(apply(pose.getTranslation(), length, width), apply(pose.getRotation()));
  }
}
