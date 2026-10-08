package frc.robot.subsystems.drive;

import java.util.Locale;
import java.util.Objects;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;

/**
 * A field pose a path passes through, facing its heading: {@code Keyframe.at(new Pose2d(4, 2,
 * Rotation2d.fromDegrees(90)))}. Named, {@code .named("pickup")}, it's a place code can wait for:
 * {@link DriveMove#passed} is a gate that opens as the robot passes it.
 *
 * @param pose where the path passes, and the heading the robot turns to as it comes from the one
 *     before
 * @param name its name, or null
 */
public record Keyframe(Pose2d pose, @Nullable String name) {
  /**
   * @throws IllegalArgumentException for a blank name
   */
  public Keyframe {
    Objects.requireNonNull(pose, "pose");
    if (name != null && name.isBlank()) {
      throw new IllegalArgumentException("a keyframe's name can't be blank");
    }
  }

  /** A keyframe at a field pose. */
  public static Keyframe at(Pose2d pose) {
    return new Keyframe(pose, null);
  }

  /** The same keyframe, named: {@code Keyframe.at(pose).named("pickup")}. */
  public Keyframe named(String name) {
    return new Keyframe(pose, name);
  }

  /** As a move's name shows it: its name, or "(4.00, 2.00)". */
  @Override
  public String toString() {
    return name != null
        ? name
        : String.format(Locale.ROOT, "(%.2f, %.2f)", pose.getX(), pose.getY());
  }
}
