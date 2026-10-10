package frc.robot.subsystems.drive;

import com.michaelgrundvig.frc.world.field.Place;
import java.util.Locale;
import java.util.Objects;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;

/**
 * A pose a path passes through, facing its heading: {@code Keyframe.at(new Pose2d(4, 2,
 * Rotation2d.fromDegrees(90)))} on the field's coordinates, or {@code Keyframe.at(place)} at a
 * {@link Place} written once for blue, which the move finds for the robot's alliance when it
 * starts. Named, {@code .named("pickup")}, it's a place code can wait for: {@link DriveMove#passed}
 * is a gate that opens as the robot passes it.
 *
 * @param pose where the path passes, in field coordinates, and the heading the robot turns to as it
 *     comes from the one before; null for a keyframe at a place
 * @param place where it passes, for the robot's alliance; null for a keyframe at a pose
 * @param name its name, or null
 */
public record Keyframe(@Nullable Pose2d pose, @Nullable Place place, @Nullable String name) {
  /**
   * @throws IllegalArgumentException for a blank name
   */
  public Keyframe {
    if ((pose == null) == (place == null)) {
      throw new IllegalArgumentException("a keyframe is at a pose or at a place, not both");
    }
    if (name != null && name.isBlank()) {
      throw new IllegalArgumentException("a keyframe's name can't be blank");
    }
  }

  /** A keyframe at a pose, in field coordinates. */
  public static Keyframe at(Pose2d pose) {
    return new Keyframe(Objects.requireNonNull(pose, "pose"), null, null);
  }

  /**
   * A keyframe at a place, as it is for the robot's alliance when the move starts: a path written
   * once is right on both. A move that starts with the alliance unknown ends REFUSED.
   */
  public static Keyframe at(Place place) {
    return new Keyframe(null, Objects.requireNonNull(place, "place"), null);
  }

  /** The same keyframe, named: {@code Keyframe.at(pose).named("pickup")}. */
  public Keyframe named(String name) {
    return new Keyframe(pose, place, name);
  }

  /** As a move's name shows it: its name, a place's name, or "(4.00, 2.00)". */
  @Override
  public String toString() {
    if (name != null) {
      return name;
    }
    return pose != null
        ? String.format(Locale.ROOT, "(%.2f, %.2f)", pose.getX(), pose.getY())
        : Objects.requireNonNull(place).name();
  }
}
