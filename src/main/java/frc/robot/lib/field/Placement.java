package frc.robot.lib.field;

import java.util.Locale;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;

/**
 * The pre-match placement check: is the robot on a start position, and on the one expected? The
 * drive team reads it while setting up, instead of an auto spending its first seconds correcting a
 * placement error. It is only as good as the pose it is given: on a robot, that takes cameras that
 * see tags.
 *
 * @param status the verdict
 * @param start the start the robot is on, or the one it should move to; null when the alliance is
 *     unknown or the field has no starts
 * @param distanceMeters how far the robot is from that start
 * @param headingErrorDegrees how far the robot is turned from that start's heading
 */
public record Placement(
    Status status, @Nullable String start, double distanceMeters, double headingErrorDegrees) {
  public enum Status {
    /** The alliance isn't known yet, so neither are the start positions. */
    ALLIANCE_UNKNOWN,
    /** On the expected start (or on any start, when none is expected), within tolerance. */
    READY,
    /** On a start, but not the expected one. */
    WRONG_START,
    /** Not on any start. */
    NOT_ON_A_START
  }

  /**
   * Checks the robot's pose against the field's starts for its alliance.
   *
   * @param expected the start the robot should be on (the selected auto's), or null for any
   * @param toleranceMeters how far from a start still counts as on it
   * @param toleranceDegrees how far turned from a start's heading still counts as on it
   */
  public static Placement check(
      FieldModel field,
      Optional<Alliance> alliance,
      Pose2d pose,
      @Nullable Place expected,
      double toleranceMeters,
      double toleranceDegrees) {
    if (alliance.isEmpty()) {
      return new Placement(Status.ALLIANCE_UNKNOWN, null, 0, 0);
    }
    Placement nearest = null;
    Placement onExpected = null;
    for (Place start : field.starts()) {
      Pose2d spot = start.on(field, alliance.get());
      Placement candidate =
          new Placement(
              Status.NOT_ON_A_START,
              start.name(),
              pose.getTranslation().getDistance(spot.getTranslation()),
              Math.abs(pose.getRotation().minus(spot.getRotation()).getDegrees()));
      if (start.equals(expected)) {
        onExpected = candidate;
      }
      if (nearest == null || candidate.distanceMeters < nearest.distanceMeters) {
        nearest = candidate;
      }
    }
    if (nearest == null) {
      return new Placement(Status.NOT_ON_A_START, null, 0, 0);
    }
    if (onExpected != null && onExpected.within(toleranceMeters, toleranceDegrees)) {
      return onExpected.withStatus(Status.READY);
    }
    if (nearest.within(toleranceMeters, toleranceDegrees)) {
      return nearest.withStatus(expected == null ? Status.READY : Status.WRONG_START);
    }
    // Off every start: point the drive team to the one the auto expects.
    return onExpected != null ? onExpected : nearest;
  }

  /** One line for the drive team, such as "Ready on Center: 2 cm, 1° off". */
  public String describe() {
    return switch (status) {
      case ALLIANCE_UNKNOWN -> "Alliance unknown";
      case READY -> "Ready on " + start + ": " + offBy();
      case WRONG_START -> "On " + start + ", not the expected start";
      case NOT_ON_A_START ->
          start == null ? "No start positions" : "Not on a start: " + start + " is " + away();
    };
  }

  private boolean within(double toleranceMeters, double toleranceDegrees) {
    return distanceMeters <= toleranceMeters && headingErrorDegrees <= toleranceDegrees;
  }

  private Placement withStatus(Status newStatus) {
    return new Placement(newStatus, start, distanceMeters, headingErrorDegrees);
  }

  private String offBy() {
    return String.format(
        Locale.ROOT, "%.0f cm, %.0f° off", distanceMeters * 100, headingErrorDegrees);
  }

  private String away() {
    return String.format(Locale.ROOT, "%.2f m and %.0f° away", distanceMeters, headingErrorDegrees);
  }
}
