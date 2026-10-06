package com.michaelgrundvig.frc.world.field;

import java.util.Arrays;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A named place on the field's floor, written as if the robot were on the blue alliance: a start
 * position, a spot to line up on. {@link #on} gives it for either alliance.
 *
 * <p>Places near AprilTags are best derived from them ({@link #between}, {@link #facing}): each
 * alliance's is then found from that alliance's own tags, as the layout measured them, instead of
 * by turning blue's numbers around.
 */
public sealed interface Place {
  /** What robot code, logs, and the drive team call it. */
  String name();

  /** Where the place is for an alliance, in field coordinates. */
  Pose2d on(FieldModel field, Alliance alliance);

  /**
   * The tags the place is found from, for an alliance: the ones a task lining up on it should
   * trust. None for a place written down as a pose.
   */
  List<Integer> tags(FieldModel field, Alliance alliance);

  /**
   * A pose written down for blue, from the game manual or CAD. Red's is its image under the field's
   * symmetry.
   */
  static Place at(String name, Pose2d blue) {
    return new Fixed(name, blue, null);
  }

  /**
   * A pose given for each alliance, where the field isn't symmetric: the symmetry is a convenience,
   * never an assumption.
   */
  static Place at(String name, Pose2d blue, Pose2d red) {
    return new Fixed(name, blue, red);
  }

  /**
   * The point midway between tags (a hub's center, between tags on opposite faces), facing away
   * from the alliance's drivers.
   *
   * @param blueTags the tags as seen on the blue alliance's half; red's are their partners
   */
  static Place between(String name, int... blueTags) {
    return new BetweenTags(name, Arrays.stream(blueTags).boxed().toList());
  }

  /**
   * A pose in front of a tag, facing it: where the robot's center is when it lines up on the tag
   * from a distance.
   *
   * @param blueTag the tag as seen on the blue alliance's half; red's is its partner
   * @param distanceMeters from the tag's face to the robot's center
   */
  static Place facing(String name, int blueTag, double distanceMeters) {
    return new FacingTag(name, blueTag, distanceMeters);
  }

  /** A place written down for blue, and optionally for red. */
  record Fixed(String name, Pose2d blue, @Nullable Pose2d red) implements Place {
    public Fixed {
      requireName(name);
    }

    @Override
    public Pose2d on(FieldModel field, Alliance alliance) {
      return alliance == Alliance.RED && red != null ? red : field.forAlliance(alliance, blue);
    }

    @Override
    public List<Integer> tags(FieldModel field, Alliance alliance) {
      return List.of();
    }
  }

  /** The point midway between tags. */
  record BetweenTags(String name, List<Integer> blueTags) implements Place {
    public BetweenTags {
      requireName(name);
      if (blueTags.isEmpty()) {
        throw new IllegalArgumentException(name + " needs at least one tag");
      }
      blueTags = List.copyOf(blueTags);
    }

    @Override
    public Pose2d on(FieldModel field, Alliance alliance) {
      Translation2d sum = Translation2d.ZERO;
      for (int tag : blueTags) {
        sum = sum.plus(field.tagPose(alliance, tag).getTranslation());
      }
      return new Pose2d(sum.div(blueTags.size()), FieldModel.driverForward(alliance));
    }

    @Override
    public List<Integer> tags(FieldModel field, Alliance alliance) {
      return blueTags.stream().map(tag -> field.tagFor(alliance, tag)).toList();
    }
  }

  /** A pose in front of a tag, facing it. */
  record FacingTag(String name, int blueTag, double distanceMeters) implements Place {
    public FacingTag {
      requireName(name);
      if (!(distanceMeters >= 0)) {
        throw new IllegalArgumentException(name + ": the distance must be zero or more");
      }
    }

    @Override
    public Pose2d on(FieldModel field, Alliance alliance) {
      // A tag's pose faces out of its face: step out along it, then turn around to face the tag.
      return field
          .tagPose(alliance, blueTag)
          .transformBy(new Transform2d(distanceMeters, 0, Rotation2d.PI));
    }

    @Override
    public List<Integer> tags(FieldModel field, Alliance alliance) {
      return List.of(field.tagFor(alliance, blueTag));
    }
  }

  private static void requireName(String name) {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a place needs a name");
    }
  }
}
