package com.michaelgrundvig.frc.world.field;

import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose3d;

/**
 * An AprilTag on the field, as known: where the game's official layout puts it, and where it was
 * found at this venue, if it was measured. Its pose faces out of its printed face, in WPILib's
 * field frame.
 *
 * <p>The official layout is nominal geometry, which the field's {@link Symmetry} pairs with the
 * other alliance's tags. A survey is a measurement, never mirrored: it says where this tag is, on
 * whichever half it stands.
 *
 * @param id its number
 * @param nominal where the official layout puts it
 * @param surveyed where a survey at this venue found it; null when nobody surveyed it
 */
public record Tag(int id, Pose3d nominal, @Nullable Pose3d surveyed) {
  public Tag {
    if (id < 0) {
      throw new IllegalArgumentException("a tag's number is zero or more, not " + id);
    }
  }

  /** A tag where the official layout puts it, nothing surveyed. */
  public static Tag nominal(int id, Pose3d pose) {
    return new Tag(id, pose, null);
  }

  /** Where it is: as surveyed, if it was, or else where the official layout puts it. */
  public Pose3d pose() {
    return surveyed != null ? surveyed : nominal;
  }
}
