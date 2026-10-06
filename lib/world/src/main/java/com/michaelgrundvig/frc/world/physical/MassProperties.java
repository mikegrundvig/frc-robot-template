package com.michaelgrundvig.frc.world.physical;

import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Kilograms;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.List;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Mass;

/**
 * How much a part weighs, where its weight is centered, and how hard it is to turn: all a physics
 * engine needs to move it, and what gravity's pull on a mechanism comes from. In the part's own
 * frame: a whole robot's, or a link's.
 *
 * <p>What isn't known is zero, not a guess ({@link #NONE}): a link nobody has weighed has no mass,
 * centered on its frame's origin, and turns freely. Everything derived from it stays consistent at
 * zero: no weight to hold up, nothing added to what it hangs from.
 *
 * @param mass how much it weighs: zero when nobody knows
 * @param cg its center of mass, in its frame, in meters
 * @param inertia how hard it is to turn, about its center of mass, along its frame's axes: zero
 *     when nobody knows, or for a point mass
 */
@ConfigRecord
public record MassProperties(Mass mass, Translation3d cg, Inertia inertia) {
  /**
   * None known: no mass, centered on its frame's origin (a link's joint), and no inertia. A link
   * starts here until CAD or a scale gives its own; the robot's missing-data report lists each.
   */
  public static final MassProperties NONE =
      MassPropertiesBuilder.builder()
          .mass(Kilograms.of(0))
          .cg(Translation3d.ZERO)
          .inertia(Inertia.ZERO)
          .build();

  public MassProperties {
    Checks.on(MassProperties.class)
        .field("mass", mass, nonNegative())
        .field(
            "cg",
            cg,
            point ->
                Double.isFinite(point.getX() + point.getY() + point.getZ())
                    ? null
                    : point + " must be a point, numbers all")
        .orThrow();
  }

  /** Whether it has no mass: none given. */
  public boolean isMassless() {
    return mass.in(Kilograms) == 0;
  }

  /** Starts describing a part's mass properties, one named value at a time. */
  public static MassPropertiesBuilder.MassStage builder() {
    return MassPropertiesBuilder.builder();
  }

  /**
   * The same part, seen from another frame: given in a frame that sits at {@code frame} in the
   * other, as a link's are given in its joint's frame and the robot's are wanted.
   */
  public MassProperties transformedBy(Transform3d frame) {
    return builder()
        .mass(mass)
        .cg(cg.rotateBy(frame.getRotation()).plus(frame.getTranslation()))
        .inertia(inertia.rotated(frame.getRotation()))
        .build();
  }

  /** Its inertia tensor about a point rather than its center of mass, row by row. */
  double[] tensorAbout(Translation3d point) {
    double m = mass.in(Kilograms);
    double[] t = inertia.entries();
    double[] d = {cg.getX() - point.getX(), cg.getY() - point.getY(), cg.getZ() - point.getZ()};
    double squared = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        t[3 * i + j] += m * ((i == j ? squared : 0) - d[i] * d[j]);
      }
    }
    return t;
  }

  /**
   * Parts given in one frame, as one: their masses summed, at their common center of mass. Parts
   * with no mass add nothing; if none has any, the sum has none ({@link #NONE}).
   *
   * @throws IllegalArgumentException if there are none
   */
  public static MassProperties sum(List<MassProperties> parts) {
    if (parts.isEmpty()) {
      throw new IllegalArgumentException("a sum of no parts has no mass");
    }
    double total = 0;
    Translation3d moment = Translation3d.ZERO;
    for (MassProperties part : parts) {
      double m = part.mass.in(Kilograms);
      total += m;
      moment = moment.plus(part.cg.times(m));
    }
    if (total == 0) {
      return NONE;
    }
    Translation3d center = moment.div(total);
    double[] tensor = new double[9];
    for (MassProperties part : parts) {
      double[] about = part.tensorAbout(center);
      for (int i = 0; i < 9; i++) {
        tensor[i] += about[i];
      }
    }
    return builder().mass(Kilograms.of(total)).cg(center).inertia(Inertia.of(tensor)).build();
  }

  /**
   * What's left once a part, given in the same frame, is taken away: a robot as weighed, less its
   * mechanisms' links, is its chassis.
   *
   * @throws IllegalArgumentException naming what isn't a body's, if what's left has no mass, or
   *     turns more easily than any body could
   */
  public MassProperties minus(MassProperties part) {
    double left = mass.in(Kilograms) - part.mass.in(Kilograms);
    if (!(left > 0)) {
      throw new IllegalArgumentException(
          "nothing is left: " + shown(part.mass) + " taken from " + shown(mass));
    }
    Translation3d center =
        cg.times(mass.in(Kilograms)).minus(part.cg.times(part.mass.in(Kilograms))).div(left);
    double[] whole = tensorAbout(center);
    double[] away = part.tensorAbout(center);
    for (int i = 0; i < 9; i++) {
      whole[i] -= away[i];
    }
    return builder().mass(Kilograms.of(left)).cg(center).inertia(Inertia.of(whole)).build();
  }
}
