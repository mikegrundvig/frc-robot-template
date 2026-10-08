package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;

/**
 * The robot as a whole: its frame, bumpers, and mass. These live here rather than under the
 * drivetrain because simulation, path planning, and rule checks all read them.
 *
 * <p>Lengths run along the robot's x axis (front to back), widths along y (side to side). The
 * robot's origin is the center of the frame, on the floor.
 *
 * <p>Its mass is the robot as weighed, bumpers and battery on, with every mechanism at its joint's
 * rest position (stowed): its links are part of it. What needs the chassis alone, the simulator,
 * takes them off ({@code RobotMath.chassis}). Until it's weighed, it's the most the season allows
 * ({@link Weight#unweighed}), and the robot's missing-data report says so. Its inertia is CAD's
 * whole tensor, or its yaw alone, as PathPlanner, Choreo or a rotation test gives it, or neither;
 * what isn't given is a solid box's of its mass, bumpers and height ({@link #massProperties}):
 * never zero, or the drivetrain couldn't turn it. Its center of mass, which only the simulator
 * reads, is the frame's center, a third of its height up, until it's given ({@link #cgOrDefault}),
 * and the missing-data report notes it in the simulator.
 *
 * @param frameLength front to back, across the outside of the frame
 * @param frameWidth side to side, across the outside of the frame
 * @param height the tallest point, in the starting configuration
 * @param bumperEnds how far the bumpers stick out beyond the frame at the front and the back
 * @param bumperSides how far they stick out at the left and the right: the same as at the ends,
 *     often, but bumpers square around a frame that isn't are thicker on its shorter sides
 * @param bumperBottom how high the bumpers' bottom edge is above the floor
 * @param bumperTop how high their top edge is
 * @param weight everything that moves, robot, bumpers, and battery, with every mechanism stowed: as
 *     weighed, or the season's most until it is
 * @param cg its center of mass from the robot's origin, every mechanism stowed; none given, the
 *     frame's center, a third of its height up
 * @param inertia how hard it is to turn about its center of mass, along the robot's axes: CAD's
 *     whole tensor ({@link Turning#tensor}); its yaw alone ({@link Turning#yaw}), the box giving
 *     its pitch and roll; or none known ({@link Turning#BOX}), the box's all
 */
@ConfigRecord
public record Body(
    Distance frameLength,
    Distance frameWidth,
    Distance height,
    Distance bumperEnds,
    Distance bumperSides,
    Distance bumperBottom,
    Distance bumperTop,
    Weight weight,
    Optional<Translation3d> cg,
    Turning inertia) {
  public Body {
    Checks.on(Body.class)
        .field("frameLength", frameLength, positive())
        .field("frameWidth", frameWidth, positive())
        .field("height", height, positive())
        .field("bumperEnds", bumperEnds, positive())
        .field("bumperSides", bumperSides, positive())
        .field("bumperBottom", bumperBottom, nonNegative())
        .field("bumperTop", bumperTop, positive())
        .check(
            "bumperTop",
            () ->
                bumperTop.gt(bumperBottom) && bumperTop.lte(height)
                    ? null
                    : shown(bumperTop)
                        + " must be above the bumpers' bottom ("
                        + shown(bumperBottom)
                        + ") and no higher than the robot ("
                        + shown(height)
                        + ")")
        .check(
            "inertia",
            () ->
                whyNotABody(
                    inertia, box(weight, frameLength, frameWidth, height, bumperEnds, bumperSides)))
        .check(
            "cg", () -> cg.isPresent() ? outside(cg.get(), frameLength, frameWidth, height) : null)
        .orThrow();
  }

  /**
   * How hard the robot is to turn, as far as anyone knows: CAD's whole tensor, its yaw alone, or
   * neither. What isn't given, a solid box of its mass, bumpers and height supplies, and the
   * robot's missing-data report says which.
   */
  public sealed interface Turning permits Turning.Tensor, Turning.Yaw, Turning.Box {
    /** Nothing given: a solid box's, all of it. */
    Turning BOX = Box.BOX;

    /** CAD's whole tensor, about the center of mass, along the robot's axes. */
    static Turning tensor(Inertia inertia) {
      return Tensor.builder().inertia(inertia).build();
    }

    /**
     * The yaw alone, as PathPlanner's or Choreo's robot MOI, CAD or a rotation test gives it: a
     * solid box gives the pitch and roll, and no products of inertia.
     */
    static Turning yaw(MomentOfInertia yaw) {
      return Yaw.builder().yaw(yaw).build();
    }

    /**
     * CAD's whole tensor.
     *
     * @param inertia about the center of mass, along the robot's axes
     */
    @ConfigRecord
    record Tensor(Inertia inertia) implements Turning {
      /** Starts describing a whole tensor, one named value at a time. */
      public static BodyTurningTensorBuilder.InertiaStage builder() {
        return BodyTurningTensorBuilder.builder();
      }
    }

    /**
     * The yaw alone.
     *
     * @param yaw how hard the robot is to spin about the vertical through its center of mass
     */
    @ConfigRecord
    record Yaw(MomentOfInertia yaw) implements Turning {
      public Yaw {
        Checks.on(Yaw.class).field("yaw", yaw, positive()).orThrow();
      }

      /** Starts describing a yaw inertia, one named value at a time. */
      public static BodyTurningYawBuilder.YawStage builder() {
        return BodyTurningYawBuilder.builder();
      }
    }

    /** Nothing given. */
    enum Box implements Turning {
      BOX
    }
  }

  /**
   * Why a robot's inertia can't be a body's, or null when it can: a given tensor that turns too
   * easily about some axis, or a given yaw the box's pitch and roll can't stand beside, each of the
   * three at most the sum of the other two. Never adjusted: give the whole tensor instead.
   */
  private static @Nullable String whyNotABody(Turning inertia, Inertia box) {
    return switch (inertia) {
      case Turning.Tensor tensor -> tensor.inertia().whyNotSolid();
      case Turning.Yaw yaw -> {
        double izz = yaw.yaw().in(KilogramSquareMeters);
        yield izz <= box.ixx() + box.iyy()
                && box.ixx() <= box.iyy() + izz
                && box.iyy() <= box.ixx() + izz
            ? null
            : "a yaw of "
                + shown(izz)
                + " kg m² can't stand beside the box's roll and pitch, "
                + shown(box.ixx())
                + " and "
                + shown(box.iyy())
                + " kg m²: each must be at most the sum of the other two. Give the whole tensor";
      }
      case Turning.Box nothing -> null;
    };
  }

  /** A solid box of the robot's mass, as long and wide as its bumpers and as tall as it is. */
  private static Inertia box(
      Weight weight,
      Distance frameLength,
      Distance frameWidth,
      Distance height,
      Distance bumperEnds,
      Distance bumperSides) {
    return Inertia.box(
        weight.mass(),
        new Translation3d(
            frameLength.plus(bumperEnds.times(2)), frameWidth.plus(bumperSides.times(2)), height));
  }

  /** Why a center of mass isn't inside the frame and under its top, or null when it is. */
  private static @Nullable String outside(
      Translation3d cg, Distance frameLength, Distance frameWidth, Distance height) {
    return Math.abs(cg.getX()) <= frameLength.in(Meters) / 2
            && Math.abs(cg.getY()) <= frameWidth.in(Meters) / 2
            && cg.getZ() > 0
            && cg.getZ() < height.in(Meters)
        ? null
        : cg
            + " must be inside the frame, above the floor and below the robot's height ("
            + shown(height)
            + ")";
  }

  /**
   * How much the whole robot weighs: on a scale, bumpers and battery on, or until it's been
   * weighed, the most the season allows it with a battery, which the season's limits give ({@code
   * SeasonLimits.unweighed}). The robot's missing-data report lists an unweighed robot.
   *
   * @param mass robot, bumpers, and battery
   * @param weighed whether it's the robot on a scale, rather than the season's most
   */
  @ConfigRecord
  public record Weight(Mass mass, boolean weighed) {
    public Weight {
      Checks.on(Weight.class).field("mass", mass, positive()).orThrow();
    }

    /** Starts describing a weight, one named value at a time. */
    public static BodyWeightBuilder.MassStage builder() {
      return BodyWeightBuilder.builder();
    }

    /** The robot on a scale, bumpers and battery on, every mechanism stowed. */
    public static Weight weighed(Mass mass) {
      return builder().mass(mass).weighed(true).build();
    }

    /**
     * Not weighed yet: the most the season allows, with a battery, which its limits give ({@code
     * SeasonLimits.unweighed}). The robot's missing-data report says so.
     */
    public static Weight unweighed(Mass allowed) {
      return builder().mass(allowed).weighed(false).build();
    }
  }

  /** Everything that moves: robot, bumpers, and battery. */
  public Mass mass() {
    return weight.mass();
  }

  /**
   * Its mass, center of mass and inertia together: what's given of its inertia, and a solid box of
   * its mass, as long and wide as its bumpers and as tall as it is, for the rest: all of it, or the
   * pitch and roll beside a given yaw, with no products of inertia.
   */
  public MassProperties massProperties() {
    Inertia box = box(weight, frameLength, frameWidth, height, bumperEnds, bumperSides);
    Inertia resolved =
        switch (inertia) {
          case Turning.Tensor tensor -> tensor.inertia();
          case Turning.Yaw yaw ->
              Inertia.builder()
                  .ixx(box.ixx())
                  .iyy(box.iyy())
                  .izz(yaw.yaw().in(KilogramSquareMeters))
                  .ixy(0)
                  .ixz(0)
                  .iyz(0)
                  .build();
          case Turning.Box nothing -> box;
        };
    return MassProperties.builder().mass(weight.mass()).cg(cgOrDefault()).inertia(resolved).build();
  }

  /** How hard the robot is to spin about the vertical axis through its center of mass. */
  public MomentOfInertia yawMomentOfInertia() {
    return KilogramSquareMeters.of(massProperties().inertia().izz());
  }

  /**
   * Its center of mass from the robot's origin, every mechanism stowed: as given, or until it is,
   * the frame's center, a third of the robot's height up.
   */
  public Translation3d cgOrDefault() {
    return cg.orElseGet(() -> new Translation3d(0, 0, height.in(Meters) / 3));
  }

  /**
   * The center of mass's height above the floor, every mechanism stowed, as {@link #cgOrDefault}.
   */
  public Distance cgHeight() {
    return Meters.of(cgOrDefault().getZ());
  }

  /** Starts building a body, one named value at a time. */
  public static BodyBuilder.FrameLengthStage builder() {
    return BodyBuilder.builder();
  }

  /** Front to back, across the outside of the bumpers. */
  public Distance bumperLength() {
    return frameLength.plus(bumperEnds.times(2));
  }

  /** Side to side, across the outside of the bumpers. */
  public Distance bumperWidth() {
    return frameWidth.plus(bumperSides.times(2));
  }

  /** Around the outside of the frame. The game rules limit it. */
  public Distance framePerimeter() {
    return frameLength.plus(frameWidth).times(2);
  }

  /**
   * Whether a point, measured from the robot's origin, lies within the robot: inside the bumpers,
   * and between the floor and the robot's height.
   */
  public boolean contains(Translation3d point) {
    return Math.abs(point.getX()) <= bumperLength().in(Meters) / 2
        && Math.abs(point.getY()) <= bumperWidth().in(Meters) / 2
        && point.getZ() >= 0
        && point.getZ() <= height.in(Meters);
  }
}
