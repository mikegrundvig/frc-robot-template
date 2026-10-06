package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.MomentOfInertia;

/**
 * A swerve drivetrain, as built: a module at each corner, each of its own type and where its wheel
 * touches the floor. What it is, not how it's driven: its limits and gains are the drivetrain's own
 * configuration, tuned while the robot runs, and this is fixed until the robot is rebuilt.
 *
 * <p>Most drivetrains have one type of module at every corner, placed symmetrically: {@link
 * #uniform} places them as Phoenix Tuner X's swerve project gives them, and {@link #inset} from a
 * frame's edges as a module's footprint drawing does. A drivetrain that mixes types (MK5i in front
 * and MK5n behind) lists its corners.
 *
 * <p>Robot frame (WPILib's): +x forward, +y left, counterclockwise positive. Corner order is always
 * front left, front right, back left, back right.
 *
 * @param corners the four modules: front left, front right, back left, back right
 */
@ConfigRecord
public record DrivetrainGeometry(List<Corner> corners) {
  /** The corners' names, in their order: front left, front right, back left, back right. */
  public static final List<String> LABELS =
      List.of("front-left", "front-right", "back-left", "back-right");

  public DrivetrainGeometry {
    corners = List.copyOf(corners);
    List<Corner> all = corners;
    Checks.on(DrivetrainGeometry.class)
        .field("corners", corners, DrivetrainGeometry::notFourCorners)
        .check("corners", () -> all.size() == 4 ? misplaced(all) : null)
        .orThrow();
  }

  /** Starts describing a swerve drivetrain, one corner at a time. */
  public static DrivetrainGeometryBuilder.CornersStage builder() {
    return DrivetrainGeometryBuilder.builder();
  }

  /**
   * The same type of module at every corner, the wheels a distance ahead of and behind the center,
   * and to either side: as Phoenix Tuner X's swerve project gives them, its front left's x and y.
   *
   * @param forward how far the front wheels are ahead of the center, and the back ones behind it
   * @param left how far the left wheels are left of the center, and the right ones right of it
   */
  public static DrivetrainGeometry uniform(ModuleType module, Distance forward, Distance left) {
    return placed(module, forward.in(Meters), left.in(Meters));
  }

  /**
   * The same type of module at every corner, each wheel's center an inset in from the outside of a
   * frame, in both directions: corner-mounted modules give it in their footprint drawings.
   *
   * @throws com.michaelgrundvig.frc.world.config.ConfigException if the inset leaves no room
   *     between the wheels
   */
  public static DrivetrainGeometry inset(ModuleType module, Body body, Distance wheelInset) {
    return placed(
        module,
        body.frameLength().in(Meters) / 2 - wheelInset.in(Meters),
        body.frameWidth().in(Meters) / 2 - wheelInset.in(Meters));
  }

  private static DrivetrainGeometry placed(ModuleType module, double x, double y) {
    double[][] signs = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    List<Corner> corners = new ArrayList<>();
    for (double[] sign : signs) {
      corners.add(
          Corner.builder()
              .module(module)
              .position(new Translation2d(sign[0] * x, sign[1] * y))
              .build());
    }
    return builder().corners(corners).build();
  }

  /**
   * One corner's module: its type, and where its wheel touches the floor.
   *
   * @param module the module's hardware
   * @param position where its wheel touches the floor, from the robot's center: forward and left
   */
  @ConfigRecord
  public record Corner(ModuleType module, Translation2d position) {
    public Corner {
      Checks.on(Corner.class)
          .field(
              "position",
              position,
              at ->
                  Double.isFinite(at.getX() + at.getY())
                      ? null
                      : at + " must be a point, numbers both")
          .orThrow();
    }

    /** Starts describing a corner, one named value at a time. */
    public static DrivetrainGeometryCornerBuilder.ModuleStage builder() {
      return DrivetrainGeometryCornerBuilder.builder();
    }
  }

  /**
   * A type of swerve module: its hardware, the same wherever one of its type sits. Wheel radius
   * shrinks as the tread wears; measure it again during the season.
   *
   * @param driveMotor the motor that drives the wheel
   * @param steerMotor the motor that turns the module
   * @param wheelRadius measured on the tread
   * @param driveGearing between the drive motor and the wheel: motor rotations per wheel rotation
   * @param steerGearing between the steering motor and the module: motor rotations per full turn of
   *     the module
   * @param wheelCoefficientOfFriction the tread's grip on carpet: about 1.0 when worn, 1.3 new
   * @param driveInertia the drive's own moving parts (the wheel with its tread, its gears, and the
   *     motor's rotor through them), as inertia at the wheel. The robot's mass the wheel moves
   *     comes on top of it, from the body.
   * @param steerInertia the module turning about its steering axis, with the steering motor's rotor
   *     through its gearing: what the steering motor turns
   * @param couplingRatio how many turns the drive motor makes for each turn of the module, with the
   *     wheel held still: turning an MK4-style module turns its wheel through the bevel gear, which
   *     the drive's encoder would otherwise count as travel. Zero for a module without the coupling
   */
  @ConfigRecord
  public record ModuleType(
      Motor driveMotor,
      Motor steerMotor,
      Distance wheelRadius,
      Gearing driveGearing,
      Gearing steerGearing,
      double wheelCoefficientOfFriction,
      MomentOfInertia driveInertia,
      MomentOfInertia steerInertia,
      double couplingRatio) {
    public ModuleType {
      Checks.on(ModuleType.class)
          .field("wheelRadius", wheelRadius, positive())
          .field("wheelCoefficientOfFriction", wheelCoefficientOfFriction, positive())
          .field("driveInertia", driveInertia, positive())
          .field("steerInertia", steerInertia, positive())
          .field("couplingRatio", couplingRatio, nonNegative())
          .orThrow();
    }

    /** Starts describing a type of module, one named value at a time. */
    public static DrivetrainGeometryModuleTypeBuilder.DriveMotorStage builder() {
      return DrivetrainGeometryModuleTypeBuilder.builder();
    }

    /** What drives the wheel: one drive motor. Its gearing is {@link #driveGearing}. */
    public Actuator.Motors driveMotors() {
      return Actuator.Motors.builder().motor(driveMotor).count(1).build();
    }

    /** What turns the module: one steering motor. Its gearing is {@link #steerGearing}. */
    public Actuator.Motors steerMotors() {
      return Actuator.Motors.builder().motor(steerMotor).count(1).build();
    }

    /**
     * One module, placed: its steering joint on the chassis where its wheel touches the floor, its
     * wheel's joint on that at the axle, the drive motor turning the wheel and, through the
     * coupling, the module's steering too, and the steering motor turning the module.
     *
     * @param label which corner, for logs: "front-left"
     * @param at where the wheel touches the floor, from the robot's center
     */
    public ModuleJoints joints(String label, Translation2d at) {
      double radius = wheelRadius.in(Meters);
      Joint.Revolute steer =
          Joint.Revolute.builder()
              .label(label + " steer")
              .parent(Part.CHASSIS)
              .origin(new Transform3d(at.getX(), at.getY(), 0, Rotation3d.ZERO))
              .axis(Vector3.UP)
              .range(Optional.empty())
              .reference(0)
              .link(disc(label + " module", steerInertia, radius, Vector3.UP))
              .positions(List.of())
              .outputs(List.of())
              .build();
      Joint.Revolute wheel =
          Joint.Revolute.builder()
              .label(label + " wheel")
              .parent(steer)
              .origin(new Transform3d(0, 0, radius, Rotation3d.ZERO))
              .axis(new Vector3(0, 1, 0))
              .range(Optional.empty())
              .reference(0)
              .link(disc(label + " wheel", driveInertia, radius, new Vector3(0, 1, 0)))
              .positions(List.of())
              .outputs(List.of())
              .build();
      Drive drive =
          Drive.builder()
              .label(label + " drive")
              .actuator(driveMotors())
              .transmission(
                  Transmission.Combined.builder()
                      .terms(terms(wheel, steer))
                      .efficiency(driveGearing.efficiency())
                      .build())
              .build();
      Drive steering =
          Drive.builder()
              .label(label + " steer")
              .actuator(steerMotors())
              .transmission(
                  Transmission.Geared.builder().joint(steer).gearing(steerGearing).build())
              .build();
      return new ModuleJoints(label, steer, wheel, drive, steering);
    }

    /** The drive motor's turns: the drive's ratio on the wheel, and the coupling's on steering. */
    private List<Transmission.Term> terms(Joint wheel, Joint steer) {
      List<Transmission.Term> terms = new ArrayList<>();
      terms.add(
          Transmission.Term.builder()
              .joint(wheel)
              .ratio(driveGearing.ratio())
              .pitchRadius(Optional.empty())
              .build());
      if (couplingRatio != 0) {
        terms.add(
            Transmission.Term.builder()
                .joint(steer)
                .ratio(couplingRatio)
                .pitchRadius(Optional.empty())
                .build());
      }
      return terms;
    }

    /**
     * A module's moving part as a joint's link: only as much as its transmission needs, a disc of
     * the wheel's radius as hard to turn about its axis as the inertia given. The module's mass is
     * the body's: the drivetrain's joints aren't among the robot's, and take nothing from it.
     */
    private static Link disc(String label, MomentOfInertia about, double radius, Vector3 axis) {
      double inertia = about.in(KilogramSquareMeters);
      double across = inertia / 2;
      return Link.builder()
          .label(label)
          .mass(
              MassProperties.builder()
                  .mass(Kilograms.of(2 * inertia / (radius * radius)))
                  .cg(Translation3d.ZERO)
                  .inertia(
                      Inertia.builder()
                          .ixx(axis.x() != 0 ? inertia : across)
                          .iyy(axis.y() != 0 ? inertia : across)
                          .izz(axis.z() != 0 ? inertia : across)
                          .ixy(0)
                          .ixz(0)
                          .iyz(0)
                          .build())
                  .build())
          .shapes(List.of())
          .build();
    }

    /** Top wheel speed at the motor's nominal voltage, with no load. The real robot is slower. */
    public LinearVelocity freeSpeed() {
      return MetersPerSecond.of(
          driveMotor.model().freeSpeed / driveGearing.ratio() * wheelRadius.in(Meters));
    }

    /**
     * The drive kV (volts per meter per second) a frictionless drivetrain would have: the motor's
     * nominal voltage over the free speed. A starting point; measure the real one.
     */
    public double idealDriveKv() {
      return driveMotor.model().nominalVoltage / freeSpeed().in(MetersPerSecond);
    }
  }

  /**
   * A swerve module at one corner, as joints and drives: what the derived functions read, as they
   * read a mechanism's (the drive motor's {@code Combined} transmission gives Phoenix its coupling,
   * and the ideal kV its wheel).
   *
   * @param label which corner: "front-left", "front-right", "back-left", "back-right"
   * @param steer the module turning on the chassis, about the vertical through its wheel
   * @param wheel the wheel turning on its axle, forward positive
   * @param drive the drive motor: the drive's ratio on the wheel, and the coupling's on steering
   * @param steering the steering motor, through the steering's gearing
   */
  public record ModuleJoints(
      String label, Joint.Revolute steer, Joint.Revolute wheel, Drive drive, Drive steering) {}

  /**
   * The four modules as joints and drives, each where its corner puts it: front left, front right,
   * back left, back right. They aren't the robot's joints: they ride inside its body, and its mass
   * has them.
   */
  public List<ModuleJoints> joints() {
    List<ModuleJoints> joints = new ArrayList<>();
    for (int i = 0; i < corners.size(); i++) {
      Corner corner = corners.get(i);
      joints.add(corner.module().joints(LABELS.get(i), corner.position()));
    }
    return joints;
  }

  /** Each corner's type of module: front left, front right, back left, back right. */
  public ModuleType module(int corner) {
    return corners.get(corner).module();
  }

  /**
   * Where each wheel touches the floor, from the robot's center: front left, front right, back
   * left, back right.
   */
  public Translation2d[] moduleTranslations() {
    Translation2d[] translations = new Translation2d[corners.size()];
    for (int i = 0; i < translations.length; i++) {
      translations[i] = corners.get(i).position();
    }
    return translations;
  }

  /**
   * Top speed at the motors' nominal voltage, with no load: its slowest module's, which the rest
   * can only wait for. The real robot is slower.
   */
  public LinearVelocity freeSpeed() {
    LinearVelocity slowest = corners.get(0).module().freeSpeed();
    for (Corner corner : corners) {
      if (corner.module().freeSpeed().lt(slowest)) {
        slowest = corner.module().freeSpeed();
      }
    }
    return slowest;
  }

  /** The treads' grip on carpet: its least, which slips first. */
  public double wheelCoefficientOfFriction() {
    double least = Double.POSITIVE_INFINITY;
    for (Corner corner : corners) {
      least = Math.min(least, corner.module().wheelCoefficientOfFriction());
    }
    return least;
  }

  /**
   * Why the wheels don't fit this body's frame, or null when they do: each wheel's center inside
   * the outside of the frame.
   */
  public @Nullable String whyWheelsDontFit(Body body) {
    double halfLength = body.frameLength().in(Meters) / 2;
    double halfWidth = body.frameWidth().in(Meters) / 2;
    List<String> outside = new ArrayList<>();
    for (int i = 0; i < corners.size(); i++) {
      Translation2d at = corners.get(i).position();
      if (!(Math.abs(at.getX()) < halfLength && Math.abs(at.getY()) < halfWidth)) {
        outside.add(LABELS.get(i) + "'s, at " + at);
      }
    }
    return outside.isEmpty()
        ? null
        : "wheels outside the frame ("
            + shown(body.frameLength())
            + " by "
            + shown(body.frameWidth())
            + "): "
            + String.join(", ", outside);
  }

  private static @Nullable String notFourCorners(List<Corner> corners) {
    return corners.size() == 4
        ? null
        : "needs four, front left, front right, back left and back right, not " + corners.size();
  }

  /** The corners not in their own quarter of the robot, or null if each is. */
  private static @Nullable String misplaced(List<Corner> corners) {
    double[][] signs = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    List<String> misplaced = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      Translation2d at = corners.get(i).position();
      if (!(at.getX() * signs[i][0] > 0 && at.getY() * signs[i][1] > 0)) {
        misplaced.add(
            LABELS.get(i)
                + " must be "
                + (signs[i][0] > 0 ? "ahead of" : "behind")
                + " the center and "
                + (signs[i][1] > 0 ? "left" : "right")
                + " of it, not at "
                + at);
      }
    }
    return misplaced.isEmpty() ? null : String.join("; ", misplaced);
  }
}
