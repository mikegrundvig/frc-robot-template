package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.given;
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
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.MomentOfInertia;

/**
 * A swerve drivetrain, as built: a module at each corner, each of its own type and where its wheel
 * touches the floor; and, in the same block, how it's driven and wired: {@code
 * DrivetrainGeometry.uniform(MODULES, forward, left).wired(WIRING).control(DRIVING)}. How it's
 * driven isn't part of what it is: two drivetrains the same but for their control or wiring are
 * one, re-tuned or re-wired.
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
 * @param wiring where its devices are on the CAN bus, and how they turn; none runs it in the
 *     simulator alone. Its controllers must drive its modules' motors, and SPARKs are on
 *     Systemcore's own CAN
 * @param control how it's driven: its limits and gains, tuned while the robot runs; a robot needs
 *     it
 */
@ConfigRecord
public record DrivetrainGeometry(
    List<Corner> corners, Optional<SwerveWiring> wiring, Optional<DrivetrainControl> control) {
  /** The corners' names, in their order: front left, front right, back left, back right. */
  public static final List<String> LABELS =
      List.of("front-left", "front-right", "back-left", "back-right");

  public DrivetrainGeometry {
    corners = List.copyOf(corners);
    List<Corner> all = corners;
    Optional<SwerveWiring> wired = wiring;
    Checks.on(DrivetrainGeometry.class)
        .field("corners", corners, DrivetrainGeometry::notFourCorners)
        .check("corners", () -> all.size() == 4 ? misplaced(all) : null)
        .check("wiring", () -> wired.map(each -> unwirable(all, each)).orElse(null))
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

  /** The same drivetrain, wired: its modules' and its gyro's devices. */
  public DrivetrainGeometry wired(SwerveWiring wiring) {
    return DrivetrainGeometryBuilder.builder(this).wiring(wiring).build();
  }

  /**
   * The same drivetrain, its modules' absolute encoders offset otherwise, front left, front right,
   * back left, back right, as Phoenix Tuner X's swerve generator gives them, the rest of its wiring
   * kept: a practice robot's, {@code DRIVETRAIN.encoderOffsets(Rotations.of(0.12),
   * Rotations.of(-0.3), Rotations.of(0.41), Rotations.of(0.07))}. A CANcoder moved to another ID in
   * the competition robot's wiring reaches it.
   *
   * @throws IllegalArgumentException if it isn't wired
   */
  public DrivetrainGeometry encoderOffsets(
      Angle frontLeft, Angle frontRight, Angle backLeft, Angle backRight) {
    SwerveWiring was =
        wiring.orElseThrow(
            () ->
                new IllegalArgumentException(
                    "the drivetrain isn't wired, so its modules have no offsets: wire it,"
                        + " .wired(...)"));
    List<Angle> offsets = List.of(frontLeft, frontRight, backLeft, backRight);
    List<SwerveWiring.ModuleWiring> modules = new ArrayList<>();
    for (int i = 0; i < offsets.size(); i++) {
      modules.add(
          SwerveWiringModuleWiringBuilder.builder(was.modules().get(i))
              .encoderOffset(offsets.get(i))
              .build());
    }
    return wired(SwerveWiringBuilder.builder(was).modules(modules).build());
  }

  /** The same drivetrain, driven so: its limits and gains. */
  public DrivetrainGeometry control(DrivetrainControl control) {
    return DrivetrainGeometryBuilder.builder(this).control(control).build();
  }

  /**
   * Equal to another drivetrain the same in everything but how it's driven and wired, and what only
   * the simulator reads: its modules' drive and steering inertias. A practice robot's drivetrain,
   * its Tuner X simulation inertias its own, is still the competition robot's.
   */
  @Override
  public boolean equals(@Nullable Object other) {
    return other instanceof DrivetrainGeometry that
        && robotReads(corners).equals(robotReads(that.corners));
  }

  @Override
  public int hashCode() {
    return robotReads(corners).hashCode();
  }

  /** Its corners as the robot reads them: each module without what only the simulator reads. */
  private static List<Corner> robotReads(List<Corner> corners) {
    return corners.stream()
        .map(
            corner ->
                DrivetrainGeometryCornerBuilder.builder(corner)
                    .module(
                        DrivetrainGeometryModuleTypeBuilder.builder(corner.module())
                            .driveInertia(Optional.empty())
                            .steerInertia(Optional.empty())
                            .build())
                    .build())
        .toList();
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
   * <p>The tread's grip and the inertias are optional, each with a default until it's measured. The
   * robot plans and judges by the grip, so it says at startup that it's the default; only the
   * simulator reads the inertias, so it says so when it simulates ({@code
   * RobotDefinition.missing}).
   *
   * @param driveMotor the motor that drives the wheel
   * @param steerMotor the motor that turns the module
   * @param wheelRadius measured on the tread
   * @param driveGearing between the drive motor and the wheel: motor rotations per wheel rotation
   * @param steerGearing between the steering motor and the module: motor rotations per full turn of
   *     the module
   * @param wheelCoefficientOfFriction the tread's grip on carpet: about 1.0 when worn, 1.3 new;
   *     none measured, {@link #DEFAULT_WHEEL_COEFFICIENT_OF_FRICTION}
   * @param driveInertia the drive's own moving parts (the wheel with its tread, its gears, and the
   *     motor's rotor through them), as inertia at the wheel. The robot's mass the wheel moves
   *     comes on top of it, from the body. None measured, {@link #DEFAULT_DRIVE_INERTIA}
   * @param steerInertia the module turning about its steering axis, with the steering motor's rotor
   *     through its gearing: what the steering motor turns. None measured, {@link
   *     #DEFAULT_STEER_INERTIA}
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
      Optional<Double> wheelCoefficientOfFriction,
      Optional<MomentOfInertia> driveInertia,
      Optional<MomentOfInertia> steerInertia,
      double couplingRatio) {
    /**
     * The tread's grip until it's measured: PathPlanner's default, which teams already plan with.
     * Tread on carpet grips about 1.0 worn and 1.3 new; a pull test measures it.
     */
    public static final double DEFAULT_WHEEL_COEFFICIENT_OF_FRICTION = 1.2;

    /**
     * The drive's own moving parts until they're measured: an estimate for a 4 in wheel, about
     * 0.0004 kg m², and a Kraken X60's rotor through 6.75:1, about 0.0006 (the rotor's inertia
     * isn't published). It's about 2% of what a wheel's share of a robot's mass puts on it, so a
     * rough one costs little.
     */
    public static final MomentOfInertia DEFAULT_DRIVE_INERTIA = KilogramSquareMeters.of(0.001);

    /**
     * The module's steering until it's measured: Phoenix Tuner X's generated simulation value, as
     * AdvantageKit's swerve templates use it.
     */
    public static final MomentOfInertia DEFAULT_STEER_INERTIA = KilogramSquareMeters.of(0.004);

    public ModuleType {
      Checks.on(ModuleType.class)
          .field("wheelRadius", wheelRadius, positive())
          .field("wheelCoefficientOfFriction", wheelCoefficientOfFriction, given(positive()))
          .field("driveInertia", driveInertia, given(positive()))
          .field("steerInertia", steerInertia, given(positive()))
          .field("couplingRatio", couplingRatio, nonNegative())
          .orThrow();
    }

    /** Starts describing a type of module, one named value at a time. */
    public static DrivetrainGeometryModuleTypeBuilder.DriveMotorStage builder() {
      return DrivetrainGeometryModuleTypeBuilder.builder();
    }

    /** The tread's grip on carpet: as measured, or until it is, PathPlanner's default. */
    public double wheelCoefficientOfFrictionOrDefault() {
      return wheelCoefficientOfFriction.orElse(DEFAULT_WHEEL_COEFFICIENT_OF_FRICTION);
    }

    /** The drive's own moving parts, as inertia at the wheel: as measured, or the estimate. */
    public MomentOfInertia driveInertiaOrDefault() {
      return driveInertia.orElse(DEFAULT_DRIVE_INERTIA);
    }

    /** The module turning about its steering axis: as measured, or Tuner X's simulation value. */
    public MomentOfInertia steerInertiaOrDefault() {
      return steerInertia.orElse(DEFAULT_STEER_INERTIA);
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
              .link(disc(label + " module", steerInertiaOrDefault(), radius, Vector3.UP))
              .build();
      Joint.Revolute wheel =
          Joint.Revolute.builder()
              .label(label + " wheel")
              .parent(steer)
              .origin(new Transform3d(0, 0, radius, Rotation3d.ZERO))
              .axis(new Vector3(0, 1, 0))
              .link(disc(label + " wheel", driveInertiaOrDefault(), radius, new Vector3(0, 1, 0)))
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
      terms.add(Transmission.Term.builder().joint(wheel).ratio(driveGearing.ratio()).build());
      if (couplingRatio != 0) {
        terms.add(Transmission.Term.builder().joint(steer).ratio(couplingRatio).build());
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

  /**
   * The treads' grip on carpet: its least, which slips first, each as measured or PathPlanner's
   * default.
   */
  public double wheelCoefficientOfFriction() {
    double least = Double.POSITIVE_INFINITY;
    for (Corner corner : corners) {
      least = Math.min(least, corner.module().wheelCoefficientOfFrictionOrDefault());
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

  /**
   * Why a wiring can't drive these modules, or null when it can: a controller that doesn't drive
   * its motor (a NEO on a TalonFX, say), as a joint's block refuses, or SPARKs on a CANivore, where
   * a SPARK can't be.
   */
  private static @Nullable String unwirable(List<Corner> corners, SwerveWiring wiring) {
    Set<String> problems = new LinkedHashSet<>();
    for (Corner corner : corners) {
      ModuleType module = corner.module();
      String drive = onController("drive", module.driveMotor(), wiring.driveController());
      String steering = onController("steering", module.steerMotor(), wiring.steerController());
      if (drive != null) {
        problems.add(drive);
      }
      if (steering != null) {
        problems.add(steering);
      }
    }
    boolean sparks =
        wiring.driveController() != MechanismWiring.Controller.TALON_FX
            || wiring.steerController() != MechanismWiring.Controller.TALON_FX;
    if (sparks && !(wiring.canBus() instanceof CanBus.Port)) {
      problems.add("its SPARKs are on a CANivore: a SPARK is on Systemcore's own CAN");
    }
    return problems.isEmpty() ? null : String.join("; ", problems);
  }

  /** Why a module's motor can't be on a controller, saying which can; null when it can. */
  private static @Nullable String onController(
      String which, Motor motor, MechanismWiring.Controller controller) {
    if (controller.drives(motor)) {
      return null;
    }
    List<String> could = new ArrayList<>();
    for (MechanismWiring.Controller each : MechanismWiring.Controller.values()) {
      if (each.drives(motor)) {
        could.add(each.shown());
      }
    }
    return "its "
        + which
        + "'s "
        + motor.name()
        + " is on "
        + controller.shown()
        + ", which doesn't drive one: "
        + String.join(" or ", could)
        + " does";
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
