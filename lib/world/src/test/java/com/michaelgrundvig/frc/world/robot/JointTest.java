package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.PITCH_UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.definition;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.encoder;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.gearing;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.linear;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.link;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.point;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.prismatic;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.revolute;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Joints, couplings, drives and sensors: each part's own checks, and the definition's across them.
 * Every reference is the part itself; what a reference can still get wrong, a part left off the
 * definition's lists, is caught there.
 */
class JointTest {
  private final Joint.Revolute arm =
      revolute(
          "arm",
          Part.CHASSIS,
          point(0.1, 0, 0.5),
          PITCH_UP,
          Optional.of(Range.degrees(-10, 110)),
          0,
          link("arm", 4, point(0.3, 0, 0), point(0.6, 0.05, 0.05)));

  /**
   * Every message shows a position on a joint one way: degrees for one that turns, meters for one
   * that slides, to four significant figures, no finer than a hundredth of a degree or a tenth of a
   * millimeter, so a position measured on the robot reads as where it is, not its noise.
   */
  @Test
  void aJointShowsAPositionInItsUnits() {
    Joint.Prismatic lift =
        prismatic(
            "lift",
            Part.CHASSIS,
            point(0, 0, 0.1),
            UP,
            Range.meters(0, 1.2),
            link("lift", 1, point(0, 0, 0), point(0.1, 0.1, 0.1)));

    assertThat(arm.shown(Math.toRadians(130))).isEqualTo("130°");
    assertThat(arm.shown(Math.toRadians(40.27312))).isEqualTo("40.27°");
    assertThat(arm.shown(Math.toRadians(-5.7296))).isEqualTo("-5.73°");
    assertThat(arm.shown(-1e-9)).as("noise about zero").isEqualTo("0°");
    assertThat(lift.shown(Inches.of(32).in(Meters))).isEqualTo("0.8128 m");
    assertThat(lift.shown(0.40000012)).isEqualTo("0.4 m");
  }

  @Test
  void aJointIsOne() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                revolute(
                    " ",
                    Part.CHASSIS,
                    point(0, 0, 0),
                    new Vector3(0, 0, 0),
                    Optional.empty(),
                    0,
                    arm.link()))
        .withMessageContaining("Revolute[ ].label")
        .withMessageContaining("an axis can't be zero");
    Joint.Revolute restingPastItsStops =
        revolute(
            "arm",
            Part.CHASSIS,
            point(0, 0, 0),
            UP,
            Optional.of(Range.degrees(0, 90)),
            2,
            arm.link());
    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(List.of(restingPastItsStops), List.of(), List.of(), List.of()))
        .withMessageContaining(
            "RobotDefinition[test].joints: arm rests at 114.6°, outside its hard stops, 0° to 90°:"
                + " it can't start there");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Joint.Prismatic.builder()
                    .label("lift")
                    .parent(Part.CHASSIS)
                    .origin(new Transform3d(Double.NaN, 0, 0, Rotation3d.ZERO))
                    .axis(UP)
                    .link(arm.link())
                    .build())
        .withMessageStartingWith("Prismatic[lift].origin");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Range.meters(1, 1))
        .withMessage("a range's stops are apart, and 1 m to 1 m is one place: give them both");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Range.degrees(135, 0))
        .withMessage(
            "a range goes from its lower stop to its upper, and 135° to 0° is the other way round:"
                + " give the lower first, Range.degrees(0, 135)");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Range.meters(Double.NaN, 1))
        .withMessage("a range's stops are numbers, not NaN m to 1 m");
    assertThat(Range.degrees(-10, 110).width()).isCloseTo(Math.toRadians(120), within(1e-12));
    assertThat(Part.CHASSIS.label()).isEqualTo("chassis");
  }

  /** A joint's motion: turning about its axis, or sliding along it however long the axis is. */
  @Test
  void aJointMovesAlongItsAxis() {
    Joint.Prismatic lift =
        prismatic(
            "lift",
            Part.CHASSIS,
            point(0, 0, 0),
            new Vector3(0, 0, 2),
            Range.meters(0, 1),
            arm.link());

    assertThat(lift.motion(0.5).getTranslation()).isEqualTo(point(0, 0, 0.5));
    assertThat(arm.motion(Math.PI / 2).getRotation().getY()).isCloseTo(-Math.PI / 2, within(1e-12));
  }

  @Test
  void aCouplingIsAFunctionOfItsLeader() {
    Joint.Revolute other =
        revolute(
            "other",
            Part.CHASSIS,
            point(0, 0, 0.3),
            PITCH_UP,
            Optional.empty(),
            0,
            link("other", 1, point(0.1, 0, 0), point(0.1, 0.1, 0.1)));
    Coupling.Curve curve =
        Coupling.Curve.builder()
            .leader(arm)
            .follower(other)
            .table(
                List.of(new Translation2d(0, 0), new Translation2d(1, 2), new Translation2d(2, 2)))
            .build();

    assertThat(curve.followerAt(-1)).as("before the table").isZero();
    assertThat(curve.followerAt(0.5)).isCloseTo(1, within(1e-12));
    assertThat(curve.followerAt(3)).as("past it").isEqualTo(2);
    assertThat(curve.slopeAt(0.5)).isCloseTo(2, within(1e-12));
    assertThat(curve.slopeAt(1.5)).isZero();
    assertThat(curve.slopeAt(5)).isZero();
    assertThat(linear(arm, other, 0.5).slopeAt(9)).isEqualTo(0.5);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> linear(arm, arm, 1))
        .withMessageContaining("a joint can't follow itself");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Coupling.Curve.builder()
                    .leader(arm)
                    .follower(other)
                    .table(List.of(new Translation2d(1, 0), new Translation2d(1, 2)))
                    .build())
        .withMessageContaining("must rise from point to point");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Coupling.Curve.builder()
                    .leader(arm)
                    .follower(other)
                    .table(List.of(new Translation2d(1, 0)))
                    .build())
        .withMessageContaining("two points or more");
  }

  /** Motors turn a shaft, so they can't push along a line; a cylinder can only. */
  @Test
  void aDrivesActuatorMustSuitItsTransmission() {
    Transmission.Between line =
        Transmission.Between.builder()
            .a(Part.CHASSIS)
            .onA(point(0, 0, 0.2))
            .b(arm)
            .onB(point(0.2, 0, 0))
            .build();
    Transmission.Geared geared =
        Transmission.Geared.builder().joint(arm).gearing(gearing(10, 1)).build();

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Drive.builder()
                    .label("arm")
                    .actuator(Actuator.Motors.builder().motor(Motor.NEO).count(1).build())
                    .transmission(line)
                    .build())
        .withMessageContaining("motors turn a shaft");
    assertThat(line.efficiency()).isEqualTo(1);
    assertThat(line.joints()).containsExactly(arm);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Transmission.Between.builder()
                    .a(arm)
                    .onA(point(0, 0, 0))
                    .b(arm)
                    .onB(point(0.2, 0, 0))
                    .build())
        .withMessageContaining("two different parts");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Transmission.Combined.builder().terms(List.of()).efficiency(1).build())
        .withMessageContaining("needs a joint");
    Transmission.Term term = Transmission.Term.builder().joint(arm).ratio(3).build();
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> Transmission.Combined.builder().terms(List.of(term, term)).efficiency(1).build())
        .withMessageContaining("name each joint once, not arm");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Transmission.Term.builder().joint(arm).ratio(0).build())
        .withMessageContaining("a number other than 0");
    Transmission.Combined combined =
        Transmission.Combined.builder().terms(List.of(term)).efficiency(0.8).build();
    assertThat(combined.efficiency()).isEqualTo(0.8);
    assertThat(combined.shaftRadiansPer(arm)).isEqualTo(3);
    assertThat(geared.shaftRadiansPer(arm)).isEqualTo(10);
    assertThat(geared.joints()).containsExactly(arm);
  }

  @Test
  void sensorsAreWhereTheirJointsAre() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Sensor.Switch.limit("stow", arm, Sensor.Switch.Limit.End.LOWER, 3))
        .withMessageContaining("must be inside arm's range");
    assertThat(Sensor.Switch.limit("stow", arm, Sensor.Switch.Limit.End.LOWER, 0).label())
        .isEqualTo("stow");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> encoder("arm-abs", arm, 0, true))
        .withMessageContaining("a number other than 0");
    // An absolute encoder's resolution is optional: only a pair decoded by their difference reads
    // it, and the definition asks for it there. One given counts something.
    assertThat(encoder("arm-abs", arm, 1, true, Optional.empty()).countsPerTurn()).isEmpty();
    assertThatIllegalArgumentException()
        .isThrownBy(() -> encoder("arm-abs", arm, 1, true, Optional.of(0)))
        .withMessageContaining("Encoder[arm-abs].countsPerTurn: 0 must be positive");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> encoder("arm-abs", arm, 1, false, Optional.of(-1)))
        .withMessageContaining("must be positive");
  }

  /** What a reference can get wrong: a part the definition doesn't list. */
  @Test
  void everythingADriveOrSensorTurnsIsOneOfTheRobotsJoints() {
    Drive drive = MechanismFixtures.motors(arm, Motor.KRAKEN_X60, 1, gearing(50, 1));
    Sensor.Encoder pivot = encoder("arm-abs", arm, 1, true);

    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(List.of(), List.of(), List.of(drive), List.of(pivot)))
        .withMessageContaining("drive arm's joint, arm, isn't one of the robot's joints: list it")
        .withMessageContaining("sensor arm-abs' joint, arm, isn't one of the robot's joints");
    Joint.Revolute wrist =
        revolute(
            "wrist",
            arm,
            point(0.6, 0, 0),
            PITCH_UP,
            Optional.empty(),
            0,
            link("claw", 1, point(0.1, 0, 0), point(0.1, 0.1, 0.1)));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(List.of(wrist), List.of(), List.of(), List.of()))
        .withMessageContaining("joint wrist's parent, arm, isn't one of the robot's joints");
  }

  @Test
  void labelsAreUniqueForLogsAndWriters() {
    Joint.Revolute twin =
        revolute(
            "arm",
            Part.CHASSIS,
            point(-0.1, 0, 0.5),
            PITCH_UP,
            Optional.empty(),
            0,
            link("other arm", 1, point(0.1, 0, 0), point(0.1, 0.1, 0.1)));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                definition(
                    List.of(arm, twin),
                    List.of(),
                    List.of(
                        MechanismFixtures.motors(arm, Motor.NEO, 1, gearing(10, 1)),
                        MechanismFixtures.motors(twin, Motor.NEO, 1, gearing(10, 1))),
                    List.of()))
        .withMessageContaining("two joints are labeled arm")
        .withMessageContaining("two drives are labeled arm");
  }

  @Test
  void aJointFollowsOneOtherAtMostAndNeverItself() {
    Joint.Prismatic carriage =
        prismatic(
            "carriage",
            Part.CHASSIS,
            point(0, 0, 0.1),
            UP,
            Range.meters(0, 1),
            link("carriage", 2, point(0, 0, 0.05), point(0.2, 0.2, 0.1)));
    Joint.Prismatic stage =
        prismatic(
            "stage",
            Part.CHASSIS,
            point(0, 0, 0.1),
            UP,
            Range.meters(0, 1),
            link("stage", 1, point(0, 0, 0.2), point(0.2, 0.05, 0.4)));
    Joint.Prismatic top =
        prismatic(
            "top",
            Part.CHASSIS,
            point(0, 0, 0.1),
            UP,
            Range.meters(0, 1),
            link("top", 1, point(0, 0, 0.2), point(0.2, 0.05, 0.4)));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                definition(
                    List.of(carriage, stage, top),
                    List.of(linear(carriage, stage, 0.5), linear(top, stage, 0.5)),
                    List.of(),
                    List.of()))
        .withMessageContaining("stage follows two joints");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                definition(
                    List.of(carriage, stage, top),
                    List.of(
                        linear(carriage, stage, 0.5),
                        linear(stage, top, 0.5),
                        linear(top, carriage, 0.5)),
                    List.of(),
                    List.of()))
        .withMessageContaining("follows itself, through its couplings");
    Coupling offset =
        Coupling.Linear.builder().leader(carriage).follower(stage).ratio(0.5).offset(0.2).build();
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> definition(List.of(carriage, stage), List.of(offset), List.of(), List.of()))
        .withMessageContaining("stage's rest (0.000) isn't where its coupling puts it");
  }

  /**
   * The body is the robot as weighed, its links a part of it: links weighing all of it together
   * leave nothing for the chassis, which carries the drivetrain, the electronics and the battery.
   */
  @Test
  void theLinksMustLeaveSomeOfTheRobotForTheChassis() {
    Joint.Revolute anvil =
        revolute(
            "anvil",
            Part.CHASSIS,
            point(0, 0, 0.3),
            PITCH_UP,
            Optional.empty(),
            0,
            link("anvil", 45, point(0.1, 0, 0), point(0.2, 0.2, 0.2)));
    Joint.Revolute hammer =
        revolute(
            "hammer",
            Part.CHASSIS,
            point(0, 0, 0.3),
            PITCH_UP,
            Optional.empty(),
            0,
            link("hammer", 15, point(0.1, 0, 0), point(0.2, 0.2, 0.2)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(List.of(anvil, hammer), List.of(), List.of(), List.of()))
        .withMessageContaining(
            "its links weigh 60.00 kg together, as much as the robot's 60.00 kg as weighed:"
                + " nothing is left for the chassis");
  }

  /** What's left once the links are off must still turn as a body does. */
  @Test
  void theChassisLeftOnceTheLinksAreOffMustBeABody() {
    Joint.Revolute boom =
        revolute(
            "boom",
            Part.CHASSIS,
            point(0, 0, 0.3),
            PITCH_UP,
            Optional.empty(),
            0,
            link("boom", 50, point(0.3, 0, 0), point(0.2, 0.2, 0.2)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(List.of(boom), List.of(), List.of(), List.of()))
        .withMessageContaining("the chassis, the body less its links at rest, isn't a body");
  }

  /** A camera on a link sits on it, and inside the robot when its joint is stowed. */
  @Test
  void aCameraOnAJointSitsOnItsLink() {
    CameraConfig onArm =
        CameraConfigBuilder.builder(ConfigFixtures.camera("arm", point(0.2, 0, 0.03)))
            .joint((Joint) arm)
            .build();
    CameraConfig floating =
        CameraConfigBuilder.builder(ConfigFixtures.camera("arm", point(0.2, 0, 0.4)))
            .joint((Joint) arm)
            .build();
    Drive drive = MechanismFixtures.motors(arm, Motor.KRAKEN_X60, 1, gearing(50, 1));

    RobotDefinition robot =
        definition(List.of(arm), List.of(), List.of(drive), List.of(), List.of(onArm));
    assertThat(robot.math().cameraPose(onArm, Map.of()).getX()).isCloseTo(0.3, within(1e-9));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> definition(List.of(arm), List.of(), List.of(drive), List.of(), List.of(floating)))
        .withMessageContaining("m off arm, the link it rides on");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(List.of(), List.of(), List.of(), List.of(), List.of(onArm)))
        .withMessageContaining("camera arm's joint, arm, isn't one of the robot's joints");
  }

  /** An absolute encoder's range must fit one turn somewhere Phoenix can wrap it. */
  @Test
  void anAbsoluteEncodersRangeMustFitWherePhoenixCanWrapIt() {
    Joint.Revolute hood =
        revolute(
            "hood",
            Part.CHASSIS,
            point(0, 0, 0.5),
            PITCH_UP,
            Optional.of(
                Range.builder()
                    .kind(Position.Kind.ANGLE)
                    .lower(1.2 * Math.PI)
                    .upper(2.6 * Math.PI)
                    .build()),
            1.2 * Math.PI,
            link("hood", 1, point(0.1, 0, 0), point(0.1, 0.1, 0.1)));
    Sensor.Encoder reads = encoder("hood-abs", hood, 1, true);

    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(List.of(hood), List.of(), List.of(), List.of(reads)))
        .withMessageContaining(
            "hood's absolute encoders read it from 0° to 360°, and its range, 216° to 468°,"
                + " crosses where they wrap");
    RobotMath math = definition(List.of(hood), List.of(), List.of(), List.of()).math();
    assertThatIllegalArgumentException()
        .isThrownBy(() -> math.discontinuityPoint(encoder("hood-rotor", hood, 40, false)))
        .withMessageContaining("isn't absolute");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> math.absoluteSpan(List.of(reads, encoder("hood-rotor", hood, 40, false))))
        .withMessageContaining("hood-rotor: a span is of absolute encoders on one joint");
    assertThatIllegalArgumentException().isThrownBy(() -> math.absoluteSpan(List.of()));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                math.rotorToSensorRatio(
                    MechanismFixtures.motors(arm, Motor.NEO, 1, gearing(5, 1)), reads))
        .withMessageContaining("doesn't turn hood");
  }

  /**
   * An absolute encoder must wrap a fortieth of its turn clear of its joint's range, not just
   * outside it: a range that fills nearly a whole turn, or one Phoenix's point, from 0 to 1, can't
   * center on, leaves a reading a hair from the wrap at its end.
   */
  @Test
  void anAbsoluteEncoderWrapsAFortiethOfATurnClearOfTheRange() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> encoded(-360, -180))
        .withMessageContaining(
            "hood's absolute encoders read it from -360° to 0°, and its range, -360° to -180°,"
                + " comes within 0° of where they wrap");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> encoded(0, 350))
        .withMessageContaining("comes within 5° of where they wrap, a fortieth of their turn");
    Sensor.Encoder clear = (Sensor.Encoder) encoded(0, 340).sensors().get(0);
    assertThat(RobotMath.wrapMargin(clear, 0.5 + 170.0 / 360))
        .isCloseTo(10.0 / 360, within(1e-12))
        .isGreaterThan(RobotMath.WRAP_MARGIN);
    assertThat(RobotMath.wrapMargin(clear, 1.0)).as("on the range's edge").isZero();
  }

  /**
   * A shaft slides a joint through a drum, a sprocket or a pinion, so a combined transmission's
   * prismatic term names its pitch radius, as a drum does, and its ratio is per turn of it; a
   * revolute term names none.
   */
  @Test
  void aCombinedTransmissionsSlideNamesItsPinion() {
    Joint.Prismatic lift =
        prismatic("lift", Part.CHASSIS, point(0, 0, 0), UP, Range.meters(0, 1), arm.link());
    Transmission.Term bare = Transmission.Term.builder().joint(lift).ratio(400).build();
    Transmission.Term pinion =
        Transmission.Term.builder().joint(lift).ratio(5).pitchRadius(Inches.of(0.5)).build();

    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> Transmission.Combined.builder().terms(List.of(bare)).efficiency(1).build())
        .withMessageContaining(
            "a shaft slides lift through a drum, a sprocket or a pinion: name its pitch radius");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Transmission.Term.builder().joint(arm).ratio(5).pitchRadius(Inches.of(0.5)).build())
        .withMessageContaining("arm turns: its ratio is per turn, through no drum");
    assertThat(bare.perTurnOrMeter()).as("read straight off the slide").isEqualTo(400);
    assertThat(pinion.perTurnOrMeter())
        .isCloseTo(5 / (2 * Math.PI * Inches.of(0.5).in(Meters)), within(1e-9));
    Transmission.Combined combined =
        Transmission.Combined.builder().terms(List.of(pinion)).efficiency(1).build();
    assertThat(combined.shaftRadiansPer(lift))
        .as("as a drum's")
        .isCloseTo(
            Transmission.Drum.builder()
                .joint(lift)
                .gearing(gearing(5, 1))
                .pitchRadius(Inches.of(0.5))
                .build()
                .shaftRadiansPer(lift),
            within(1e-9));
  }

  /** A hood from one angle to another, its absolute encoder on its shaft. */
  private static RobotDefinition encoded(double lowerDegrees, double upperDegrees) {
    Joint.Revolute hood =
        revolute(
            "hood",
            Part.CHASSIS,
            point(0, 0, 0.5),
            PITCH_UP,
            Optional.of(Range.degrees(lowerDegrees, upperDegrees)),
            Math.toRadians(lowerDegrees),
            link("hood", 1, point(0.1, 0, 0), point(0.1, 0.1, 0.1)));
    return definition(
        List.of(hood), List.of(), List.of(), List.of(encoder("hood-abs", hood, 1, true)));
  }
}
