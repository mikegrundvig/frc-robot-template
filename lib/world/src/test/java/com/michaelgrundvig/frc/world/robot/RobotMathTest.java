package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Finding.Scope.ROBOT;
import static com.michaelgrundvig.frc.world.config.Finding.Scope.SIMULATOR;
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
import static com.michaelgrundvig.frc.world.robot.RobotMath.GRAVITY;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatIllegalStateException;
import static org.assertj.core.api.Assertions.tuple;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;

/**
 * Every mechanism in the audit's table ("How FRC mechanisms read"), described and checked against
 * physics worked by hand: kG from m g r through the gearing, a turret's span from its gears' teeth,
 * a cascade's stages from their ratios.
 */
class RobotMathTest {
  private static final DCMotor KRAKEN = Motor.KRAKEN_X60.model();
  private static final DCMotor KRAKEN_X44 = Motor.KRAKEN_X44.model();
  private static final double KRAKEN_ROTOR =
      Motor.KRAKEN_X60.rotorInertia().in(KilogramSquareMeters);

  /**
   * An arm, two Krakens at 60:1 losing a tenth, a through-bore encoder on the pivot, and a linear
   * servo under it: 4 kg, its center 0.3 m out along a 0.6 m bar, pivoting 0.5 m up, from -10° to
   * 110°, stowed at -10°.
   */
  @Nested
  class ArmWithAServo {
    final Joint.Revolute arm =
        revolute(
            "arm",
            Part.CHASSIS,
            point(0.1, 0, 0.5),
            PITCH_UP,
            Optional.of(Range.degrees(-10, 110)),
            Math.toRadians(-10),
            link("arm", 4, point(0.3, 0, 0), point(0.6, 0.05, 0.05)));
    final Drive motors = MechanismFixtures.motors(arm, Motor.KRAKEN_X60, 2, gearing(60, 0.9));
    final Sensor.Encoder pivot = encoder("arm-abs", arm, 1, true);
    final Actuator.Servo linear =
        Actuator.Servo.builder().from(0.25).to(0.4).force(50).speed(0.03).build();
    final Drive servo =
        Drive.builder()
            .label("arm-servo")
            .actuator(linear)
            .transmission(
                Transmission.Between.builder()
                    .a(Part.CHASSIS)
                    .onA(point(0.1, 0, 0.2))
                    .b(arm)
                    .onB(point(0.15, 0, 0))
                    .build())
            .build();
    final RobotMath math =
        definition(List.of(arm), List.of(), List.of(motors, servo), List.of(pivot)).math();

    @Test
    void gravityIsItsWeightTimesItsLeverFromThePivot() {
      double weight = 4 * GRAVITY;

      assertThat(math.gravityTorque(arm, Map.of(arm, 0.0))).isCloseTo(weight * 0.3, within(1e-9));
      assertThat(math.gravityTorque(arm, Map.of(arm, Math.toRadians(60))))
          .isCloseTo(weight * 0.3 / 2, within(1e-9));
      assertThat(math.gravityTorque(arm, Map.of(arm, Math.PI / 2))).isCloseTo(0, within(1e-9));
      assertThat(math.gravityOffset(arm)).as("its zero is horizontal").isCloseTo(0, within(1e-12));
    }

    /** kG = m g r R / (n Kt G η): the audit's formula, from WPILib's own motor model. */
    @Test
    void itsIdealGainsComeFromItsMotorsGearingAndMass() {
      RobotMath.Gains gains = math.idealGains(arm);

      double kG = 4 * GRAVITY * 0.3 * KRAKEN.R / (2 * KRAKEN.Kt * 60 * 0.9);
      double kV = 60 * 12 / KRAKEN.freeSpeed;
      // The bar about its own center, then 0.3 m out, then two rotors through 60:1.
      double inertia = 4 * (0.36 + 0.0025) / 12 + 4 * 0.09 + 2 * KRAKEN_ROTOR * 3600;
      double kA = inertia * KRAKEN.R / (2 * KRAKEN.Kt * 60 * 0.9);
      assertThat(gains.kG()).isCloseTo(kG, within(1e-9));
      assertThat(gains.kV()).isCloseTo(kV, within(1e-9));
      assertThat(gains.kA()).isCloseTo(kA, within(1e-9));
      assertThat(math.inertia(arm, Map.of())).isCloseTo(inertia, within(1e-9));
      assertThat(math.freeSpeed(arm)).isCloseTo(KRAKEN.freeSpeed / 60, within(1e-9));
    }

    @Test
    void phoenixsRatiosComeFromWhereTheEncoderSits() {
      assertThat(math.sensorToMechanismRatio(pivot)).isEqualTo(1);
      assertThat(math.rotorToSensorRatio(motors, pivot)).isCloseTo(60, within(1e-12));
      // Its range, -10° to 110°, is -0.028 to 0.306 of a turn: centered, it wraps at 0.639.
      assertThat(math.discontinuityPoint(pivot))
          .isCloseTo((-10.0 + 110.0) / 2 / 360 + 0.5, within(1e-12));
      assertThat(math.absoluteSpan(List.of(pivot))).isCloseTo(2 * Math.PI, within(1e-12));
    }

    /** The servo's torque at an angle: its push along the line, times the line's lever arm. */
    @Test
    void theServoTurnsItByItsLeverArm() {
      Map<Joint, Double> level = Map.of(arm, 0.0);
      // Pivot (0.1, 0, 0.5), the servo's base 0.3 m below it and its rod end 0.15 m out the arm.
      double length = Math.hypot(0.15, 0.3);
      double push = linear.forceOrTorque(1, length, 0);
      double lever = 0.15 * 0.3 / length; // the line's distance from the pivot

      assertThat(math.coordinate(servo, level)).isCloseTo(length, within(1e-12));
      assertThat(math.leverage(servo, arm, level)).isCloseTo(lever, within(1e-9));
      assertThat(push).as("told to extend, it pushes").isPositive();
      assertThat(math.torque(servo, arm, level, 1)).isCloseTo(push * lever, within(1e-9));
      // The lever arm by finite differences, at another angle.
      double h = 1e-6;
      double at = Math.toRadians(40);
      double ahead = math.coordinate(servo, Map.of(arm, at + h));
      double behind = math.coordinate(servo, Map.of(arm, at - h));
      assertThat(math.leverage(servo, arm, Map.of(arm, at)))
          .isCloseTo((ahead - behind) / (2 * h), within(1e-6));
    }

    @Test
    void itsLinkIsWhereItsAngleTurnsIt() {
      Pose3d raised = math.pose(arm, Map.of(arm, Math.PI / 2));

      assertThat(raised.getTranslation()).isEqualTo(point(0.1, 0, 0.5));
      Translation3d tip =
          point(0.6, 0, 0).rotateBy(raised.getRotation()).plus(raised.getTranslation());
      assertThat(tip.getDistance(point(0.1, 0, 1.1))).isLessThan(1e-9);
      // Raised, it stands up out of the robot; level, it reaches past the frame's front.
      assertThat(math.top(Map.of(arm, Math.PI / 2))).isCloseTo(1.1, within(1e-9));
      double front = Inches.of(14).in(Meters);
      assertThat(math.extension(Map.of(arm, 0.0)))
          .isCloseTo(Math.hypot(0.1 + 0.6 - front, 0), within(1e-9));
      assertThat(math.pose(Part.CHASSIS, Map.of())).isEqualTo(Pose3d.ZERO);
    }
  }

  /** An L-shaped link, its center raised 0.1 m above its zero direction 0.3 m out. */
  @Test
  void aCenterOffItsZeroGivesTheOffsetFromHorizontal() {
    Joint.Revolute arm =
        revolute(
            "arm",
            Part.CHASSIS,
            point(0, 0, 0.5),
            PITCH_UP,
            Optional.empty(),
            0,
            link("arm", 2, point(0.3, 0, 0.1), point(0.1, 0.1, 0.1)));
    RobotMath math =
        definition(
                List.of(arm),
                List.of(),
                List.of(MechanismFixtures.motors(arm, Motor.KRAKEN_X60, 1, gearing(50, 1))),
                List.of())
            .math();
    double offset = Math.atan2(0.1, 0.3);

    assertThat(math.gravityOffset(arm)).isCloseTo(offset, within(1e-12));
    assertThat(math.gravityTorque(arm, Map.of(arm, -offset)))
        .as("level, its center straight out")
        .isCloseTo(2 * GRAVITY * Math.hypot(0.3, 0.1), within(1e-9));
  }

  /**
   * A cascade elevator: a carriage and two stages, each measured from the chassis, the stages
   * moving a third and two thirds of the carriage's travel, as 6800 draws them. The carriage is
   * wound up by two Krakens at 5:1 on a 1 in drum.
   */
  @Nested
  class CascadeElevator {
    final Range travel = Range.meters(0, 1.2);
    final Joint.Prismatic carriage =
        prismatic(
            "carriage",
            Part.CHASSIS,
            point(0, 0, 0.1),
            UP,
            travel,
            link("carriage", 3, point(0, 0, 0.05), point(0.2, 0.2, 0.1)));
    final Joint.Prismatic stage1 =
        prismatic(
            "stage1",
            Part.CHASSIS,
            point(0, 0, 0.1),
            UP,
            Range.meters(0, 0.4),
            link("stage1", 2, point(0, 0, 0.3), point(0.25, 0.05, 0.6)));
    final Joint.Prismatic stage2 =
        prismatic(
            "stage2",
            Part.CHASSIS,
            point(0, 0, 0.1),
            UP,
            Range.meters(0, 0.8),
            link("stage2", 1.5, point(0, 0, 0.3), point(0.22, 0.05, 0.6)));
    final Drive winch =
        MechanismFixtures.motors(carriage, Motor.KRAKEN_X60, 2, gearing(5, 1), Inches.of(1));
    final RobotMath math =
        definition(
                List.of(carriage, stage1, stage2),
                List.of(linear(carriage, stage1, 1.0 / 3), linear(carriage, stage2, 2.0 / 3)),
                List.of(winch),
                List.of())
            .math();

    @Test
    void theStagesFollowTheCarriage() {
      Map<Joint, Double> raised = math.positions(Map.of(carriage, 0.9));

      assertThat(raised.get(stage1)).isCloseTo(0.3, within(1e-12));
      assertThat(raised.get(stage2)).isCloseTo(0.6, within(1e-12));
      assertThat(math.pose(stage2, Map.of(carriage, 0.9)).getZ()).isCloseTo(0.7, within(1e-12));
      assertThatIllegalArgumentException()
          .isThrownBy(() -> math.positions(Map.of(stage1, 0.2)))
          .withMessageContaining("stage1 follows carriage");
      assertThatIllegalArgumentException()
          .isThrownBy(() -> math.gravityTorque(stage2, Map.of()))
          .withMessageContaining("ask about carriage");
      // Stowed is the carriage alone: the stages are its to place, so any method takes it.
      assertThat(math.rests()).containsExactly(Map.entry(carriage, 0.0));
      assertThat(math.positions(math.rests())).containsKeys(stage1, stage2);
      assertThat(math.extension(math.rests())).isZero();
      assertThat(math.motorsAlone(carriage)).isTrue();
      assertThat(math.motorsAlone(stage1)).as("its leader's moves carry it").isFalse();
    }

    /** Each stage's weight counts by how far it moves for the carriage's travel. */
    @Test
    void theCarriageHoldsEveryStagesWeightByItsRatio() {
      double force = GRAVITY * (3 + 2.0 / 3 + 1.5 * 2.0 / 3);
      double drum = Inches.of(1).in(Meters);
      double rate = 5 / drum; // motor radians per meter
      double moved = 3 + 2.0 / 9 + 1.5 * 4.0 / 9 + 2 * KRAKEN_ROTOR * rate * rate;
      RobotMath.Gains gains = math.idealGains(carriage);

      assertThat(math.gravityTorque(carriage, Map.of(carriage, 0.2)))
          .isCloseTo(force, within(1e-9));
      assertThat(math.gravityTorque(carriage, Map.of(carriage, 1.1)))
          .isCloseTo(force, within(1e-9));
      assertThat(gains.kG()).isCloseTo(force * KRAKEN.R / (2 * KRAKEN.Kt * rate), within(1e-9));
      assertThat(gains.kV()).isCloseTo(rate * 12 / KRAKEN.freeSpeed, within(1e-9));
      assertThat(gains.kA()).isCloseTo(moved * KRAKEN.R / (2 * KRAKEN.Kt * rate), within(1e-9));
      assertThat(math.freeSpeed(carriage)).isCloseTo(KRAKEN.freeSpeed / rate, within(1e-9));
      assertThatIllegalArgumentException()
          .isThrownBy(() -> math.gravityOffset(carriage))
          .withMessageContaining("slides");
    }

    /** An arm on the carriage: its own gravity doesn't change with height; the carriage's does. */
    @Test
    void anArmRidesTheCarriage() {
      Joint.Revolute arm =
          revolute(
              "arm",
              carriage,
              point(0.1, 0, 0.2),
              PITCH_UP,
              Optional.of(Range.degrees(-90, 90)),
              0,
              link("arm", 2, point(0.25, 0, 0), point(0.5, 0.05, 0.05)));
      RobotMath withArm =
          definition(
                  List.of(carriage, stage1, stage2, arm),
                  List.of(linear(carriage, stage1, 1.0 / 3), linear(carriage, stage2, 2.0 / 3)),
                  List.of(
                      winch, MechanismFixtures.motors(arm, Motor.KRAKEN_X60, 1, gearing(40, 1))),
                  List.of())
              .math();

      double low = withArm.gravityTorque(arm, Map.of(carriage, 0.0, arm, 0.3));
      double high = withArm.gravityTorque(arm, Map.of(carriage, 1.0, arm, 0.3));
      assertThat(high).isCloseTo(low, within(1e-9));
      assertThat(low).isCloseTo(2 * GRAVITY * 0.25 * Math.cos(0.3), within(1e-9));
      assertThat(withArm.gravityTorque(carriage, Map.of(arm, 1.0)))
          .as("the elevator lifts the arm, wherever it points")
          .isCloseTo(GRAVITY * (3 + 2.0 / 3 + 1.0 + 2), within(1e-9));
      assertThat(withArm.pose(arm, Map.of(carriage, 1.0)).getZ()).isCloseTo(1.3, within(1e-12));
    }
  }

  /**
   * A continuous elevator, as 422's: each stage still until the carriage reaches it, at 0.45 m and
   * 0.9 m, then moving with it. The weight the carriage lifts steps at each.
   */
  @Test
  void aContinuousElevatorsLoadStepsAtEachStage() {
    Joint.Prismatic carriage =
        prismatic(
            "carriage",
            Part.CHASSIS,
            point(0, 0, 0.1),
            UP,
            Range.meters(0, 1.2),
            link("carriage", 3, point(0, 0, 0.05), point(0.2, 0.2, 0.1)));
    Joint.Prismatic stage1 =
        prismatic(
            "stage1",
            Part.CHASSIS,
            point(0, 0, 0.1),
            UP,
            Range.meters(0, 0.75),
            link("stage1", 2, point(0, 0, 0.3), point(0.25, 0.05, 0.6)));
    Joint.Prismatic stage2 =
        prismatic(
            "stage2",
            Part.CHASSIS,
            point(0, 0, 0.1),
            UP,
            Range.meters(0, 0.3),
            link("stage2", 1.5, point(0, 0, 0.3), point(0.22, 0.05, 0.6)));
    RobotMath math =
        definition(
                List.of(carriage, stage1, stage2),
                List.of(
                    curve(carriage, stage1, 0.45, 1.2, 0.75),
                    curve(carriage, stage2, 0.9, 1.2, 0.3)),
                List.of(
                    MechanismFixtures.motors(
                        carriage, Motor.KRAKEN_X60, 2, gearing(5, 1), Inches.of(1))),
                List.of())
            .math();

    assertThat(math.gravityTorque(carriage, Map.of(carriage, 0.3)))
        .isCloseTo(3 * GRAVITY, within(1e-9));
    assertThat(math.gravityTorque(carriage, Map.of(carriage, 0.6)))
        .isCloseTo(5 * GRAVITY, within(1e-9));
    assertThat(math.gravityTorque(carriage, Map.of(carriage, 1.0)))
        .isCloseTo(6.5 * GRAVITY, within(1e-9));
    assertThat(math.positions(Map.of(carriage, 1.0)).get(stage1)).isCloseTo(0.55, within(1e-12));
    assertThat(math.positions(Map.of(carriage, 1.0)).get(stage2)).isCloseTo(0.1, within(1e-12));
  }

  /**
   * An intake whose angle drives a hopper out along a linkage CAD solved into a table, with a
   * camera on the hopper, as a top team's.
   */
  @Test
  void aHopperOnACurveCarriesACamera() {
    Joint.Revolute intake =
        revolute(
            "intake",
            Part.CHASSIS,
            point(0.25, 0, 0.15),
            PITCH_UP,
            Optional.of(Range.builder().kind(Position.Kind.ANGLE).lower(0).upper(1.5).build()),
            0,
            link("intake", 2.5, point(0.15, 0, 0), point(0.3, 0.4, 0.05)));
    Joint.Prismatic hopper =
        prismatic(
            "hopper",
            Part.CHASSIS,
            point(0, 0, 0.2),
            new Vector3(1, 0, 0),
            Range.meters(0, 0.25),
            link("hopper", 1.5, point(0, 0, 0.1), point(0.3, 0.4, 0.2)));
    Coupling linkage =
        Coupling.Curve.builder()
            .leader(intake)
            .follower(hopper)
            .table(
                List.of(
                    new Translation2d(0, 0),
                    new Translation2d(0.5, 0.05),
                    new Translation2d(1.0, 0.15),
                    new Translation2d(1.5, 0.25)))
            .build();
    CameraConfig camera =
        CameraConfigBuilder.builder(ConfigFixtures.camera("hopper", point(0.05, 0, 0.15)))
            .joint(hopper)
            .build();
    RobotDefinition robot =
        definition(
            List.of(intake, hopper),
            List.of(linkage),
            List.of(MechanismFixtures.motors(intake, Motor.KRAKEN_X60, 1, gearing(30, 1))),
            List.of(),
            List.of(camera));
    RobotMath math = robot.math();

    Pose3d seen = math.cameraPose(camera, Map.of(intake, 1.0));
    assertThat(seen.getTranslation().getDistance(point(0.15 + 0.05, 0, 0.2 + 0.15)))
        .isLessThan(1e-12);
    assertThat(math.cameraPose(camera, Map.of()).getX())
        .as("stowed")
        .isCloseTo(0.05, within(1e-12));
    // The hopper slides level, so only the intake's own weight turns it; but it rides the intake's
    // speed at the curve's slope, 0.2 m a radian between 0.5 and 1.
    assertThat(math.gravityTorque(intake, Map.of(intake, 0.75)))
        .isCloseTo(2.5 * GRAVITY * 0.15 * Math.cos(0.75), within(1e-9));
    double intakeAlone =
        2.5 * (0.3 * 0.3 + 0.05 * 0.05) / 12 + 2.5 * 0.15 * 0.15 + KRAKEN_ROTOR * 900;
    assertThat(math.inertia(intake, Map.of(intake, 0.75)))
        .isCloseTo(intakeAlone + 1.5 * 0.2 * 0.2, within(1e-9));
  }

  /**
   * A turret from -390° to 40°, read by two encoders on 18- and 17-tooth gears against its 92-tooth
   * ring, as a top team's: together they tell lcm(18, 17) = 306 teeth of the ring apart, 3.33 of
   * its turns.
   */
  @Test
  void aTurretsTwoEncodersTellMoreThanATurnApart() {
    Joint.Revolute turret =
        revolute(
            "turret",
            Part.CHASSIS,
            point(0, 0, 0.6),
            UP,
            Optional.of(Range.degrees(-390, 40)),
            0,
            link("turret", 3, point(0.05, 0, 0.1), point(0.3, 0.3, 0.2)));
    Sensor.Encoder a = encoder("turret-a", turret, 92.0 / 18, true);
    Sensor.Encoder b = encoder("turret-b", turret, 92.0 / 17, true);
    Drive drive = MechanismFixtures.motors(turret, Motor.KRAKEN_X60, 1, gearing(50, 1));
    RobotMath math = definition(List.of(turret), List.of(), List.of(drive), List.of(a, b)).math();

    assertThat(math.absoluteSpan(List.of(a, b))).isCloseTo(2 * Math.PI * 306 / 92, within(1e-9));
    assertThat(math.absoluteSpan(List.of(a))).isCloseTo(2 * Math.PI * 18 / 92, within(1e-9));
    // The pair's window is centered on the range, -175°: 598.7° each way.
    Range window = math.absoluteWindow(List.of(a, b));
    assertThat(Math.toDegrees(window.lower())).isCloseTo(-175 - 180.0 * 306 / 92, within(1e-9));
    assertThat(Math.toDegrees(window.upper())).isCloseTo(-175 + 180.0 * 306 / 92, within(1e-9));
    // At 16 counts a turn, a count off on each moves their difference more than half the slower
    // one's turn: the pair can't pick it, and tells apart only what one does alone.
    Sensor.Encoder coarseA = encoder("turret-a", turret, 92.0 / 18, true, Optional.of(16));
    Sensor.Encoder coarseB = encoder("turret-b", turret, 92.0 / 17, true, Optional.of(16));
    assertThat(math.absoluteSpan(List.of(coarseA, coarseB)))
        .isCloseTo(2 * Math.PI * 18 / 92, within(1e-9));
    // Without each one's counts, nothing says whether their difference picks the turn: refused.
    Sensor.Encoder uncountedA = encoder("turret-a", turret, 92.0 / 18, true, Optional.empty());
    Sensor.Encoder uncountedB = encoder("turret-b", turret, 92.0 / 17, true, Optional.empty());
    assertThatIllegalArgumentException()
        .isThrownBy(() -> math.absoluteSpan(List.of(a, uncountedB)))
        .withMessage(
            "decoding turret-a and turret-b by their difference needs each encoder's counts per"
                + " turn: give them");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                definition(
                    List.of(turret), List.of(), List.of(drive), List.of(uncountedA, uncountedB)))
        .withMessageContaining(
            "joints: decoding turret-a and turret-b by their difference needs each encoder's counts"
                + " per turn: give them");
    assertThat(math.absoluteSpan(List.of(uncountedA)))
        .as("one alone reads no counts")
        .isCloseTo(2 * Math.PI * 18 / 92, within(1e-9));
    assertThat(math.gravityOffset(turret)).as("gravity doesn't turn it").isZero();
    assertThat(math.idealGains(turret).kG()).isCloseTo(0, within(1e-12));
    assertThatIllegalArgumentException()
        .as("one encoder can't tell its range apart")
        .isThrownBy(() -> definition(List.of(turret), List.of(), List.of(drive), List.of(a)))
        .withMessageContaining("turret's absolute encoders tell 70.43° of its 430° apart");
  }

  /**
   * A turret read coarse then fine, as a top team's: an absolute encoder geared down to 0.536 of a
   * turn for each of the turret's tells 671° apart; another on the motor's shaft, through 56.6:1,
   * turns a hundred times faster. Their readings repeat together only every 59.7 turns, but a
   * decoder reads the fine one only within the coarse one's turn: the span is the coarse one's, and
   * the window it reads sits where its discontinuity point puts it, the range centered.
   */
  @Test
  void aCoarseEncoderSetsTheSpanAndAFineOneOnlyRefinesIt() {
    Joint.Revolute turret =
        revolute(
            "turret",
            Part.CHASSIS,
            point(0, 0, 0.6),
            UP,
            Optional.of(Range.degrees(-390, 40)),
            0,
            link("turret", 3, point(0.05, 0, 0.1), point(0.3, 0.3, 0.2)));
    double coarseRatio = (220.0 / 25) * (8.0 / 30) * (8.0 / 35);
    Sensor.Encoder coarse = encoder("turret coarse", turret, coarseRatio, true);
    Sensor.Encoder fine = encoder("turret fine", turret, (220.0 / 14) * (36.0 / 10), true);
    Drive drive =
        MechanismFixtures.motors(
            turret, Motor.KRAKEN_X60, 1, gearing((220.0 / 14) * (36.0 / 10), 1));
    RobotMath math =
        definition(List.of(turret), List.of(), List.of(drive), List.of(coarse, fine)).math();

    assertThat(Math.toDegrees(math.absoluteSpan(List.of(coarse, fine))))
        .isCloseTo(360 / coarseRatio, within(1e-9));
    Range window = math.absoluteWindow(List.of(coarse));
    assertThat(Math.toDegrees(window.lower())).isCloseTo(-175 - 180 / coarseRatio, within(1e-9));
    assertThat(window.width()).isCloseTo(2 * Math.PI / coarseRatio, within(1e-9));
    assertThat(math.absoluteWindow(List.of(coarse, fine))).isEqualTo(window);
    // Read coarse then fine, a pair reads neither one's counts: they needn't be given.
    Sensor.Encoder uncountedCoarse =
        encoder("turret coarse", turret, coarseRatio, true, Optional.empty());
    Sensor.Encoder uncountedFine =
        encoder("turret fine", turret, (220.0 / 14) * (36.0 / 10), true, Optional.empty());
    assertThat(
            definition(
                    List.of(turret),
                    List.of(),
                    List.of(drive),
                    List.of(uncountedCoarse, uncountedFine))
                .math()
                .absoluteSpan(List.of(uncountedCoarse, uncountedFine)))
        .isCloseTo(2 * Math.PI / coarseRatio, within(1e-9));
    // The motor's rotor reads it too, but never absolutely: it counts from where it started.
    Sensor.Encoder rotor = drive.rotor();
    assertThat(rotor.absolute()).isFalse();
    assertThat(rotor.label()).isEqualTo("turret rotor");
    assertThat(math.sensorToMechanismRatio(rotor)).isEqualTo(fine.ratio(turret));
    assertThat(math.rotorToSensorRatio(drive, coarse))
        .isCloseTo(fine.ratio(turret) / coarseRatio, within(1e-12));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> math.absoluteSpan(List.of(coarse, rotor)))
        .withMessageContaining("turret rotor: a span is of absolute encoders on one joint");
  }

  /**
   * The same turret, its coarse encoder read once at startup: a reading becomes the turret's
   * position through the encoder's ratio, wrapped into the window it tells apart. Half a turn of
   * the encoder is 336° of turret, past the window's top, so it's the -336° the turret can reach.
   */
  @Test
  void aReadingBecomesThePositionInItsWindow() {
    Joint.Revolute turret =
        revolute(
            "turret",
            Part.CHASSIS,
            point(0, 0, 0.6),
            UP,
            Optional.of(Range.degrees(-390, 40)),
            0,
            link("turret", 3, point(0.05, 0, 0.1), point(0.3, 0.3, 0.2)));
    double ratio = (220.0 / 25) * (8.0 / 30) * (8.0 / 35);
    Sensor.Encoder coarse = encoder("turret coarse", turret, ratio, true);
    Drive drive = MechanismFixtures.motors(turret, Motor.KRAKEN_X60, 1, gearing(56.6, 1));
    RobotMath math = definition(List.of(turret), List.of(), List.of(drive), List.of(coarse)).math();
    double perTurn = 360 / ratio;

    assertThat(Math.toDegrees(math.fromAbsolute(coarse, 0))).isCloseTo(0, within(1e-9));
    assertThat(Math.toDegrees(math.fromAbsolute(coarse, -0.1)))
        .isCloseTo(-0.1 * perTurn, within(1e-9));
    assertThat(Math.toDegrees(math.fromAbsolute(coarse, 0.5)))
        .isCloseTo(0.5 * perTurn - perTurn, within(1e-9));
    assertThat(Math.toDegrees(math.fromAbsolute(coarse, 1.5)))
        .as("a whole turn more is the same place")
        .isCloseTo(0.5 * perTurn - perTurn, within(1e-9));
  }

  /**
   * A differential wrist: two motors, each turning pitch and roll together, one the roll backwards.
   * Their average is Phoenix's differential's average axis, pitch; half their difference, roll.
   */
  @Test
  void aDifferentialWristsMotorsShareItsPitchAndSplitItsRoll() {
    Joint.Revolute pitch =
        revolute(
            "pitch",
            Part.CHASSIS,
            point(0, 0, 0.6),
            PITCH_UP,
            Optional.of(Range.degrees(-90, 90)),
            0,
            link("wrist", 1.2, point(0.1, 0, 0), point(0.2, 0.1, 0.1)));
    Joint.Revolute roll =
        revolute(
            "roll",
            pitch,
            point(0.1, 0, 0),
            new Vector3(1, 0, 0),
            Optional.empty(),
            0,
            link("hand", 0.8, point(0.05, 0, 0), point(0.1, 0.15, 0.05)));
    Drive a = differential("a", pitch, roll, 20, 20);
    Drive b = differential("b", pitch, roll, 20, -20);
    RobotMath math = definition(List.of(pitch, roll), List.of(), List.of(a, b), List.of()).math();
    assertThat(math.motorsAlone(pitch)).as("two drives turn it together").isFalse();
    assertThat(math.motorsAlone(roll)).isFalse();
    DCMotor x44 = KRAKEN_X44;
    double perVolt = x44.stallTorque / x44.nominalVoltage;
    Map<Joint, Double> at = Map.of(pitch, 0.5, roll, 0.2);

    double motorA = math.coordinate(a, at);
    double motorB = math.coordinate(b, at);
    assertThat((motorA + motorB) / 2).isCloseTo(20 * 0.5, within(1e-12));
    assertThat((motorA - motorB) / 2).isCloseTo(20 * 0.2, within(1e-12));
    // Each motor's rotor reads both, as the motor turns: Phoenix's ratio on the average axis.
    Sensor.Encoder rotorB = b.rotor();
    assertThat(rotorB.joints()).containsExactly(pitch, roll);
    assertThat(rotorB.ratio(roll)).isEqualTo(-20);
    assertThat(math.sensorToMechanismRatio(rotorB, pitch)).isEqualTo(20);
    assertThatIllegalStateException()
        .isThrownBy(() -> math.sensorToMechanismRatio(rotorB))
        .withMessageContaining("b rotor reads 2 joints");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Sensor.Encoder.builder()
                    .label("wrist cancoder")
                    .terms(rotorB.terms())
                    .absolute(true)
                    .countsPerTurn(4096)
                    .build())
        .withMessageContaining("an absolute encoder reads one joint, not 2");
    RobotMath.Gains pitching = math.idealGains(pitch);
    RobotMath.Gains rolling = math.idealGains(roll);
    assertThat(pitching.kV()).isCloseTo(20 * 12 / x44.freeSpeed, within(1e-9));
    assertThat(rolling.kV()).isCloseTo(20 * 12 / x44.freeSpeed, within(1e-9));
    double torque = GRAVITY * (1.2 * 0.1 + 0.8 * 0.15);
    assertThat(pitching.kG())
        .as("both motors hold it up")
        .isCloseTo(torque / (2 * 20 * perVolt), within(1e-9));
    assertThat(rolling.kG()).isCloseTo(0, within(1e-9));
  }

  /**
   * A wrist chained through the shoulder's axle: its motor, on the chassis, turns the wrist and the
   * shoulder together, so with the motor still, the wrist keeps its angle to the floor as the
   * shoulder turns. Its gravity is the angle to the floor's, through the shoulder.
   */
  @Test
  void aWristChainedThroughTheShoulderFeelsTheShouldersAngle() {
    Joint.Revolute shoulder =
        revolute(
            "shoulder",
            Part.CHASSIS,
            point(0, 0, 0.5),
            PITCH_UP,
            Optional.of(Range.degrees(-30, 120)),
            0,
            link("upper arm", 3, point(0.25, 0, 0), point(0.5, 0.05, 0.05)));
    Joint.Revolute wrist =
        revolute(
            "wrist",
            shoulder,
            point(0.5, 0, 0),
            PITCH_UP,
            Optional.of(Range.degrees(-120, 120)),
            0,
            link("claw", 1, point(0.12, 0, 0), point(0.2, 0.1, 0.1)));
    Drive chain =
        Drive.builder()
            .label("wrist")
            .actuator(Actuator.Motors.builder().motor(Motor.KRAKEN_X44).count(1).build())
            .transmission(
                Transmission.Combined.builder()
                    .terms(List.of(term(wrist, 25), term(shoulder, 25)))
                    .efficiency(1)
                    .build())
            .build();
    RobotMath math =
        definition(
                List.of(shoulder, wrist),
                List.of(),
                List.of(
                    MechanismFixtures.motors(shoulder, Motor.KRAKEN_X60, 2, gearing(80, 1)), chain),
                List.of())
            .math();

    assertThat(math.gravityTorque(wrist, Map.of(shoulder, 0.6, wrist, -0.2)))
        .isCloseTo(GRAVITY * 0.12 * Math.cos(0.4), within(1e-9));
    assertThat(math.coordinate(chain, Map.of(shoulder, 0.6, wrist, -0.6)))
        .as("the claw level to the floor: the motor where it started")
        .isCloseTo(0, within(1e-12));
    assertThat(math.gravityOffset(wrist)).isCloseTo(0, within(1e-12));
    assertThat(math.idealGains(wrist).kV()).isCloseTo(25 * 12 / KRAKEN_X44.freeSpeed, within(1e-9));
    // The wrist's motor turns as the shoulder does, but it's the wrist's: the shoulder's seeds are
    // its own two motors'.
    assertThat(math.drivesOf(shoulder)).extracting(Drive::label).containsExactly("shoulder");
    assertThat(math.drivesOf(wrist)).containsExactly(chain);
    assertThat(math.motorsAlone(wrist))
        .as("its motor turns the shoulder too, and it's the wrist's")
        .isTrue();
    assertThat(math.idealGains(shoulder).kV()).isCloseTo(80 * 12 / KRAKEN.freeSpeed, within(1e-9));
    assertThat(math.freeSpeed(shoulder)).isCloseTo(KRAKEN.freeSpeed / 80, within(1e-9));
    assertThat(math.gravityTorque(shoulder, Map.of(shoulder, 0.0, wrist, 0.0)))
        .as("the shoulder holds the claw at the arm's end too")
        .isCloseTo(GRAVITY * (3 * 0.25 + 1 * 0.62), within(1e-9));
  }

  /**
   * A swerve module's drive motor turns its wheel through the drive's gearing, and turns it too as
   * the module steers, through the coupling: Tuner X's coupling ratio, now the module's own.
   */
  @Test
  void aSwerveDriveMotorTurnsWithItsModulesSteering() {
    Body body = ConfigFixtures.body();
    DrivetrainGeometry.ModuleType hardware =
        DrivetrainGeometryModuleTypeBuilder.builder(ConfigFixtures.modules())
            .couplingRatio(50.0 / 14)
            .build();
    DrivetrainGeometry drivetrain = DrivetrainGeometry.inset(hardware, body, Inches.of(2.625));
    DrivetrainGeometry.ModuleJoints frontLeft = drivetrain.joints().get(0);
    Sensor.Encoder cancoder = encoder("front-left cancoder", frontLeft.steer(), 1, true);
    Sensor.Encoder rotor = frontLeft.drive().rotor();
    RobotMath math =
        RobotMath.of(
            body,
            List.of(frontLeft.steer(), frontLeft.wheel()),
            List.of(),
            List.of(frontLeft.drive(), frontLeft.steering()),
            List.of(cancoder));
    double radius = Inches.of(2).in(Meters);

    assertThat(drivetrain.joints())
        .extracting(DrivetrainGeometry.ModuleJoints::label)
        .containsExactly("front-left", "front-right", "back-left", "back-right");
    assertThat(frontLeft.steer().origin().getTranslation().toTranslation2d())
        .isEqualTo(drivetrain.moduleTranslations()[0]);
    assertThat(math.coordinate(frontLeft.drive(), Map.of(frontLeft.steer(), 2 * Math.PI)))
        .as("a turn of the module, wheel still: the coupling's turns of the motor")
        .isCloseTo(2 * Math.PI * 50 / 14, within(1e-9));
    assertThat(math.coordinate(frontLeft.drive(), Map.of(frontLeft.wheel(), 2 * Math.PI)))
        .isCloseTo(2 * Math.PI * 6.75, within(1e-9));
    assertThat(math.rotorToSensorRatio(frontLeft.steering(), cancoder))
        .isCloseTo(12.8, within(1e-12));
    assertThat(math.sensorToMechanismRatio(rotor, frontLeft.wheel())).isEqualTo(6.75);
    assertThat(rotor.ratio(frontLeft.steer())).as("the coupling").isEqualTo(50.0 / 14);
    assertThat(math.discontinuityPoint(cancoder)).as("as the module's IO sets it").isEqualTo(0.5);
    // The wheel's ideal kV, per meter at the tread, is the drivetrain's own.
    assertThat(math.idealGains(frontLeft.wheel()).kV() / radius)
        .isCloseTo(hardware.idealDriveKv(), within(1e-9));
    assertThat(math.freeSpeed(frontLeft.wheel()) * radius)
        .isCloseTo(hardware.freeSpeed().in(MetersPerSecond), within(1e-9));
    // The steering is its steering motor's alone: the drive motor turns as it steers, through the
    // coupling, but it's the wheel's, which stays still as the module steers.
    DCMotor steering = hardware.steerMotor().model();
    double steerGearing = hardware.steerGearing().ratio();
    assertThat(math.drivesOf(frontLeft.steer())).containsExactly(frontLeft.steering());
    assertThat(math.drivesOf(frontLeft.wheel())).containsExactly(frontLeft.drive());
    assertThat(math.idealGains(frontLeft.steer()).kV())
        .isCloseTo(steerGearing * 12 / steering.freeSpeed, within(1e-9));
    assertThat(math.freeSpeed(frontLeft.steer()))
        .isCloseTo(steering.freeSpeed / steerGearing, within(1e-9));
  }

  /** A shooter's flywheel: a joint that turns freely, geared up 2:3. */
  @Test
  void aFlywheelTurnsFreelyFasterThanItsMotor() {
    Joint.Revolute flywheel =
        revolute(
            "flywheel",
            Part.CHASSIS,
            point(-0.2, 0, 0.5),
            new Vector3(0, 1, 0),
            Optional.empty(),
            0,
            link("flywheel", 1.2, point(0, 0, 0), point(0.1, 0.2, 0.1)));
    RobotMath math =
        definition(
                List.of(flywheel),
                List.of(),
                List.of(
                    MechanismFixtures.motors(flywheel, Motor.KRAKEN_X60, 2, gearing(2.0 / 3, 1))),
                List.of())
            .math();

    assertThat(math.freeSpeed(flywheel)).isCloseTo(KRAKEN.freeSpeed * 1.5, within(1e-9));
    assertThat(math.idealGains(flywheel).kG()).isCloseTo(0, within(1e-12));
    assertThat(math.idealGains(flywheel).kV())
        .isCloseTo((2.0 / 3) * 12 / KRAKEN.freeSpeed, within(1e-12));
    assertThat(math.sensors()).isEmpty();
    assertThat(math.joints()).containsExactly(flywheel);
    assertThat(math.drives()).hasSize(1);
  }

  /** The chassis is the body as weighed, less every link where it was stowed. */
  @Test
  void theChassisIsTheBodyLessItsLinks() {
    Joint.Revolute arm =
        revolute(
            "arm",
            Part.CHASSIS,
            point(0.1, 0, 0.5),
            PITCH_UP,
            Optional.empty(),
            0,
            link("arm", 4, point(0.3, 0, 0), point(0.6, 0.05, 0.05)));
    RobotDefinition robot = definition(List.of(arm), List.of(), List.of(), List.of());

    var chassis = robot.math().chassis();
    assertThat(chassis.mass().in(Kilograms)).isCloseTo(56, within(1e-9));
    // 60 kg centered over the origin, less 4 kg 0.4 m forward: the rest 0.4 * 4 / 56 behind.
    assertThat(chassis.cg().getX()).isCloseTo(-0.4 * 4 / 56, within(1e-12));
  }

  /**
   * An intake whose angle swings a hopper's floor through a linkage at 40/130 of its own, as a top
   * team's: the floor's weight isn't one cosine of the intake's angle, so Phoenix's Arm_Cosine
   * misses it, by as much as the gap the fit reports, where it reports it. The arm alone fits
   * exactly.
   */
  @Test
  void aFloorALinkageSwingsBendsGravityOffOneCosine() {
    Joint.Revolute intake =
        revolute(
            "intake",
            Part.CHASSIS,
            point(0.3, 0, 0.17),
            PITCH_UP,
            Optional.of(Range.degrees(0, 130)),
            Math.toRadians(130),
            link("intake", 6.3, point(0.16, 0, 0), point(0.33, 0.5, 0.05)));
    Joint.Revolute floor =
        revolute(
            "floor",
            Part.CHASSIS,
            point(0, 0, 0.12),
            PITCH_UP,
            Optional.of(Range.degrees(0, 40)),
            Math.toRadians(40),
            link("floor", 1.5, point(0.16, 0, 0), point(0.32, 0.5, 0.02)));
    Drive motor = MechanismFixtures.motors(intake, Motor.KRAKEN_X44, 1, gearing(48.8, 1));
    RobotMath alone = definition(List.of(intake), List.of(), List.of(motor), List.of()).math();
    RobotMath linked =
        definition(
                List.of(intake, floor),
                List.of(linear(intake, floor, 40.0 / 130)),
                List.of(motor),
                List.of())
            .math();

    RobotMath.GravityFit exact = alone.gravityFit(intake);
    assertThat(exact.misfit()).isCloseTo(0, within(1e-12));
    assertThat(exact.peak()).isCloseTo(6.3 * GRAVITY * 0.16, within(1e-9));
    RobotMath.GravityFit fit = linked.gravityFit(intake);
    double cosine = fit.peak() * Math.cos(fit.worstAt() + fit.offset());
    double real = linked.gravityTorque(intake, Map.of(intake, fit.worstAt()));
    assertThat(fit.offset()).isEqualTo(linked.gravityOffset(intake));
    assertThat(fit.misfit()).as("5% of the peak").isCloseTo(0.050, within(0.001));
    assertThat(Math.abs(cosine - real) / fit.peak()).isCloseTo(fit.misfit(), within(1e-12));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> linked.gravityFit(floor))
        .withMessageContaining("floor follows intake");
  }

  /**
   * An arm nobody has weighed: its link's mass is none, zero, and everything derived stays
   * consistent at zero. No gravity to hold, so no kG seed and no offset; its kA from its motors'
   * rotors through the gearing alone; and the chassis is the body as weighed.
   */
  @Test
  void anArmWithNoMassHasNoGravityAndItsRotorsInertiaAlone() {
    Joint.Revolute arm =
        revolute(
            "arm",
            Part.CHASSIS,
            point(0.1, 0, 0.5),
            PITCH_UP,
            Optional.of(Range.degrees(-10, 110)),
            0,
            Link.builder().label("arm").mass(MassProperties.NONE).build());
    Drive motors = MechanismFixtures.motors(arm, Motor.KRAKEN_X60, 2, gearing(60, 0.9));
    RobotDefinition robot = definition(List.of(arm), List.of(), List.of(motors), List.of());
    RobotMath math = robot.math();
    RobotMath.Gains gains = math.idealGains(arm);
    double rotors = 2 * KRAKEN_ROTOR * 3600;

    assertThat(math.gravityTorque(arm, Map.of(arm, 0.0))).isZero();
    assertThat(math.gravityOffset(arm)).isZero();
    assertThat(gains.kG()).isZero();
    assertThat(math.gravityFit(arm))
        .isEqualTo(new RobotMath.GravityFit(0, 0, 0, -Math.toRadians(10)));
    assertThat(math.inertia(arm, Map.of())).isCloseTo(rotors, within(1e-12));
    assertThat(gains.kA()).isCloseTo(rotors * KRAKEN.R / (2 * KRAKEN.Kt * 60 * 0.9), within(1e-12));
    assertThat(math.chassis()).isEqualTo(robot.body().massProperties());
  }

  /**
   * Gravity can move an arm about a level axis and a slide that isn't level, by their axes alone,
   * with no mass at all; not a turret, about the vertical, nor a slide that runs level. A ramp, a
   * slide along its own x on a frame tipped up 30°, is read in the robot's frame.
   */
  @Test
  void gravityActsOnJointsByTheirAxesNotTheirMass() {
    Joint.Revolute arm =
        revolute(
            "arm",
            Part.CHASSIS,
            point(0.1, 0, 0.5),
            PITCH_UP,
            Optional.of(Range.degrees(-10, 110)),
            0,
            Link.unseen("arm"));
    Joint.Revolute turret =
        revolute(
            "turret",
            Part.CHASSIS,
            point(0, 0, 0.6),
            UP,
            Optional.empty(),
            0,
            Link.unseen("turret"));
    Joint.Prismatic lift =
        prismatic(
            "lift", Part.CHASSIS, point(-0.2, 0, 0.1), UP, Range.meters(0, 1), Link.unseen("lift"));
    Joint.Prismatic hopper =
        prismatic(
            "hopper",
            Part.CHASSIS,
            point(0, 0, 0.2),
            new Vector3(1, 0, 0),
            Range.meters(0, 0.3),
            Link.unseen("hopper"));
    Joint.Prismatic ramp =
        JointPrismaticBuilder.builder(
                prismatic(
                    "ramp",
                    Part.CHASSIS,
                    point(0, 0, 0.3),
                    new Vector3(1, 0, 0),
                    Range.meters(0, 0.3),
                    Link.unseen("ramp")))
            .origin(new Transform3d(point(0, 0, 0.3), new Rotation3d(0, Math.toRadians(-30), 0)))
            .build();
    RobotMath math =
        definition(List.of(arm, turret, lift, hopper, ramp), List.of(), List.of(), List.of())
            .math();

    assertThat(math.gravityActsOn(arm)).isTrue();
    assertThat(math.gravityActsOn(turret)).isFalse();
    assertThat(math.gravityActsOn(lift)).isTrue();
    assertThat(math.gravityActsOn(hopper)).isFalse();
    assertThat(math.gravityActsOn(ramp)).isTrue();
  }

  /**
   * What a definition doesn't know, each with its cost: the test robot weighed and its inertia
   * given, so only the share of its mass no link holds; then the same robot unweighed, its inertia
   * a box's, its center of mass not given, with an arm nobody weighed, a roller weighed but not
   * spun, and a joint nothing moves.
   */
  @Test
  void theMissingDataReportSaysWhatIsntKnownAndWhatItCosts() {
    Joint.Revolute arm =
        revolute(
            "arm",
            Part.CHASSIS,
            point(0.1, 0, 0.5),
            PITCH_UP,
            Optional.of(Range.degrees(-10, 110)),
            0,
            Link.builder().label("arm").mass(MassProperties.NONE).build());
    Joint.Revolute roller =
        revolute(
            "roller",
            arm,
            point(0.5, 0, 0),
            new Vector3(0, 1, 0),
            Optional.empty(),
            0,
            Link.builder()
                .label("roller")
                .mass(
                    MassProperties.builder()
                        .mass(Kilograms.of(0.5))
                        .cg(point(0, 0, 0))
                        .inertia(com.michaelgrundvig.frc.world.physical.Inertia.ZERO)
                        .build())
                .shapes(link("roller", 0.5, point(0, 0, 0), point(0.05, 0.4, 0.05)).shapes())
                .build());
    RobotDefinition weighed =
        definition(
            List.of(arm, roller),
            List.of(),
            List.of(MechanismFixtures.motors(arm, Motor.KRAKEN_X60, 1, gearing(50, 1))),
            List.of());
    RobotDefinition unweighed =
        RobotDefinitionBuilder.builder(weighed)
            .body(
                BodyBuilder.builder(weighed.body())
                    .weight(Body.Weight.unweighed(Kilograms.of(68)))
                    .cg(Optional.empty())
                    .inertia(Body.Turning.BOX)
                    .build())
            .build();

    assertThat(notes(ConfigFixtures.definition(List.of())))
        .extracting(Finding::message, Finding::scope)
        .containsExactly(
            tuple(
                "60.00 kg of the robot's 60.00 kg is in no link: the chassis, its electronics,"
                    + " bumpers and battery: carried as one rigid body",
                SIMULATOR));
    assertThat(notes(unweighed))
        .extracting(Finding::message, Finding::scope)
        .containsExactly(
            tuple(
                "the robot isn't weighed: its mass is the season's most, 68 Kg: the drive's moves"
                    + " and feedforward, the simulator's acceleration and the links' mass budget"
                    + " rest on the rules, not the robot",
                ROBOT),
            tuple(
                "the body's inertia is a box's, its mass spread through its bumpers' outline and"
                    + " height: how fast the drive's moves turn, its turning feedforward, and how"
                    + " the simulator tips and turns the robot are approximate",
                ROBOT),
            tuple(
                "67.50 kg of the robot's 68.00 kg is in no link: the chassis, its electronics,"
                    + " bumpers and battery: carried as one rigid body",
                SIMULATOR),
            tuple(
                "arm has no mass: arm's gravity comes from its kG, and arm adds nothing to arm's"
                    + " inertia",
                SIMULATOR),
            tuple(
                "arm has no shape: it isn't drawn, nor checked against the starting height and the"
                    + " extension limit",
                ROBOT),
            tuple(
                "roller has mass but no inertia: its own turning adds nothing to roller's inertia",
                SIMULATOR),
            tuple(
                "roller has nothing moving it: no drive turns it and no coupling leads it: no ideal"
                    + " gains, and the simulator leaves it where it is",
                ROBOT));
    assertThat(notes(unweighed))
        .extracting(Finding::subject)
        .containsExactly("test", "test", "test", "arm", "arm", "roller", "roller");
  }

  /**
   * Modules that leave their tread's grip and inertias to the defaults say so where each is read:
   * the grip on the robot, which plans its drive moves and judges slip by it; the inertias in the
   * simulator alone. Each once, however many corners leave it out, and only what's left out.
   */
  @Test
  void theModulesDefaultsAreSaidWhereTheyreRead() {
    RobotDefinition robot = ConfigFixtures.definition(List.of());
    DrivetrainGeometry.ModuleType unmeasured =
        DrivetrainGeometryModuleTypeBuilder.builder(ConfigFixtures.modules())
            .wheelCoefficientOfFriction(Optional.empty())
            .driveInertia(Optional.empty())
            .steerInertia(Optional.empty())
            .build();
    DrivetrainGeometry.ModuleType steerUnmeasured =
        DrivetrainGeometryModuleTypeBuilder.builder(ConfigFixtures.modules())
            .steerInertia(Optional.empty())
            .build();
    RobotDefinition defaulted =
        RobotDefinitionBuilder.builder(robot)
            .drivetrain(DrivetrainGeometry.inset(unmeasured, robot.body(), Inches.of(2.625)))
            .build();
    RobotDefinition steerDefaulted =
        RobotDefinitionBuilder.builder(robot)
            .drivetrain(DrivetrainGeometry.inset(steerUnmeasured, robot.body(), Inches.of(2.625)))
            .build();

    assertThat(notes(defaulted))
        .extracting(Finding::message, Finding::scope)
        .containsExactly(
            tuple(
                "60.00 kg of the robot's 60.00 kg is in no link: the chassis, its electronics,"
                    + " bumpers and battery: carried as one rigid body",
                SIMULATOR),
            tuple(
                "the tread's grip isn't measured: it's 1.2, PathPlanner's default: drive moves"
                    + " plan their acceleration on it, and the slip check judges by it: measure it"
                    + " with a pull test (docs/characterization.md, tread friction)",
                ROBOT),
            tuple(
                "the drive's own inertia isn't measured: it's 0.001 kg m², an estimate for a 4 in"
                    + " wheel and a Kraken X60's rotor: how fast the simulator's wheels spin up is"
                    + " a guess, though a small one beside the robot's mass they carry",
                SIMULATOR),
            tuple(
                "the steering's inertia isn't measured: it's 0.004 kg m², Phoenix Tuner X's"
                    + " simulation value: how fast the simulator's modules steer at their current"
                    + " limit is a guess",
                SIMULATOR));
    assertThat(MechanismFixtures.notes(steerDefaulted))
        .hasSize(2)
        .last()
        .asString()
        .startsWith(
            "the steering's inertia isn't measured: it's 0.004 kg m², Phoenix Tuner X's simulation"
                + " value:");
  }

  /**
   * A joint gravity acts on, held at positions, is held by its kG: one not written is doubted on
   * the robot, as missing, or as seeded from a mass nobody tuned. A kG written, 0 included, clears
   * it; a roller, a turret, a slide that runs level, and a joint with no control never need one.
   */
  @Test
  void aJointGravityActsOnWithNoKgWrittenIsDoubted() {
    MechanismControl control = MechanismFixtures.stowed(30);
    Joint.Revolute bare =
        revolute(
                "bare",
                Part.CHASSIS,
                point(0.1, 0, 0.5),
                PITCH_UP,
                Optional.of(Range.degrees(-10, 110)),
                0,
                Link.unseen("bare arm"))
            .control(control);
    Joint.Revolute weighed =
        revolute(
                "weighed",
                Part.CHASSIS,
                point(0.1, 0.3, 0.5),
                PITCH_UP,
                Optional.of(Range.degrees(-10, 110)),
                0,
                link("weighed arm", 2, point(0.3, 0, 0), point(0.6, 0.05, 0.05)))
            .control(control);
    Joint.Revolute counterbalanced =
        revolute(
                "counterbalanced",
                Part.CHASSIS,
                point(0.1, -0.3, 0.5),
                PITCH_UP,
                Optional.of(Range.degrees(-10, 110)),
                0,
                Link.unseen("balanced arm"))
            .control(control.withKg(0));
    Joint.Revolute roller =
        revolute(
                "roller",
                bare,
                point(0.5, 0, 0),
                PITCH_UP,
                Optional.empty(),
                0,
                Link.unseen("roller"))
            .control(MechanismControl.velocity(0.02));
    Joint.Revolute turret =
        revolute(
                "turret",
                Part.CHASSIS,
                point(0, 0, 0.6),
                UP,
                Optional.of(Range.degrees(-90, 90)),
                0,
                Link.unseen("turret"))
            .control(control);
    Joint.Prismatic lift =
        prismatic(
                "lift",
                Part.CHASSIS,
                point(-0.2, 0, 0.1),
                UP,
                Range.meters(0, 1),
                Link.unseen("lift"))
            .control(control);
    Joint.Prismatic hopper =
        prismatic(
                "hopper",
                Part.CHASSIS,
                point(0, 0, 0.2),
                new Vector3(1, 0, 0),
                Range.meters(0, 0.3),
                Link.unseen("hopper"))
            .control(control);
    Joint.Revolute idle =
        revolute(
            "idle",
            Part.CHASSIS,
            point(0.1, 0, 0.2),
            PITCH_UP,
            Optional.of(Range.degrees(0, 90)),
            0,
            Link.unseen("idle arm"));
    List<Joint> joints =
        List.of(bare, weighed, counterbalanced, roller, turret, lift, hopper, idle);
    List<Drive> drives = new ArrayList<>();
    for (Joint joint : joints) {
      drives.add(
          joint instanceof Joint.Revolute revolute
              ? MechanismFixtures.motors(revolute, Motor.KRAKEN_X60, 1, gearing(50, 1))
              : MechanismFixtures.motors(
                  (Joint.Prismatic) joint, Motor.KRAKEN_X60, 1, gearing(10, 1), Inches.of(1)));
    }
    RobotDefinition robot = definition(joints, List.of(), drives, List.of());

    assertThat(robot.findings())
        .filteredOn(finding -> finding.scope() == ROBOT && finding.message().contains("kG"))
        .extracting(Finding::message, Finding::scope)
        .containsExactly(
            tuple(
                "bare has no kG: it holds against gravity by kP alone, so it sits below its"
                    + " targets until its kG is tuned and written, withKg(...), or written as 0"
                    + " for a counterbalanced joint",
                ROBOT),
            tuple(
                "weighed's kG is seeded from its mass, not tuned: it holds only as well as its"
                    + " links' mass and where it sits are right: tune its kG and write it,"
                    + " withKg(...)",
                ROBOT),
            tuple(
                "lift has no kG: it holds against gravity by kP alone, so it sits below its"
                    + " targets until its kG is tuned and written, withKg(...), or written as 0"
                    + " for a counterbalanced joint",
                ROBOT));
    assertThat(MechanismFixtures.notes(robot.with(bare.retuned(tuned -> tuned.withKg(0.4)))))
        .noneMatch(note -> note.startsWith("bare has no kG"));
  }

  private static Coupling.Curve curve(
      Joint leader, Joint follower, double breakpoint, double end, double top) {
    return Coupling.Curve.builder()
        .leader(leader)
        .follower(follower)
        .table(
            List.of(
                new Translation2d(0, 0),
                new Translation2d(breakpoint, 0),
                new Translation2d(end, top)))
        .build();
  }

  private static Transmission.Term term(Joint joint, double ratio) {
    return Transmission.Term.builder().joint(joint).ratio(ratio).build();
  }

  private static Drive differential(
      String label, Joint pitch, Joint roll, double onPitch, double onRoll) {
    return Drive.builder()
        .label(label)
        .actuator(Actuator.Motors.builder().motor(Motor.KRAKEN_X44).count(1).build())
        .transmission(
            Transmission.Combined.builder()
                .terms(List.of(term(pitch, onPitch), term(roll, onRoll)))
                .efficiency(1)
                .build())
        .build();
  }

  /** What a definition says it doesn't know. */
  private static List<Finding> notes(RobotDefinition robot) {
    return robot.findings().stream()
        .filter(finding -> finding.level() == Finding.Level.NOTE)
        .toList();
  }
}
