package frc.robot.lib.phoenix;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.Coupling;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Transmission;
import frc.robot.Robots;
import frc.robot.lib.wiring.CanBus;
import frc.robot.lib.wiring.MechanismWiring;
import frc.robot.lib.wiring.MechanismWiringBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;

/**
 * The helper's settings for a robot made up for it: an arm with a wrist, an elevator, a turret, a
 * flywheel and a differential, each conversion and each refusal on its own. The 2026 robots'
 * mechanisms are checked against their teams' settings in {@code frc.robot.robots2026}.
 */
class PhoenixMechanismTest {
  private static final Body BODY = Robots.SIM.definition().body();
  private static final Vector3 RAISE = new Vector3(0, -1, 0); // turns +x up as it grows
  private static final Vector3 LOWER = new Vector3(0, 1, 0); // turns +x down as it grows
  private static final double TURN = 2 * Math.PI;
  private static final MechanismControl.NeutralMode BRAKE = MechanismControl.NeutralMode.BRAKE;

  /** The arm: 4 kg, 0.3 m out along its zero, horizontal; two Krakens through 60:1. */
  private static final Joint.Revolute ARM =
      revolute(
          "arm",
          Part.CHASSIS,
          new Translation3d(0.1, 0, 0.5),
          RAISE,
          Optional.of(Range.degrees(-30, 120)),
          Math.toRadians(-30),
          weighed("arm link", 4, new Translation3d(0.3, 0, 0)));

  /** A wrist on the arm's end, which the arm's turning tilts. */
  private static final Joint.Revolute WRIST =
      revolute(
          "wrist",
          ARM,
          new Translation3d(0.6, 0, 0),
          RAISE,
          Optional.of(Range.degrees(-90, 90)),
          0,
          weighed("claw", 1, new Translation3d(0.1, 0, 0)));

  /** An elevator's carriage, 5 kg, on a 2 cm drum through 10:1. */
  private static final Joint.Prismatic ELEVATOR =
      Joint.Prismatic.builder()
          .label("elevator")
          .parent(Part.CHASSIS)
          .origin(new Transform3d(new Translation3d(-0.2, 0, 0.1), Rotation3d.ZERO))
          .axis(Vector3.UP)
          .range(Optional.of(Range.meters(0, 1.2)))
          .reference(0)
          .link(weighed("carriage", 5, Translation3d.ZERO))
          .positions(List.of())
          .outputs(List.of())
          .build();

  /** A turret, unweighed, on 340° of travel. */
  private static final Joint.Revolute TURRET =
      revolute(
          "turret",
          Part.CHASSIS,
          new Translation3d(0, 0, 0.3),
          Vector3.UP,
          Optional.of(Range.degrees(-160, 180)),
          0,
          unseen("turret plate"));

  /** A flywheel, which turns freely. */
  private static final Joint.Revolute FLYWHEEL =
      revolute(
          "flywheel",
          Part.CHASSIS,
          new Translation3d(-0.2, 0, 0.4),
          LOWER,
          Optional.empty(),
          0,
          unseen("wheels"));

  private static final Drive ARM_MOTORS = ARM.motors(Motor.KRAKEN_X60, 2, gearing(60));
  private static final Drive WRIST_MOTOR = WRIST.motors(Motor.KRAKEN_X44, 1, gearing(30));
  private static final Drive ELEVATOR_MOTORS =
      ELEVATOR.motors(Motor.KRAKEN_X60, 2, gearing(10), Meters.of(0.02));
  private static final Drive TURRET_MOTOR = TURRET.motors(Motor.KRAKEN_X60, 1, gearing(50));
  private static final Drive FLYWHEEL_MOTOR = FLYWHEEL.motors(Motor.KRAKEN_X60, 1, gearing(1));

  /** The turret's CANcoder, geared to turn back as the turret turns forward. */
  private static final Sensor.Encoder TURRET_CANCODER =
      Sensor.Encoder.builder()
          .label("turret cancoder")
          .terms(List.of(term(TURRET, -1)))
          .absolute(true)
          .countsPerTurn(4096)
          .build();

  /** The arm's CANcoder, on its shaft. */
  private static final Sensor.Encoder ARM_CANCODER = encoder("arm cancoder", ARM, 1);

  private final RobotMath math =
      RobotMath.of(
          BODY,
          List.of(ARM, WRIST, ELEVATOR, TURRET, FLYWHEEL),
          List.of(),
          List.of(ARM_MOTORS, WRIST_MOTOR, ELEVATOR_MOTORS, TURRET_MOTOR, FLYWHEEL_MOTOR),
          List.of(TURRET_CANCODER, ARM_CANCODER));

  // ---- Ratios, directions, limits and followers ----

  @Test
  void anArmsRotorReadsItThroughItsGearingInItsRotations() {
    PhoenixMechanism.Settings arm =
        PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring(11, 12), control());
    TalonFXConfiguration leader = arm.leader();

    assertThat(leader.Feedback.FeedbackSensorSource)
        .isEqualTo(FeedbackSensorSourceValue.RotorSensor);
    assertThat(leader.Feedback.SensorToMechanismRatio).isEqualTo(60);
    assertThat(leader.Feedback.RotorToSensorRatio).as("Phoenix's default").isEqualTo(1);
    assertThat(leader.MotorOutput.Inverted).isEqualTo(InvertedValue.CounterClockwise_Positive);
    assertThat(leader.MotorOutput.NeutralMode).isEqualTo(NeutralModeValue.Brake);
    assertThat(leader.ClosedLoopGeneral.ContinuousWrap).as("it has stops").isFalse();
    assertThat(arm.cancoder()).isEmpty();
  }

  @Test
  void everyCurrentLimitIsSetWhereItShows() {
    TalonFXConfiguration leader =
        PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring(11, 12), control()).leader();

    assertThat(leader.CurrentLimits.StatorCurrentLimitEnable).isTrue();
    assertThat(leader.CurrentLimits.StatorCurrentLimit).isEqualTo(60);
    assertThat(leader.CurrentLimits.SupplyCurrentLimitEnable).isTrue();
    assertThat(leader.CurrentLimits.SupplyCurrentLimit).isEqualTo(50);
    assertThat(leader.CurrentLimits.SupplyCurrentLowerLimit).isEqualTo(35);
    assertThat(leader.CurrentLimits.SupplyCurrentLowerTime).isEqualTo(0.5);
  }

  @Test
  void aMotorTurningClockwiseToRaiseTheJointIsInverted() {
    MechanismWiring wiring = wiring(motor(11, true), motor(12, true));

    TalonFXConfiguration leader =
        PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring, coasting()).leader();

    assertThat(leader.MotorOutput.Inverted).isEqualTo(InvertedValue.Clockwise_Positive);
    assertThat(leader.MotorOutput.NeutralMode).isEqualTo(NeutralModeValue.Coast);
  }

  /**
   * A follower turning the leader's way is aligned with it, one turning the other way opposed: the
   * wiring says which way each turns as the joint moves forward, and alignment follows.
   */
  @Test
  void followersAreAlignedOrOpposedByWhichWayTheyTurn() {
    PhoenixMechanism.Settings same =
        PhoenixMechanism.configure(
            math, ARM, ARM_MOTORS, wiring(motor(11, false), motor(12, false)), control());
    PhoenixMechanism.Settings facing =
        PhoenixMechanism.configure(
            math, ARM, ARM_MOTORS, wiring(motor(11, false), motor(12, true)), control());
    PhoenixMechanism.FollowerSettings follower = facing.followers().get(0);

    assertThat(same.followers().get(0).request().MotorAlignment)
        .isEqualTo(MotorAlignmentValue.Aligned);
    assertThat(follower.request().MotorAlignment).isEqualTo(MotorAlignmentValue.Opposed);
    assertThat(follower.request().LeaderID).isEqualTo(11);
    assertThat(follower.id()).isEqualTo(12);
    assertThat(follower.configuration().MotorOutput.Inverted)
        .as("its own direction, for its own readings")
        .isEqualTo(InvertedValue.Clockwise_Positive);
    assertThat(follower.configuration().Feedback.SensorToMechanismRatio).isEqualTo(60);
    assertThat(follower.configuration().CurrentLimits.StatorCurrentLimit).isEqualTo(60);
    assertThat(follower.configuration().MotorOutput.NeutralMode).isEqualTo(NeutralModeValue.Brake);
  }

  @Test
  void aDrivesMotorsAreAllWired() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring(11), control()))
        .withMessage("arm is 2 motors, and its wiring names 1");
  }

  @Test
  void aSoftLimitIsInPhoenixsRotationsAndInsideTheStops() {
    MechanismControl limited = control(soft(Math.toRadians(-20), Math.toRadians(110)));

    TalonFXConfiguration leader =
        PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring(11, 12), limited).leader();

    assertThat(leader.SoftwareLimitSwitch.ForwardSoftLimitEnable).isTrue();
    assertThat(leader.SoftwareLimitSwitch.ForwardSoftLimitThreshold)
        .isCloseTo(110.0 / 360, within(1e-12));
    assertThat(leader.SoftwareLimitSwitch.ReverseSoftLimitEnable).isTrue();
    assertThat(leader.SoftwareLimitSwitch.ReverseSoftLimitThreshold)
        .isCloseTo(-20.0 / 360, within(1e-12));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                PhoenixMechanism.configure(
                    math,
                    ARM,
                    ARM_MOTORS,
                    wiring(11, 12),
                    control(soft(Math.toRadians(-40), Math.toRadians(110)))))
        .withMessage(
            "arm's soft limits, -40.0° to 110.0°, must be inside its hard stops, -30.0° to 120.0°");
  }

  @Test
  void noSoftLimitsLeavesPhoenixsOff() {
    TalonFXConfiguration leader =
        PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring(11, 12), control()).leader();

    assertThat(leader.SoftwareLimitSwitch.ForwardSoftLimitEnable).isFalse();
    assertThat(leader.SoftwareLimitSwitch.ReverseSoftLimitEnable).isFalse();
  }

  @Test
  void aJointThatTurnsFreelyTakesTheShortWayRound() {
    TalonFXConfiguration leader =
        PhoenixMechanism.configure(math, FLYWHEEL, FLYWHEEL_MOTOR, wiring(20), control()).leader();

    assertThat(leader.ClosedLoopGeneral.ContinuousWrap).isTrue();
    assertThat(leader.Feedback.SensorToMechanismRatio).isEqualTo(1);
  }

  @Test
  void anElevatorIsInItsDrumsRotations() {
    PhoenixMechanism.Settings elevator =
        PhoenixMechanism.configure(
            math,
            ELEVATOR,
            ELEVATOR_MOTORS,
            wiring(30, 31),
            control(seeded(), Optional.of(soft(0.1, 1.0)), Optional.of(profile(1, 3, 0)), BRAKE));
    TalonFXConfiguration leader = elevator.leader();
    double drum = TURN * 0.02;

    assertThat(elevator.units().name()).isEqualTo("rotations of elevator's drum");
    assertThat(leader.Feedback.SensorToMechanismRatio)
        .as("the gearing")
        .isCloseTo(10, within(1e-12));
    assertThat(leader.SoftwareLimitSwitch.ReverseSoftLimitThreshold)
        .isCloseTo(0.1 / drum, within(1e-12));
    assertThat(leader.SoftwareLimitSwitch.ForwardSoftLimitThreshold)
        .isCloseTo(1.0 / drum, within(1e-12));
    assertThat(leader.MotionMagic.MotionMagicCruiseVelocity).isCloseTo(1 / drum, within(1e-12));
    assertThat(leader.MotionMagic.MotionMagicAcceleration).isCloseTo(3 / drum, within(1e-12));
    assertThat(leader.ClosedLoopGeneral.ContinuousWrap).isFalse();
  }

  /**
   * A winch, 100:1 on a ½ in spool: its rotor turns 2506 times a meter, past Phoenix's 1000, but
   * its Phoenix unit is the spool's rotation, so its ratio is the gearing, 100. Its soft limit and
   * its cruise, in meters in its control, are in spool rotations in Phoenix.
   */
  @Test
  void aWinchIsInItsSpoolsRotations() {
    Joint.Prismatic winch =
        Joint.Prismatic.builder()
            .label("winch")
            .parent(Part.CHASSIS)
            .origin(Transform3d.ZERO)
            .axis(Vector3.UP)
            .range(Optional.of(Range.meters(0, 0.5)))
            .reference(0)
            .link(unseen("hook"))
            .positions(List.of())
            .outputs(List.of())
            .build();
    Drive slow = winch.motors(Motor.KRAKEN_X60, 1, gearing(100), Inches.of(0.25));
    RobotMath robot = RobotMath.of(BODY, List.of(winch), List.of(), List.of(slow), List.of());
    double spool = TURN * 0.25 * 0.0254;
    double free = robot.freeSpeed(winch);

    TalonFXConfiguration leader =
        PhoenixMechanism.configure(
                robot,
                winch,
                slow,
                wiring(1),
                control(
                    seeded(),
                    Optional.of(soft(0, 0.5)),
                    Optional.of(profile(free / 2, 1, 0)),
                    BRAKE))
            .leader();

    assertThat(slow.rotor().ratio(winch)).isGreaterThan(PhoenixMechanism.MAX_RATIO);
    assertThat(leader.Feedback.SensorToMechanismRatio).isCloseTo(100, within(1e-9));
    assertThat(leader.SoftwareLimitSwitch.ForwardSoftLimitThreshold)
        .isCloseTo(0.5 / spool, within(1e-9));
    assertThat(leader.MotionMagic.MotionMagicCruiseVelocity)
        .as("half the spool's free speed, a Kraken's 100 rps through 100:1")
        .isCloseTo(0.5, within(1e-9));
    assertThat(leader.Slot0.kV).as("12 V for the spool's 1 rps").isCloseTo(12, within(1e-9));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                PhoenixMechanism.configure(
                    robot,
                    winch,
                    slow,
                    wiring(1),
                    control(
                        seeded(), Optional.empty(), Optional.of(profile(free * 2, 1, 0)), BRAKE)))
        .withMessage(
            "winch's Motion Magic cruise, 2 drum rot/s, is 2 times its free speed, 1 drum rot/s:"
                + " the profile would ask more than its motors turn it at 12 V");
  }

  /** Geared past 1000:1, a ratio Phoenix can't hold is refused. */
  @Test
  void aRatioPhoenixCantHoldIsRefused() {
    Drive crawl = TURRET.motors(Motor.KRAKEN_X60, 1, gearing(1500));
    RobotMath robot = RobotMath.of(BODY, List.of(TURRET), List.of(), List.of(crawl), List.of());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> PhoenixMechanism.configure(robot, TURRET, crawl, wiring(1), control()))
        .withMessage(
            "turret's SensorToMechanismRatio would be 1500, and Phoenix holds from -1000 to 1000,"
                + " never 0");
  }

  @Test
  void onlyMotorsAreConfigured() {
    Drive servo =
        Drive.builder()
            .label("arm servo")
            .actuator(Actuator.Servo.builder().from(0).to(1).force(1).speed(1).build())
            .transmission(Transmission.Geared.builder().joint(ARM).gearing(gearing(1)).build())
            .build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> PhoenixMechanism.configure(math, ARM, servo, wiring(11), control()))
        .withMessage("arm servo isn't motors: Phoenix drives motors");
  }

  @Test
  void theDriveMustMoveTheJoint() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> PhoenixMechanism.configure(math, WRIST, ARM_MOTORS, wiring(1, 2), control()))
        .withMessage("arm doesn't move wrist");
  }

  // ---- Gains and gravity ----

  /**
   * The seeds, in Phoenix's units: a Kraken X60's 12 V over its 100 rps, through 60:1, is 7.2 V per
   * arm rotation per second; the feedback gains, per radian in the control, per rotation in
   * Phoenix.
   */
  @Test
  void theSeedsAndTheFeedbackGainsArePerRotation() {
    MechanismControl tuned =
        control(gains(2, 0.5, 0.1, 0.2, Optional.empty(), Optional.empty(), Optional.empty()));

    TalonFXConfiguration leader =
        PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring(11, 12), tuned).leader();
    RobotMath.Gains seed = math.idealGains(ARM);

    assertThat(leader.Slot0.kV).isCloseTo(7.2, within(1e-9));
    assertThat(leader.Slot0.kV).isCloseTo(seed.kV() * TURN, within(1e-12));
    assertThat(leader.Slot0.kA).isCloseTo(seed.kA() * TURN, within(1e-12));
    assertThat(leader.Slot0.kP).isCloseTo(2 * TURN, within(1e-12));
    assertThat(leader.Slot0.kI).isCloseTo(0.5 * TURN, within(1e-12));
    assertThat(leader.Slot0.kD).isCloseTo(0.1 * TURN, within(1e-12));
    assertThat(leader.Slot0.kS).as("volts").isEqualTo(0.2);
  }

  @Test
  void anArmHoldsAgainstOneCosineFromItsDefinition() {
    PhoenixMechanism.Settings arm =
        PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring(11, 12), control());
    RobotMath.Gains seed = math.idealGains(ARM);

    assertThat(arm.leader().Slot0.GravityType).isEqualTo(GravityTypeValue.Arm_Cosine);
    assertThat(arm.leader().Slot0.GravityArmPositionOffset)
        .as("its center of mass on its zero, horizontal")
        .isCloseTo(0, within(1e-12));
    assertThat(arm.leader().Slot0.kG).isCloseTo(seed.kG(), within(1e-12)).isPositive();
    assertThat(arm.notes()).isEmpty();
  }

  @Test
  void tunedGainsWinOverTheSeeds() {
    double idealKv = math.idealGains(ARM).kV();
    MechanismControl tuned =
        control(gains(0, 0, 0, 0, Optional.of(idealKv * 1.1), Optional.of(0.05), Optional.of(0.3)));

    PhoenixMechanism.Settings arm =
        PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring(11, 12), tuned);

    assertThat(arm.leader().Slot0.kV).isCloseTo(idealKv * 1.1 * TURN, within(1e-12));
    assertThat(arm.leader().Slot0.kA).isCloseTo(0.05 * TURN, within(1e-12));
    assertThat(arm.leader().Slot0.kG).isEqualTo(0.3);
    assertThat(arm.notes()).as("each near its ideal").isEmpty();
  }

  /**
   * A kV worked out per turn of the arm and entered as if per turn of its rotor, converted to the
   * control's radians from there: 60 times the ideal, the gearing, named.
   */
  @Test
  void aTunedKvInAnotherFrameIsNamed() {
    double perArmTurn = math.idealGains(ARM).kV() * TURN;
    double enteredPerRotorTurn = perArmTurn * 60 / TURN; // in V/(rad/s), as the control takes it
    MechanismControl tuned =
        control(
            gains(
                0, 0, 0, 0, Optional.of(enteredPerRotorTurn), Optional.empty(), Optional.empty()));

    List<String> notes =
        PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring(11, 12), tuned).notes();

    assertThat(notes)
        .containsExactly(
            "arm's tuned kV, 68.75 V/(rad/s), is 60 times the ideal, 1.146 V/(rad/s), from its"
                + " motors and what it moves: the ideal per turn of arm, within 1%, entered as if"
                + " per turn of its rotor, by the look of it",
            "arm's kV and kA, 432 and 0.08436, are outside what Motion Magic Expo takes (0.001 to"
                + " 100, 0.00001 to 100): Expo is left at Phoenix's defaults, a bare rotor's");
  }

  @Test
  void aTunedKvFarFromTheIdealInNoFrameIsStillNoted() {
    double ideal = math.idealGains(ARM).kV();
    MechanismControl tuned =
        control(gains(0, 0, 0, 0, Optional.of(ideal * 3.7), Optional.empty(), Optional.empty()));

    assertThat(PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring(11, 12), tuned).notes())
        .singleElement()
        .asString()
        .startsWith("arm's tuned kV, 4.24 V/(rad/s), is 3.7 times the ideal")
        .doesNotContain("look of it");
  }

  @Test
  void aTunedKgFarFromTheLinksWeightIsNoted() {
    double ideal = math.idealGains(ARM).kG();
    MechanismControl tuned =
        control(gains(0, 0, 0, 0, Optional.empty(), Optional.empty(), Optional.of(ideal / 3)));

    assertThat(PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring(11, 12), tuned).notes())
        .singleElement()
        .asString()
        .startsWith("arm's tuned kG, ")
        .contains("is 0.3333 times the ideal");
  }

  @Test
  void anElevatorHoldsItsCarriagesWeightEverywhere() {
    TalonFXConfiguration leader =
        PhoenixMechanism.configure(math, ELEVATOR, ELEVATOR_MOTORS, wiring(30, 31), control())
            .leader();
    RobotMath.Gains seed = math.idealGains(ELEVATOR);

    assertThat(leader.Slot0.GravityType).isEqualTo(GravityTypeValue.Elevator_Static);
    assertThat(leader.Slot0.kG).isCloseTo(seed.kG(), within(1e-12)).isPositive();
    assertThat(leader.Slot0.kV)
        .as("per drum rotation")
        .isCloseTo(seed.kV() * TURN * 0.02, within(1e-12));
    assertThat(leader.Slot0.kA).isCloseTo(seed.kA() * TURN * 0.02, within(1e-12));
  }

  @Test
  void aWristOnAnArmIsHeldOnlyWithTheArmAtItsReference() {
    PhoenixMechanism.Settings wrist =
        PhoenixMechanism.configure(math, WRIST, WRIST_MOTOR, wiring(40), control());

    assertThat(wrist.leader().Slot0.GravityType).isEqualTo(GravityTypeValue.Arm_Cosine);
    assertThat(wrist.notes())
        .containsExactly(
            "wrist rides arm, whose turning tilts its gravity: Phoenix's onboard gravity holds it"
                + " only with arm at its reference, so feed RobotMath.gravityTorque at the joints'"
                + " positions as the request's feedforward");
  }

  /**
   * An unweighed arm has no gravity seed, and the helper says so; an unweighed turret or flywheel
   * needs none, since gravity doesn't turn it however much it weighs.
   */
  @Test
  void anUnweighedArmHasNoGravitySeed() {
    Joint.Revolute bare =
        revolute(
            "bare arm",
            Part.CHASSIS,
            new Translation3d(0, 0, 0.5),
            RAISE,
            Optional.of(Range.degrees(0, 90)),
            0,
            unseen("bare link"));
    Drive motor = bare.motors(Motor.KRAKEN_X60, 1, gearing(40));
    RobotMath robot = RobotMath.of(BODY, List.of(bare), List.of(), List.of(motor), List.of());
    PhoenixMechanism.Settings arm =
        PhoenixMechanism.configure(robot, bare, motor, wiring(1), control());

    assertThat(arm.leader().Slot0.kG).isZero();
    assertThat(arm.leader().Slot0.GravityType).isEqualTo(GravityTypeValue.Elevator_Static);
    assertThat(arm.notes())
        .containsExactly(
            "bare link has no mass: bare arm has no gravity seed, kG 0, and no offset for"
                + " Phoenix's Arm_Cosine");
    assertThat(
            PhoenixMechanism.configure(math, TURRET, TURRET_MOTOR, wiring(50), control()).notes())
        .isEmpty();
    assertThat(
            PhoenixMechanism.configure(math, FLYWHEEL, FLYWHEEL_MOTOR, wiring(20), control())
                .notes())
        .isEmpty();
  }

  @Test
  void anUnweighedCarriageHasNoGravitySeed() {
    Joint.Prismatic slide =
        Joint.Prismatic.builder()
            .label("slide")
            .parent(Part.CHASSIS)
            .origin(Transform3d.ZERO)
            .axis(Vector3.UP)
            .range(Optional.of(Range.meters(0, 0.5)))
            .reference(0)
            .link(unseen("slide carriage"))
            .positions(List.of())
            .outputs(List.of())
            .build();
    Drive motor = slide.motors(Motor.KRAKEN_X60, 1, gearing(5), Meters.of(0.02));
    RobotMath robot = RobotMath.of(BODY, List.of(slide), List.of(), List.of(motor), List.of());

    assertThat(PhoenixMechanism.configure(robot, slide, motor, wiring(1), control()).notes())
        .containsExactly("slide carriage has no mass: slide has no gravity seed, kG 0");
  }

  /**
   * A tuned kG on a joint gravity doesn't turn is refused: Arm_Cosine's offset would be a guess,
   * and a turret's kG is nothing at all.
   */
  @Test
  void aTunedKgWhereGravityDoesntTurnTheJointIsRefused() {
    MechanismControl tuned =
        control(gains(0, 0, 0, 0, Optional.empty(), Optional.empty(), Optional.of(0.3)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> PhoenixMechanism.configure(math, TURRET, TURRET_MOTOR, wiring(50), tuned))
        .withMessage(
            "turret has a tuned kG of 0.3 V, but gravity doesn't turn it in the definition (turret"
                + " plate has no mass): Phoenix's Arm_Cosine needs where its weight sits, which"
                + " only its mass gives");
  }

  @Test
  void voltsPhoenixCantHoldAreRefused() {
    MechanismControl tuned =
        control(gains(0, 0, 0, 200, Optional.empty(), Optional.empty(), Optional.empty()));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring(11, 12), tuned))
        .withMessage("arm's kS, 200 V, is past what Phoenix holds, -128 to 127 V");
  }

  /**
   * An arm whose zero points straight up has its gravity offset at Phoenix's limit, a quarter turn:
   * Phoenix takes it, and the helper says how close it is.
   */
  @Test
  void anArmZeroedStraightUpSitsAtPhoenixsLimit() {
    Joint.Revolute upright = upright(0);
    Drive motor = upright.motors(Motor.KRAKEN_X60, 1, gearing(25));
    RobotMath robot = RobotMath.of(BODY, List.of(upright), List.of(), List.of(motor), List.of());

    PhoenixMechanism.Settings settings =
        PhoenixMechanism.configure(robot, upright, motor, wiring(1), control());

    assertThat(settings.leader().Slot0.GravityArmPositionOffset).isCloseTo(0.25, within(1e-12));
    assertThat(settings.notes())
        .containsExactly(
            "upright's gravity offset is 0.25 rotations, at Phoenix's limit of ±0.25: its center of"
                + " mass is 0.0° from straight above or below the pivot at its zero, and a CAD"
                + " change that moves it past makes the offset one Phoenix can't take. Zero"
                + " upright nearer horizontal in CAD");
  }

  /**
   * Its center of mass two degrees forward of straight up, Phoenix can't take it, and the helper
   * says what can.
   */
  @Test
  void anOffsetPastPhoenixsLimitIsRefused() {
    Joint.Revolute upright = upright(Math.toRadians(2));
    Drive motor = upright.motors(Motor.KRAKEN_X60, 1, gearing(25));
    RobotMath robot = RobotMath.of(BODY, List.of(upright), List.of(), List.of(motor), List.of());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> PhoenixMechanism.configure(robot, upright, motor, wiring(1), control()))
        .withMessage(
            "upright's gravity offset is 0.2556 rotations, past Phoenix's ±0.25: its positive"
                + " direction lowers it. Turn its axis around in the definition (and its motors'"
                + " and encoder's directions in the wiring), so that positive raises it; the offset"
                + " is then 0.2444");
  }

  /** An arm zeroed level whose positive direction lowers it: half a turn off. */
  @Test
  void anArmWhosePositiveLowersItIsRefusedWithTheTurnedOffset() {
    Joint.Revolute lowering =
        revolute(
            "lowering",
            Part.CHASSIS,
            new Translation3d(0, 0, 0.5),
            LOWER,
            Optional.of(Range.degrees(-30, 90)),
            0,
            weighed("lowering link", 2, new Translation3d(0.3, 0, 0)));
    Drive motor = lowering.motors(Motor.KRAKEN_X60, 1, gearing(25));
    RobotMath robot = RobotMath.of(BODY, List.of(lowering), List.of(), List.of(motor), List.of());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> PhoenixMechanism.configure(robot, lowering, motor, wiring(1), control()))
        .withMessageContaining("is 0.5 rotations, past Phoenix's ±0.25")
        .withMessageEndingWith("the offset is then 0");
  }

  /**
   * A flap the arm swings at half its angle, weighed: the arm's gravity isn't one cosine, and the
   * helper says by how much.
   */
  @Test
  void gravityOneCosineMissesIsNoted() {
    Joint.Revolute flap =
        revolute(
            "flap",
            Part.CHASSIS,
            new Translation3d(0.1, 0, 0.3),
            RAISE,
            Optional.of(Range.degrees(-15, 60)),
            Math.toRadians(-15),
            weighed("flap plate", 3, new Translation3d(0.4, 0, 0)));
    Coupling linkage =
        Coupling.Linear.builder().leader(ARM).follower(flap).ratio(0.5).offset(0).build();
    RobotMath robot =
        RobotMath.of(BODY, List.of(ARM, flap), List.of(linkage), List.of(ARM_MOTORS), List.of());

    List<String> notes =
        PhoenixMechanism.configure(robot, ARM, ARM_MOTORS, wiring(11, 12), control()).notes();

    assertThat(robot.gravityFit(ARM).misfit()).isGreaterThan(PhoenixMechanism.MISFIT);
    assertThat(notes)
        .singleElement()
        .asString()
        .startsWith("one cosine misses arm's gravity by ")
        .endsWith(
            "Phoenix's Arm_Cosine can't follow it, so feed RobotMath.gravityTorque at the joint's"
                + " angle as the request's feedforward");
  }

  // ---- Motion Magic ----

  @Test
  void aProfileIsInPhoenixsRotationsAndExpoFollowsTheSlot() {
    MechanismControl profiled = control(profile(1.0, 4.0, 40));

    TalonFXConfiguration leader =
        PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wiring(11, 12), profiled).leader();

    assertThat(leader.MotionMagic.MotionMagicCruiseVelocity).isCloseTo(1 / TURN, within(1e-12));
    assertThat(leader.MotionMagic.MotionMagicAcceleration).isCloseTo(4 / TURN, within(1e-12));
    assertThat(leader.MotionMagic.MotionMagicJerk).isCloseTo(40 / TURN, within(1e-12));
    assertThat(leader.MotionMagic.MotionMagicExpo_kV).isEqualTo(leader.Slot0.kV);
    assertThat(leader.MotionMagic.MotionMagicExpo_kA).isEqualTo(leader.Slot0.kA);
  }

  /** The arm's free speed through 60:1 is 1/0.6 of a turn a second: a cruise past it is refused. */
  @Test
  void aCruiseAboveTheFreeSpeedIsRefused() {
    double free = math.freeSpeed(ARM);

    assertThat(free / TURN).isCloseTo(100.0 / 60, within(1e-9));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                PhoenixMechanism.configure(
                    math, ARM, ARM_MOTORS, wiring(11, 12), control(profile(free * 1.5, 4, 0))))
        .withMessage(
            "arm's Motion Magic cruise, 2.5 rot/s, is 1.5 times its free speed, 1.667 rot/s: the"
                + " profile would ask more than its motors turn it at 12 V");
  }

  @Test
  void aProfilePhoenixCantHoldIsRefused() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                PhoenixMechanism.configure(
                    math, ELEVATOR, ELEVATOR_MOTORS, wiring(30, 31), control(profile(1, 2000, 0))))
        .withMessage(
            "elevator's Motion Magic acceleration, 15920, is past the most Phoenix holds, 9999");
  }

  @Test
  void anExpoProfilePhoenixCantTakeIsLeftAtItsDefaultAndNoted() {
    MechanismControl untuned =
        control(gains(0, 0, 0, 0, Optional.of(0.0), Optional.empty(), Optional.empty()));

    PhoenixMechanism.Settings flywheel =
        PhoenixMechanism.configure(math, FLYWHEEL, FLYWHEEL_MOTOR, wiring(20), untuned);

    assertThat(flywheel.leader().MotionMagic.MotionMagicExpo_kV).isEqualTo(0.12);
    assertThat(flywheel.notes())
        .anySatisfy(note -> assertThat(note).startsWith("flywheel's kV and kA, 0 and "));
  }

  // ---- CANcoders ----

  /**
   * A CANcoder on the arm's shaft, read remotely: the TalonFX reads the arm through it, 1:1, and
   * its rotor 60 turns for each of the CANcoder's.
   */
  @Test
  void aRemoteCancoderReadsTheJointAndItsRotorTheGearing() {
    PhoenixMechanism.Settings arm =
        PhoenixMechanism.configure(
            math,
            ARM,
            ARM_MOTORS,
            ARM_CANCODER,
            wiring(cancoder(MechanismWiring.Feedback.REMOTE), 11, 12),
            control());
    TalonFXConfiguration leader = arm.leader();
    CANcoderConfiguration cancoder = arm.cancoder().orElseThrow();

    assertThat(leader.Feedback.FeedbackSensorSource)
        .isEqualTo(FeedbackSensorSourceValue.RemoteCANcoder);
    assertThat(leader.Feedback.FeedbackRemoteSensorID).isEqualTo(9);
    assertThat(leader.Feedback.SensorToMechanismRatio).isEqualTo(1);
    assertThat(leader.Feedback.RotorToSensorRatio).isEqualTo(60);
    assertThat(arm.followers().get(0).configuration().Feedback.SensorToMechanismRatio)
        .as("a follower reads its own rotor")
        .isEqualTo(60);
    assertThat(cancoder.MagnetSensor.MagnetOffset).isEqualTo(-0.125);
    assertThat(cancoder.MagnetSensor.SensorDirection)
        .isEqualTo(SensorDirectionValue.Clockwise_Positive);
    assertThat(cancoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint)
        .as("the middle of -30° to 120°, plus half a turn")
        .isCloseTo(45.0 / 360 + 0.5, within(1e-12));
    assertThat(arm.notes()).isEmpty();
  }

  @Test
  void eachFeedbackIsItsSource() {
    for (var feedback : MechanismWiring.Feedback.values()) {
      TalonFXConfiguration leader =
          PhoenixMechanism.configure(
                  math,
                  ARM,
                  ARM_MOTORS,
                  ARM_CANCODER,
                  wiring(cancoder(feedback), 11, 12),
                  control())
              .leader();
      FeedbackSensorSourceValue expected =
          switch (feedback) {
            case ROTOR -> FeedbackSensorSourceValue.RotorSensor;
            case REMOTE -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FUSED -> FeedbackSensorSourceValue.FusedCANcoder;
            case SYNC -> FeedbackSensorSourceValue.SyncCANcoder;
          };

      assertThat(leader.Feedback.FeedbackSensorSource).as(feedback.name()).isEqualTo(expected);
      assertThat(leader.Feedback.SensorToMechanismRatio)
          .as(feedback.name())
          .isEqualTo(feedback == MechanismWiring.Feedback.ROTOR ? 60 : 1);
    }
  }

  /**
   * A CANcoder geared to turn back as the turret turns forward reads forward once its wiring says
   * it turns the other way: its ratio is a magnitude, and its wrap is the definition's turned
   * round.
   */
  @Test
  void aCancoderGearedBackwardsWrapsWhereTheDefinitionsTurnedRoundDoes() {
    double definitions = math.discontinuityPoint(TURRET_CANCODER);
    PhoenixMechanism.Settings turret =
        PhoenixMechanism.configure(
            math,
            TURRET,
            TURRET_MOTOR,
            TURRET_CANCODER,
            wiring(cancoder(MechanismWiring.Feedback.FUSED), 50),
            control());

    assertThat(turret.leader().Feedback.SensorToMechanismRatio).isEqualTo(1);
    assertThat(turret.leader().Feedback.RotorToSensorRatio).isEqualTo(50);
    assertThat(turret.cancoder().orElseThrow().MagnetSensor.AbsoluteSensorDiscontinuityPoint)
        .isCloseTo(1 - definitions, within(1e-12));
  }

  /** The turret's 340° leave 10° either side of the wrap: closer than 9° would be noted. */
  @Test
  void aRangeNearlyATurnLeavesLittleMargin() {
    double point =
        PhoenixMechanism.cancoder(math, TURRET_CANCODER, cancoder(MechanismWiring.Feedback.REMOTE))
            .MagnetSensor
            .AbsoluteSensorDiscontinuityPoint;

    assertThat(PhoenixMechanism.wrapMargin(TURRET_CANCODER, point))
        .isCloseTo(10.0 / 360, within(1e-12));
    assertThat(PhoenixMechanism.wrapProblem(TURRET_CANCODER, point)).isNull();
    assertThat(PhoenixMechanism.wrapProblem(TURRET_CANCODER, point + 0.01))
        .isEqualTo(
            "turret cancoder wraps at 0.5378 of its turn, 6.4° from turret's range (-160.0° to"
                + " 180.0°), and 166.4° from where it stows (0.0°): a reading a hair past the"
                + " wrap reads a turn away");
  }

  /** A team's own wrap, across the range: where the arm reads a turn away inside its travel. */
  @Test
  void aWrapInsideTheRangeIsNamed() {
    assertThat(PhoenixMechanism.wrapMargin(ARM_CANCODER, 1.0)).isNegative();
    assertThat(PhoenixMechanism.wrapProblem(ARM_CANCODER, 1.0))
        .startsWith("arm cancoder wraps at 1 of its turn, inside arm's range");
    assertThat(PhoenixMechanism.wrapMargin(encoder("flywheel encoder", FLYWHEEL, 1), 0.5))
        .as("nothing to keep clear")
        .isInfinite();
  }

  @Test
  void aCancoderMustBeWiredAndAbsolute() {
    Sensor.Encoder relative =
        Sensor.Encoder.builder()
            .label("arm quadrature")
            .terms(List.of(term(ARM, 1)))
            .absolute(false)
            .countsPerTurn(2048)
            .build();
    MechanismWiring wired = wiring(cancoder(MechanismWiring.Feedback.REMOTE), 11, 12);

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                PhoenixMechanism.configure(
                    math, ARM, ARM_MOTORS, ARM_CANCODER, wiring(11, 12), control()))
        .withMessage("arm's wiring names no CANcoder for arm cancoder: wire it");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PhoenixMechanism.configure(math, ARM, ARM_MOTORS, wired, control()))
        .withMessage(
            "arm's wiring names a CANcoder: give the definition's encoder it is, or wire none");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> PhoenixMechanism.configure(math, ARM, ARM_MOTORS, relative, wired, control()))
        .withMessage("arm quadrature must be an absolute encoder on arm alone, as a CANcoder is");
  }

  // ---- Coupling and differentials ----

  /** A differential wrist: pitch both motors turn together, roll they turn against each other. */
  private static final Joint.Revolute PITCH =
      revolute(
          "pitch",
          Part.CHASSIS,
          new Translation3d(0.2, 0, 0.6),
          RAISE,
          Optional.of(Range.degrees(-90, 90)),
          0,
          unseen("pitch yoke"));

  private static final Joint.Revolute ROLL =
      revolute(
          "roll",
          PITCH,
          Translation3d.ZERO,
          new Vector3(1, 0, 0),
          Optional.of(Range.degrees(-180, 180)),
          0,
          unseen("roll head"));

  private static final Drive LEFT = side("left", 20, 12);
  private static final Drive RIGHT = side("right", 20, -12);

  private final RobotMath wrist =
      RobotMath.of(BODY, List.of(PITCH, ROLL), List.of(), List.of(LEFT, RIGHT), List.of());

  @Test
  void aMotorTurningTwoJointsSaysPhoenixReadsBoth() {
    assertThat(PhoenixMechanism.configure(wrist, PITCH, LEFT, wiring(1), control()).notes().get(0))
        .isEqualTo(
            "left's rotor turns 12 times for each turn of roll too, which Phoenix reads as pitch"
                + " moving: take it out in code, as a swerve module's coupling is");
  }

  @Test
  void aCanivoreIsPhoenixsBusByName() {
    assertThat(PhoenixMechanism.bus(CanBus.canivore("arm bus")).getName()).isEqualTo("arm bus");
    assertThat(PhoenixMechanism.bus(CanBus.port(CANPort.CAN_S1)).getName()).isEqualTo("can_s1");
  }

  // ---- What the tests are built from ----

  private static Joint.Revolute revolute(
      String label,
      Part parent,
      Translation3d at,
      Vector3 axis,
      Optional<Range> range,
      double reference,
      Link link) {
    return Joint.Revolute.builder()
        .label(label)
        .parent(parent)
        .origin(new Transform3d(at, Rotation3d.ZERO))
        .axis(axis)
        .range(range)
        .reference(reference)
        .link(link)
        .positions(List.of())
        .outputs(List.of())
        .build();
  }

  /**
   * An arm turning forward and down from straight up, its center of mass on its zero, or tipped
   * forward of it by an angle.
   */
  private static Joint.Revolute upright(double tipped) {
    return revolute(
        "upright",
        Part.CHASSIS,
        new Translation3d(0.2, 0, 0.3),
        LOWER,
        Optional.of(Range.degrees(0, 135)),
        Math.toRadians(1.5),
        weighed(
            "upright link",
            2,
            new Translation3d(0.3 * Math.sin(tipped), 0, 0.3 * Math.cos(tipped))));
  }

  private static Link weighed(String label, double kilograms, Translation3d cg) {
    return Link.builder()
        .label(label)
        .mass(
            MassProperties.builder()
                .mass(Kilograms.of(kilograms))
                .cg(cg)
                .inertia(Inertia.box(Kilograms.of(kilograms), new Translation3d(0.1, 0.1, 0.1)))
                .build())
        .shapes(List.of())
        .build();
  }

  private static Link unseen(String label) {
    return Link.builder().label(label).mass(MassProperties.NONE).shapes(List.of()).build();
  }

  private static Gearing gearing(double ratio) {
    return Gearing.builder().ratio(ratio).efficiency(1).build();
  }

  private static Transmission.Term term(Joint joint, double ratio) {
    return Transmission.Term.builder()
        .joint(joint)
        .ratio(ratio)
        .pitchRadius(Optional.empty())
        .build();
  }

  private static Sensor.Encoder encoder(String label, Joint joint, double ratio) {
    return Sensor.Encoder.builder()
        .label(label)
        .terms(List.of(term(joint, ratio)))
        .absolute(true)
        .countsPerTurn(4096)
        .build();
  }

  /** One side of the wrist: a Kraken X44 turning pitch and roll. */
  private static Drive side(String label, double pitch, double roll) {
    return Drive.builder()
        .label(label)
        .actuator(Actuator.Motors.builder().motor(Motor.KRAKEN_X44).count(1).build())
        .transmission(
            Transmission.Combined.builder()
                .terms(List.of(term(PITCH, pitch), term(ROLL, roll)))
                .efficiency(1)
                .build())
        .build();
  }

  private static MechanismWiring.Motor motor(int id, boolean clockwise) {
    return MechanismWiring.Motor.builder().id(id).clockwise(clockwise).build();
  }

  private static MechanismWiring wiring(int... ids) {
    List<MechanismWiring.Motor> motors = new ArrayList<>();
    for (int id : ids) {
      motors.add(motor(id, false));
    }
    return wiring(motors.toArray(MechanismWiring.Motor[]::new));
  }

  private static MechanismWiring wiring(MechanismWiring.Motor... motors) {
    return MechanismWiring.builder()
        .canBus(CanBus.port(CANPort.CAN_S0))
        .controller(MechanismWiring.Controller.TALON_FX)
        .motors(List.of(motors))
        .cancoder(Optional.empty())
        .dataPort(Optional.empty())
        .build();
  }

  private static MechanismWiring wiring(MechanismWiring.Cancoder cancoder, int... ids) {
    return MechanismWiringBuilder.builder(wiring(ids)).cancoder(Optional.of(cancoder)).build();
  }

  private static MechanismWiring.Cancoder cancoder(MechanismWiring.Feedback feedback) {
    Angle offset = Rotations.of(-0.125);
    return MechanismWiring.Cancoder.builder()
        .id(9)
        .offset(offset)
        .clockwise(true)
        .feedback(feedback)
        .build();
  }

  private static MechanismControl.Gains gains(
      double kP,
      double kI,
      double kD,
      double kS,
      Optional<Double> kV,
      Optional<Double> kA,
      Optional<Double> kG) {
    return MechanismControl.Gains.builder()
        .kP(kP)
        .kI(kI)
        .kD(kD)
        .kS(kS)
        .kV(kV)
        .kA(kA)
        .kG(kG)
        .build();
  }

  private static MechanismControl.Gains seeded() {
    return MechanismControl.Gains.seeded();
  }

  private static MechanismControl.SoftLimits soft(double lower, double upper) {
    return MechanismControl.SoftLimits.builder().lower(lower).upper(upper).build();
  }

  private static MechanismControl.Profile profile(double cruise, double acceleration, double jerk) {
    return MechanismControl.Profile.builder()
        .cruiseVelocity(cruise)
        .acceleration(acceleration)
        .jerk(jerk)
        .build();
  }

  private static MechanismControl control() {
    return control(
        seeded(), Optional.empty(), Optional.empty(), MechanismControl.NeutralMode.BRAKE);
  }

  private static MechanismControl coasting() {
    return control(
        seeded(), Optional.empty(), Optional.empty(), MechanismControl.NeutralMode.COAST);
  }

  private static MechanismControl control(MechanismControl.Gains gains) {
    return control(gains, Optional.empty(), Optional.empty(), MechanismControl.NeutralMode.BRAKE);
  }

  private static MechanismControl control(MechanismControl.SoftLimits limits) {
    return control(
        seeded(), Optional.of(limits), Optional.empty(), MechanismControl.NeutralMode.BRAKE);
  }

  private static MechanismControl control(MechanismControl.Profile profile) {
    return control(
        seeded(), Optional.empty(), Optional.of(profile), MechanismControl.NeutralMode.BRAKE);
  }

  private static MechanismControl control(
      MechanismControl.Gains gains,
      Optional<MechanismControl.SoftLimits> limits,
      Optional<MechanismControl.Profile> profile,
      MechanismControl.NeutralMode neutral) {
    return MechanismControl.builder()
        .gains(gains)
        .softLimits(limits)
        .profile(profile)
        .currentLimits(
            MechanismControl.CurrentLimits.builder()
                .stator(Amps.of(60))
                .supply(Amps.of(50))
                .supplyLower(Amps.of(35))
                .supplyLowerTime(Seconds.of(0.5))
                .build())
        .neutralMode(neutral)
        .bands(Optional.empty())
        .homing(Optional.empty())
        .build();
  }
}
