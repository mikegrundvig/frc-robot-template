package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.PITCH_UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.definition;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.encoder;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.gearing;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.link;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.motors;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.point;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.prismatic;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.revolute;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * How the program drives a joint, worked out once: its gains tuned where given and seeded where
 * not, gravity's kind and where it pulls hardest, the one feedforward law, its followers'
 * directions, and what no motor controller could do or what's doubtful. An arm of 4 kg 0.3 m out on
 * two Krakens through 60:1, a wrist on it, an elevator, a turret.
 */
class ControllerPlanTest {
  private static final Joint.Revolute ARM =
      revolute(
          "arm",
          Part.CHASSIS,
          point(0.1, 0, 0.5),
          PITCH_UP,
          Optional.of(Range.degrees(-30, 120)),
          Math.toRadians(-30),
          link("arm link", 4, point(0.3, 0, 0), point(0.6, 0.05, 0.05)));

  private static final Joint.Revolute WRIST =
      revolute(
          "wrist",
          ARM,
          point(0.6, 0, 0),
          PITCH_UP,
          Optional.of(Range.degrees(-90, 90)),
          0,
          link("claw", 1, point(0.1, 0, 0), point(0.2, 0.05, 0.05)));

  private static final Joint.Prismatic ELEVATOR =
      prismatic(
          "elevator",
          Part.CHASSIS,
          point(-0.2, 0, 0.1),
          UP,
          Range.meters(0, 1.2),
          link("carriage", 5, point(0, 0, 0), point(0.3, 0.3, 0.1)));

  private static final Joint.Revolute TURRET =
      revolute(
          "turret",
          Part.CHASSIS,
          point(0, 0, 0.3),
          UP,
          Optional.of(Range.degrees(-160, 180)),
          0,
          link("turret plate", 3, point(0, 0, 0), point(0.3, 0.3, 0.02)));

  private static final Drive ARM_MOTORS = motors(ARM, Motor.KRAKEN_X60, 2, gearing(60, 1));
  private static final Drive WRIST_MOTOR = motors(WRIST, Motor.KRAKEN_X44, 1, gearing(30, 1));
  private static final Drive ELEVATOR_MOTORS =
      motors(ELEVATOR, Motor.KRAKEN_X60, 2, gearing(10, 1), Meters.of(0.02));
  private static final Drive TURRET_MOTOR = motors(TURRET, Motor.KRAKEN_X60, 1, gearing(50, 1));

  private static final Sensor.Encoder ARM_ENCODER = encoder("arm encoder", ARM, 1, true);

  private static final RobotMath MATH =
      definition(
              List.of(ARM, WRIST, ELEVATOR, TURRET),
              List.of(),
              List.of(ARM_MOTORS, WRIST_MOTOR, ELEVATOR_MOTORS, TURRET_MOTOR),
              List.of(ARM_ENCODER))
          .math();

  /** Two TalonFXs, the second turned the other way. */
  private static final List<MechanismWiring.Device> PAIR =
      List.of(
          MechanismWiring.Controller.TALON_FX.can(11),
          MechanismWiring.Controller.TALON_FX.can(12).clockwise());

  private static ControllerPlan arm(MechanismControl control) {
    return ControllerPlan.of(MATH, ARM, ARM_MOTORS, PAIR, control);
  }

  /** Friction in the direction of travel, kV at the speed, kA at the acceleration, and gravity. */
  @Test
  void itsFeedforwardIsFrictionSpeedAccelerationAndGravity() {
    ControllerPlan plan =
        arm(
            MechanismControl.position(30)
                .withKs(0.1)
                .withKv(2)
                .withKa(0.05)
                .withKg(0.5, Degrees.of(0)));

    assertThat(plan.feedforward(0, 0, 0)).as("level: all of kG").isCloseTo(0.5, within(1e-12));
    assertThat(plan.feedforward(Math.PI / 2, 0, 0)).as("straight up").isCloseTo(0, within(1e-12));
    assertThat(plan.feedforward(0, 1, 2)).isCloseTo(0.1 + 2 + 0.1 + 0.5, within(1e-12));
    assertThat(plan.feedforward(0, -1, 0)).isCloseTo(-0.1 - 2 + 0.5, within(1e-12));
    assertThat(plan.kP()).isEqualTo(30);
  }

  /** What isn't tuned is the definition's seed, from its motors, gearing and mass. */
  @Test
  void untunedGainsAreTheDefinitionsSeeds() {
    RobotMath.Gains seed = MATH.idealGains(ARM);
    ControllerPlan seeded = arm(MechanismControl.position(30));
    ControllerPlan tuned = arm(MechanismControl.position(30).withKv(seed.kV() * 1.1).withKa(0.05));

    assertThat(seeded.kV()).isEqualTo(seed.kV());
    assertThat(seeded.kA()).isEqualTo(seed.kA());
    assertThat(seeded.gravity().kG()).isEqualTo(seed.kG()).isPositive();
    assertThat(seeded.gravity().kind()).isEqualTo(ControllerPlan.Gravity.Kind.COSINE);
    assertThat(seeded.gravity().offset()).as("its mass on its zero").isCloseTo(0, within(1e-12));
    assertThat(seeded.notes()).isEmpty();
    assertThat(tuned.kV()).isEqualTo(seed.kV() * 1.1);
    assertThat(tuned.kA()).isEqualTo(0.05);
    assertThat(tuned.notes()).as("near its ideal").isEmpty();
  }

  /** An arm's tuned kG pulls hardest at the angle given with it, along one cosine. */
  @Test
  void aGivenGravityAnglePullsHardest() {
    ControllerPlan plan = arm(MechanismControl.position(30).withKg(0.4, Degrees.of(-30)));

    assertThat(plan.gravity().offset()).isCloseTo(Math.toRadians(30), within(1e-12));
    assertThat(plan.feedforward(Math.toRadians(-30), 0, 0)).isCloseTo(0.4, within(1e-12));
    assertThat(plan.feedforward(Math.toRadians(60), 0, 0)).isCloseTo(0, within(1e-12));
  }

  /** A slide's gravity is the same all along; a turret's is none. */
  @Test
  void aSlidesGravityIsSteadyAndATurretsNone() {
    ControllerPlan elevator =
        ControllerPlan.of(
            MATH, ELEVATOR, ELEVATOR_MOTORS, List.of(), MechanismControl.position(60).withKg(0.9));
    ControllerPlan turret =
        ControllerPlan.of(MATH, TURRET, TURRET_MOTOR, List.of(), MechanismControl.position(20));

    assertThat(elevator.gravity().kind()).isEqualTo(ControllerPlan.Gravity.Kind.STEADY);
    assertThat(elevator.feedforward(0.1, 0, 0)).isEqualTo(0.9);
    assertThat(elevator.feedforward(1.1, 0, 0)).isEqualTo(0.9);
    assertThat(elevator.rotations().perRotation()).isCloseTo(2 * Math.PI * 0.02, within(1e-15));
    assertThat(elevator.rotorRatio()).isCloseTo(10 / (2 * Math.PI * 0.02), within(1e-9));
    assertThat(elevator.notes())
        .singleElement()
        .asString()
        .startsWith("elevator's tuned kG, 0.9 V, is ");
    assertThat(turret.gravity().kind()).isEqualTo(ControllerPlan.Gravity.Kind.NONE);
    assertThat(turret.feedforward(1, 0, 0)).isZero();
    assertThat(turret.turnsFreely()).as("it has a range").isFalse();
  }

  /**
   * A gain entered in another frame names the frames: per turn of the arm, as if per rotor turn.
   */
  @Test
  void aTunedGainInAnotherFrameIsNamed() {
    double perArmTurn = MATH.idealGains(ARM).kV() * 2 * Math.PI;

    assertThat(arm(MechanismControl.position(30).withKv(perArmTurn * 60 / (2 * Math.PI))).notes())
        .singleElement()
        .asString()
        .endsWith(
            ": the ideal per turn of arm, within 1%, entered as if per turn of its rotor, by the"
                + " look of it");
    assertThat(arm(MechanismControl.position(30).withKg(MATH.idealGains(ARM).kG() / 3)).notes())
        .singleElement()
        .asString()
        .contains("is 0.3333 times the ideal");
  }

  /** A wrist on an arm: its gravity tilts as the arm turns, which one cosine can't follow. */
  @Test
  void aWristOnATurningArmIsNoted() {
    ControllerPlan wrist =
        ControllerPlan.of(MATH, WRIST, WRIST_MOTOR, List.of(), MechanismControl.position(10));

    assertThat(wrist.notes())
        .containsExactly(
            "wrist rides arm, whose turning tilts its gravity: a controller's gravity term holds it"
                + " only with arm at rest, so feed RobotMath.gravityTorque at the joints'"
                + " positions as the request's feedforward");
  }

  /** A follower turned the other way from its leader opposes it; one turned alike doesn't. */
  @Test
  void itsFollowersDirectionsAreTheWirings() {
    ControllerPlan plan = arm(MechanismControl.position(30));

    assertThat(plan.controllers()).isEqualTo(PAIR);
    assertThat(plan.motors()).isEqualTo(2);
    assertThat(plan.opposes(PAIR.get(1))).isTrue();
    assertThat(plan.opposes(MechanismWiring.Controller.TALON_FX.can(13))).isFalse();
    assertThat(plan.brakes()).isTrue();
    assertThat(plan.softLimits()).isEmpty();
    assertThat(plan.currentLimits()).isEqualTo(MechanismControl.CurrentLimits.DEFAULTS);
    assertThat(plan.joint()).isEqualTo(ARM);
    assertThat(plan.drive()).isEqualTo(ARM_MOTORS);
  }

  /** What no motor controller could do is refused, saying what. */
  @Test
  void itRefusesWhatNoMotorControllerCouldDo() {
    MechanismControl control = MechanismControl.position(30);

    assertThatIllegalArgumentException()
        .isThrownBy(() -> ControllerPlan.of(MATH, ARM, ARM_MOTORS, PAIR.subList(0, 1), control))
        .withMessage("arm is 2 motors, and its wiring names 1");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ControllerPlan.of(MATH, WRIST, ARM_MOTORS, PAIR, control))
        .withMessage("arm doesn't move wrist");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> ControllerPlan.of(MATH, TURRET, TURRET_MOTOR, List.of(), control.withKg(0.3)))
        .withMessage(
            "turret has a tuned kG of 0.3 V, but gravity can't move it: it turns about the"
                + " vertical. Leave its kG out, or write 0");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ControllerPlan.of(MATH, ARM))
        .withMessage("arm has no control");
  }

  /**
   * An encoder on the arm's shaft wraps half a turn from the middle of its range, as its motor
   * controller reads it: clear of it, and doubted when it wraps inside it.
   */
  @Test
  void anAbsoluteEncoderWrapsClearOfItsJointsRange() {
    double point = ControllerPlan.wrapPoint(MATH, ARM_ENCODER);

    assertThat(point).isCloseTo(45.0 / 360 + 0.5, within(1e-12));
    assertThat(ControllerPlan.wrapMargin(ARM_ENCODER, point))
        .as("75° each way, less the range's half")
        .isCloseTo(105.0 / 360, within(1e-12));
    assertThat(ControllerPlan.wrapProblem(ARM_ENCODER, point)).isNull();
    assertThat(ControllerPlan.wrapProblem(ARM_ENCODER, 1.0))
        .startsWith("arm encoder wraps at 1 of its turn, inside arm's range (-30° to 120°)");
  }
}
