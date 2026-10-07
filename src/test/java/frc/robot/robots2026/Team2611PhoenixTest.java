package frc.robot.robots2026;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static frc.robot.robots2026.Parts.CANCODER_COUNTS;
import static frc.robot.robots2026.PhoenixParts.configure;
import static frc.robot.robots2026.PhoenixParts.control;
import static frc.robot.robots2026.PhoenixParts.corner;
import static frc.robot.robots2026.PhoenixParts.gains;
import static frc.robot.robots2026.PhoenixParts.motor;
import static frc.robot.robots2026.PhoenixParts.seeded;
import static frc.robot.robots2026.PhoenixParts.units;
import static frc.robot.robots2026.PhoenixParts.wiring;
import static frc.robot.robots2026.Team2611.BODY;
import static frc.robot.robots2026.Team2611.DRIVETRAIN;
import static frc.robot.robots2026.Team2611.PIVOT;
import static frc.robot.robots2026.Team2611.ROBOT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.lib.phoenix.PhoenixMechanism;
import frc.robot.subsystems.mechanisms.JointFixtures;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/**
 * The helper's Phoenix settings for 2611's 2026 robot ({@link Team2611}) against what its code
 * configures. Citations are to {@code ~/src/2611-Vectors/2026-Beta-Bot} at 6a5da76, under {@code
 * src/main/java/frc/robot/}.
 *
 * <p>Every ratio here is the definition's gearing, a {@code double}, never typed again: 2611's 2025
 * elevator wrote its ratio as {@code 34 / 12}, integer division, so 2 (2025-Beta-Bot's
 * Constants.java:81), the kind of slip the helper leaves no place for.
 */
class Team2611PhoenixTest {
  private final RobotMath math = ROBOT.math();

  /**
   * Each TalonFX reads its joint through its drive's gearing. 2611 sets no ratio, so Phoenix reads
   * rotor turns and the code divides by the ratio (Pivot.java:83, Intake.java:43, FullSend.java:42,
   * Transition.java:72): its Constants.java writes each output over motor (1.0 / 25, :85).
   */
  @Test
  void eachMotorReadsItsJointThroughTheRatioItsCodeDividesBy() {
    record Mechanism(Joint joint, Drive drive, double theirs) {}
    for (Mechanism each :
        List.of(
            new Mechanism(PIVOT, motors(Team2611.PIVOT), 1 / (1.0 / 25.0)),
            new Mechanism(Team2611.INTAKE, motors(Team2611.INTAKE), 1 / (1.0 / 3.0)),
            new Mechanism(Team2611.FULL_SEND, motors(Team2611.FULL_SEND), 1 / (1.0 / 4.0)),
            new Mechanism(Team2611.TRANSITION, motors(Team2611.TRANSITION), 1 / (1.0 / 2.0)),
            new Mechanism(Team2611.SHOOTER, motors(Team2611.SHOOTER), 1))) {
      assertThat(seeded(math, each.joint(), each.drive()).leader().Feedback.SensorToMechanismRatio)
          .as(each.joint().label())
          .isCloseTo(each.theirs(), within(1e-12));
    }
  }

  /**
   * The velocity loops' tuned kV, per rotor rotation per second (Shooter.java:29, Intake.java:24,
   * FullSend.java:24, Transition.java:26), in the helper's units: the same number times the
   * gearing, per rotation of the joint, and each near the ideal, so nothing to note.
   */
  @Test
  void theirVelocityKvsAreTheirsPerTurnOfTheJoint() {
    record Loop(Joint joint, Drive drive, double kV, double ratio) {}
    for (Loop loop :
        List.of(
            new Loop(Team2611.SHOOTER, motors(Team2611.SHOOTER), 0.12, 1),
            new Loop(Team2611.INTAKE, motors(Team2611.INTAKE), 0.13, 3),
            new Loop(Team2611.FULL_SEND, motors(Team2611.FULL_SEND), 0.13, 4),
            new Loop(Team2611.TRANSITION, motors(Team2611.TRANSITION), 0.14, 2))) {
      PhoenixMechanism.Settings settings =
          configure(
              math,
              loop.joint(),
              loop.drive(),
              wiring(loop.drive()),
              control(
                  gains(
                      units(loop.joint(), loop.drive()),
                      0,
                      0,
                      loop.kV(),
                      Double.NaN,
                      loop.ratio())));

      assertThat(settings.leader().Slot0.kV)
          .as(loop.joint().label())
          .isCloseTo(loop.kV() * loop.ratio(), within(1e-12));
      assertThat(settings.notes()).as(loop.joint().label()).isEmpty();
    }
  }

  /**
   * The shooter's four motors (IDs 41 to 44, Constants.java:51-55): the first leads,
   * counterclockwise (Shooter.java:38); the second left one follows aligned, the right ones opposed
   * (Shooter.java:34-36). Wired as each turns, the helper aligns them so.
   */
  @Test
  void theShootersFollowersAreAlignedAsTheirs() {
    List<MechanismWiring.Device> wired =
        wiring(motor(41, false), motor(42, false), motor(43, true), motor(44, true));

    PhoenixMechanism.Settings shooter =
        configure(math, Team2611.SHOOTER, motors(Team2611.SHOOTER), wired, control());

    assertThat(shooter.leader().MotorOutput.Inverted)
        .isEqualTo(InvertedValue.CounterClockwise_Positive);
    assertThat(shooter.followers())
        .extracting(follower -> follower.request().MotorAlignment)
        .containsExactly(
            MotorAlignmentValue.Aligned, MotorAlignmentValue.Opposed, MotorAlignmentValue.Opposed);
    assertThat(shooter.followers()).allMatch(follower -> follower.request().LeaderID == 41);
  }

  /**
   * The pivot's absolute encoder wraps where 2611's code wraps it, at its reading's 0, the pivot's
   * 0° (VectorKit/hardware/AbsoluteEncoder.java:233-235): 1.5° from where it stows
   * (Constants.java:89), and at the range's end. The helper flags it, and its own point, the
   * definition's, leaves a third of a turn either side. Both judge a point by one margin, the
   * definition's fortieth of a turn ({@code RobotMath.WRAP_MARGIN}), which theirs, on the range's
   * edge, is inside.
   */
  @Test
  void thePivotsEncoderWrapIsFlaggedBesideItsStow() {
    Sensor.Encoder encoder = absolute(Team2611.PIVOT);
    double ours =
        PhoenixMechanism.cancoder(
                math, encoder, CANCODER.can(Team2611.CANIVORE, 1).offset(Rotations.of(0)))
            .MagnetSensor
            .AbsoluteSensorDiscontinuityPoint;

    assertThat(ControllerPlan.wrapProblem(encoder, 1.0))
        .isEqualTo(
            "pivot absolute encoder wraps at 1 of its turn, 0° from pivot's range (0° to 135°), and"
                + " 1.5° from where it stows (1.5°): a reading a hair past the wrap reads a turn"
                + " away");
    assertThat(ControllerPlan.wrapMargin(encoder, 1.0)).isCloseTo(0, within(1e-12));
    assertThat(ours).isCloseTo(0.6875, within(1e-12));
    assertThat(ControllerPlan.wrapProblem(encoder, ours)).isNull();
    assertThat(ControllerPlan.wrapMargin(encoder, ours)).isCloseTo(0.3125, within(1e-12));
  }

  /**
   * The pivot as 2611 drives it: a Kraken X60 turning clockwise to swing it out (Pivot.java:30-32),
   * braking; its arm unweighed and its kG not written, as 2611's has none: kG 0, about its zero.
   */
  @Test
  void thePivotHasNoGravitySeedUnweighed() {
    PhoenixMechanism.Settings pivot =
        configure(math, PIVOT, motors(Team2611.PIVOT), wiring(motor(32, true)), control());

    assertThat(pivot.leader().MotorOutput.Inverted).isEqualTo(InvertedValue.Clockwise_Positive);
    assertThat(pivot.leader().Slot0.kG).isZero();
    assertThat(pivot.notes()).isEmpty();
  }

  /**
   * Weighed, with its center of mass anywhere along its zero, which the fixture draws straight up,
   * the pivot's gravity offset is a quarter turn, whatever it weighs: exactly Phoenix's limit. The
   * helper takes it and flags it, since a center of mass a hair forward of that is an offset
   * Phoenix can't hold. The masses are no guesses of 2611's: any gives the same offset.
   */
  @Test
  void weighedItsGravityOffsetWouldSitAtPhoenixsLimit() {
    for (double kilograms : new double[] {1, 6}) {
      Joint.Revolute weighed =
          JointRevoluteBuilder.builder(PIVOT)
              .link(
                  Link.builder()
                      .label("intake arm")
                      .mass(
                          MassProperties.builder()
                              .mass(Kilograms.of(kilograms))
                              .cg(new Translation3d(0, 0, 0.2))
                              .inertia(
                                  Inertia.box(
                                      Kilograms.of(kilograms), new Translation3d(0.05, 0.55, 0.36)))
                              .build())
                      .build())
              .build();
      Drive motor = JointFixtures.motors(weighed, Motor.KRAKEN_X60, 1, Gearing.of(25));
      RobotMath alone = RobotMath.of(BODY, List.of(weighed), List.of(), List.of(motor), List.of());

      PhoenixMechanism.Settings pivot =
          configure(alone, weighed, motor, wiring(motor(32, true)), control());

      assertThat(pivot.leader().Slot0.GravityType).isEqualTo(GravityTypeValue.Arm_Cosine);
      assertThat(pivot.leader().Slot0.GravityArmPositionOffset)
          .as(kilograms + " kg")
          .isCloseTo(0.25, within(1e-12));
      assertThat(pivot.notes())
          .singleElement()
          .asString()
          .startsWith("pivot's gravity offset is 0.25 rotations, at Phoenix's limit of ±0.25");
    }
  }

  /**
   * The swerve as ModuleIOTalonFX configures it from Tuner's constants: the drive's
   * SensorToMechanismRatio its gearing (subsystems/drive/ModuleIOTalonFX.java:100), with the
   * coupling taken out in code; the steering fused with its CANcoder
   * (generated/TunerConstants.java: 50), its RotorToSensorRatio the steering's gearing
   * (ModuleIOTalonFX.java:122), wrapping the short way (:126), the front left's CANcoder offset
   * 0.3779 and counterclockwise (TunerConstants.java:131-133), its discontinuity a fresh
   * configuration's. Tuner's kV, 0.72373 per wheel rotation per second for the drive and 2.49 per
   * module rotation per second for the steering (TunerConstants.java:28, 34), are each the ideal's,
   * near enough.
   */
  @Test
  void theSwerveModulesAreTunersOwn() {
    DrivetrainGeometry.ModuleJoints frontLeft = DRIVETRAIN.joints().get(0);
    Sensor.Encoder cancoder =
        Sensor.Encoder.absolute("front-left cancoder", frontLeft.steer(), CANCODER_COUNTS);
    RobotMath corner = corner(BODY, frontLeft, cancoder);
    double driveGearing = Team2611.MODULES.driveGearing().ratio();
    double steerGearing = Team2611.MODULES.steerGearing().ratio();

    PhoenixMechanism.Settings drive =
        configure(
            corner,
            frontLeft.wheel(),
            frontLeft.drive(),
            wiring(motor(1, false)),
            control(
                gains(
                    units(frontLeft.wheel(), frontLeft.drive()), 0.1, 0, 0.72373, Double.NaN, 1)));
    PhoenixMechanism.Settings steer =
        configure(
            corner,
            frontLeft.steer(),
            frontLeft.steering(),
            cancoder,
            CANCODER.can(Team2611.CANIVORE, 3).offset(Rotations.of(0.3779296875)).fused(),
            wiring(motor(2, false)),
            control(
                gains(
                    units(frontLeft.steer(), frontLeft.steering()),
                    100,
                    0.5,
                    2.49,
                    Double.NaN,
                    1)));

    assertThat(drive.leader().Feedback.SensorToMechanismRatio).isEqualTo(driveGearing);
    assertThat(drive.leader().Slot0.kV).isCloseTo(0.72373, within(1e-12));
    assertThat(drive.notes())
        .containsExactly(
            "front-left drive's rotor turns 3.857 times for each turn of front-left steer too,"
                + " which the controller reads as front-left wheel moving: take it out in code,"
                + " as a swerve module's coupling is");
    assertThat(steer.leader().Feedback.FeedbackSensorSource)
        .isEqualTo(FeedbackSensorSourceValue.FusedCANcoder);
    assertThat(steer.leader().Feedback.FeedbackRemoteSensorID).isEqualTo(3);
    assertThat(steer.leader().Feedback.SensorToMechanismRatio).isEqualTo(1);
    assertThat(steer.leader().Feedback.RotorToSensorRatio).isCloseTo(steerGearing, within(1e-12));
    assertThat(steer.leader().ClosedLoopGeneral.ContinuousWrap).isTrue();
    assertThat(steer.leader().Slot0.kP).isCloseTo(100, within(1e-12));
    assertThat(steer.cancoder().orElseThrow().MagnetSensor.MagnetOffset).isEqualTo(0.3779296875);
    assertThat(steer.cancoder().orElseThrow().MagnetSensor.SensorDirection)
        .isEqualTo(SensorDirectionValue.CounterClockwise_Positive);
    assertThat(steer.cancoder().orElseThrow().MagnetSensor.AbsoluteSensorDiscontinuityPoint)
        .isEqualTo(0.5);
    assertThat(steer.notes()).isEmpty();
  }

  /** A joint's own motors, from its block. */
  private static Drive motors(Joint joint) {
    return joint.ownDrives().getFirst();
  }

  /** A joint's own absolute encoder, from its block. */
  private static Sensor.Encoder absolute(Joint joint) {
    return (Sensor.Encoder) joint.ownSensors().getFirst();
  }
}
