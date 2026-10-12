package frc.robot.robots2026;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static frc.robot.DeltaRobot.BODY;
import static frc.robot.DeltaRobot.DRIVETRAIN;
import static frc.robot.DeltaRobot.PIVOT;
import static frc.robot.DeltaRobot.ROBOT;
import static frc.robot.robots2026.Parts.CANCODER_COUNTS;
import static frc.robot.robots2026.PhoenixParts.configure;
import static frc.robot.robots2026.PhoenixParts.control;
import static frc.robot.robots2026.PhoenixParts.corner;
import static frc.robot.robots2026.PhoenixParts.gains;
import static frc.robot.robots2026.PhoenixParts.motor;
import static frc.robot.robots2026.PhoenixParts.seeded;
import static frc.robot.robots2026.PhoenixParts.units;
import static frc.robot.robots2026.PhoenixParts.wiring;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures;
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
import frc.robot.DeltaRobot;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/**
 * The helper's Phoenix settings for a team's 2026 robot, Delta ({@link DeltaRobot}), against what
 * its code configures. Citations are to the robot's original code, under {@code
 * src/main/java/frc/robot/}. Every ratio here is the definition's gearing, a {@code double}, never
 * typed again, as in {@link BetaPhoenixTest}.
 */
class DeltaPhoenixTest {
  private final RobotMath math = ROBOT.math();

  /**
   * Each TalonFX reads its joint through its drive's gearing. The robot's code sets no ratio, so
   * Phoenix reads rotor turns and the code divides by the ratio (Pivot.java:96, Intake.java:42,
   * FullSend.java:37, Transition.java:74): its Constants.java writes each output over motor (1.0 /
   * 25, :84).
   */
  @Test
  void eachMotorReadsItsJointThroughTheRatioItsCodeDividesBy() {
    record Mechanism(Joint joint, Drive drive, double theirs) {}
    for (Mechanism each :
        List.of(
            new Mechanism(PIVOT, motors(DeltaRobot.PIVOT), 1 / (1.0 / 25.0)),
            new Mechanism(DeltaRobot.INTAKE, motors(DeltaRobot.INTAKE), 1 / (1.0 / 2.0)),
            new Mechanism(DeltaRobot.FULL_SEND, motors(DeltaRobot.FULL_SEND), 1 / (1.0 / 4.0)),
            new Mechanism(DeltaRobot.TRANSITION, motors(DeltaRobot.TRANSITION), 1 / (1.0 / 2.0)),
            new Mechanism(DeltaRobot.SHOOTER, motors(DeltaRobot.SHOOTER), 1))) {
      assertThat(seeded(math, each.joint(), each.drive()).leader().Feedback.SensorToMechanismRatio)
          .as(each.joint().label())
          .isCloseTo(each.theirs(), within(1e-12));
    }
  }

  /**
   * The velocity loops' tuned kV, per rotor rotation per second (Shooter.java:23, Intake.java:20,
   * FullSend.java:18, Transition.java:24-25), in the helper's units: the same number times the
   * gearing, per rotation of the joint, and each near the ideal, so nothing to note.
   */
  @Test
  void theirVelocityKvsAreTheirsPerTurnOfTheJoint() {
    record Loop(Joint joint, Drive drive, double kV, double ratio) {}
    for (Loop loop :
        List.of(
            new Loop(DeltaRobot.SHOOTER, motors(DeltaRobot.SHOOTER), 0.12, 1),
            new Loop(DeltaRobot.INTAKE, motors(DeltaRobot.INTAKE), 0.13, 2),
            new Loop(DeltaRobot.FULL_SEND, motors(DeltaRobot.FULL_SEND), 0.13, 4),
            new Loop(DeltaRobot.TRANSITION, motors(DeltaRobot.TRANSITION), 0.14, 2))) {
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
   * The shooter's four motors (IDs 41 to 44, Constants.java:49-52): 43 leads, clockwise
   * (Shooter.java:32); 44 follows aligned, 41 and 42 opposed (Shooter.java:28-30), in that order.
   * Wired as each turns, the helper aligns them so; Beta's leader is 41, counterclockwise.
   */
  @Test
  void theShootersFollowersAreAlignedAsTheirs() {
    List<MechanismWiring.Device> wired =
        wiring(motor(43, true), motor(44, true), motor(41, false), motor(42, false));

    PhoenixMechanism.Settings shooter =
        configure(math, DeltaRobot.SHOOTER, motors(DeltaRobot.SHOOTER), wired, control());

    assertThat(shooter.leader().MotorOutput.Inverted).isEqualTo(InvertedValue.Clockwise_Positive);
    assertThat(shooter.followers())
        .extracting(follower -> follower.request().MotorAlignment)
        .containsExactly(
            MotorAlignmentValue.Aligned, MotorAlignmentValue.Opposed, MotorAlignmentValue.Opposed);
    assertThat(shooter.followers()).allMatch(follower -> follower.request().LeaderID == 43);
  }

  /**
   * The intake roller's two motors (IDs 31 and 33, Constants.java:78-79): 31 leads,
   * counterclockwise (Intake.java:25); 33 follows it opposed (Intake.java:23), so turns clockwise.
   * Beta's roller is one motor.
   */
  @Test
  void theIntakesFollowerIsOpposedAsTheirs() {
    List<MechanismWiring.Device> wired = wiring(motor(31, false), motor(33, true));

    PhoenixMechanism.Settings intake =
        configure(math, DeltaRobot.INTAKE, motors(DeltaRobot.INTAKE), wired, control());

    assertThat(intake.leader().MotorOutput.Inverted)
        .isEqualTo(InvertedValue.CounterClockwise_Positive);
    assertThat(intake.followers())
        .extracting(follower -> follower.request().MotorAlignment)
        .containsExactly(MotorAlignmentValue.Opposed);
    assertThat(intake.followers()).allMatch(follower -> follower.request().LeaderID == 31);
  }

  /**
   * The pivot's absolute encoder wraps where the robot's code wraps it, at its reading's 0, the
   * pivot's 0° (hardware/AbsoluteEncoder.java:58-60): 3° from where it stows (Constants.java:88),
   * and at the range's end. The helper flags it, and its own point, the definition's, leaves a
   * third of a turn either side. Both judge a point by one margin, the definition's fortieth of a
   * turn ({@code RobotMath.WRAP_MARGIN}), which theirs, on the range's edge, is inside.
   */
  @Test
  void thePivotsEncoderWrapIsFlaggedBesideItsStow() {
    Sensor.Encoder encoder = absolute(DeltaRobot.PIVOT);
    double ours =
        PhoenixMechanism.cancoder(
                math, encoder, CANCODER.can(DeltaRobot.CANIVORE, 1).offset(Rotations.of(0)))
            .MagnetSensor
            .AbsoluteSensorDiscontinuityPoint;

    assertThat(ControllerPlan.wrapProblem(encoder, 1.0))
        .isEqualTo(
            "pivot absolute encoder wraps at 1 of its turn, 0° from pivot's range (0° to 135°), and"
                + " 3° from where it stows (3°): a reading a hair past the wrap reads a turn"
                + " away");
    assertThat(ControllerPlan.wrapMargin(encoder, 1.0)).isCloseTo(0, within(1e-12));
    assertThat(ours).isCloseTo(0.6875, within(1e-12));
    assertThat(ControllerPlan.wrapProblem(encoder, ours)).isNull();
    assertThat(ControllerPlan.wrapMargin(encoder, ours)).isCloseTo(0.3125, within(1e-12));
  }

  /**
   * The pivot as the robot's code drives it: a Kraken X60 turning clockwise to swing it out
   * (Pivot.java:26-27), braking; its arm unweighed and its kG not written, as the robot's code has
   * none: kG 0, about its zero.
   */
  @Test
  void thePivotHasNoGravitySeedUnweighed() {
    PhoenixMechanism.Settings pivot =
        configure(math, PIVOT, motors(DeltaRobot.PIVOT), wiring(motor(32, true)), control());

    assertThat(pivot.leader().MotorOutput.Inverted).isEqualTo(InvertedValue.Clockwise_Positive);
    assertThat(pivot.leader().Slot0.kG).isZero();
    assertThat(pivot.notes()).isEmpty();
  }

  /**
   * Weighed, with its center of mass anywhere along its zero, which the fixture draws straight up,
   * the pivot's gravity offset is a quarter turn, whatever it weighs: exactly Phoenix's limit. The
   * helper takes it and flags it, since a center of mass a hair forward of that is an offset
   * Phoenix can't hold. The masses are no guesses of the robot's code: any gives the same offset.
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
   * SensorToMechanismRatio its gearing (subsystems/drive/ModuleIOTalonFX.java:106), with the
   * coupling taken out in code; the steering fused with its CANcoder
   * (generated/TunerConstants.java:54), its RotorToSensorRatio the steering's gearing
   * (ModuleIOTalonFX.java:131), the front left's CANcoder (ID 12) offset 0.1069 and
   * counterclockwise (TunerConstants.java:139-142), its discontinuity a fresh configuration's.
   * Their steering wraps the short way (:137), which the helper sets for no joint: the template's
   * module driver sets its own steering to, and a joint's profile picks its turn. Tuner's kV, 0.72
   * per wheel rotation per second for the drive and 2.49 per module rotation per second for the
   * steering (TunerConstants.java:36, 30), are each the ideal's, near enough.
   */
  @Test
  void theSwerveModulesAreTunersOwn() {
    DrivetrainGeometry.ModuleJoints frontLeft = DRIVETRAIN.joints().get(0);
    Sensor.Encoder cancoder =
        JointFixtures.absoluteEncoder("front-left cancoder", frontLeft.steer(), CANCODER_COUNTS);
    RobotMath corner = corner(BODY, frontLeft, cancoder);
    double driveGearing = DeltaRobot.MODULES.driveGearing().ratio();
    double steerGearing = DeltaRobot.MODULES.steerGearing().ratio();

    PhoenixMechanism.Settings drive =
        configure(
            corner,
            frontLeft.wheel(),
            frontLeft.drive(),
            wiring(motor(1, false)),
            control(
                gains(units(frontLeft.wheel(), frontLeft.drive()), 0.1, 0, 0.72, Double.NaN, 1)));
    PhoenixMechanism.Settings steer =
        configure(
            corner,
            frontLeft.steer(),
            frontLeft.steering(),
            cancoder,
            CANCODER.can(DeltaRobot.CANIVORE, 12).offset(Rotations.of(0.10693359375)).fused(),
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
    assertThat(drive.leader().Slot0.kV).isCloseTo(0.72, within(1e-12));
    assertThat(drive.notes())
        .containsExactly(
            "front-left drive's rotor turns 3.375 times for each turn of front-left steer too,"
                + " which the controller reads as front-left wheel moving: take it out in code,"
                + " as a swerve module's coupling is");
    assertThat(steer.leader().Feedback.FeedbackSensorSource)
        .isEqualTo(FeedbackSensorSourceValue.FusedCANcoder);
    assertThat(steer.leader().Feedback.FeedbackRemoteSensorID).isEqualTo(12);
    assertThat(steer.leader().Feedback.SensorToMechanismRatio).isEqualTo(1);
    assertThat(steer.leader().Feedback.RotorToSensorRatio).isCloseTo(steerGearing, within(1e-12));
    assertThat(steer.leader().ClosedLoopGeneral.ContinuousWrap)
        .as("the helper never sets a joint's loop to wrap: a module's driver sets its steering's")
        .isFalse();
    assertThat(steer.leader().Slot0.kP).isCloseTo(100, within(1e-12));
    assertThat(steer.cancoder().orElseThrow().MagnetSensor.MagnetOffset)
        .isCloseTo(0.10693359375, within(1e-12));
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
