package frc.robot.robots2026;

import static frc.robot.DeltaRobot.BODY;
import static frc.robot.DeltaRobot.DRIVETRAIN;
import static frc.robot.DeltaRobot.MODULES;
import static frc.robot.DeltaRobot.ROBOT;
import static frc.robot.robots2026.Compare.SANITY_HIGH;
import static frc.robot.robots2026.Compare.SANITY_LOW;
import static frc.robot.robots2026.Compare.freeRpm;
import static frc.robot.robots2026.Compare.kV;
import static frc.robot.robots2026.Compare.sameMount;
import static frc.robot.robots2026.Parts.CANCODER_COUNTS;
import static frc.robot.robots2026.Parts.IN;
import static frc.robot.robots2026.Parts.inches;
import static frc.robot.robots2026.Parts.swapped;
import static frc.robot.robots2026.Parts.withOutputs;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.tuple;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.MetersPerSecond;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures;
import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.config.Finding.Scope;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.BodyBuilder;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.DeltaRobot;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A team's 2026 robot, Delta ({@link DeltaRobot}), against what its code configures. Citations are
 * to the robot's original code, under {@code src/main/java/frc/robot/}. What differs from Beta's
 * ({@link BetaRobotTest}) is tested as Delta's; what is Beta's, copied, is not tested again.
 */
class DeltaRobotTest {
  private final RobotMath math = ROBOT.math();

  @Test
  void itFitsTheSeason() {
    assertThat(Rebuilt.LIMITS.problems(ROBOT)).isEmpty();
    assertThat(BODY.mass().in(Kilograms))
        .as("Constants.java:193, PathPlanner's file's too (settings.json:15)")
        .isCloseTo(62.57, within(0.01));
  }

  /**
   * The robot the simulator runs is theirs, its drivetrain given the template's control through
   * {@code with}: their wiring kept, Tuner X's, and the tread's grip the default the robot plans on
   * until it's measured.
   */
  @Test
  void theSimulatorsRobotKeepsTheirWiring() {
    assertThat(DeltaRobot.CONFIG.definition().drivetrain()).isEqualTo(ROBOT.drivetrain());
    assertThat(DeltaRobot.CONFIG.drivetrainWiring()).isEqualTo(ROBOT.drivetrain().wiring());
    assertThat(DeltaRobot.CONFIG.drivetrain().gains().steerKp()).isEqualTo(16.0);
    assertThat(ROBOT.drivetrain().wheelCoefficientOfFriction()).isEqualTo(1.2);
  }

  /**
   * The drivetrain's Phoenix ratios as ModuleIOTalonFX sets them from Tuner's constants
   * (subsystems/drive/ModuleIOTalonFX.java:106,131; generated/TunerConstants.java:86-90), and its
   * modules where TunerConstants.java:144-178 puts them. The CAN bus is as Tuner X left it,
   * "Default Name" (TunerConstants.java:78), and the Pigeon 2 is at ID 1 (:95).
   */
  @Test
  void theSwerveModulesAreTunersOwn() {
    DrivetrainGeometry.ModuleJoints frontLeft = DRIVETRAIN.joints().get(0);
    Sensor.Encoder cancoder =
        JointFixtures.absoluteEncoder("front-left cancoder", frontLeft.steer(), CANCODER_COUNTS);
    Sensor.Encoder rotor = frontLeft.drive().rotor();
    RobotMath corner =
        RobotMath.of(
            BODY,
            List.of(frontLeft.steer(), frontLeft.wheel()),
            List.of(),
            List.of(frontLeft.drive(), frontLeft.steering()),
            List.of(cancoder));

    assertThat(corner.sensorToMechanismRatio(rotor, frontLeft.wheel())).isEqualTo(5.2734375);
    assertThat(rotor.ratio(frontLeft.steer()))
        .as("kCoupleRatio, read by the drive's rotor as the module turns")
        .isCloseTo(3.375, within(1e-12));
    assertThat(corner.rotorToSensorRatio(frontLeft.steering(), cancoder))
        .isCloseTo(26.09090909090909, within(1e-12));
    assertThat(corner.discontinuityPoint(cancoder))
        .as("a fresh CANcoderConfiguration's, TunerConstants.java:72")
        .isEqualTo(0.5);
    assertThat(corner.coordinate(frontLeft.drive(), Map.of(frontLeft.steer(), 2 * Math.PI)))
        .as("kCoupleRatio's drive turns for a turn of the module, TunerConstants.java:86")
        .isCloseTo(2 * Math.PI * 3.375, within(1e-9));
    assertThat(
            ROBOT.moduleTranslations()[0].getDistance(new Translation2d(10.375 * IN, 10.375 * IN)))
        .isLessThan(1e-12);
    assertThat(DRIVETRAIN.wiring().orElseThrow().canBus()).isEqualTo(DeltaRobot.CANIVORE);
    assertThat(DRIVETRAIN.wiring().orElseThrow().gyroId()).isEqualTo(1);
  }

  /**
   * The steering's seeds are its steering motor's alone, as for Beta: a Kraken X44 through 26.09:1,
   * 2.42 V per module rotation per second beside Tuner's 2.49 (generated/TunerConstants.java:30),
   * turning the module at most 4.96 rotations a second.
   */
  @Test
  void theSteeringIsSeededFromItsOwnMotor() {
    DrivetrainGeometry.ModuleJoints frontLeft = DRIVETRAIN.joints().get(0);
    RobotMath corner =
        RobotMath.of(
            BODY,
            List.of(frontLeft.steer(), frontLeft.wheel()),
            List.of(),
            List.of(frontLeft.drive(), frontLeft.steering()),
            List.of());

    assertThat(corner.drivesOf(frontLeft.steer())).containsExactly(frontLeft.steering());
    assertThat(kV(corner, frontLeft.steer(), 1)).isCloseTo(2.42, within(0.005));
    assertThat(corner.freeSpeed(frontLeft.steer()) / (2 * Math.PI)).isCloseTo(4.96, within(0.005));
  }

  /**
   * kSpeedAt12Volts, 5.85 m/s (TunerConstants.java:82), is what the motors and gearing reach, free,
   * to a few thousandths: unlike Beta's, which asked for more than its drive has.
   */
  @Test
  void theTunersSpeedAt12VoltsIsWhatTheMotorsReach() {
    double free = MODULES.freeSpeed().in(MetersPerSecond);

    assertThat(free).isCloseTo(5.85, within(0.005));
    assertThat(3.25).as("the drivers' top speed, Constants.java:45").isLessThan(free);
  }

  /**
   * The pivot's absolute encoder wraps where AbsoluteEncoder.get puts it, at its reading's 0
   * (hardware/AbsoluteEncoder.java:58-60: a reading less its offset, plus a turn when negative),
   * which is the pivot's 0°: 3° from where it stows (Constants.java:88). The in-drive ends on a
   * reading at or under 3° (Pivot.java:70), so a pivot swung in past 0° reads 359° and the drive
   * goes on, at -3 V, until the button is let go. The model puts the wrap as far from the range as
   * a turn allows. The team's hazard, with the stops guessed: theirs sits on the range's edge,
   * inside the fortieth of a turn the definition keeps clear, so a definition whose encoder wrapped
   * there would be refused.
   */
  @Test
  void thePivotsEncoderWrapsBesideItsStow() {
    double derived = math.discontinuityPoint(absolute(DeltaRobot.PIVOT));
    var range = DeltaRobot.PIVOT.range().orElseThrow();

    assertThat(derived).isCloseTo(0.6875, within(1e-12));
    assertThat(RobotMath.wrapMargin(absolute(DeltaRobot.PIVOT), derived))
        .isCloseTo(0.3125, within(1e-12));
    assertThat(RobotMath.wrapMargin(absolute(DeltaRobot.PIVOT), 1.0))
        .as("theirs")
        .isCloseTo(0, within(1e-12))
        .isLessThan(RobotMath.WRAP_MARGIN);
    for (double setpoint : new double[] {3, 130}) {
      assertThat(range.contains(Math.toRadians(setpoint))).as(setpoint + "°").isTrue();
    }
  }

  /**
   * Phoenix's SensorToMechanismRatio for each motor's rotor, which its drive implies. The robot's
   * code sets none, so Phoenix reads every mechanism in rotor turns and the code divides by its
   * ratio (Pivot.java:96, Intake.java:42, FullSend.java:37, Transition.java:74); each rotor's ratio
   * here is the one its code divides by, written the motor-over-output way round.
   */
  @Test
  void eachRotorTurnsByItsDrivesRatio() {
    assertThat(math.sensorToMechanismRatio(motors(DeltaRobot.PIVOT).rotor()))
        .isEqualTo(1 / (1.0 / 25));
    assertThat(math.sensorToMechanismRatio(motors(DeltaRobot.INTAKE).rotor())).isEqualTo(2);
    assertThat(math.sensorToMechanismRatio(motors(DeltaRobot.FULL_SEND).rotor())).isEqualTo(4);
    assertThat(math.sensorToMechanismRatio(motors(DeltaRobot.TRANSITION).rotor())).isEqualTo(2);
    assertThat(math.sensorToMechanismRatio(motors(DeltaRobot.SHOOTER).rotor())).isEqualTo(1);
    assertThat(motors(DeltaRobot.PIVOT).rotor().label()).isEqualTo("pivot rotor");
    assertThat(math.rotorToSensorRatio(motors(DeltaRobot.PIVOT), absolute(DeltaRobot.PIVOT)))
        .as("the rotor's turns for the absolute encoder's, were the two fused")
        .isCloseTo(25, within(1e-12));
  }

  /**
   * Every velocity setpoint stays at or under its mechanism's free speed: the shooter's cap, 4500
   * rpm (Constants.java:68); the full send's 1000 (FullSend.java:46) and the transition's 2000
   * (Transition.java:83), output rpm. The intake's 3000 output rpm (Intake.java:51) is exactly its
   * motors' free speed through 2:1, so its velocity loop can't reach it under any load.
   */
  @Test
  void noSetpointIsFasterThanItsMotorsCanTurnIt() {
    assertThat(freeRpm(math, DeltaRobot.SHOOTER)).isGreaterThan(4500);
    assertThat(freeRpm(math, DeltaRobot.FULL_SEND)).isGreaterThan(1000);
    assertThat(freeRpm(math, DeltaRobot.TRANSITION)).isGreaterThan(2000);
    assertThat(freeRpm(math, DeltaRobot.INTAKE)).isCloseTo(3000, within(1e-6));
  }

  /**
   * The names their code gives its places, now the definition's: the pivot's angles
   * (Constants.java:88-90) and the two voltages it's driven at (Pivot.java:53, :68), the rollers'
   * speeds (Intake.java:51-52, FullSend.java:46, Transition.java:83), the driver's two feeds
   * (RobotContainer.java:254-266), and the auto's guard on the intake's roller
   * (RobotContainer.java:164-165), an interlock.
   */
  @Test
  void theirPlacesAreNamedInTheDefinition() {
    assertThat(DeltaRobot.PIVOT.positions())
        .extracting(Position::name)
        .containsExactly("in", "out");
    assertThat(DeltaRobot.PIVOT.outputs())
        .extracting(Output::name, Output::kind, Output::value)
        .containsExactly(
            tuple("swinging out", Output.Kind.VOLTAGE, 5.0),
            tuple("swinging in", Output.Kind.VOLTAGE, -3.0));
    assertThat(DeltaRobot.INTAKE.output("outtaking").value() * 60 / (2 * Math.PI))
        .isCloseTo(-500, within(1e-9));
    assertThat(ROBOT.configurations())
        .extracting(Configuration::name)
        .containsExactly("feeding", "unjamming");
    assertThat(DeltaRobot.FEEDING.joints())
        .containsExactly(DeltaRobot.FULL_SEND, DeltaRobot.TRANSITION);
    assertThat(ROBOT.everyInterlock())
        .singleElement()
        .hasToString("intake runs only when pivot at least 130°");
  }

  /**
   * Their targeting feeds at 5000 rpm (RobotContainer.java:171, RobotTarget.java:38,
   * AutoTarget.java:70), commanded as 20,000 rpm of the motor through 4:1 (FullSend.java:37): 3.3
   * times what a Kraken X60 turns. Driver B starts it (RobotContainer.java:276-278). The definition
   * refuses it, naming both; the fixture names the driver's 1000 instead.
   */
  @Test
  void theTargetingFeederSpeedIsRefused() {
    Joint.Revolute targetingFeed =
        withOutputs(
            DeltaRobot.FULL_SEND, Output.rpm("feeding", 5000), Output.rpm("unjamming", -1000));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                swapped(
                    ROBOT,
                    DeltaRobot.FULL_SEND,
                    targetingFeed,
                    motors(DeltaRobot.FULL_SEND),
                    JointFixtures.motors(targetingFeed, Motor.KRAKEN_X60, 1, Gearing.of(4))))
        .withMessage(
            "RobotDefinition[delta].joints: full send's feeding, 5000 rpm, is 3.333 times its"
                + " free speed, 1500 rpm: its motors can't turn it that fast; give it a speed under"
                + " that");
  }

  /**
   * The intake's 3000 rpm is exactly its motors' free speed through 2:1, so its velocity loop has
   * nothing left to hold it under load: noted, not refused.
   */
  @Test
  void theIntakesSpeedIsNotedAtItsFreeSpeed() {
    assertThat(ROBOT.findings())
        .filteredOn(finding -> finding.level() == Finding.Level.DOUBT)
        .extracting(Finding::message)
        .containsExactly(
            "intake's intaking, 3000 rpm, is 100% of its free speed, 3000 rpm: a velocity loop has"
                + " little or nothing left to hold it under load");
  }

  /**
   * The interlock holds for every configuration: one that runs the intake's roller with the pivot
   * stowed is refused, and one that puts the pivot out first is taken.
   */
  @Test
  void aConfigurationThatRunsTheIntakeStowedIsRefused() {
    Configuration stowed = Configuration.of("intaking", DeltaRobot.INTAKE.at(DeltaRobot.INTAKING));
    Configuration out =
        Configuration.of(
            "intaking",
            DeltaRobot.PIVOT.at(DeltaRobot.PIVOT_OUT),
            DeltaRobot.INTAKE.at(DeltaRobot.INTAKING));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> RobotDefinitionBuilder.builder(ROBOT).configurations(List.of(stowed)).build())
        .withMessage(
            "RobotDefinition[delta].configurations: configuration intaking runs intake, but"
                + " intake runs only when pivot at least 130°, and pivot is at 3°, where it rests");
    assertThat(
            RobotDefinitionBuilder.builder(ROBOT)
                .configurations(List.of(out))
                .build()
                .configurations())
        .containsExactly(out);
  }

  /**
   * The ideal kV against each velocity loop's tuned one, in volts per rotor rotation per second
   * (Shooter.java:23, Intake.java:20, FullSend.java:18, Transition.java:24-25): never equal, since
   * tuned ones carry friction, but the same mechanism.
   */
  @Test
  void theIdealKvIsNearEachTunedOne() {
    double[][] pairs = {
      {kV(math, DeltaRobot.SHOOTER, 1), 0.12},
      {kV(math, DeltaRobot.INTAKE, 2), 0.13},
      {kV(math, DeltaRobot.FULL_SEND, 4), 0.13},
      {kV(math, DeltaRobot.TRANSITION, 2), 0.14},
    };
    for (double[] pair : pairs) {
      assertThat(pair[0] / pair[1]).isBetween(SANITY_LOW, SANITY_HIGH);
    }
    assertThat(pairs[0][0])
        .as("a Kraken X60's 12 V over its 100 rps")
        .isCloseTo(0.12, within(1e-9));
  }

  /**
   * Each camera's mount as Constants.java:119-161 gives it, in WPILib's own transform. The right
   * rear's lift is 1.938 in and the wheel's radius (Constants.java:111-116), Delta's 2 in: 3.938 in
   * where Beta's is 3.868, so that camera sits a hair further forward than Beta's.
   */
  @Test
  void theCamerasAreWhereTheirCodePutsThem() {
    sameMount(
        DeltaRobot.LEFT_REAR,
        new Transform3d(
            inches(-10.5, 12, 21),
            new Rotation3d(Math.toRadians(-2), Math.toRadians(-10), Math.toRadians(180.985))));
    sameMount(
        DeltaRobot.LEFT_FRONT,
        new Transform3d(
            inches(-7.75, 12.25, 21),
            new Rotation3d(0, Math.toRadians(-10), Math.toRadians(23.162))));
    sameMount(
        DeltaRobot.RIGHT_FRONT,
        new Transform3d(
            inches(-8.625, -12.25, 20.875),
            new Rotation3d(0, Math.toRadians(-8.2), Math.toRadians(-23.162))));
    Transform3d rightRear =
        new Transform3d(
                inches(-11.6875, -12, 16.837),
                new Rotation3d(0, Math.toRadians(-12.5), Math.toRadians(180.985)))
            .plus(new Transform3d(inches(0, 0, 1.938 + 2), Rotation3d.ZERO));
    assertThat(DeltaRobot.LEFT_FRONT.name()).as("Beta's is FrontLeft").isEqualTo("Camera5");
    assertThat(
            DeltaRobot.RIGHT_REAR
                .robotToCamera()
                .getTranslation()
                .getDistance(rightRear.getTranslation()))
        .as("worked by hand to a ten-thousandth of an inch")
        .isLessThan(3e-6);
    assertThat(
            DeltaRobot.RIGHT_REAR
                .robotToCamera()
                .getTranslation()
                .getDistance(inches(-11.6875, -12, 16.837 + 1.938 + 2)))
        .as("a lift straight up would be 0.86 in away")
        .isCloseTo(0.8574 * IN, within(1e-4 * IN));
  }

  /**
   * The pivot's gravity, which the robot's code doesn't model (its controller's gains are all 0:
   * Pivot.java:21-22), nor can the model: no seed, no mass. The arm's mass is unknown, so it has no
   * gravity to hold, no offset from horizontal, and its kA seeds from its motor's rotor through
   * 25:1 alone.
   */
  @Test
  void thePivotHasNoGravitySeedWithoutAMass() {
    RobotMath.Gains gains = math.idealGains(DeltaRobot.PIVOT);
    double rotor = Motor.KRAKEN_X60.rotorInertia().in(KilogramSquareMeters) * 25 * 25;

    assertThat(gains.kG()).isZero();
    assertThat(math.gravityOffset(DeltaRobot.PIVOT)).isZero();
    assertThat(math.gravityFit(DeltaRobot.PIVOT).peak()).isZero();
    assertThat(math.inertia(DeltaRobot.PIVOT, Map.of())).isCloseTo(rotor, within(1e-12));
  }

  /**
   * What the definition doesn't know. No link's mass, which only the simulator reads; the tread's
   * grip, which the code gives only as a placeholder and Beta's unread value, which the robot says;
   * the pivot's kG not written, as the robot's code has none, which the robot says; the robot
   * weighed, as the code gives it; its yaw the code's 5.5 kg m² (Constants.java:194), its pitch and
   * roll the model's box, which would put the yaw at 6.89, the PathPlanner app's 6.883
   * (settings.json:16) to a tenth of a percent.
   */
  @Test
  void theMissingDataReportListsEveryLinksMass() {
    List<Finding> notes =
        ROBOT.findings().stream().filter(finding -> finding.level() == Finding.Level.NOTE).toList();

    assertThat(notes)
        .extracting(Finding::message, Finding::scope)
        .containsExactly(
            tuple(
                "the body's pitch and roll are a box's, beside its given yaw: how the simulator"
                    + " tips the robot is approximate",
                Scope.SIMULATOR),
            tuple(
                "62.57 kg of the robot's 62.57 kg is in no link: the chassis, its electronics,"
                    + " bumpers and battery: carried as one rigid body",
                Scope.SIMULATOR),
            tuple(
                "the tread's grip isn't measured: it's 1.2, PathPlanner's default: drive moves"
                    + " plan their acceleration on it, and the slip check judges by it: measure it"
                    + " with a pull test (docs/characterization.md, tread friction)",
                Scope.ROBOT),
            tuple(
                "intake arm has no mass: pivot's gravity comes from its kG, and intake arm adds"
                    + " nothing to pivot's inertia",
                Scope.SIMULATOR),
            tuple(
                "pivot has no kG: it holds against gravity by kP alone, so it sits below its"
                    + " targets until its kG is tuned and written, withKg(...), or written as 0"
                    + " for a counterbalanced joint",
                Scope.ROBOT),
            tuple(
                "intake roller has no mass: intake roller adds nothing to intake's inertia, so its"
                    + " rotors alone bring it to intaking, 3000 rpm, in about 11 ms",
                Scope.SIMULATOR),
            tuple(
                "shooter wheels has no mass: shooter wheels adds nothing to shooter's inertia",
                Scope.SIMULATOR),
            tuple(
                "full send rollers has no mass: full send rollers adds nothing to full send's"
                    + " inertia, so its rotors alone bring it to feeding, 1000 rpm, in about 6 ms",
                Scope.SIMULATOR),
            tuple(
                "transition rollers has no mass: transition rollers adds nothing to transition's"
                    + " inertia, so its rotors alone bring it to feeding, 2000 rpm, in about 5 ms",
                Scope.SIMULATOR));
    assertThat(notes)
        .extracting(Finding::subject)
        .containsExactly(
            "delta",
            "delta",
            "delta",
            "pivot",
            "pivot",
            "intake",
            "shooter",
            "full send",
            "transition");
    assertThat(BODY.yawMomentOfInertia().in(KilogramSquareMeters)).isEqualTo(5.5);
    Body boxed = BodyBuilder.builder(BODY).inertia(Body.Turning.BOX).build();
    assertThat(boxed.yawMomentOfInertia().in(KilogramSquareMeters)).isCloseTo(6.883, within(0.01));
  }

  /** Delta has no hood: its code never builds one, and its joints name none. */
  @Test
  void deltaHasNoHood() {
    assertThat(ROBOT.joints()).extracting(Joint::label).doesNotContain("hood");
    assertThat(ROBOT.joints())
        .extracting(Joint::label)
        .containsExactly("pivot", "intake", "shooter", "full send", "transition");
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
