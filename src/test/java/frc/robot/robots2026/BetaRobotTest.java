package frc.robot.robots2026;

import static frc.robot.BetaRobot.BODY;
import static frc.robot.BetaRobot.DRIVETRAIN;
import static frc.robot.BetaRobot.MODULES;
import static frc.robot.BetaRobot.ROBOT;
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

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.config.Finding.Scope;
import com.michaelgrundvig.frc.world.robot.Actuator;
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
import frc.robot.BetaRobot;
import frc.robot.game.Rebuilt;
import frc.robot.subsystems.mechanisms.JointFixtures;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A team's 2026 robot, Beta ({@link BetaRobot}), against what its code configures. Citations are to
 * the robot's original code, under {@code src/main/java/frc/robot/}.
 */
class BetaRobotTest {
  private final RobotMath math = ROBOT.math();

  @Test
  void itFitsTheSeason() {
    assertThat(Rebuilt.LIMITS.problems(ROBOT)).isEmpty();
    assertThat(BODY.mass().in(Kilograms))
        .as("PathPlanner's mass, settings.json:15")
        .isCloseTo(62.57, within(0.01));
  }

  /**
   * The robot the simulator runs is theirs, its drivetrain given the template's control through
   * {@code with}: their wiring kept, Tuner X's, and the tread's grip the default the robot plans on
   * until it's measured.
   */
  @Test
  void theSimulatorsRobotKeepsTheirWiring() {
    assertThat(BetaRobot.CONFIG.definition().drivetrain()).isEqualTo(ROBOT.drivetrain());
    assertThat(BetaRobot.CONFIG.drivetrainWiring()).isEqualTo(ROBOT.drivetrain().wiring());
    assertThat(BetaRobot.CONFIG.drivetrain().gains().steerKp()).isEqualTo(16.0);
    assertThat(ROBOT.drivetrain().wheelCoefficientOfFriction()).isEqualTo(1.2);
  }

  /**
   * The drivetrain's Phoenix ratios as ModuleIOTalonFX sets them from Tuner's constants
   * (subsystems/drive/ModuleIOTalonFX.java:100,122; generated/TunerConstants.java:80-83), and its
   * modules where TunerConstants.java:135-169 puts them.
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

    assertThat(corner.sensorToMechanismRatio(rotor, frontLeft.wheel()))
        .isEqualTo(6.026785714285714);
    assertThat(rotor.ratio(frontLeft.steer()))
        .as("kCoupleRatio, read by the drive's rotor as the module turns")
        .isCloseTo(3.857142857142857, within(1e-12));
    assertThat(corner.rotorToSensorRatio(frontLeft.steering(), cancoder))
        .isCloseTo(26.09090909090909, within(1e-12));
    assertThat(corner.discontinuityPoint(cancoder))
        .as("a fresh CANcoderConfiguration's, TunerConstants.java:65")
        .isEqualTo(0.5);
    assertThat(corner.coordinate(frontLeft.drive(), Map.of(frontLeft.steer(), 2 * Math.PI)))
        .as("kCoupleRatio's drive turns for a turn of the module, TunerConstants.java:80")
        .isCloseTo(2 * Math.PI * 3.857142857142857, within(1e-9));
    assertThat(
            ROBOT.moduleTranslations()[0].getDistance(new Translation2d(10.375 * IN, 10.375 * IN)))
        .isLessThan(1e-12);
  }

  /**
   * The steering's seeds are its steering motor's alone: a Kraken X44 through 26.09:1, 2.42 V per
   * module rotation per second beside Tuner's 2.49 (generated/TunerConstants.java:34), turning the
   * module at most 4.96 rotations a second. The drive motor turns as the module steers, through the
   * coupling, but it's the wheel's, which stays still meanwhile: counted as pushing the steering
   * too, it put the seed at 1.93 and the free speed at 6.21.
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
   * kSpeedAt12Volts, 5.4 m/s (TunerConstants.java:76), is the team's own: the motors and gearing
   * reach 4.94 m/s free. It's what Drive.java desaturates the wheels to and gives PathPlanner, so
   * every speed above 4.94 asks the drive for more than it has. The team's quirk, not the model's.
   */
  @Test
  void theTeamsTopSpeedIsAboveWhatTheMotorsReach() {
    double free = MODULES.freeSpeed().in(MetersPerSecond);

    assertThat(free).isCloseTo(4.9404, within(1e-4));
    assertThat(5.4).isGreaterThan(free);
    assertThat(3.25).as("the drivers' top speed, Constants.java:46").isLessThan(free);
  }

  /**
   * The pivot's absolute encoder wraps where AbsoluteEncoder.get puts it, at its reading's 0
   * (hardware/AbsoluteEncoder.java:233-235: a reading less its offset, plus a turn when negative),
   * which is the pivot's 0°: 1.5° from where it stows (Constants.java:89). A reading a hair below 0
   * is 359°, and Pivot.intakeIsOut (Pivot.java:69) says out. The model puts the wrap as far from
   * the range as a turn allows. The team's hazard, with the stops guessed: theirs sits on the
   * range's edge, inside the fortieth of a turn the definition keeps clear, so a definition whose
   * encoder wrapped there would be refused.
   */
  @Test
  void thePivotsEncoderWrapsBesideItsStow() {
    double derived = math.discontinuityPoint(absolute(BetaRobot.PIVOT));
    var range = BetaRobot.PIVOT.range().orElseThrow();

    assertThat(derived).isCloseTo(0.6875, within(1e-12));
    assertThat(RobotMath.wrapMargin(absolute(BetaRobot.PIVOT), derived))
        .isCloseTo(0.3125, within(1e-12));
    assertThat(RobotMath.wrapMargin(absolute(BetaRobot.PIVOT), 1.0))
        .as("theirs")
        .isCloseTo(0, within(1e-12))
        .isLessThan(RobotMath.WRAP_MARGIN);
    for (double setpoint : new double[] {1.5, 110, 130}) {
      assertThat(range.contains(Math.toRadians(setpoint))).as(setpoint + "°").isTrue();
    }
  }

  /**
   * Phoenix's SensorToMechanismRatio for each motor's rotor, which its drive implies. the robot's
   * code sets none, so Phoenix reads every mechanism in rotor turns and the code divides by its
   * ratio (Pivot.java:83, Intake.java:43, FullSend.java:42, Transition.java:72); each rotor's ratio
   * here is the one its code divides by, written the motor-over-output way round.
   */
  @Test
  void eachRotorTurnsByItsDrivesRatio() {
    assertThat(math.sensorToMechanismRatio(motors(BetaRobot.PIVOT).rotor()))
        .isEqualTo(1 / (1.0 / 25));
    assertThat(math.sensorToMechanismRatio(motors(BetaRobot.INTAKE).rotor())).isEqualTo(3);
    assertThat(math.sensorToMechanismRatio(motors(BetaRobot.FULL_SEND).rotor())).isEqualTo(4);
    assertThat(math.sensorToMechanismRatio(motors(BetaRobot.TRANSITION).rotor())).isEqualTo(2);
    assertThat(math.sensorToMechanismRatio(motors(BetaRobot.SHOOTER).rotor())).isEqualTo(1);
    assertThat(motors(BetaRobot.PIVOT).rotor().label()).isEqualTo("pivot rotor");
    assertThat(math.rotorToSensorRatio(motors(BetaRobot.PIVOT), absolute(BetaRobot.PIVOT)))
        .as("the rotor's turns for the absolute encoder's, were the two fused")
        .isCloseTo(25, within(1e-12));
  }

  /**
   * Every velocity setpoint stays at or under its mechanism's free speed: the shooter's cap, 4500
   * rpm (Constants.java:70); the full send's 1000 (FullSend.java:50) and the transition's 2000
   * (Transition.java:80), output rpm. The intake's 2000 output rpm (Intake.java:51) is exactly its
   * motor's free speed through 3:1, so its velocity loop can't reach it under any load.
   */
  @Test
  void noSetpointIsFasterThanItsMotorsCanTurnIt() {
    assertThat(freeRpm(math, BetaRobot.SHOOTER)).isGreaterThan(4500);
    assertThat(freeRpm(math, BetaRobot.FULL_SEND)).isGreaterThan(1000);
    assertThat(freeRpm(math, BetaRobot.TRANSITION)).isGreaterThan(2000);
    assertThat(freeRpm(math, BetaRobot.INTAKE)).isCloseTo(2000, within(1e-6));
  }

  /**
   * The names their code gives its places, now the definition's: the pivot's three angles
   * (Constants.java:89-91), the rollers' speeds (Intake.java:51-52, FullSend.java:50,
   * Transition.java:80), the driver's two feeds (RobotContainer.java:220-228), and the auto's guard
   * on the intake's roller (RobotContainer.java:143), an interlock.
   */
  @Test
  void theirPlacesAreNamedInTheDefinition() {
    assertThat(BetaRobot.PIVOT.positions()).extracting(Position::name).containsExactly("in", "out");
    assertThat(BetaRobot.INTAKE.output("outtaking").value() * 60 / (2 * Math.PI))
        .isCloseTo(-500, within(1e-9));
    assertThat(ROBOT.configurations())
        .extracting(Configuration::name)
        .containsExactly("feeding", "unjamming");
    assertThat(BetaRobot.FEEDING.joints())
        .containsExactly(BetaRobot.FULL_SEND, BetaRobot.TRANSITION);
    assertThat(ROBOT.everyInterlock())
        .singleElement()
        .hasToString("intake runs only when pivot at least 110°");
  }

  /**
   * Their autos feed at 5000 rpm (RobotContainer.java:146, AutoTarget.java:62,
   * RobotTarget.java:36), commanded as 20,000 rpm of the motor through 4:1 (FullSend.java:42): 3.3
   * times what a Kraken X60 turns. The definition refuses it, naming both; the fixture names the
   * driver's 1000 instead.
   */
  @Test
  void theAutosFeederSpeedIsRefused() {
    Joint.Revolute autoFeed =
        withOutputs(
            BetaRobot.FULL_SEND, Output.rpm("feeding", 5000), Output.rpm("unjamming", -1000));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                swapped(
                    ROBOT,
                    BetaRobot.FULL_SEND,
                    autoFeed,
                    motors(BetaRobot.FULL_SEND),
                    JointFixtures.motors(autoFeed, Motor.KRAKEN_X60, 1, Gearing.of(4))))
        .withMessage(
            "RobotDefinition[beta].joints: full send's feeding, 5000 rpm, is 3.333 times its"
                + " free speed, 1500 rpm: its motors can't turn it that fast; give it a speed under"
                + " that");
  }

  /**
   * The intake's 2000 rpm is exactly its motor's free speed through 3:1, so its velocity loop has
   * nothing left to hold it under load: noted, not refused.
   */
  @Test
  void theIntakesSpeedIsNotedAtItsFreeSpeed() {
    assertThat(ROBOT.findings())
        .filteredOn(finding -> finding.level() == Finding.Level.DOUBT)
        .extracting(Finding::message)
        .containsExactly(
            "intake's intaking, 2000 rpm, is 100% of its free speed, 2000 rpm: a velocity loop has"
                + " little or nothing left to hold it under load");
  }

  /**
   * The interlock holds for every configuration: one that runs the intake's roller with the pivot
   * stowed is refused, and one that puts the pivot out first is taken.
   */
  @Test
  void aConfigurationThatRunsTheIntakeStowedIsRefused() {
    Configuration stowed = Configuration.of("intaking", BetaRobot.INTAKE.at(BetaRobot.INTAKING));
    Configuration out =
        Configuration.of(
            "intaking",
            BetaRobot.PIVOT.at(BetaRobot.PIVOT_OUT),
            BetaRobot.INTAKE.at(BetaRobot.INTAKING));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> RobotDefinitionBuilder.builder(ROBOT).configurations(List.of(stowed)).build())
        .withMessage(
            "RobotDefinition[beta].configurations: configuration intaking runs intake, but"
                + " intake runs only when pivot at least 110°, and pivot is at 1.5°, where it rests");
    assertThat(
            RobotDefinitionBuilder.builder(ROBOT)
                .configurations(List.of(out))
                .build()
                .configurations())
        .containsExactly(out);
  }

  /**
   * The ideal kV against each velocity loop's tuned one, in volts per rotor rotation per second
   * (Shooter.java:29, Intake.java:24, FullSend.java:24, Transition.java:26): never equal, since
   * tuned ones carry friction, but the same mechanism.
   */
  @Test
  void theIdealKvIsNearEachTunedOne() {
    double[][] pairs = {
      {kV(math, BetaRobot.SHOOTER, 1), 0.12},
      {kV(math, BetaRobot.INTAKE, 3), 0.13},
      {kV(math, BetaRobot.FULL_SEND, 4), 0.13},
      {kV(math, BetaRobot.TRANSITION, 2), 0.14},
    };
    for (double[] pair : pairs) {
      assertThat(pair[0] / pair[1]).isBetween(SANITY_LOW, SANITY_HIGH);
    }
    assertThat(pairs[0][0])
        .as("a Kraken X60's 12 V over its 100 rps")
        .isCloseTo(0.12, within(1e-9));
  }

  /** Each camera's mount as Constants.java:119-157 gives it, in WPILib's own transform. */
  @Test
  void theCamerasAreWhereTheirCodePutsThem() {
    sameMount(
        BetaRobot.LEFT_REAR,
        new Transform3d(
            inches(-10.5, 12, 21),
            new Rotation3d(Math.toRadians(-2), Math.toRadians(-10), Math.toRadians(180.985))));
    sameMount(
        BetaRobot.LEFT_FRONT,
        new Transform3d(
            inches(-7.75, 12.25, 21),
            new Rotation3d(0, Math.toRadians(-10), Math.toRadians(23.162))));
    sameMount(
        BetaRobot.RIGHT_FRONT,
        new Transform3d(
            inches(-8.625, -12.25, 20.875),
            new Rotation3d(0, Math.toRadians(-8.2), Math.toRadians(-23.162))));
    Transform3d rightRear =
        new Transform3d(
                inches(-11.6875, -12, 16.837),
                new Rotation3d(0, Math.toRadians(-12.5), Math.toRadians(180.985)))
            .plus(new Transform3d(inches(0, 0, 1.938 + 1.93), Rotation3d.ZERO));
    assertThat(
            BetaRobot.RIGHT_REAR
                .robotToCamera()
                .getTranslation()
                .getDistance(rightRear.getTranslation()))
        .as("worked by hand to a ten-thousandth of an inch")
        .isLessThan(3e-6);
    assertThat(
            BetaRobot.RIGHT_REAR
                .robotToCamera()
                .getTranslation()
                .getDistance(inches(-11.6875, -12, 16.837 + 1.938 + 1.93)))
        .as("a lift straight up would be 0.84 in away")
        .isCloseTo(0.8422 * IN, within(1e-4 * IN));
  }

  /**
   * The pivot's gravity, which the robot's code doesn't model (its controller's kS and kV are 0,
   * and it has no kG: Pivot.java:26-27), nor can the model: no seed, no mass. The arm's mass is
   * unknown, so it has no gravity to hold, no offset from horizontal, and its kA seeds from its
   * motor's rotor through 25:1 alone.
   */
  @Test
  void thePivotHasNoGravitySeedWithoutAMass() {
    RobotMath.Gains gains = math.idealGains(BetaRobot.PIVOT);
    double rotor = Motor.KRAKEN_X60.rotorInertia().in(KilogramSquareMeters) * 25 * 25;

    assertThat(gains.kG()).isZero();
    assertThat(math.gravityOffset(BetaRobot.PIVOT)).isZero();
    assertThat(math.gravityFit(BetaRobot.PIVOT).peak()).isZero();
    assertThat(math.inertia(BetaRobot.PIVOT, Map.of())).isCloseTo(rotor, within(1e-12));
  }

  /**
   * What the definition doesn't know. No link's mass, which only the simulator reads; the tread's
   * grip, which their code gives only as PathPlanner's tuning value, which the robot says; the
   * pivot's kG not written, as the robot's code has none (Pivot.java:26-27), which the robot says;
   * the robot weighed, as the code gives it; its yaw the code's 5.5 kg m² (Constants.java:188), its
   * pitch and roll the model's box, which would put the yaw at 6.89, the PathPlanner app's 6.883
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
                "intake roller has no mass: intake roller adds nothing to intake's inertia",
                Scope.SIMULATOR),
            tuple(
                "shooter wheels has no mass: shooter wheels adds nothing to shooter's inertia",
                Scope.SIMULATOR),
            tuple(
                "full send rollers has no mass: full send rollers adds nothing to full send's"
                    + " inertia",
                Scope.SIMULATOR),
            tuple(
                "transition rollers has no mass: transition rollers adds nothing to transition's"
                    + " inertia",
                Scope.SIMULATOR),
            tuple(
                "hood has no mass: hood's gravity comes from its kG, and hood adds nothing to"
                    + " hood's inertia",
                Scope.SIMULATOR),
            tuple(
                "hood has no shape: it isn't drawn, nor checked against the starting height and"
                    + " the extension limit",
                Scope.ROBOT));
    assertThat(notes)
        .extracting(Finding::subject)
        .containsExactly(
            "beta",
            "beta",
            "beta",
            "pivot",
            "pivot",
            "intake",
            "shooter",
            "full send",
            "transition",
            "hood",
            "hood");
    assertThat(BODY.yawMomentOfInertia().in(KilogramSquareMeters)).isEqualTo(5.5);
    Body boxed = BodyBuilder.builder(BODY).inertia(Body.Turning.BOX).build();
    assertThat(boxed.yawMomentOfInertia().in(KilogramSquareMeters)).isCloseTo(6.883, within(0.01));
  }

  /**
   * The hood's servos, told a share of their travel as WPILib's Servo.set takes it: the code's
   * lowest, 0.65 (Constants.java:59), is the hood at 0°, and its highest, 1.0, full extension at
   * 40°, on the guessed geometry. Both servos are told the same (Hood.java:30-31), and told more,
   * they push the hood up.
   */
  @Test
  void theHoodsServosAreToldTheirShareOfTheirTravel() {
    Actuator.Servo servo = BetaRobot.WCP_0408;
    Map<Joint, Double> lowered = Map.of(BetaRobot.HOOD, 0.0);
    Map<Joint, Double> raised = Map.of(BetaRobot.HOOD, Math.toRadians(40));

    assertThat(servo.command(math.coordinate(BetaRobot.HOOD.ownDrives().get(0), lowered)))
        .isCloseTo(0.65, within(1e-9));
    assertThat(servo.command(math.coordinate(BetaRobot.HOOD.ownDrives().get(0), raised)))
        .isCloseTo(1.0, within(0.001));
    assertThat(math.coordinate(BetaRobot.HOOD.ownDrives().get(1), raised))
        .isCloseTo(math.coordinate(BetaRobot.HOOD.ownDrives().get(0), raised), within(1e-12));
    assertThat(math.torque(BetaRobot.HOOD.ownDrives().get(0), BetaRobot.HOOD, lowered, 1.0))
        .as("told to extend, it lifts the hood")
        .isPositive();
    assertThat(math.torque(BetaRobot.HOOD.ownDrives().get(0), BetaRobot.HOOD, lowered, 0.65))
        .as("told where it is, it holds")
        .isCloseTo(0, within(1e-9));
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
