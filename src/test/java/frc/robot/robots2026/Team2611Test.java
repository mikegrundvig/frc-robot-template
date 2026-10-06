package frc.robot.robots2026;

import static frc.robot.robots2026.Compare.SANITY_HIGH;
import static frc.robot.robots2026.Compare.SANITY_LOW;
import static frc.robot.robots2026.Compare.freeRpm;
import static frc.robot.robots2026.Compare.kV;
import static frc.robot.robots2026.Compare.sameMount;
import static frc.robot.robots2026.Parts.CANCODER;
import static frc.robot.robots2026.Parts.IN;
import static frc.robot.robots2026.Parts.encoder;
import static frc.robot.robots2026.Parts.inches;
import static frc.robot.robots2026.Parts.swapped;
import static frc.robot.robots2026.Parts.withOutputs;
import static frc.robot.robots2026.Team2611.BODY;
import static frc.robot.robots2026.Team2611.DRIVETRAIN;
import static frc.robot.robots2026.Team2611.MODULES;
import static frc.robot.robots2026.Team2611.ROBOT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.MetersPerSecond;

import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.BodyBuilder;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.game.Rebuilt;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * 2611's 2026 robot ({@link Team2611}) against what its code configures. Citations are to {@code
 * ~/src/2611-Vectors/2026-Beta-Bot} at 6a5da76, under {@code src/main/java/frc/robot/}.
 */
class Team2611Test {
  private final RobotMath math = ROBOT.math();

  @Test
  void itFitsTheSeason() {
    assertThat(Rebuilt.LIMITS.problems(ROBOT)).isEmpty();
    assertThat(BODY.mass().in(Kilograms))
        .as("PathPlanner's mass, settings.json:15")
        .isCloseTo(62.57, within(0.01));
  }

  /**
   * The drivetrain's Phoenix ratios as ModuleIOTalonFX sets them from Tuner's constants
   * (subsystems/drive/ModuleIOTalonFX.java:100,122; generated/TunerConstants.java:80-83), and its
   * modules where TunerConstants.java:135-169 puts them.
   */
  @Test
  void theSwerveModulesAreTunersOwn() {
    DrivetrainGeometry.ModuleJoints frontLeft = DRIVETRAIN.joints().get(0);
    Sensor.Encoder cancoder = encoder("front-left cancoder", frontLeft.steer(), 1, true, CANCODER);
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
   * (VectorKit/hardware/AbsoluteEncoder.java:233-235: a reading less its offset, plus a turn when
   * negative), which is the pivot's 0°: 1.5° from where it stows (Constants.java:89). A reading a
   * hair below 0 is 359°, and Pivot.intakeIsOut (Pivot.java:69) says out. The model puts the wrap
   * as far from the range as a turn allows. The team's hazard, with the stops guessed: theirs sits
   * on the range's edge, inside the fortieth of a turn the definition keeps clear, so a definition
   * whose encoder wrapped there would be refused.
   */
  @Test
  void thePivotsEncoderWrapsBesideItsStow() {
    double derived = math.discontinuityPoint(Team2611.PIVOT_ABSOLUTE);
    var range = Team2611.PIVOT.range().orElseThrow();

    assertThat(derived).isCloseTo(0.6875, within(1e-12));
    assertThat(RobotMath.wrapMargin(Team2611.PIVOT_ABSOLUTE, derived))
        .isCloseTo(0.3125, within(1e-12));
    assertThat(RobotMath.wrapMargin(Team2611.PIVOT_ABSOLUTE, 1.0))
        .as("theirs")
        .isCloseTo(0, within(1e-12))
        .isLessThan(RobotMath.WRAP_MARGIN);
    for (double setpoint : new double[] {1.5, 110, 130}) {
      assertThat(range.contains(Math.toRadians(setpoint))).as(setpoint + "°").isTrue();
    }
  }

  /**
   * Phoenix's SensorToMechanismRatio for each motor's rotor, which its drive implies. 2611 sets
   * none, so Phoenix reads every mechanism in rotor turns and the code divides by its ratio
   * (Pivot.java:83, Intake.java:43, FullSend.java:42, Transition.java:72); each rotor's ratio here
   * is the one its code divides by, written the motor-over-output way round.
   */
  @Test
  void eachRotorTurnsByItsDrivesRatio() {
    assertThat(math.sensorToMechanismRatio(Team2611.PIVOT_MOTOR.rotor())).isEqualTo(1 / (1.0 / 25));
    assertThat(math.sensorToMechanismRatio(Team2611.INTAKE_MOTOR.rotor())).isEqualTo(3);
    assertThat(math.sensorToMechanismRatio(Team2611.FULL_SEND_MOTOR.rotor())).isEqualTo(4);
    assertThat(math.sensorToMechanismRatio(Team2611.TRANSITION_MOTOR.rotor())).isEqualTo(2);
    assertThat(math.sensorToMechanismRatio(Team2611.SHOOTER_MOTORS.rotor())).isEqualTo(1);
    assertThat(Team2611.PIVOT_MOTOR.rotor().label()).isEqualTo("pivot rotor");
    assertThat(math.rotorToSensorRatio(Team2611.PIVOT_MOTOR, Team2611.PIVOT_ABSOLUTE))
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
    assertThat(freeRpm(math, Team2611.SHOOTER)).isGreaterThan(4500);
    assertThat(freeRpm(math, Team2611.FULL_SEND)).isGreaterThan(1000);
    assertThat(freeRpm(math, Team2611.TRANSITION)).isGreaterThan(2000);
    assertThat(freeRpm(math, Team2611.INTAKE)).isCloseTo(2000, within(1e-6));
  }

  /**
   * The names their code gives its places, now the definition's: the pivot's three angles
   * (Constants.java:89-91), the rollers' speeds (Intake.java:51-52, FullSend.java:50,
   * Transition.java:80), the driver's two feeds (RobotContainer.java:220-228), and the auto's guard
   * on the intake's roller (RobotContainer.java:143), an interlock.
   */
  @Test
  void theirPlacesAreNamedInTheDefinition() {
    assertThat(Team2611.PIVOT.positions())
        .extracting(Position::name)
        .containsExactly("in", "run", "out");
    assertThat(Team2611.PIVOT.position("run").value())
        .isCloseTo(Math.toRadians(110), within(1e-12));
    assertThat(Team2611.INTAKE.output("outtaking").value() * 60 / (2 * Math.PI))
        .isCloseTo(-500, within(1e-9));
    assertThat(ROBOT.configurations())
        .extracting(Configuration::name)
        .containsExactly("feeding", "unjamming");
    assertThat(Team2611.FEEDING.joints()).containsExactly(Team2611.FULL_SEND, Team2611.TRANSITION);
    assertThat(ROBOT.interlocks())
        .singleElement()
        .hasToString("intake runs only when pivot past run");
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
            Team2611.FULL_SEND, Output.rpm("feeding", 5000), Output.rpm("unjamming", -1000));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                swapped(
                    ROBOT,
                    Team2611.FULL_SEND,
                    autoFeed,
                    Team2611.FULL_SEND_MOTOR,
                    autoFeed.motors(Motor.KRAKEN_X60, 1, Parts.gearing(4))))
        .withMessage(
            "RobotDefinition[2611 2026].joints: full send's feeding, 5000 rpm, is 3.333 times its"
                + " free speed, 1500 rpm: its motors can't turn it that fast");
  }

  /**
   * The intake's 2000 rpm is exactly its motor's free speed through 3:1, so its velocity loop has
   * nothing left to hold it under load: noted, not refused.
   */
  @Test
  void theIntakesSpeedIsNotedAtItsFreeSpeed() {
    assertThat(ROBOT.notes())
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
    Configuration stowed = Configuration.of("intaking", Team2611.INTAKE.at("intaking"));
    Configuration out =
        Configuration.of("intaking", Team2611.PIVOT.at("out"), Team2611.INTAKE.at("intaking"));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> RobotDefinitionBuilder.builder(ROBOT).configurations(List.of(stowed)).build())
        .withMessage(
            "RobotDefinition[2611 2026].configurations: configuration intaking runs intake, but"
                + " intake runs only when pivot past run, and pivot is at 1.5°, its reference");
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
      {kV(math, Team2611.SHOOTER, 1), 0.12},
      {kV(math, Team2611.INTAKE, 3), 0.13},
      {kV(math, Team2611.FULL_SEND, 4), 0.13},
      {kV(math, Team2611.TRANSITION, 2), 0.14},
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
        Team2611.LEFT_REAR,
        new Transform3d(
            inches(-10.5, 12, 21),
            new Rotation3d(Math.toRadians(-2), Math.toRadians(-10), Math.toRadians(180.985))));
    sameMount(
        Team2611.LEFT_FRONT,
        new Transform3d(
            inches(-7.75, 12.25, 21),
            new Rotation3d(0, Math.toRadians(-10), Math.toRadians(23.162))));
    sameMount(
        Team2611.RIGHT_FRONT,
        new Transform3d(
            inches(-8.625, -12.25, 20.875),
            new Rotation3d(0, Math.toRadians(-8.2), Math.toRadians(-23.162))));
    Transform3d rightRear =
        new Transform3d(
                inches(-11.6875, -12, 16.837),
                new Rotation3d(0, Math.toRadians(-12.5), Math.toRadians(180.985)))
            .plus(new Transform3d(inches(0, 0, 1.938 + 1.93), Rotation3d.ZERO));
    assertThat(
            Team2611.RIGHT_REAR
                .robotToCamera()
                .getTranslation()
                .getDistance(rightRear.getTranslation()))
        .as("worked by hand to a ten-thousandth of an inch")
        .isLessThan(3e-6);
    assertThat(
            Team2611.RIGHT_REAR
                .robotToCamera()
                .getTranslation()
                .getDistance(inches(-11.6875, -12, 16.837 + 1.938 + 1.93)))
        .as("a lift straight up would be 0.84 in away")
        .isCloseTo(0.8422 * IN, within(1e-4 * IN));
  }

  /**
   * The pivot's gravity, which 2611 doesn't model (its controller's kS and kV are 0, and it has no
   * kG: Pivot.java:26-27), nor can the model: no seed, no mass. The arm's mass is unknown, so it
   * has no gravity to hold, no offset from horizontal, and its kA seeds from its motor's rotor
   * through 25:1 alone.
   */
  @Test
  void thePivotHasNoGravitySeedWithoutAMass() {
    RobotMath.Gains gains = math.idealGains(Team2611.PIVOT);
    double rotor = Motor.KRAKEN_X60.rotorInertia().in(KilogramSquareMeters) * 25 * 25;

    assertThat(gains.kG()).isZero();
    assertThat(math.gravityOffset(Team2611.PIVOT)).isZero();
    assertThat(math.gravityFit(Team2611.PIVOT).peak()).isZero();
    assertThat(math.inertia(Team2611.PIVOT, Map.of())).isCloseTo(rotor, within(1e-12));
  }

  /**
   * What the definition doesn't know. No link's mass, so no gravity seeds; the robot weighed, as
   * the code gives it; its yaw the code's 5.5 kg m² (Constants.java:188), its pitch and roll the
   * model's box, which would put the yaw at 6.89, the PathPlanner app's 6.883 (settings.json:16) to
   * a tenth of a percent.
   */
  @Test
  void theMissingDataReportListsEveryLinksMass() {
    ROBOT.missing().forEach(System.out::println);

    assertThat(ROBOT.missing())
        .extracting(RobotDefinition.Missing::what)
        .containsExactly(
            "the body's pitch and roll are a box's, beside its given yaw",
            "62.57 kg of the robot's 62.57 kg is in no link",
            "intake arm has no mass",
            "intake roller has no mass",
            "shooter wheels has no mass",
            "full send rollers has no mass",
            "transition rollers has no mass",
            "hood has no mass",
            "hood has no shape");
    assertThat(ROBOT.missing().get(2).costs())
        .isEqualTo(
            "pivot has no gravity seed (kG 0), no gravity torque, and no center of mass to shift;"
                + " its kA seeds from its motors' rotors alone");
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
    Actuator.Servo servo = Team2611.WCP_0408;
    Map<Joint, Double> lowered = Map.of(Team2611.HOOD, 0.0);
    Map<Joint, Double> raised = Map.of(Team2611.HOOD, Math.toRadians(40));

    assertThat(servo.command(math.coordinate(Team2611.HOOD_LEFT, lowered)))
        .isCloseTo(0.65, within(1e-9));
    assertThat(servo.command(math.coordinate(Team2611.HOOD_LEFT, raised)))
        .isCloseTo(1.0, within(0.001));
    assertThat(math.coordinate(Team2611.HOOD_RIGHT, raised))
        .isCloseTo(math.coordinate(Team2611.HOOD_LEFT, raised), within(1e-12));
    assertThat(math.torque(Team2611.HOOD_LEFT, Team2611.HOOD, lowered, 1.0))
        .as("told to extend, it lifts the hood")
        .isPositive();
    assertThat(math.torque(Team2611.HOOD_LEFT, Team2611.HOOD, lowered, 0.65))
        .as("told where it is, it holds")
        .isCloseTo(0, within(1e-9));
  }
}
