package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.camera;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.definition;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.drivetrain;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Rotations;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/** A robot's parts are checked against each other when the definition is written down. */
class RobotDefinitionTest {
  private static final Translation3d AXLE = new Translation3d(0.25, 0, 0.32);

  @Test
  void wheelPositionsComeFromTheDrivetrain() {
    assertThat(definition(List.of()).moduleTranslations())
        .containsExactly(drivetrain().moduleTranslations());
  }

  @Test
  void theWheelsMustFitTheFrame() {
    var tooWide =
        DrivetrainGeometry.uniform(ConfigFixtures.modules(), Inches.of(11), Inches.of(13));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> RobotDefinitionBuilder.builder(definition(List.of())).drivetrain(tooWide).build())
        .withMessageStartingWith("RobotDefinition[test].drivetrain.corners");
  }

  /** A robot has a name, which every message about its definition starts with. */
  @Test
  void aRobotNeedsAName() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> RobotDefinitionBuilder.builder(definition(List.of())).name(" ").build())
        .withMessageStartingWith("RobotDefinition[ ].name");
  }

  @Test
  void cameraNamesMustBeUnique() {
    var cameras = List.of(camera("front", inches(10, 0, 8)), camera("front", inches(-10, 0, 8)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(cameras))
        .withMessageContaining("two cameras are named front");
  }

  @Test
  void cameraNamesMustBeUniqueAsIdsToo() {
    var cameras =
        List.of(camera("Front Left", inches(10, 0, 8)), camera("front-left", inches(-10, 0, 8)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(cameras))
        .withMessageContaining(
            "cameras \"Front Left\" and \"front-left\" differ only in case or punctuation, so"
                + " they're one id, \"front-left\": name them apart");
  }

  @Test
  void camerasMustBeOnTheRobot() {
    // 10 m ahead: the kind of mistake made by typing inches where meters belong.
    var cameras = List.of(camera("front", new Translation3d(10, 0, 0.2)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(cameras))
        .withMessageContaining("camera front is outside the robot");
  }

  /**
   * An IMU is mounted inside the robot, as a camera is: the backup 2 m up, or the gyro 1 m behind,
   * is a mistake in units or sign.
   */
  @Test
  void imusMustBeOnTheRobot() {
    ImuMount high = ImuMountBuilder.builder(ImuMount.UNCHECKED).up(Inches.of(80)).build();
    ImuMount behind = ImuMountBuilder.builder(ImuMount.UNCHECKED).forward(Inches.of(-40)).build();
    RobotDefinition robot = definition(List.of());

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(robot)
                    .imus(ImusBuilder.builder(Imus.UNCHECKED).backup(high).build())
                    .build())
        .withMessageContaining("the backup IMU is outside the robot");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(robot)
                    .imus(ImusBuilder.builder(Imus.UNCHECKED).gyro(behind).build())
                    .build())
        .withMessageContaining("the gyro is outside the robot");
    assertThat(
            RobotDefinitionBuilder.builder(robot)
                .imus(ImusBuilder.builder(Imus.UNCHECKED).backup(Optional.empty()).build())
                .build()
                .imus()
                .backup())
        .as("a roboRIO robot has no backup IMU")
        .isEmpty();
  }

  @Test
  void anImusOrientationIsARotation() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                ImuMountBuilder.builder(ImuMount.UNCHECKED)
                    .orientation(new Rotation3d(Double.NaN, 0, 0))
                    .build())
        .withMessageContaining("ImuMount.orientation");
  }

  @Test
  void theCameraListCannotChangeAfterwards() {
    List<CameraConfig> cameras = new ArrayList<>(List.of(camera("front", inches(10, 0, 8))));
    RobotDefinition robot = definition(cameras);

    cameras.clear();

    assertThat(robot.cameras()).hasSize(1);
  }

  /**
   * A practice robot is the competition robot's joints, each changed where it's given and kept
   * otherwise: a CAD revision that regears the competition robot's pivot regears the practice
   * robot's, whose motors are only re-wired. Restating its motors, which would keep the old
   * gearing, is refused, saying how to re-wire them.
   */
  @Test
  void aRegearOfTheCompetitionRobotReachesThePracticeRobot() {
    for (double ratio : new double[] {25, 30}) {
      Joint.Revolute pivot = pivot(Gearing.of(ratio));
      RobotDefinition practice = robot(pivot).with(pivot.motorsOn(TALON_FX.can(33)));

      Driven.Motors motors = practice.joints().getFirst().driven().motors().orElseThrow();
      assertThat(motors.gearing()).isEqualTo(Gearing.of(ratio));
      assertThat(motors.controllers()).containsExactly(TALON_FX.can(33));
    }
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> pivot(Gearing.of(30)).motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(33)))
        .withMessageStartingWith("pivot already has its motors: to re-wire them, motorsOn(...)");
  }

  /**
   * A practice robot is the same robot with its parts re-wired: each joint, sensor or drivetrain
   * given replaces the robot's own it's equal to, the same but for how it's driven or wired. One
   * the robot hasn't is refused; so is one given twice, whose second would drop the first's change;
   * a joint's own switch or encoder, re-wired with its joint; and a drivetrain of other modules.
   */
  @Test
  void aPracticeRobotIsTheSameRobotWithItsPartsRewired() {
    Joint.Revolute pivot =
        pivot(Gearing.of(25))
            .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.2)))
            .limitSwitch(
                Sensor.Switch.Limit.End.LOWER,
                Degrees.of(0),
                SensorWiring.port(1).normallyClosed());
    Sensor.Switch beam =
        Sensor.Switch.beam("claw beam", pivot).on(SensorWiring.port(2).normallyOpen());
    RobotDefinition competition =
        RobotDefinitionBuilder.builder(robot(pivot)).sensors(List.of(beam)).build();
    SwerveWiring wiring =
        SwerveWiring.builder()
            .canBus(CanBus.SYSTEMCORE)
            .driveController(TALON_FX)
            .steerController(TALON_FX)
            .gyroId(1)
            .gyroYawScale(1.0)
            .modules(List.of(module(1, 2, 1), module(3, 4, 2), module(5, 6, 3), module(7, 8, 4)))
            .build();

    RobotDefinition practice =
        competition
            .with(
                pivot
                    .motorsOn(TALON_FX.can(33))
                    .limitSwitchOn(
                        Sensor.Switch.Limit.End.LOWER, SensorWiring.port(3).normallyClosed()))
            .with(beam.on(CANDI.can(36).s1().normallyOpen()))
            .with(competition.drivetrain().wired(wiring));

    assertThat(practice.joints()).containsExactly(pivot);
    assertThat(practice.joints().getFirst().switchAt(Sensor.Switch.Limit.End.LOWER).wiring())
        .contains(SensorWiring.port(3).normallyClosed());
    assertThat(practice.sensors()).containsExactly(beam);
    assertThat(((Sensor.Switch) practice.sensors().getFirst()).wiring())
        .contains(CANDI.can(36).s1().normallyOpen());
    assertThat(practice.drivetrain()).isEqualTo(competition.drivetrain());
    assertThat(practice.drivetrain().wiring()).contains(wiring);
    assertThat(practice.devices())
        .extracting(CanDevice::name)
        .contains("pivot motor 33", "CANdi 36", "the gyro");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(Joint.roller("roller", Part.CHASSIS, AXLE)))
        .withMessage(
            "test has no joint roller the same but for how it's driven, its stops, where it rests"
                + " and where its places are");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                competition.with(
                    pivot.motorsOn(TALON_FX.can(33)), pivot.retuned(control -> control.withKd(1))))
        .withMessage(
            "pivot is given twice, and the second would drop the first's change: give it once,"
                + " its changes one after another");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(Sensor.Switch.beam("hopper beam", Part.CHASSIS)))
        .withMessage("test has no sensor hopper beam the same but for its wiring");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                competition.with(
                    pivot
                        .switchAt(Sensor.Switch.Limit.End.LOWER)
                        .on(SensorWiring.port(4).normallyClosed())))
        .withMessage(
            "pivot lower limit is pivot's own: re-wire it with its joint,"
                + " limitSwitchOn(End.LOWER, ...)");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(pivot.ownSensors().getFirst()))
        .withMessage(
            "pivot absolute encoder is pivot's own: re-wire it with its joint, encoderOn(...)");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                competition.with(
                    DrivetrainGeometry.uniform(
                        ConfigFixtures.modules(), Inches.of(10), Inches.of(10))))
        .withMessage(
            "test's drivetrain isn't this one: their modules, or where they sit, differ. Give its"
                + " own, re-wired, .wired(...), or re-tuned, .control(...)");
  }

  /**
   * A practice robot whose encoders differ only in their offsets says the offsets alone, so the
   * competition robot's CANcoders moved to other IDs, its joint's and its modules', reach it: the
   * silent divergence a whole CANcoder said again would leave. A drivetrain that isn't wired has no
   * offsets to change.
   */
  @Test
  void aCompetitionRobotsMovedCanIdReachesItsPracticeRobot() {
    for (int id : new int[] {34, 44}) {
      Joint.Revolute pivot =
          pivot(Gearing.of(25)).absoluteEncoder(CANCODER.can(id).offset(Rotations.of(0.2)));
      RobotDefinition competition =
          RobotDefinitionBuilder.builder(robot(pivot))
              .drivetrain(
                  drivetrain()
                      .wired(
                          SwerveWiring.builder()
                              .canBus(CanBus.SYSTEMCORE)
                              .driveController(TALON_FX)
                              .steerController(TALON_FX)
                              .gyroId(1)
                              .gyroYawScale(1.0)
                              .modules(
                                  List.of(
                                      module(1, 2, id + 1),
                                      module(3, 4, id + 2),
                                      module(5, 6, id + 3),
                                      module(7, 8, id + 4)))
                              .build()))
              .build();

      RobotDefinition practice =
          competition
              .with(pivot.encoderOffset(Rotations.of(0.47)))
              .with(
                  competition
                      .drivetrain()
                      .encoderOffsets(
                          Rotations.of(0.1),
                          Rotations.of(0.2),
                          Rotations.of(0.3),
                          Rotations.of(0.4)));

      assertThat(
              practice
                  .joints()
                  .getFirst()
                  .driven()
                  .absoluteEncoder()
                  .flatMap(Driven.AbsoluteEncoder::cancoder))
          .contains(CANCODER.can(id).offset(Rotations.of(0.47)));
      List<SwerveWiring.ModuleWiring> modules =
          practice.drivetrain().wiring().orElseThrow().modules();
      assertThat(modules)
          .extracting(SwerveWiring.ModuleWiring::encoderId)
          .containsExactly(
              Optional.of(id + 1), Optional.of(id + 2), Optional.of(id + 3), Optional.of(id + 4));
      assertThat(modules)
          .extracting(SwerveWiring.ModuleWiring::encoderOffset)
          .containsExactly(
              Rotations.of(0.1), Rotations.of(0.2), Rotations.of(0.3), Rotations.of(0.4));
      assertThat(modules.getFirst().driveId()).isEqualTo(1);
    }
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                drivetrain()
                    .encoderOffsets(
                        Rotations.of(0), Rotations.of(0), Rotations.of(0), Rotations.of(0)))
        .withMessage(
            "the drivetrain isn't wired, so its modules have no offsets: wire it, .wired(...)");
  }

  /** A practice robot is named on its definition, as on its configuration, the rest kept. */
  @Test
  void aPracticeRobotIsTheRobotUnderAnotherName() {
    RobotDefinition competition = robot(pivot(Gearing.of(25)));

    RobotDefinition practice = competition.named("practice");

    assertThat(practice.name()).isEqualTo("practice");
    assertThat(practice.joints()).isEqualTo(competition.joints());
    assertThat(practice.drivetrain()).isEqualTo(competition.drivetrain());
  }

  /**
   * A camera mounted otherwise is given whole, and replaces the robot's camera of its name: a
   * practice robot's back camera, a fraction of an inch lower. One the robot has no camera of the
   * name of is refused, and so is one given twice.
   */
  @Test
  void aPracticeRobotsCameraIsMountedOtherwise() {
    CameraConfig back = camera("back", inches(-10, 0, 20));
    CameraConfig front = camera("front", inches(10, 0, 20));
    RobotDefinition competition = definition(List.of(front, back));
    CameraConfig lower = CameraConfigBuilder.builder(back).up(Inches.of(19.81)).build();

    RobotDefinition practice = competition.with(lower);

    assertThat(practice.cameras()).containsExactly(front, lower);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(camera("side", inches(0, 10, 20))))
        .withMessage("test has no camera named side: it has front, back");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(lower, back))
        .withMessage(
            "back is given twice, and the second would drop the first's change: give it once,"
                + " its changes one after another");
  }

  /**
   * A drivetrain whose modules differ only in what the simulator alone reads, their inertias, as a
   * practice robot's Tuner X simulation values can, is the robot's own, and the robot takes them.
   * One whose tread grips otherwise, which the robot plans by, is another.
   */
  @Test
  void aDrivetrainDifferingOnlyInItsSimulatorsValuesIsTheRobotsOwn() {
    RobotDefinition competition = definition(List.of());
    DrivetrainGeometry.ModuleType heavier =
        DrivetrainGeometryModuleTypeBuilder.builder(ConfigFixtures.modules())
            .driveInertia(KilogramSquareMeters.of(0.01))
            .steerInertia(KilogramSquareMeters.of(0.035))
            .build();
    DrivetrainGeometry practiceModules =
        DrivetrainGeometry.inset(heavier, ConfigFixtures.body(), Inches.of(2.625));

    RobotDefinition practice = competition.with(practiceModules);

    assertThat(practice.drivetrain().module(0).steerInertia())
        .contains(KilogramSquareMeters.of(0.035));
    DrivetrainGeometry.ModuleType grippier =
        DrivetrainGeometryModuleTypeBuilder.builder(ConfigFixtures.modules())
            .wheelCoefficientOfFriction(1.0)
            .build();
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                competition.with(
                    DrivetrainGeometry.inset(grippier, ConfigFixtures.body(), Inches.of(2.625))))
        .withMessageStartingWith("test's drivetrain isn't this one");
  }

  /** A pivot of the robot's, as its block has it, geared as CAD has it. */
  private static Joint.Revolute pivot(Gearing gearing) {
    return Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_DOWN)
        .range(Range.degrees(0, 135))
        .rest(Degrees.of(1.5))
        .motor(Motor.KRAKEN_X60, gearing, TALON_FX.can(32))
        .control(
            MechanismControl.position(30)
                .withHoming(MechanismControl.Homing.toHardStop(Sensor.Switch.Limit.End.LOWER)));
  }

  /** The test robot with these joints. */
  private static RobotDefinition robot(Joint... joints) {
    return RobotDefinitionBuilder.builder(definition(List.of())).joints(List.of(joints)).build();
  }

  private static SwerveWiring.ModuleWiring module(int drive, int steer, int encoder) {
    return SwerveWiring.ModuleWiring.builder()
        .driveId(drive)
        .steerId(steer)
        .encoderOffset(Rotations.of(0))
        .driveInverted(false)
        .steerInverted(false)
        .encoderInverted(false)
        .encoderId(encoder)
        .build();
  }

  private static Translation3d inches(double x, double y, double z) {
    return new Translation3d(Inches.of(x), Inches.of(y), Inches.of(z));
  }
}
