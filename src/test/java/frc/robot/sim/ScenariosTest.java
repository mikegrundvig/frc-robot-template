package frc.robot.sim;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatIllegalStateException;
import static org.assertj.core.api.Assertions.within;

import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.game.Rebuilt;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.Bump;
import frc.robot.lib.field.Terrain;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.RobotController;

/**
 * The simulator's scenarios: each is found by its name, a misspelled name lists them all, and each
 * does to the simulated world what it says. What the robot's code makes of each is for the
 * whole-robot tests ({@code ScenarioSimTest}).
 */
@ExtendWith(RobotSim.class)
class ScenariosTest {
  private static final List<CameraConfig> CAMERAS = Robots.SIM.cameras();

  /** Where the robot stands for the events: on blue's side, clear of everything. */
  private static final Pose2d STANDING = new Pose2d(2.0, 4.0, Rotation2d.ZERO);

  private final List<SimulatedWorld> worlds = new ArrayList<>();

  @AfterEach
  void tearDown() {
    worlds.forEach(SimulatedWorld::close);
  }

  /** The simulator's world on 2026's field, without cameras, sensors exact, the robot standing. */
  private SimulatedWorld world() {
    SimulatedWorld built =
        new SimulatedWorld(
            RobotSim.FIELD,
            Robots.SIM.drivetrain(),
            Robots.SIM.body(),
            List.of(),
            SensorErrors.NONE);
    worlds.add(built);
    built.place(STANDING);
    step(built);
    return built;
  }

  private static void step(SimulatedWorld world) {
    RobotSim.stepClock();
    world.step(RobotController.getTime());
  }

  private static SensorErrors sensorsIn(String scenario) {
    return Scenarios.named(scenario).errorsOn(CAMERAS);
  }

  private static void start(String scenario, SimulatedWorld world) {
    Scenarios.named(scenario).startIn(world);
  }

  static Stream<String> names() {
    return Scenarios.ALL.stream().map(Scenario::name);
  }

  @Test
  void everyScenarioHasItsOwnName() {
    List<String> names = names().toList();

    assertThat(names).doesNotHaveDuplicates();
    for (String name : names) {
      assertThat(Scenarios.named(name).name()).isEqualTo(name);
    }
  }

  /**
   * Every scenario does something: its faults make the sensors err differently from a season-worn
   * robot's, or its event changes where the robot is or what its sensors read, a step later, from a
   * world it didn't happen in.
   */
  @ParameterizedTest
  @MethodSource("names")
  void everyScenarioDoesSomething(String name) {
    Scenario scenario = Scenarios.named(name);
    SimulatedWorld untouched = world();
    SimulatedWorld started = world();

    scenario.startIn(started);

    boolean faults = !scenario.errorsOn(CAMERAS).equals(SensorErrors.TYPICAL);
    boolean happened = !afterATurn(started).equals(afterATurn(untouched));
    assertThat(faults || happened).isTrue();
  }

  /**
   * A step after the robot is turned in place (so a frozen gyro shows): where it is, its tilt, and
   * what its gyros and wheels read.
   */
  private static List<Object> afterATurn(SimulatedWorld world) {
    world.drivetrain().shove(new Transform2d(0, 0, Rotation2d.fromDegrees(30)));
    step(world);
    GyroIO.GyroIOInputs gyro = read(world.drivetrain().gyro());
    GyroIO.GyroIOInputs backup = read(world.drivetrain().backupGyro());
    List<Object> seen =
        new ArrayList<>(
            List.of(
                world.truePose(),
                world.drivetrain().pitchRadians(),
                world.drivetrain().rollRadians(),
                gyro.connected,
                gyro.yaw,
                backup.connected,
                backup.yaw));
    for (ModuleIO module : world.drivetrainHardware().modules()) {
      seen.add(read(module).drivePositionRad);
    }
    return seen;
  }

  @Test
  void aMisspelledNameListsEveryScenario() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Scenarios.named("colision"))
        .withMessageStartingWith("No simulator scenario is named \"colision\". The scenarios: ")
        .withMessageContaining(
            Scenarios.ALL.stream().map(Scenario::name).collect(Collectors.joining(", ")));
  }

  @Test
  void theListingGivesEachNameAndWhatItDoes() {
    String listing = Scenarios.listing();

    assertThat(listing.lines()).hasSize(Scenarios.ALL.size());
    assertThat(listing)
        .contains("collision        Another robot hits it")
        .contains("camera-low       The robot's first camera knocked 4 in low");
    assertThat(Scenarios.line(Scenarios.named("shove")))
        .startsWith("shove            Pushed 0.6 m")
        .endsWith(System.lineSeparator());
  }

  @Test
  void aNameIsLowercaseWordsJoinedByHyphensAndADescriptionSaysSomething() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Scenario.builder()
                    .name("Camera Low")
                    .description(" ")
                    .sensorFaults(Scenario.NO_FAULTS)
                    .event(Scenario.NOTHING)
                    .build())
        .withMessageContaining(
            "Scenario.name: \"Camera Low\" must be lowercase words joined by hyphens")
        .withMessageContaining("Scenario.description: must not be blank");
  }

  @Test
  void aCameraKnockedLowIsTheFirstCameraAlone() {
    SensorErrors errors = sensorsIn("camera-low");

    String first = CAMERAS.get(0).name();
    assertThat(errors.cameraMountError(first).getZ()).isCloseTo(-0.1016, within(1e-9));
    assertThat(errors.cameraMountError(CAMERAS.get(1).name()).getZ()).isZero();
    assertThat(errors.gyro()).isEqualTo(SensorErrors.TYPICAL.gyro());
  }

  @Test
  void aCameraTurnedIsTwoDegreesOfYaw() {
    SensorErrors errors = sensorsIn("camera-turned");

    var turn = errors.cameraMountError(CAMERAS.get(0).name()).getRotation();
    assertThat(Math.toDegrees(turn.getZ())).isCloseTo(2, within(1e-9));
    assertThat(turn.getX()).isCloseTo(0, within(1e-12));
    assertThat(turn.getY()).isCloseTo(0, within(1e-12));
  }

  @Test
  void aCameraScenarioNeedsACamera() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Scenarios.named("camera-low").errorsOn(List.of()))
        .withMessage("this scenario knocks a camera, and the robot has none");
  }

  @Test
  void theGyroScaleIsTheGyrosAlone() {
    SensorErrors errors = sensorsIn("gyro-scale");

    assertThat(errors.gyro().scale()).isEqualTo(1.15);
    assertThat(errors.gyro().driftDegreesPerMinute())
        .isEqualTo(SensorErrors.TYPICAL.gyro().driftDegreesPerMinute());
    assertThat(errors.backupGyro()).isEqualTo(SensorErrors.TYPICAL.backupGyro());
  }

  @Test
  void wornTreadAndAModuleZeroedOffChangeTheWheels() {
    assertThat(sensorsIn("worn-tread").wheelRadiusScale()).containsOnly(0.97);
    assertThat(sensorsIn("module-zero-off").steerOffsetDegrees())
        .containsExactly(
            5.0,
            SensorErrors.TYPICAL.steerOffsetDegrees().get(1),
            SensorErrors.TYPICAL.steerOffsetDegrees().get(2),
            SensorErrors.TYPICAL.steerOffsetDegrees().get(3));
    assertThat(sensorsIn("exact-sensors")).isEqualTo(SensorErrors.NONE);
  }

  @Test
  void aCollisionMovesTheRobotAndJoltsItsImu() {
    SimulatedWorld world = world();

    start("collision", world);

    assertThat(world.truePose().getY()).isCloseTo(4.6, within(1e-9));
    assertThat(world.drivetrain().feltAcceleration().getY())
        .isCloseTo(DriveSimulation.HIT_JOLT_METERS_PER_SECOND_SQUARED, within(1e-9));
  }

  @Test
  void aShoveMovesTheRobotUnfelt() {
    SimulatedWorld world = world();

    start("shove", world);

    assertThat(world.truePose().getY()).isCloseTo(4.6, within(1e-9));
    assertThat(world.drivetrain().feltAcceleration().getNorm()).isCloseTo(0, within(1e-9));
  }

  @Test
  void aSlippingWheelTurnsWithoutMovingTheRobot() {
    SimulatedWorld world = world();
    ModuleIO[] modules = world.drivetrainHardware().modules().toArray(ModuleIO[]::new);
    double before = read(modules[0]).drivePositionRad;

    start("wheel-slip", world);
    step(world);

    assertThat(read(modules[0]).drivePositionRad - before).isCloseTo(Math.PI, within(1e-9));
    assertThat(read(modules[1]).drivePositionRad).isCloseTo(0, within(1e-9));
    assertThat(world.truePose()).isEqualTo(STANDING);
  }

  /** Every wheel pushed the same, the robot goes three quarters as far with a corner up. */
  @Test
  void aLiftedWheelTiltsTheRobotAndItsWheelsMoveItLess() {
    double flat = distanceDriven(world());
    SimulatedWorld world = world();

    start("wheel-lifted", world);
    double lifted = distanceDriven(world);

    assertThat(Math.toDegrees(world.drivetrain().pitchRadians()))
        .as("nose up")
        .isCloseTo(-5, within(1e-9));
    assertThat(Math.toDegrees(world.drivetrain().rollRadians()))
        .as("left side up")
        .isCloseTo(5, within(1e-9));
    assertThat(lifted / flat).isCloseTo(0.75, within(1e-9));
  }

  /** How far the robot goes with every drive motor at 3 V for a second. */
  private static double distanceDriven(SimulatedWorld world) {
    for (ModuleIO module : world.drivetrainHardware().modules()) {
      module.setDriveVoltage(3.0);
    }
    for (int i = 0; i < RobotLoop.loops(1.0); i++) {
      step(world);
    }
    return world.truePose().getTranslation().getDistance(STANDING.getTranslation());
  }

  @Test
  void theGyroUnpluggedLeavesTheBackup() {
    SimulatedWorld world = world();

    start("gyro-unplugged", world);
    step(world);

    assertThat(read(world.drivetrain().gyro()).connected).isFalse();
    assertThat(read(world.drivetrain().backupGyro()).connected).isTrue();
  }

  @Test
  void bothImusUnpluggedLeaveNeither() {
    SimulatedWorld world = world();

    start("imus-unplugged", world);
    step(world);

    assertThat(read(world.drivetrain().gyro()).connected).isFalse();
    assertThat(read(world.drivetrain().backupGyro()).connected).isFalse();
  }

  @Test
  void aFrozenGyroMissesATurn() {
    SimulatedWorld world = world();
    double before = read(world.drivetrain().gyro()).yaw.getDegrees();

    start("gyro-frozen", world);
    world.place(new Pose2d(STANDING.getTranslation(), Rotation2d.fromDegrees(30)));
    step(world);

    GyroIO.GyroIOInputs gyro = read(world.drivetrain().gyro());
    assertThat(gyro.connected).isTrue();
    assertThat(gyro.yaw.getDegrees()).isEqualTo(before);
    assertThat(read(world.drivetrain().backupGyro()).yaw.getDegrees()).isCloseTo(30, within(1e-9));
  }

  /** From either side, the robot ends up clear of the nearest bump's ramp, facing its crest. */
  @Test
  void aBumpCrossingSetsTheRobotBeforeTheNearestBumpFacingIt() {
    for (double x : new double[] {3.0, 6.0}) { // before blue's left bump, and past it
      SimulatedWorld world = world();
      world.place(new Pose2d(x, 5.5, Rotation2d.fromDegrees(70)));

      start("bump-crossing", world);

      Bump bump = Rebuilt.LEFT_BUMP;
      Translation2d spot = world.truePose().getTranslation();
      double offset = bump.acrossOffset(spot);
      assertThat(Math.abs(offset))
          .as("from x = " + x)
          .isCloseTo(bump.depthMeters() / 2 + Scenarios.BUMP_CLEARANCE_METERS, within(1e-9));
      assertThat(Math.signum(offset))
          .isEqualTo(Math.signum(bump.acrossOffset(new Translation2d(x, 5.5))));
      assertThat(bump.alongOffset(spot)).isCloseTo(bump.length() / 2, within(1e-9));
      Translation2d ahead = spot.plus(new Translation2d(0.1, world.truePose().getRotation()));
      assertThat(Math.abs(bump.acrossOffset(ahead)))
          .as("facing the crest")
          .isLessThan(Math.abs(offset));
    }
  }

  @Test
  void aBumpCrossingNeedsABump() {
    SimulatedWorld world = world();
    world.drivetrain().setTerrain(Terrain.FLAT);

    assertThatIllegalStateException()
        .isThrownBy(() -> start("bump-crossing", world))
        .withMessage("bump-crossing needs a field with bumps, and this one has none");
  }

  private static ModuleIOInputs read(ModuleIO module) {
    ModuleIOInputs inputs = new ModuleIOInputs();
    module.updateInputs(inputs);
    return inputs;
  }

  private static GyroIO.GyroIOInputs read(GyroIO gyro) {
    GyroIO.GyroIOInputs inputs = new GyroIO.GyroIOInputs();
    gyro.updateInputs(inputs);
    return inputs;
  }
}
