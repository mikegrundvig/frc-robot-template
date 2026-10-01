package frc.robot.sim;

import static frc.robot.sim.Faults.BACKUP_IMU;
import static frc.robot.sim.Faults.GYRO;
import static frc.robot.sim.Faults.ROBOT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.RobotController;

/**
 * The simulated world's live faults, as the page applies them: each changes what the robot's code
 * reads from its sensors (or where the robot really is), and undoing it puts that back. The robot's
 * code reads only its IO's inputs, so these tests read the same. The cameras' faults are in {@code
 * CameraFaultsSimTest}.
 */
@ExtendWith(RobotSim.class)
class FaultsTest {
  /** Where the robot stands: on blue's side, clear of everything. */
  private static final Pose2d STANDING = new Pose2d(2.0, 4.0, Rotation2d.ZERO);

  private final List<SimulatedWorld> worlds = new ArrayList<>();
  private final List<String> said = new ArrayList<>();
  private Components components = new Components(said::add);

  @AfterEach
  void tearDown() {
    worlds.forEach(SimulatedWorld::close);
  }

  /** A world with sensors exactly as configured, its parts declared, the robot standing. */
  private SimulatedWorld world() {
    SimulatedWorld built =
        new SimulatedWorld(
            RobotSim.FIELD,
            Robots.SIM.drivetrain(),
            Robots.SIM.body(),
            List.of(),
            SensorErrors.NONE);
    worlds.add(built);
    components = new Components(said::add);
    built.addTo(components);
    built.place(STANDING);
    step(built);
    return built;
  }

  private static void step(SimulatedWorld world) {
    RobotSim.stepClock();
    world.step(RobotController.getTime());
  }

  private void apply(String component, String fault, Map<String, Double> values) {
    assertThat(components.apply(Setting.of(component, fault, values))).isNull();
  }

  private void apply(String component, String fault) {
    apply(component, fault, Map.of());
  }

  private void undo(String component, String fault) {
    assertThat(components.undo(component, fault)).isNull();
  }

  private static GyroIO imu(SimulatedWorld world, String name) {
    return name.equals(GYRO) ? world.drivetrain().gyro() : world.drivetrain().backupGyro();
  }

  /** Turns the robot in place without its wheels: its IMUs feel it. */
  private static void turn(SimulatedWorld world, double degrees) {
    world.drivetrain().shove(new Transform2d(0, 0, Rotation2d.fromDegrees(degrees)));
    step(world);
  }

  @Test
  void asBuiltNothingIsOn() {
    world();

    assertThat(components.on()).isEmpty();
    assertThat(components.components())
        .extracting(c -> c.name())
        .containsExactly(
            GYRO,
            BACKUP_IMU,
            Faults.module(0),
            Faults.module(1),
            Faults.module(2),
            Faults.module(3),
            ROBOT);
  }

  @ParameterizedTest
  @ValueSource(strings = {GYRO, BACKUP_IMU})
  void anUnpluggedImuReadsNothingUntilPluggedBackIn(String name) {
    SimulatedWorld world = world();
    read(imu(world, name)); // what it read before

    apply(name, Faults.UNPLUGGED);
    step(world);
    GyroIO.GyroIOInputs unplugged = read(imu(world, name));
    undo(name, Faults.UNPLUGGED);
    step(world);

    assertThat(unplugged.connected).isFalse();
    assertThat(unplugged.odometryYawsRad).isEmpty();
    assertThat(read(imu(world, name)).connected).isTrue();
    assertThat(components.on()).isEmpty();
  }

  @ParameterizedTest
  @ValueSource(strings = {GYRO, BACKUP_IMU})
  void aFrozenImuMissesTurnsUntilUndone(String name) {
    SimulatedWorld world = world();

    apply(name, Faults.FROZEN);
    turn(world, 30);
    double frozen = read(imu(world, name)).yaw.getDegrees();
    undo(name, Faults.FROZEN);
    turn(world, 10);

    assertThat(frozen).isCloseTo(0, within(1e-9));
    assertThat(read(imu(world, name)).yaw.getDegrees()).isCloseTo(40, within(1e-9));
  }

  /** A scale error changes the turns it reads from then on, without a jump; undoing it too. */
  @ParameterizedTest
  @ValueSource(strings = {GYRO, BACKUP_IMU})
  void anImuErringReadsTurnsByItsNewScaleFromNowOn(String name) {
    SimulatedWorld world = world();
    turn(world, 20);

    apply(name, Faults.ERRORS, Map.of(Faults.SCALE, 1.5));
    step(world);
    double atTheChange = read(imu(world, name)).yaw.getDegrees();
    turn(world, 30);
    double erring = read(imu(world, name)).yaw.getDegrees();
    undo(name, Faults.ERRORS);
    turn(world, 30);

    assertThat(atTheChange).as("no jump").isCloseTo(20, within(1e-9));
    assertThat(erring).isCloseTo(20 + 45, within(1e-9));
    assertThat(read(imu(world, name)).yaw.getDegrees()).isCloseTo(20 + 45 + 30, within(1e-9));
  }

  @ParameterizedTest
  @ValueSource(strings = {GYRO, BACKUP_IMU})
  void aResetImuReadsZeroWhereverTheRobotPoints(String name) {
    SimulatedWorld world = world();
    turn(world, 30);

    apply(name, Faults.RESET);
    step(world);
    double reset = read(imu(world, name)).yaw.getDegrees();
    turn(world, 10);

    assertThat(reset).isCloseTo(0, within(1e-9));
    assertThat(read(imu(world, name)).yaw.getDegrees()).isCloseTo(10, within(1e-9));
    assertThat(components.undo(name, Faults.RESET))
        .as("an event isn't undone")
        .contains("happens once, and nothing undoes it");
  }

  /**
   * A stale module's readings stop while its wheel turns and steers, and catch up once it's undone.
   */
  @Test
  void aStaleModuleStopsCountingUntilItCatchesUp() {
    SimulatedWorld world = world();
    ModuleIO module = world.drivetrainHardware().modules().get(0);
    drive(world, 3.0, 0.2);

    apply(Faults.module(0), Faults.STALE);
    ModuleIOInputs before = read(module);
    module.configure(Robots.SIM.drivetrain().gains(), Robots.SIM.drivetrain().limits());
    module.setSteerAngle(Rotation2d.fromDegrees(30));
    drive(world, 3.0, 0.5);
    ModuleIOInputs stale = read(module);
    drive(world, 3.0, 0.1);
    double truth = world.drivetrain().moduleSim(0).wheelRotationRadians();
    undo(Faults.module(0), Faults.STALE);
    step(world);
    ModuleIOInputs caughtUp = read(module);

    assertThat(stale.drivePositionRad).isEqualTo(before.drivePositionRad);
    assertThat(stale.odometryDrivePositionsRad).containsOnly(before.drivePositionRad);
    assertThat(stale.steerAngle).isEqualTo(before.steerAngle);
    assertThat(read(world.drivetrainHardware().modules().get(1)).drivePositionRad)
        .as("the others count on")
        .isGreaterThan(before.drivePositionRad + 1);
    assertThat(caughtUp.drivePositionRad).as("caught up").isGreaterThan(truth);
    assertThat(caughtUp.steerAngle.getDegrees())
        .as("steered while stale")
        .isCloseTo(30, within(1.0));
  }

  /** A zero knocked off: the module reads that far from where it points, until undone. */
  @Test
  void aZeroKnockedOffShiftsWhatTheModuleReads() {
    SimulatedWorld world = world();
    ModuleIO module = world.drivetrainHardware().modules().get(2);
    double before = read(module).steerAngle.getDegrees();

    apply(Faults.module(2), Faults.ZEROED_OFF, Map.of(Faults.OFFSET, 10.0));
    step(world);
    double off = read(module).steerAngle.getDegrees();
    undo(Faults.module(2), Faults.ZEROED_OFF);
    step(world);

    assertThat(off - before).isCloseTo(-10, within(1e-6));
    assertThat(read(module).steerAngle.getDegrees()).isCloseTo(before, within(1e-6));
  }

  /** Worn tread: the robot goes less far than its encoders count, until it's new again. */
  @Test
  void wornTreadMovesTheRobotLessThanItsEncodersCount() {
    double fresh = drive(world(), 3.0, 1.0);
    SimulatedWorld worn = world();
    for (int i = 0; i < 4; i++) {
      apply(Faults.module(i), Faults.WORN, Map.of(Faults.SCALE, 0.9));
    }

    double wornDistance = drive(worn, 3.0, 1.0);
    for (int i = 0; i < 4; i++) {
      undo(Faults.module(i), Faults.WORN);
    }

    assertThat(wornDistance / fresh).isCloseTo(0.9, within(1e-6));
    assertThat(worn.drivetrain().treadScale(0)).isEqualTo(1.0);
    assertThat(components.on()).isEmpty();
  }

  @Test
  void aWheelSlipCountsTurnsTheFloorNeverSaw() {
    SimulatedWorld world = world();
    ModuleIO module = world.drivetrainHardware().modules().get(3);
    double before = read(module).drivePositionRad;

    apply(Faults.module(3), Faults.SLIPS, Map.of(Faults.TURNS, 1.0));
    step(world);

    assertThat(read(module).drivePositionRad - before).isCloseTo(2 * Math.PI, within(1e-9));
    assertThat(world.truePose()).isEqualTo(STANDING);
  }

  @Test
  void aShoveMovesTheRobotInItsOwnFrameAndItsGyroFeelsTheTurn() {
    SimulatedWorld world = world();

    apply(ROBOT, Faults.SHOVED, Map.of(Faults.FORWARD, 1.0, Faults.LEFT, 0.0, "Turn", 90.0));
    step(world);

    assertThat(world.truePose().getX()).isCloseTo(3.0, within(1e-9));
    assertThat(read(world.drivetrain().gyro()).yaw.getDegrees()).isCloseTo(90, within(1e-9));
  }

  @Test
  void aHitJoltsTheImu() {
    SimulatedWorld world = world();

    apply(ROBOT, Faults.HIT, Map.of(Faults.FORWARD, 0.5, Faults.LEFT, 0.0));
    step(world);

    assertThat(read(world.drivetrain().gyro()).accelerationXMetersPerSecSq)
        .isGreaterThan(DriveSimulation.HIT_JOLT_METERS_PER_SECOND_SQUARED / 2);
  }

  /** Tilted by hand, its IMU reads the tilt; undone, it's back on the (flat) ground. */
  @Test
  void aTiltIsReadByTheImuUntilTheRobotIsSetBackDown() {
    SimulatedWorld world = world();

    apply(ROBOT, Faults.TILTED, Map.of(Faults.PITCH, -5.0, Faults.ROLL, 3.0));
    step(world);
    GyroIO.GyroIOInputs tilted = read(world.drivetrain().gyro());
    undo(ROBOT, Faults.TILTED);
    step(world);

    assertThat(Math.toDegrees(tilted.pitchRad)).isCloseTo(-5, within(1e-9));
    assertThat(Math.toDegrees(tilted.rollRad)).isCloseTo(3, within(1e-9));
    assertThat(read(world.drivetrain().gyro()).pitchRad).isCloseTo(0, within(1e-9));
  }

  @Test
  void slippingMovesTheRobotLessUntilItGripsAgain() {
    double gripping = drive(world(), 3.0, 1.0);
    SimulatedWorld world = world();

    apply(ROBOT, Faults.SLIPPING, Map.of(Faults.GRIP, 0.5));
    double slipping = drive(world, 3.0, 1.0);
    undo(ROBOT, Faults.SLIPPING);

    assertThat(slipping / gripping).isCloseTo(0.5, within(1e-6));
    assertThat(world.drivetrain().grip()).isEqualTo(1);
  }

  @Test
  void placedPutsTheRobotWhereAskedAndItsGyroFeelsTheTurn() {
    SimulatedWorld world = world();

    apply(ROBOT, Faults.PLACED, Map.of("X", 5.0, "Y", 2.0, "Heading", -90.0));
    step(world);

    assertThat(world.truePose().getTranslation().getDistance(new Translation2d(5, 2)))
        .isCloseTo(0, within(1e-9));
    assertThat(read(world.drivetrain().gyro()).yaw.getDegrees()).isCloseTo(-90, within(1e-9));
  }

  /** Undoing everything: the robot as built, its sensors reading as they did. */
  @Test
  void undoingEveryFaultPutsTheRobotBackAsBuilt() {
    SimulatedWorld world = world();
    apply(GYRO, Faults.UNPLUGGED);
    apply(BACKUP_IMU, Faults.ERRORS, Map.of(Faults.SCALE, 1.2));
    apply(Faults.module(1), Faults.STALE);
    apply(ROBOT, Faults.SLIPPING, Map.of(Faults.GRIP, 0.3));
    assertThat(components.on()).hasSize(4);

    assertThat(components.undoAll()).isNull();
    step(world);

    assertThat(components.on()).isEmpty();
    assertThat(read(world.drivetrain().gyro()).connected).isTrue();
    assertThat(world.drivetrain().backupGyroSim().errors())
        .isEqualTo(SensorErrors.NONE.backupGyro());
    assertThat(world.drivetrain().grip()).isEqualTo(1);
  }

  /** What's done is said, for the console and the log. */
  @Test
  void eachFaultAppliedOrUndoneIsSaid() {
    world();

    apply(GYRO, Faults.FROZEN);
    undo(GYRO, Faults.FROZEN);
    apply(ROBOT, Faults.HIT, Map.of(Faults.LEFT, 0.6));

    assertThat(said)
        .containsExactly(
            "Gyro: Frozen, on",
            "Gyro: Frozen, undone: Reports the robot's turns again.",
            "Robot: Hit: Forward 0 m, Left 0.6 m");
  }

  /** Drives every wheel at a voltage for a time; how far the robot went from where it stood. */
  private static double drive(SimulatedWorld world, double volts, double seconds) {
    for (ModuleIO module : world.drivetrainHardware().modules()) {
      module.setDriveVoltage(volts);
    }
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      step(world);
    }
    return world.truePose().getTranslation().getDistance(STANDING.getTranslation());
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
