package frc.robot.studies;

import frc.robot.RobotConfig;
import frc.robot.RobotConfigBuilder;
import frc.robot.Robots;
import frc.robot.SimulatedRobot;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.lib.localization.VisionConfig;
import frc.robot.lib.localization.VisionConfigBuilder;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.sim.Scenario;
import frc.robot.sim.Scenarios;
import frc.robot.sim.SensorErrors;
import frc.robot.sim.SensorErrorsBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.SplittableRandom;
import java.util.stream.Collectors;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * The situations the consistency study puts the simulated robot through, each a script of what the
 * drive team does and what happens to the robot, run on a season-worn robot ({@link
 * SensorErrors#TYPICAL}) whose cameras each sit a little off their configured mounts, as a
 * calibrated camera still does.
 *
 * <p>Every run starts the same way: the robot placed on its start, disabled for a second while the
 * cameras find it, then enabled. From then on every loop is a sample: the estimate, what it claims,
 * and the truth.
 */
final class ConsistencyRuns {
  /**
   * A route in view of the field's tags, as the drive team's sticks hold it: seconds, then forward,
   * left, and turn, each from -1 to 1. About 8.5 s; driven there and back.
   */
  private static final double[][] ROUTE = {
    {1.5, -0.5, 0, 0},
    {1.0, 0, 0.7, 0},
    {1.5, 0, 0, 0.6},
    {2.0, 0.4, -0.5, 0.2},
    {1.5, -0.3, -0.3, -0.4},
    {1.0, 0, 0, 0}
  };

  /**
   * How long the gyro-scale procedure's part of its run lasts: the procedure (about 20 s: still,
   * five turns, still again), then standing until this is up. Fixed, so every seed's run has the
   * same loops, however long the procedure took.
   */
  static final double PROCEDURE_SECONDS = 25;

  /** What a run does to the robot. */
  @FunctionalInterface
  interface Script {
    void play(Driver robot);
  }

  /**
   * One situation.
   *
   * @param name how it's asked for
   * @param description what happens in it
   * @param faults how the robot's sensors err in it, beyond a season's wear and the mounts' own
   *     small errors
   * @param script what the drive team does, and what happens to the robot
   */
  record Run(String name, String description, Scenario.SensorFaults faults, Script script) {}

  /** Every run, in the order the report shows them. */
  static final List<Run> ALL =
      List.of(
          new Run(
              "driving",
              "The route in view of the tags, there and back, then 3 s standing.",
              Scenario.NO_FAULTS,
              robot -> {
                robot.route(2);
                robot.stand(3);
              }),
          new Run(
              "standing",
              "The route once, then 20 s standing still.",
              Scenario.NO_FAULTS,
              robot -> {
                robot.route(1);
                robot.stand(20);
              }),
          new Run(
              "turning",
              "5 s standing, two turns in place at about 1.2 rad/s, then 5 s standing.",
              Scenario.NO_FAULTS,
              robot -> {
                robot.stand(5);
                robot.hold(10, 0, 0, 0.5);
                robot.stand(5);
              }),
          new Run(
              "gyro-scale",
              "The gyro-scale procedure (five turns), then standing: 40 s in all.",
              Scenario.NO_FAULTS,
              robot -> {
                robot.gyroScaleProcedure();
                robot.stand(15);
              }),
          named(
              "collision",
              robot -> {
                robot.route(0, 3);
                robot.start(Scenarios.named("collision"));
                robot.route(3, ROUTE.length);
                robot.stand(3);
              }),
          named(
              "camera-low",
              robot -> {
                robot.route(2);
                robot.stand(3);
              }),
          named(
              "camera-turned",
              robot -> {
                robot.route(2);
                robot.stand(3);
              }));

  private ConsistencyRuns() {}

  /** A run of one of the simulator's named scenarios: its faults, and this script. */
  private static Run named(String scenario, Script script) {
    Scenario named = Scenarios.named(scenario);
    return new Run(
        scenario,
        named.description() + " Driving the route, there and back.",
        named.sensorFaults(),
        script);
  }

  /**
   * The run with this name.
   *
   * @throws IllegalArgumentException naming every run there is, when none has this name
   */
  static Run named(String name) {
    for (Run run : ALL) {
      if (run.name().equals(name)) {
        return run;
      }
    }
    throw new IllegalArgumentException(
        "No consistency run is named \""
            + name
            + "\". The runs: "
            + ALL.stream().map(Run::name).collect(Collectors.joining(", ")));
  }

  /**
   * How far each camera sits off its configured mount, at most, along and about each axis: what a
   * camera calibrated with the mount procedure (docs/calibration.md) still has. Each camera's own
   * is drawn within these for each seed, and stays put for the run, as a mount's does.
   *
   * @param millimeters along each axis
   * @param degrees about each axis
   */
  record MountErrors(double millimeters, double degrees) {
    /** A few millimeters and a quarter of a degree. */
    static final MountErrors CALIBRATED = new MountErrors(5, 0.25);

    static final MountErrors NONE = new MountErrors(0, 0);

    MountErrors {
      if (!(millimeters >= 0) || !(degrees >= 0)) {
        throw new IllegalArgumentException(
            "mount errors are zero or more: " + millimeters + " mm, " + degrees + "°");
      }
    }

    /** One camera's error, for a seed: the same every time for the same seed and camera. */
    Transform3d draw(long seed, int camera) {
      if (millimeters == 0 && degrees == 0) {
        return Transform3d.ZERO;
      }
      SplittableRandom random = new SplittableRandom(Seeds.mix(seed, camera + 1L));
      double meters = millimeters / 1000;
      double radians = Math.toRadians(degrees);
      return new Transform3d(
          within(random, meters),
          within(random, meters),
          within(random, meters),
          new Rotation3d(
              within(random, radians), within(random, radians), within(random, radians)));
    }

    private static double within(SplittableRandom random, double bound) {
      return bound * (2 * random.nextDouble() - 1);
    }
  }

  /**
   * How the robot's sensors err in a run with a seed: a season's wear, each camera's own small
   * mount error, then the run's faults, which replace a knocked camera's.
   */
  static SensorErrors errors(Run run, long seed, MountErrors mounts) {
    List<CameraConfig> cameras = Robots.SIM.cameras();
    SensorErrors errors = SensorErrorsBuilder.builder(SensorErrors.TYPICAL).seed(seed).build();
    for (int i = 0; i < cameras.size(); i++) {
      errors = errors.withCameraMountError(cameras.get(i).name(), mounts.draw(seed, i));
    }
    return run.faults().apply(errors, cameras);
  }

  /**
   * The simulator's robot, with its vision pipeline's noise model scaled: what if the model were
   * fit tighter (below 1) or looser (above 1) than the one it ships with?
   */
  static RobotConfig robot(double noiseScale) {
    if (noiseScale == 1) {
      return Robots.SIM;
    }
    VisionConfig vision = Robots.SIM.vision();
    return RobotConfigBuilder.builder(Robots.SIM)
        .vision(
            VisionConfigBuilder.builder(vision)
                .linearStdDevBaseline(noiseScale * vision.linearStdDevBaseline())
                .angularStdDevBaseline(noiseScale * vision.angularStdDevBaseline())
                .build())
        .build();
  }

  /**
   * Plays a run with a seed on the simulator's robot.
   *
   * @param noiseScale how much to scale the vision pipeline's noise model by ({@link #robot})
   * @return every loop once the robot is enabled
   */
  static List<Consistency.Sample> play(Run run, long seed, MountErrors mounts, double noiseScale) {
    try (SimulatedRobot robot = new SimulatedRobot(robot(noiseScale), errors(run, seed, mounts))) {
      for (int i = 0; i < RobotLoop.loops(1.0); i++) {
        robot.loop(); // placed on its start, and the cameras find it
      }
      robot.setEnabled(true);
      Driver driver = new Driver(robot);
      run.script().play(driver);
      return driver.samples;
    }
  }

  /** The robot as a script drives it: every loop it runs is sampled. */
  static final class Driver {
    private final SimulatedRobot robot;
    private final List<Consistency.Sample> samples = new ArrayList<>();

    Driver(SimulatedRobot robot) {
      this.robot = robot;
    }

    /** Holds the sticks for a while: forward, left, and turn, each from -1 to 1. */
    void hold(double seconds, double forward, double left, double turn) {
      robot.sticks(forward, left, turn);
      for (int i = 0; i < RobotLoop.loops(seconds); i++) {
        loop();
      }
    }

    /** Stands still for a while, enabled. */
    void stand(double seconds) {
      hold(seconds, 0, 0, 0);
    }

    /** Drives the route there and back, as many times as asked: odd times back. */
    void route(int times) {
      for (int time = 0; time < times; time++) {
        double back = time % 2 == 0 ? 1 : -1;
        for (double[] leg : ROUTE) {
          hold(leg[0], leg[1] * back, leg[2] * back, leg[3]);
        }
      }
    }

    /** Drives part of the route: its legs from the first up to, not including, the last. */
    void route(int first, int last) {
      for (int i = first; i < last; i++) {
        double[] leg = ROUTE[i];
        hold(leg[0], leg[1], leg[2], leg[3]);
      }
    }

    /** Starts a scenario's event, now. */
    void start(Scenario scenario) {
      robot.start(scenario);
    }

    /**
     * Runs the gyro-scale procedure, as the Pitwall page runs it, then stands until {@link
     * #PROCEDURE_SECONDS} are up.
     *
     * @throws IllegalStateException if it doesn't end in time, or ends without measuring
     */
    void gyroScaleProcedure() {
      robot.sticks(0, 0, 0);
      Command procedure = robot.startGyroScaleProcedure();
      for (int i = 0; i < RobotLoop.loops(PROCEDURE_SECONDS); i++) {
        loop();
      }
      if (robot.isRunning(procedure) || !robot.lastCalibration().startsWith("Gyro scale")) {
        throw new IllegalStateException(
            "the gyro-scale procedure didn't finish: " + robot.lastCalibration());
      }
    }

    private void loop() {
      robot.loop();
      LocalizationEstimate estimate = robot.estimate();
      samples.add(Consistency.Sample.of(estimate.pose(), estimate.covariance(), robot.truePose()));
    }
  }
}
