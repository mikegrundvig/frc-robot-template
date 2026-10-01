package frc.robot.studies;

import frc.robot.RobotConfig;
import frc.robot.RobotConfigBuilder;
import frc.robot.Robots;
import frc.robot.SimulatedRobot;
import frc.robot.game.Rebuilt;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.lib.localization.VisionConfig;
import frc.robot.lib.localization.VisionConfigBuilder;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.sim.Scenario;
import frc.robot.sim.Scenarios;
import frc.robot.sim.SensorErrors;
import frc.robot.sim.SensorErrorsBuilder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.SplittableRandom;
import java.util.stream.Collectors;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The situations the consistency study puts the simulated robot through, each a script of what the
 * drive team does and what happens to the robot, run on a season-worn robot ({@link
 * SensorErrors#TYPICAL}) whose cameras each sit a little off their configured mounts, as a
 * calibrated camera still does, on a field whose tags each sit a little off its layout, as a field
 * built to its tolerance does.
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

  /** The field the simulator's robot plays on: its tags, which each seed places a little off. */
  private static final FieldModel FIELD = Rebuilt.field(Robots.SIM.body());

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
   * @param scenario the simulator's scenario it runs in, whose sensor faults are applied before the
   *     robot's first loop, on top of a season's wear and the mounts' and tags' own small errors;
   *     null for none
   * @param script what the drive team does, and what happens to the robot
   */
  record Run(String name, String description, @Nullable Scenario scenario, Script script) {}

  /** Every run, in the order the report shows them. */
  static final List<Run> ALL =
      List.of(
          new Run(
              "driving",
              "The route in view of the tags, there and back, then 3 s standing.",
              null,
              robot -> {
                robot.route(2);
                robot.stand(3);
              }),
          new Run(
              "standing",
              "The route once, then 20 s standing still.",
              null,
              robot -> {
                robot.route(1);
                robot.stand(20);
              }),
          new Run(
              "turning",
              "5 s standing, two turns in place at about 1.2 rad/s, then 5 s standing.",
              null,
              robot -> {
                robot.stand(5);
                robot.hold(10, 0, 0, 0.5);
                robot.stand(5);
              }),
          new Run(
              "gyro-scale",
              "The gyro-scale procedure (five turns), then standing: 40 s in all.",
              null,
              robot -> {
                robot.gyroScaleProcedure();
                robot.stand(15);
              }),
          named(
              "collision",
              robot -> {
                robot.route(0, 3);
                robot.start(); // the hit
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

  /** A run in one of the simulator's named scenarios, with this script. */
  private static Run named(String scenario, Script script) {
    Scenario named = Scenarios.named(scenario);
    return new Run(
        scenario, named.description() + " Driving the route, there and back.", named, script);
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
   * Where the cameras and the field's tags really are, against where the robot's configuration and
   * the field's layout say: how far each may be off, at most, along and about each axis. Each
   * camera's and each tag's own error is drawn within these for each seed, and stays put for the
   * run, as a mount's and a tag's do.
   *
   * @param mountMillimeters a camera's mount, along each axis: what calibrating it leaves
   * @param mountDegrees and about each axis
   * @param tagMillimeters a tag, along each axis: the field's build tolerance
   */
  record Placement(double mountMillimeters, double mountDegrees, double tagMillimeters) {
    /**
     * Cameras calibrated with the mount procedure (docs/calibration.md), a few millimeters and a
     * quarter of a degree off, on a field built to its half-inch tolerance.
     */
    static final Placement CALIBRATED = new Placement(5, 0.25, 12.7);

    static final Placement EXACT = new Placement(0, 0, 0);

    Placement {
      if (!(mountMillimeters >= 0) || !(mountDegrees >= 0) || !(tagMillimeters >= 0)) {
        throw new IllegalArgumentException(
            "how far cameras and tags are off is zero or more: "
                + mountMillimeters
                + " mm, "
                + mountDegrees
                + "°, "
                + tagMillimeters
                + " mm");
      }
    }

    /** One camera's error, for a seed: the same every time for the same seed and camera. */
    Transform3d mount(long seed, int camera) {
      if (mountMillimeters == 0 && mountDegrees == 0) {
        return Transform3d.ZERO;
      }
      SplittableRandom random = new SplittableRandom(Seeds.mix(seed, camera + 1L));
      double meters = mountMillimeters / 1000;
      double radians = Math.toRadians(mountDegrees);
      return new Transform3d(
          within(random, meters),
          within(random, meters),
          within(random, meters),
          new Rotation3d(
              within(random, radians), within(random, radians), within(random, radians)));
    }

    /** One tag's error, for a seed: the same every time for the same seed and tag. */
    Translation3d tag(long seed, int id) {
      if (tagMillimeters == 0) {
        return Translation3d.ZERO;
      }
      SplittableRandom random = new SplittableRandom(Seeds.mix(seed, -id - 1L));
      double meters = tagMillimeters / 1000;
      return new Translation3d(
          within(random, meters), within(random, meters), within(random, meters));
    }

    private static double within(SplittableRandom random, double bound) {
      return bound * (2 * random.nextDouble() - 1);
    }
  }

  /**
   * How the robot is built for a seed: a season's wear, each camera's own small mount error, and
   * each tag's own placement. A run's scenario then knocks a camera from there, through the
   * simulator's controls.
   */
  static SensorErrors errors(long seed, Placement placement) {
    List<CameraConfig> cameras = Robots.SIM.cameras();
    SensorErrors errors = SensorErrorsBuilder.builder(SensorErrors.TYPICAL).seed(seed).build();
    for (int i = 0; i < cameras.size(); i++) {
      errors = errors.withCameraMountError(cameras.get(i).name(), placement.mount(seed, i));
    }
    Map<Integer, Translation3d> tags = new HashMap<>();
    for (int id : FIELD.tags().keySet()) {
      tags.put(id, placement.tag(seed, id));
    }
    return errors.withTagErrors(tags);
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
  static List<Consistency.Sample> play(Run run, long seed, Placement placement, double noiseScale) {
    try (SimulatedRobot robot = new SimulatedRobot(robot(noiseScale), errors(seed, placement))) {
      Scenario scenario = run.scenario();
      if (scenario != null) {
        robot.applyFaults(scenario);
      }
      for (int i = 0; i < RobotLoop.loops(1.0); i++) {
        robot.loop(); // placed on its start, and the cameras find it
      }
      robot.setEnabled(true);
      Driver driver = new Driver(robot, scenario);
      run.script().play(driver);
      return driver.samples;
    }
  }

  /** The robot as a script drives it: every loop it runs is sampled. */
  static final class Driver {
    private final SimulatedRobot robot;
    private final @Nullable Scenario scenario;
    private final List<Consistency.Sample> samples = new ArrayList<>();

    Driver(SimulatedRobot robot, @Nullable Scenario scenario) {
      this.robot = robot;
      this.scenario = scenario;
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

    /**
     * Makes what happens in the run's scenario happen, now: a collision.
     *
     * @throws IllegalStateException if the run has no scenario
     */
    void start() {
      Scenario events = scenario;
      if (events == null) {
        throw new IllegalStateException("this run has no scenario to start");
      }
      robot.start(events);
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
