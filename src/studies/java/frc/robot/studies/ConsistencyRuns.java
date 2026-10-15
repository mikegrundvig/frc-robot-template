package frc.robot.studies;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.EstimatorConfigBuilder;
import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.localization.VisionConfigBuilder;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotConfigBuilder;
import com.michaelgrundvig.frc.runtime.SimulatedRobot;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.Scenario;
import com.michaelgrundvig.frc.runtime.sim.Scenarios;
import com.michaelgrundvig.frc.simulation.FaultCatalog;
import com.michaelgrundvig.frc.simulation.Seeds;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.simulation.SensorErrorsBuilder;
import com.michaelgrundvig.frc.simulation.SensorErrorsGyroErrorsBuilder;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
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
 * built to its tolerance does, each seed drawing its own ({@link Spread}).
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
  private static final FieldModel FIELD = Rebuilt.field(ExampleRobot.SIM.definition().body());

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
              "The route, hit partway, then 3 s standing.",
              robot -> {
                robot.route(0, 3);
                robot.start(); // the hit
                robot.route(3, ROUTE.length);
                robot.stand(3);
              }),
          new Run(
              "gyro-lost",
              "5 s standing; a turn in place on the gyro; the gyro unplugged; a turn back on the"
                  + " backup IMU; 5 s standing.",
              Scenarios.named("gyro-unplugged"),
              robot -> {
                robot.stand(5);
                robot.hold(5, 0, 0, 0.5);
                robot.start(); // the gyro unplugged
                robot.hold(5, 0, 0, -0.5);
                robot.stand(5);
              }),
          named(
              "camera-low",
              "The route four times, then 3 s standing.",
              robot -> {
                robot.route(4);
                robot.stand(3);
              }),
          named(
              "camera-turned",
              "The route four times, then 3 s standing.",
              robot -> {
                robot.route(4);
                robot.stand(3);
              }),
          new Run(
              "camera-turned-4",
              "The robot's first camera turned 4° on its mount. The route four times, then 3 s"
                  + " standing.",
              Scenario.builder()
                  .name("camera-turned-4")
                  .description("The robot's first camera turned 4° on its mount.")
                  .sensorFaults(
                      world ->
                          List.of(
                              Setting.of(
                                  FaultCatalog.camera(world.cameraNames().get(0)),
                                  FaultCatalog.TURNED,
                                  Map.of(
                                      FaultCatalog.ROLL,
                                      0.0,
                                      FaultCatalog.PITCH,
                                      0.0,
                                      FaultCatalog.YAW,
                                      4.0))))
                  .events(Scenario.NOTHING)
                  .build(),
              robot -> {
                robot.route(4);
                robot.stand(3);
              }),
          new Run(
              "tag-moved",
              "Tag 26, on the blue hub's face toward the route, moved 4 in along the field. The"
                  + " route four times, then 3 s standing.",
              Scenario.builder()
                  .name("tag-moved")
                  .description("Tag 26 moved 4 in along the field.")
                  .sensorFaults(
                      Scenario.faults(
                          Setting.of(
                              FaultCatalog.FIELD,
                              FaultCatalog.TAG_MOVED,
                              Map.of(
                                  FaultCatalog.TAG,
                                  26.0,
                                  FaultCatalog.X,
                                  4.0,
                                  FaultCatalog.Y,
                                  0.0,
                                  FaultCatalog.UP,
                                  0.0))))
                  .events(Scenario.NOTHING)
                  .build(),
              robot -> {
                robot.route(4);
                robot.stand(3);
              }));

  private ConsistencyRuns() {}

  /** A run in one of the simulator's named scenarios, with this script, described so. */
  private static Run named(String scenario, String script, Script plays) {
    Scenario named = Scenarios.named(scenario);
    return new Run(scenario, named.description() + " " + script, named, plays);
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
   * How widely each seed's errors are drawn: where the cameras and the field's tags really are,
   * against where the robot's configuration and the field's layout say, and how far the IMUs'
   * scales are off. Each is drawn from a Gaussian with these standard deviations, once per seed (a
   * camera's along and about each axis, a tag's along each), and stays put for the run, as a
   * mount's, a tag's, and a gyro's scale do.
   *
   * <p>Each kind of error draws from its own stream of the seed, so no two seeds share a draw: one
   * seed's second camera is never another's first.
   *
   * @param mountMillimeters a camera's mount, along each axis
   * @param mountDegrees and about each axis
   * @param tagMillimeters a tag, along each axis
   * @param gyroScale the gyro's scale error: its reading per true turn, less 1
   * @param backupScale the same for the backup IMU
   */
  record Spread(
      double mountMillimeters,
      double mountDegrees,
      double tagMillimeters,
      double gyroScale,
      double backupScale) {
    /** Cameras, tags, and IMUs exactly as configured. */
    static final Spread EXACT = new Spread(0, 0, 0, 0, 0);

    /**
     * How far a calibrated camera's mount is still off along each axis: a few millimeters, what the
     * mount procedure leaves. The estimator's {@code cameraShiftStdDevMeters} is wider: it holds
     * this and what the camera itself repeats from image to image.
     */
    static final double MOUNT_MILLIMETERS = 3;

    // Each kind of error's stream of a seed: constants far apart and far from any seed, since one
    // step of mixing depends on the seed and the kind only through their sum (with 1, 2, and 3,
    // each seed's mounts were the seed before it's tags).
    private static final long MOUNTS = 0x6d6f756e7473L; // "mounts"
    private static final long TAGS = 0x74616773L; // "tags"
    private static final long IMUS = 0x696d7573L; // "imus"

    Spread {
      if (!(mountMillimeters >= 0)
          || !(mountDegrees >= 0)
          || !(tagMillimeters >= 0)
          || !(gyroScale >= 0)
          || !(backupScale >= 0)) {
        throw new IllegalArgumentException(
            "how widely errors are drawn is zero or more: "
                + mountMillimeters
                + " mm, "
                + mountDegrees
                + "°, "
                + tagMillimeters
                + " mm, scales "
                + gyroScale
                + " and "
                + backupScale);
      }
    }

    /**
     * The errors the estimator's configuration allows for, drawn at the spread it claims: each
     * camera turned about each axis by its mount's tolerance's spread ({@link
     * CameraConfig.MountTolerance#turnStdDevRadians}), each tag off by {@code
     * tagPlacementStdDevMeters} along each, the gyro's scale off by {@code
     * headingStdDevDegreesPerTurn} and the backup IMU's by {@code
     * backupHeadingStdDevDegreesPerTurn}, and a few millimeters of mount ({@link
     * #MOUNT_MILLIMETERS}). An estimator honest about these reads a NEES near its number of
     * dimensions.
     */
    static Spread matched(EstimatorConfig config) {
      return new Spread(
          MOUNT_MILLIMETERS,
          Math.toDegrees(CameraConfig.MountTolerance.MEASURED.turnStdDevRadians()),
          1000 * config.tagPlacementStdDevMeters(),
          config.headingStdDevDegreesPerTurn() / 360,
          config.backupHeadingStdDevDegreesPerTurn() / 360);
    }

    /** One camera's error, for a seed: the same every time for the same seed and camera. */
    Transform3d mount(long seed, int camera) {
      if (mountMillimeters == 0 && mountDegrees == 0) {
        return Transform3d.ZERO;
      }
      SplittableRandom random = stream(seed, MOUNTS, camera);
      double meters = mountMillimeters / 1000;
      double radians = Math.toRadians(mountDegrees);
      return new Transform3d(
          meters * random.nextGaussian(),
          meters * random.nextGaussian(),
          meters * random.nextGaussian(),
          new Rotation3d(
              radians * random.nextGaussian(),
              radians * random.nextGaussian(),
              radians * random.nextGaussian()));
    }

    /** One tag's error, for a seed: the same every time for the same seed and tag. */
    Translation3d tag(long seed, int id) {
      if (tagMillimeters == 0) {
        return Translation3d.ZERO;
      }
      SplittableRandom random = stream(seed, TAGS, id);
      double meters = tagMillimeters / 1000;
      return new Translation3d(
          meters * random.nextGaussian(),
          meters * random.nextGaussian(),
          meters * random.nextGaussian());
    }

    /** The gyro's scale, for a seed: its reading per true turn. */
    double gyroScale(long seed) {
      return 1 + gyroScale * stream(seed, IMUS, 0).nextGaussian();
    }

    /** The backup IMU's scale, for a seed. */
    double backupScale(long seed) {
      return 1 + backupScale * stream(seed, IMUS, 1).nextGaussian();
    }

    /** A seed's stream for one kind of error and one item of it. */
    private static SplittableRandom stream(long seed, long kind, long item) {
      return new SplittableRandom(Seeds.stream(seed, kind, item));
    }
  }

  /**
   * How the robot is built for a seed: a season's wear, with its own IMU scales, each camera's own
   * small mount error, and each tag's own placement. A run's scenario then knocks a camera from
   * there, through the simulator's controls.
   */
  static SensorErrors errors(long seed, Spread spread) {
    List<CameraConfig> cameras = ExampleRobot.SIM.definition().cameras();
    SensorErrors typical = SensorErrors.TYPICAL;
    SensorErrors errors =
        SensorErrorsBuilder.builder(typical)
            .seed(seed)
            .gyro(
                SensorErrorsGyroErrorsBuilder.builder(typical.gyro())
                    .scale(spread.gyroScale(seed))
                    .build())
            .backupGyro(
                SensorErrorsGyroErrorsBuilder.builder(typical.backupGyro())
                    .scale(spread.backupScale(seed))
                    .build())
            .build();
    for (int i = 0; i < cameras.size(); i++) {
      errors = errors.withCameraMountError(cameras.get(i).name(), spread.mount(seed, i));
    }
    Map<Integer, Translation3d> tags = new HashMap<>();
    for (int id : FIELD.tags().keySet()) {
      tags.put(id, spread.tag(seed, id));
    }
    return errors.withTagErrors(tags);
  }

  /**
   * How a study runs: how widely each seed's errors are drawn, and the vision noise model scaled.
   *
   * @param spread how widely each seed's errors are drawn
   * @param noiseScale the vision pipeline's noise model scaled by this ({@link #robot})
   * @param heading what may correct the heading beside the gyro
   */
  record Setup(Spread spread, double noiseScale, EstimatorConfig.HeadingCorrection heading) {
    /**
     * Half the 2026 noise model, a little wider than the simulated cameras' noise: the study reads
     * a position NEES of 1.0 to 2.0 on a calibrated robot at half, and 1.3 to 2.5 at a quarter
     * (runtime/docs/testing.md). Half was the match while the simulator stamped its images up to a
     * loop late, which added error of its own; it stays, so the studies before and since compare.
     */
    static final double MATCHED_NOISE_SCALE = 0.5;

    /**
     * The errors and the noise model the robot's estimator configuration claims, drawn as it claims
     * them: an honest estimator reads a NEES near its number of dimensions.
     */
    static final Setup MATCHED =
        new Setup(
            Spread.matched(ExampleRobot.SIM.estimator()),
            MATCHED_NOISE_SCALE,
            ExampleRobot.SIM.estimator().headingCorrection());

    /** The same, with another way to correct the heading. */
    Setup with(EstimatorConfig.HeadingCorrection other) {
      return new Setup(spread, noiseScale, other);
    }

    Setup {
      if (!(noiseScale > 0)) {
        throw new IllegalArgumentException("a noise scale is more than 0: " + noiseScale);
      }
    }
  }

  /**
   * The simulator's robot as a setup runs it: its vision pipeline's noise model scaled (what if the
   * model were fit tighter, below 1, or looser, above 1, than the one it ships with?), and its
   * estimator correcting the heading as the setup says.
   */
  static RobotConfig robot(Setup setup) {
    VisionConfig vision = ExampleRobot.SIM.vision();
    return RobotConfigBuilder.builder(ExampleRobot.SIM)
        .vision(
            VisionConfigBuilder.builder(vision)
                .linearStdDevBaseline(setup.noiseScale() * vision.linearStdDevBaseline())
                .angularStdDevBaseline(setup.noiseScale() * vision.angularStdDevBaseline())
                .build())
        .estimator(
            EstimatorConfigBuilder.builder(ExampleRobot.SIM.estimator())
                .headingCorrection(setup.heading())
                .build())
        .build();
  }

  /**
   * One run with one seed: every loop once the robot is enabled, and what the mount check made of
   * each camera.
   *
   * @param mostDoubtMeters by camera, the most its mount was doubted at any loop
   * @param offsetMeters by camera, how far its tags landed at the end (NaN if never judged)
   */
  record Played(
      List<Consistency.Sample> samples,
      Map<String, Double> mostDoubtMeters,
      Map<String, Double> offsetMeters) {}

  /**
   * Plays a run with a seed on the simulator's robot.
   *
   * @return every loop once the robot is enabled, and the mount check's verdicts
   */
  static Played play(Run run, long seed, Setup setup) {
    try (SimulatedRobot robot = new SimulatedRobot(robot(setup), errors(seed, setup.spread()))) {
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
      Map<String, Double> offsets = new LinkedHashMap<>();
      for (CameraConfig camera : ExampleRobot.SIM.definition().cameras()) {
        offsets.put(camera.name(), robot.mountOffsetMeters(camera.name()));
      }
      return new Played(driver.samples, Map.copyOf(driver.mostDoubt), offsets);
    }
  }

  /** The robot as a script drives it: every loop it runs is sampled. */
  static final class Driver {
    private final SimulatedRobot robot;
    private final @Nullable Scenario scenario;
    private final List<Consistency.Sample> samples = new ArrayList<>();
    private final Map<String, Double> mostDoubt = new LinkedHashMap<>();

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
      samples.add(Consistency.Sample.of(estimate, robot.truePose()));
      for (CameraConfig camera : ExampleRobot.SIM.definition().cameras()) {
        mostDoubt.merge(camera.name(), robot.mountDoubtMeters(camera.name()), Math::max);
      }
    }
  }
}
