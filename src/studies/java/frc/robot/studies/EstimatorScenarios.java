package frc.robot.studies;

import static com.michaelgrundvig.frc.simulation.FaultCatalog.BACKUP_IMU;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.FIELD;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.GYRO;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.ROBOT;

import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.simulation.FaultCatalog;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import frc.robot.studies.EstimatorRig.Cameras.Misread;
import frc.robot.studies.EstimatorRuns.Rest;
import frc.robot.studies.EstimatorRuns.Route;
import frc.robot.studies.EstimatorRuns.Scenario;
import frc.robot.studies.EstimatorRuns.Stage;
import frc.robot.studies.EstimatorRuns.Timed;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.LongFunction;
import java.util.stream.Collectors;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

/**
 * The estimator study's scenarios: a route, a robot, and what goes wrong on the way, through the
 * simulator's own faults ({@link FaultCatalog}). The two misreads are the study's own, since the
 * simulator has no fault for a camera that reads a tag wrong ({@link EstimatorRig.Cameras}).
 *
 * <p>The routes are written against the 2026 field, on the floor: the robot starts before its hub,
 * works its alliance zone, and reaches the middle of the field through the trenches. A stop near a
 * tag is a meter before the hub's face; a stop far from tags is in the middle of the field, three
 * meters and more from the nearest.
 */
final class EstimatorScenarios {
  private EstimatorScenarios() {}

  /** When a short route's fault happens: on the way to the trench, two stops still to come. */
  static final double FAULT_SECONDS = 11;

  /** A stop near a tag: a meter before the hub's face. */
  private static final double NEAR_X = 3.1;

  private static final double NEAR_Y = 4.2;

  /** About 36 s: sprints, a strafe while turning, a stop near a tag, the trench, a stop far. */
  static final Route SHORT =
      Route.named("short")
          .drive(2.5, 2.2, 2.2, 0, 1.0)
          .drive(2.5, 2.2, 5.8, 90, 1.0)
          .drive(2.5, NEAR_X, NEAR_Y, 0, 0.8)
          .stop(2, Rest.NEAR_A_TAG)
          .drive(3.0, 3.0, 0.75, 0, 1.0)
          .drive(3.5, 8.0, 0.75, 0, 1.0)
          .drive(2.5, 8.3, 2.5, 180, 0.8)
          .stop(2, Rest.FAR_FROM_TAGS)
          .drive(3.0, 8.0, 0.75, 180, 1.0)
          .drive(3.5, 3.0, 0.75, 180, 1.0)
          .drive(3.0, NEAR_X, NEAR_Y, 0, 0.8)
          .stop(2, Rest.NEAR_A_TAG)
          .build();

  /** About 39 s: over the bump beside the hub and back, twice, a stop on each side. */
  static final Route BUMPS = bumps();

  /** A match, 2 minutes 30 seconds: an opening in the alliance zone, then five cycles. */
  static final Route MATCH = match();

  private static Route bumps() {
    Route.Builder route = Route.named("bumps").drive(2.5, 2.8, 2.5, 0, 0.8);
    for (int crossing = 0; crossing < 2; crossing++) {
      route
          .drive(4.5, 6.8, 2.5, 0, 0.6)
          .stop(2, Rest.FAR_FROM_TAGS)
          .drive(4.5, 2.8, 2.5, 0, 0.6)
          .drive(2.5, NEAR_X, NEAR_Y, 0, 0.8)
          .stop(2, Rest.NEAR_A_TAG)
          .drive(2.0, 2.8, 2.5, 0, 0.8);
    }
    return route.build();
  }

  private static Route match() {
    Route.Builder route =
        Route.named("match")
            .drive(2.5, 2.2, 2.2, 0, 1.0)
            .drive(2.5, 2.2, 5.8, 90, 1.0)
            .drive(2.5, NEAR_X, NEAR_Y, 0, 0.8)
            .stop(2.5, Rest.NEAR_A_TAG);
    for (int cycle = 0; cycle < 5; cycle++) {
      // Out through one trench and back through the other, the other way round each cycle.
      double out = cycle % 2 == 0 ? 0.75 : 7.3;
      double back = cycle % 2 == 0 ? 7.3 : 0.75;
      double sweepFrom = cycle % 2 == 0 ? 2.8 : 5.3;
      double sweepTo = cycle % 2 == 0 ? 5.3 : 2.8;
      route
          .drive(3.0, 3.0, out, 0, 1.0)
          .drive(3.5, 8.0, out, 0, 1.0)
          .drive(2.0, 8.3, sweepFrom, 90, 0.8)
          .drive(2.0, 8.3, sweepTo, 90, 0.6)
          .stop(1.0, Rest.FAR_FROM_TAGS)
          .drive(2.5, 8.0, back, 180, 1.0)
          .drive(3.5, 3.0, back, 180, 1.0)
          .drive(3.0, NEAR_X, NEAR_Y, 0, 0.8)
          .stop(2.0, Rest.NEAR_A_TAG);
    }
    return route.stop(8, Rest.NEAR_A_TAG).build();
  }

  /** Every scenario, in the report's order. */
  static final List<Scenario> ALL = all();

  private static List<Scenario> all() {
    List<Scenario> all = new ArrayList<>();
    LongFunction<SensorErrors> typical = EstimatorRuns::typical;

    // 1 and 2: nothing goes wrong.
    all.add(scenario("clean", "A season-worn robot, and nothing goes wrong.", SHORT));
    all.add(scenario("clean-match", "The same, for a whole match.", MATCH));
    all.add(
        new Scenario(
            "ideal",
            "Wheels and IMUs exactly as configured; the cameras' noise stays.",
            SHORT,
            EstimatorRuns::ideal,
            0,
            List.of()));
    all.add(
        new Scenario(
            "ideal-match",
            "The same, for a whole match.",
            MATCH,
            EstimatorRuns::ideal,
            0,
            List.of()));

    // 3: a camera misreads a tag.
    all.add(
        scenario(
            "misread-once",
            "One image, at rest before the hub, reads its nearest tag as another of the field's.",
            SHORT,
            at(9, stage -> stage.misread(Misread.WRONG_ID, 1))));
    all.add(
        scenario(
            "misread-repeated",
            "One camera reads its nearest tag as another of the field's, 60 images running.",
            SHORT,
            at(FAULT_SECONDS, stage -> stage.misread(Misread.WRONG_ID, 60))));
    all.add(
        scenario(
            "flipped-once",
            "One image, at rest before the hub, has its nearest tag's two poses the wrong way"
                + " round, and says it's fairly sure (ambiguity 0.15).",
            SHORT,
            at(9, stage -> stage.misread(Misread.FLIPPED, 1))));
    all.add(
        scenario(
            "flipped-repeated",
            "One camera has its nearest tag's two poses the wrong way round, 60 images running.",
            SHORT,
            at(FAULT_SECONDS, stage -> stage.misread(Misread.FLIPPED, 60))));

    // 4: the robot is moved without its wheels.
    all.add(
        scenario(
            "shove",
            "Pushed 0.6 m to its left without a jolt: neither its wheels nor its IMU notice.",
            SHORT,
            at(
                FAULT_SECONDS,
                stage ->
                    stage.knock(
                        robot(
                            FaultCatalog.SHOVED,
                            Map.of(
                                FaultCatalog.FORWARD,
                                0.0,
                                FaultCatalog.LEFT,
                                0.6,
                                FaultCatalog.TURN,
                                0.0))))));
    all.add(
        scenario(
            "collision",
            "Hit: knocked 0.6 m to its left, with a 4 g jolt its IMU feels.",
            SHORT,
            at(
                FAULT_SECONDS,
                stage ->
                    stage.knock(
                        robot(
                            FaultCatalog.HIT,
                            Map.of(FaultCatalog.FORWARD, 0.0, FaultCatalog.LEFT, 0.6))))));
    all.add(
        scenario(
            "pushed",
            "Pushed to its left at 1 m/s for a second, its wheels skidding.",
            SHORT,
            at(
                FAULT_SECONDS,
                stage ->
                    stage.fault(
                        robot(
                            FaultCatalog.PUSHED,
                            Map.of(FaultCatalog.FORWARD, 0.0, FaultCatalog.LEFT, 1.0)))),
            at(FAULT_SECONDS + 1, stage -> stage.undo(ROBOT, FaultCatalog.PUSHED))));

    // 5: the wheels lie.
    all.add(
        scenario(
            "low-grip",
            "For 5 s, only 60% of its wheels' rolling moves it: every wheel slips alike.",
            SHORT,
            at(
                FAULT_SECONDS,
                stage -> stage.fault(robot(FaultCatalog.SLIPPING, Map.of(FaultCatalog.GRIP, 0.6)))),
            at(FAULT_SECONDS + 5, stage -> stage.undo(ROBOT, FaultCatalog.SLIPPING))));
    all.add(
        scenario(
            "wheel-slip",
            "The front left wheel spins half a turn without moving the robot.",
            SHORT,
            at(
                FAULT_SECONDS,
                stage ->
                    stage.fault(
                        Setting.of(
                            FaultCatalog.module(0),
                            FaultCatalog.SLIPS,
                            Map.of(FaultCatalog.TURNS, 0.5))))));
    all.add(
        scenario(
            "wheel-spinning",
            "For 3 s the front left wheel spins 1 m/s faster than the floor moves it.",
            SHORT,
            at(
                FAULT_SECONDS,
                stage ->
                    stage.fault(
                        Setting.of(
                            FaultCatalog.module(0),
                            FaultCatalog.SPINNING,
                            Map.of(FaultCatalog.SPEED, 1.0)))),
            at(
                FAULT_SECONDS + 3,
                stage -> stage.undo(FaultCatalog.module(0), FaultCatalog.SPINNING))));
    all.add(
        scenario(
            "worn-tread",
            "Every wheel's tread worn 3%: the robot travels 3% less than its wheels count.",
            SHORT,
            atTheStart(
                stage -> {
                  for (int module = 0; module < FaultCatalog.MODULES.size(); module++) {
                    stage.fault(
                        Setting.of(
                            FaultCatalog.module(module),
                            FaultCatalog.WORN,
                            Map.of(FaultCatalog.SCALE, 0.97)));
                  }
                })));
    all.add(
        scenario(
            "module-zero-off",
            "The front left module's absolute encoder zeroed 5° off.",
            SHORT,
            atTheStart(
                stage ->
                    stage.fault(
                        Setting.of(
                            FaultCatalog.module(0),
                            FaultCatalog.ZEROED_OFF,
                            Map.of(FaultCatalog.OFFSET, 5.0))))));
    all.add(
        scenario(
            "encoder-stale",
            "For 2 s the front left module's encoders stop changing while the wheel turns on.",
            SHORT,
            at(
                FAULT_SECONDS,
                stage -> stage.fault(Setting.of(FaultCatalog.module(0), FaultCatalog.STALE))),
            at(
                FAULT_SECONDS + 2,
                stage -> stage.undo(FaultCatalog.module(0), FaultCatalog.STALE))));

    // 6: the cameras.
    all.add(
        scenario(
            "camera-blind",
            "The first camera unplugged from 5 s on.",
            SHORT,
            at(5, stage -> stage.fault(Setting.of(camera(stage, 0), FaultCatalog.UNPLUGGED)))));
    all.add(blind("cameras-blind-3s", 3));
    all.add(blind("cameras-blind-10s", 10));
    all.add(
        scenario(
            "camera-knocked",
            "At 5 s the first camera is knocked on its mount: 1 in forward, 1.5 in down, turned 3°"
                + " about the vertical and pitched 2°.",
            SHORT,
            at(
                5,
                stage ->
                    stage.fault(
                        Setting.of(
                            camera(stage, 0),
                            FaultCatalog.MOVED,
                            Map.of(
                                FaultCatalog.FORWARD,
                                1.0,
                                FaultCatalog.LEFT,
                                0.0,
                                FaultCatalog.UP,
                                -1.5)),
                        Setting.of(
                            camera(stage, 0),
                            FaultCatalog.TURNED,
                            Map.of(
                                FaultCatalog.ROLL,
                                0.0,
                                FaultCatalog.PITCH,
                                2.0,
                                FaultCatalog.YAW,
                                3.0))))));
    all.add(
        scenario(
            "camera-clock-off",
            "The first camera's clock 20 ms off the robot's: its images are stamped a loop late.",
            SHORT,
            atTheStart(
                stage ->
                    stage.fault(
                        Setting.of(
                            camera(stage, 0),
                            FaultCatalog.CLOCK_OFFSET,
                            Map.of(FaultCatalog.OFFSET, 20.0))))));
    all.add(
        scenario(
            "camera-late",
            "The first camera's images arrive 150 ms later than its configuration says, rightly"
                + " stamped.",
            SHORT,
            atTheStart(
                stage ->
                    stage.fault(
                        Setting.of(
                            camera(stage, 0),
                            FaultCatalog.LATE,
                            Map.of(FaultCatalog.EXTRA, 150.0))))));
    all.add(
        scenario(
            "camera-frozen",
            "For 3 s the first camera keeps sending its last image's tags as new ones.",
            SHORT,
            at(
                FAULT_SECONDS,
                stage -> stage.fault(Setting.of(camera(stage, 0), FaultCatalog.FROZEN))),
            at(FAULT_SECONDS + 3, stage -> stage.undo(camera(stage, 0), FaultCatalog.FROZEN))));
    all.add(
        scenario(
            "tag-moved",
            "Tag 26, on the hub's face toward the start, sits 4 in off the field's layout.",
            SHORT,
            atTheStart(
                stage ->
                    stage.fault(
                        Setting.of(
                            FIELD,
                            FaultCatalog.TAG_MOVED,
                            Map.of(
                                FaultCatalog.TAG,
                                26.0,
                                FaultCatalog.X,
                                4.0,
                                FaultCatalog.Y,
                                0.0,
                                FaultCatalog.UP,
                                0.0))))));

    // 7: the gyro.
    all.add(
        scenario(
            "gyro-scale",
            "The gyro reads 3% more turn than the robot makes; the backup IMU doesn't.",
            SHORT,
            atTheStart(
                stage ->
                    stage.fault(
                        Setting.of(GYRO, FaultCatalog.ERRORS, Map.of(FaultCatalog.SCALE, 1.03))))));
    all.add(
        scenario(
            "gyro-drift",
            "The gyro drifts 10° a minute.",
            SHORT,
            atTheStart(
                stage ->
                    stage.fault(
                        Setting.of(GYRO, FaultCatalog.ERRORS, Map.of(FaultCatalog.DRIFT, 10.0))))));
    all.add(
        scenario(
            "gyro-frozen",
            "For 5 s the gyro keeps reporting the same yaw, however the robot turns.",
            SHORT,
            at(FAULT_SECONDS, stage -> stage.fault(Setting.of(GYRO, FaultCatalog.FROZEN))),
            at(FAULT_SECONDS + 5, stage -> stage.undo(GYRO, FaultCatalog.FROZEN))));
    all.add(
        scenario(
            "gyro-unplugged",
            "The gyro unplugged for the rest of the run: the backup IMU has the heading.",
            SHORT,
            at(FAULT_SECONDS, stage -> stage.fault(Setting.of(GYRO, FaultCatalog.UNPLUGGED)))));
    all.add(
        scenario(
            "imus-unplugged",
            "The gyro and the backup IMU both unplugged for the rest of the run.",
            SHORT,
            at(
                FAULT_SECONDS,
                stage ->
                    stage.fault(
                        Setting.of(GYRO, FaultCatalog.UNPLUGGED),
                        Setting.of(BACKUP_IMU, FaultCatalog.UNPLUGGED)))));
    all.add(
        scenario(
            "gyro-reset",
            "The gyro's yaw resets to zero mid-run, as one that browns out and restarts does.",
            SHORT,
            at(FAULT_SECONDS, stage -> stage.fault(Setting.of(GYRO, FaultCatalog.RESET)))));

    // 8: tilt.
    all.add(scenario("bump", "Over the bump beside the hub and back, twice.", BUMPS));
    all.add(
        scenario(
            "tilted",
            "For 5 s a corner rides up on a game piece: tilted 5° nose up and 5° left side up,"
                + " moving three quarters as far as its wheels turn.",
            SHORT,
            at(
                FAULT_SECONDS,
                stage ->
                    stage.fault(
                        robot(
                            FaultCatalog.TILTED,
                            Map.of(FaultCatalog.PITCH, -5.0, FaultCatalog.ROLL, 5.0)),
                        robot(FaultCatalog.SLIPPING, Map.of(FaultCatalog.GRIP, 0.75)))),
            at(
                FAULT_SECONDS + 5,
                stage -> {
                  stage.undo(ROBOT, FaultCatalog.TILTED);
                  stage.undo(ROBOT, FaultCatalog.SLIPPING);
                })));

    // 9 and 10: placed wrong, and carried.
    all.add(
        scenario(
            "placed-wrong",
            "As it's placed, the robot believes a start 0.5 m and 15° from where it stands.",
            SHORT,
            atTheStart(
                stage -> stage.misplace(new Transform2d(0.4, 0.3, Rotation2d.fromDegrees(15))))));
    all.add(
        scenario(
            "disabled-carried",
            "Disabled at 11 s, carried across the alliance zone and turned a quarter turn, and"
                + " enabled again 3 s later.",
            SHORT,
            at(FAULT_SECONDS, stage -> stage.enable(false)),
            at(
                FAULT_SECONDS + 1,
                stage ->
                    stage.knock(
                        robot(
                            FaultCatalog.PLACED,
                            Map.of(
                                FaultCatalog.X,
                                2.0,
                                FaultCatalog.Y,
                                6.0,
                                FaultCatalog.HEADING,
                                90.0)))),
            at(FAULT_SECONDS + 3, stage -> stage.enable(true))));

    // Several together.
    all.add(
        scenario(
            "bad-match",
            "A match with worn tread (3%), the first camera knocked at 20 s, a collision at 40 s,"
                + " every camera blind for 3 s at 60 s, a wheel spinning for 2 s at 80 s, a shove"
                + " at 100 s, a camera misreading 60 images at 115 s, and the gyro unplugged at"
                + " 130 s.",
            MATCH,
            badMatch()));

    // The honesty checks: the clean match with errors the simulator's own noise doesn't have.
    all.add(
        new Scenario(
            "clean-match-mounts",
            "The clean match, each camera a little off its mount and each tag a little off the"
                + " layout, each seed its own: errors that repeat, and that cameras share.",
            MATCH,
            seed -> ConsistencyRuns.errors(seed, mounts(1, 1)),
            0,
            List.of()));
    all.add(
        new Scenario(
            "clean-match-mounts-2x",
            "The same, twice as far off.",
            MATCH,
            seed -> ConsistencyRuns.errors(seed, mounts(2, 2)),
            0,
            List.of()));
    all.add(
        new Scenario(
            "clean-match-tags-off",
            "The clean match, each tag a little off the layout (a centimeter along each axis, as a"
                + " standard deviation) and every camera exactly on its mount.",
            MATCH,
            seed -> ConsistencyRuns.errors(seed, mounts(0, 1)),
            0,
            List.of()));
    all.add(
        new Scenario(
            "clean-match-mounts-off",
            "The clean match, each camera a little off its mount (3 mm along and a quarter degree"
                + " about each axis) and every tag exactly on the layout.",
            MATCH,
            seed -> ConsistencyRuns.errors(seed, mounts(1, 0)),
            0,
            List.of()));
    all.add(
        new Scenario(
            "clean-match-flips",
            "The clean match, 2% of every camera's images with a tag's two poses the wrong way"
                + " round: errors with long tails.",
            MATCH,
            typical,
            0.02,
            List.of()));
    return List.copyOf(all);
  }

  /**
   * How far off each seed's cameras and tags sit: the estimator's own allowance for each, times
   * these.
   */
  private static ConsistencyRuns.Spread mounts(double mountTimes, double tagTimes) {
    ConsistencyRuns.Spread claimed = ConsistencyRuns.Spread.matched(ExampleRobot.SIM.estimator());
    return new ConsistencyRuns.Spread(
        mountTimes * claimed.mountMillimeters(),
        mountTimes * claimed.mountDegrees(),
        tagTimes * claimed.tagMillimeters(),
        claimed.gyroScale(),
        claimed.backupScale());
  }

  private static Timed[] badMatch() {
    return new Timed[] {
      atTheStart(
          stage -> {
            for (int module = 0; module < FaultCatalog.MODULES.size(); module++) {
              stage.fault(
                  Setting.of(
                      FaultCatalog.module(module),
                      FaultCatalog.WORN,
                      Map.of(FaultCatalog.SCALE, 0.97)));
            }
          }),
      at(
          20,
          stage ->
              stage.fault(
                  Setting.of(
                      camera(stage, 0),
                      FaultCatalog.TURNED,
                      Map.of(
                          FaultCatalog.ROLL,
                          0.0,
                          FaultCatalog.PITCH,
                          2.0,
                          FaultCatalog.YAW,
                          3.0)))),
      at(
          40,
          stage ->
              stage.knock(
                  robot(
                      FaultCatalog.HIT,
                      Map.of(FaultCatalog.FORWARD, 0.0, FaultCatalog.LEFT, 0.6)))),
      at(60, stage -> unplugEveryCamera(stage)),
      at(63, stage -> plugEveryCameraIn(stage)),
      at(
          80,
          stage ->
              stage.fault(
                  Setting.of(
                      FaultCatalog.module(0),
                      FaultCatalog.SPINNING,
                      Map.of(FaultCatalog.SPEED, 1.0)))),
      at(82, stage -> stage.undo(FaultCatalog.module(0), FaultCatalog.SPINNING)),
      at(
          100,
          stage ->
              stage.knock(
                  robot(
                      FaultCatalog.SHOVED,
                      Map.of(
                          FaultCatalog.FORWARD,
                          0.0,
                          FaultCatalog.LEFT,
                          0.6,
                          FaultCatalog.TURN,
                          0.0)))),
      at(115, stage -> stage.misread(Misread.WRONG_ID, 60)),
      at(130, stage -> stage.fault(Setting.of(GYRO, FaultCatalog.UNPLUGGED)))
    };
  }

  private static Scenario blind(String name, double seconds) {
    return scenario(
        name,
        String.format("Every camera unplugged for %.0f s, from 11 s.", seconds),
        SHORT,
        at(FAULT_SECONDS, EstimatorScenarios::unplugEveryCamera),
        at(FAULT_SECONDS + seconds, EstimatorScenarios::plugEveryCameraIn));
  }

  private static void unplugEveryCamera(Stage stage) {
    for (String camera : stage.cameraNames()) {
      stage.fault(Setting.of(FaultCatalog.camera(camera), FaultCatalog.UNPLUGGED));
    }
  }

  private static void plugEveryCameraIn(Stage stage) {
    for (String camera : stage.cameraNames()) {
      stage.undo(FaultCatalog.camera(camera), FaultCatalog.UNPLUGGED);
    }
  }

  /** A season-worn robot on a route, with what happens on the way. */
  private static Scenario scenario(String name, String description, Route route, Timed... actions) {
    return new Scenario(name, description, route, EstimatorRuns::typical, 0, List.of(actions));
  }

  private static Timed at(double seconds, Consumer<Stage> action) {
    return new Timed(seconds, action);
  }

  /** As the robot is placed: before its second loop, a second before it's enabled. */
  private static Timed atTheStart(Consumer<Stage> action) {
    return new Timed(-EstimatorRuns.DISABLED_SECONDS, action);
  }

  private static Setting robot(String fault, Map<String, Double> values) {
    return Setting.of(ROBOT, fault, values);
  }

  /** One of the robot's cameras, as the simulator's controls name it. */
  private static String camera(Stage stage, int index) {
    return FaultCatalog.camera(stage.cameraNames().get(index));
  }

  /**
   * The scenario with this name.
   *
   * @throws IllegalArgumentException naming every scenario there is, when none has this name
   */
  static Scenario named(String name) {
    for (Scenario scenario : ALL) {
      if (scenario.name().equals(name)) {
        return scenario;
      }
    }
    throw new IllegalArgumentException(
        "No estimator scenario is named \""
            + name
            + "\". The scenarios: "
            + ALL.stream().map(Scenario::name).collect(Collectors.joining(", ")));
  }
}
