package frc.robot.studies;

import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.localization.LoopContext;
import com.michaelgrundvig.frc.localization.VisionDecision;
import com.michaelgrundvig.frc.localization.VisionMeasurement;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.SimulatedRobot;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.simulation.SensorErrorsBuilder;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.LongFunction;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.system.RobotController;

/**
 * How the estimator study runs the simulated robot: a route driven as a driver drives it, a
 * scenario's faults at their moments, and beside the robot's own estimator, in the same loop, each
 * of WPILib's ({@link WpilibContender}) given the same wheel readings and yaw and the same cameras'
 * images.
 *
 * <p>Every run starts the same way: the robot placed on its start, a second disabled while the
 * cameras see it, then enabled and driven. Every loop from the first is scored. Times in a scenario
 * count from the moment the robot is enabled.
 */
final class EstimatorRuns {
  /** The robot's own estimator's name in the report. */
  static final String OURS = "A";

  /** How long the robot stands disabled on its start before it's enabled. */
  static final double DISABLED_SECONDS = 1.0;

  /** The field the simulator's robot plays on. */
  static final FieldModel FIELD = Rebuilt.field(ExampleRobot.SIM.definition().body());

  private EstimatorRuns() {}

  /**
   * One leg of a route: where the driver steers for, and for how long.
   *
   * @param seconds how long the leg lasts
   * @param target where the driver steers the robot, by eye; null to let the sticks go
   * @param stick the most stick the driver gives, from 0 to 1
   * @param rest whether the leg is a stop to score at rest: {@link Rest#NONE} for a leg that moves
   */
  record Leg(double seconds, Pose2d target, double stick, Rest rest) {}

  /** What a stop is for. */
  enum Rest {
    NONE,
    NEAR_A_TAG,
    FAR_FROM_TAGS
  }

  /** A route: its legs, in order. */
  record Route(String name, List<Leg> legs) {
    Route {
      legs = List.copyOf(legs);
    }

    double seconds() {
      return legs.stream().mapToDouble(Leg::seconds).sum();
    }

    /** Starts a route. */
    static Builder named(String name) {
      return new Builder(name);
    }

    /** A route being written, leg by leg. */
    static final class Builder {
      private final String name;
      private final List<Leg> legs = new ArrayList<>();
      private Pose2d last = Pose2d.ZERO;

      private Builder(String name) {
        this.name = name;
      }

      /** Drives toward a pose for a while, at up to this much stick. */
      Builder drive(double seconds, double x, double y, double degrees, double stick) {
        last = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
        legs.add(new Leg(seconds, last, stick, Rest.NONE));
        return this;
      }

      /** Settles on the last pose, then lets the sticks go and stands: a stop that is scored. */
      Builder stop(double seconds, Rest rest) {
        legs.add(new Leg(1.0, last, 0.5, Rest.NONE));
        legs.add(new Leg(seconds, last, 0, rest));
        return this;
      }

      Route build() {
        return new Route(name, legs);
      }
    }
  }

  /** The robot and its cameras, as a scenario's actions reach them. */
  static final class Stage {
    private final SimulatedRobot robot;
    private final EstimatorRig.Cameras cameras;
    private final EstimatorScores.Marks marks;
    private final List<WpilibContender> contenders;
    private int loop;
    private boolean enabled;

    private Stage(
        SimulatedRobot robot,
        EstimatorRig.Cameras cameras,
        EstimatorScores.Marks marks,
        List<WpilibContender> contenders) {
      this.robot = robot;
      this.cameras = cameras;
      this.marks = marks;
      this.contenders = contenders;
    }

    /** A fault that lasts starts now, through the simulator's controls. */
    void fault(Setting... faults) {
      robot.apply(faults);
      marks.faults().add(loop);
    }

    /** Something moves the robot at once, now: a shove, a hit, a carry. */
    void knock(Setting... events) {
      robot.apply(events);
      marks.faults().add(loop);
      marks.movedByHand().add(loop);
    }

    /** Undoes a fault that lasts. */
    void undo(String component, String fault) {
      robot.undo(component, fault);
    }

    /** A camera misreads its next images of a tag. */
    void misread(EstimatorRig.Cameras.Misread how, int images) {
      cameras.misread(how, images);
      marks.faults().add(loop);
    }

    /** Every camera reads every tag this many times as far as it is, from now. */
    void rangeBias(double scale) {
      cameras.rangeBias(scale);
    }

    /** Every camera's ranges share one wandering error of this spread, from now. */
    void sharedRangeNoise(double stdDev, double seconds) {
      cameras.sharedRangeNoise(stdDev, seconds);
    }

    /** Disables the robot, or enables it again. */
    void enable(boolean on) {
      enabled = on;
      robot.setEnabled(on);
    }

    /**
     * The robot believes it was placed this far from where it stands: its own localizer is told so,
     * as a placement on its start tells it, and so is every other.
     */
    void misplace(Transform2d off) {
      Pose2d believed = robot.truePose().plus(off);
      robot.believePlaced(believed);
      for (WpilibContender contender : contenders) {
        contender.place(believed, RobotController.getTime());
      }
      marks.faults().add(loop);
    }

    /** The names of the robot's cameras. */
    List<String> cameraNames() {
      return robot.cameraNames();
    }
  }

  /** Something a scenario does at a moment. */
  record Timed(double seconds, Consumer<Stage> action) {}

  /**
   * One scenario.
   *
   * @param name how it's asked for
   * @param description what happens in it
   * @param route the route driven
   * @param errors how the robot is built for a seed
   * @param flipShare the share of every camera's images that come with a tag's poses flipped
   * @param actions what happens, each at its moment after the robot is enabled
   */
  record Scenario(
      String name,
      String description,
      Route route,
      LongFunction<SensorErrors> errors,
      double flipShare,
      List<Timed> actions) {
    Scenario {
      actions = List.copyOf(actions);
    }
  }

  /** A season-worn robot, its noise drawn from a seed. */
  static SensorErrors typical(long seed) {
    return SensorErrorsBuilder.builder(SensorErrors.TYPICAL).seed(seed).build();
  }

  /** Wheels and sensors exactly as configured; the cameras' noise drawn from a seed. */
  static SensorErrors ideal(long seed) {
    return SensorErrorsBuilder.builder(SensorErrors.NONE).seed(seed).build();
  }

  /**
   * One run: each estimator's measures, by its name, and what WPILib did with what it was given.
   *
   * @param measures each estimator's measures, the robot's own first
   * @param tracks each estimator's pose and the truth, loop by loop
   * @param added by WPILib estimator, the measurements that corrected it
   * @param laterDiscarded by WPILib estimator, the corrections it discarded for an older
   *     measurement
   * @param offered by WPILib estimator, the measurements it was offered
   * @param oursOffered the measurements the robot's own estimator was handed
   * @param oursAccepted and how many of them it accepted
   * @param misreads how many images were misread
   * @param samplesPerLoop how many wheel readings a loop brought, on average
   * @param loops every loop's inputs, when they were asked for; otherwise none
   * @param start where the robot was placed
   */
  record Played(
      Map<String, EstimatorScores.Measures> measures,
      Map<String, EstimatorScores.Track> tracks,
      Map<String, Integer> added,
      Map<String, Integer> laterDiscarded,
      Map<String, Integer> offered,
      int oursOffered,
      int oursAccepted,
      int misreads,
      double samplesPerLoop,
      List<EstimatorCost.Loop> loops,
      Pose2d start) {}

  /** The estimators WPILib's is run as, in the report's order. */
  static List<WpilibContender> contenders() {
    RobotDefinition kinematics = ExampleRobot.SIM.definition();
    WpilibContender.Rules template = WpilibContender.Rules.TEMPLATE;
    WpilibContender.Rules tighter = template.withState(0.03, Math.toRadians(1));
    Map<String, WpilibContender.Rules> team = new LinkedHashMap<>();
    team.put("C", template);
    team.put("C.03", tighter);
    team.put("C.01", template.withState(0.01, 0.002));
    team.put("C.003", template.withState(0.003, 0.002));
    team.put("C.003n2", template.withState(0.003, 0.002).withNoise(0.01, 0.03, 2));
    team.put("C.03sorted", tighter.with(true, true));
    team.put("C.03strict", tighter.withLimits(0.2, 0.2));
    team.put("C.03pick", tighter.pickingByHeading());
    team.put("C.03gyro", tighter.with(false, false));
    List<WpilibContender> all = new ArrayList<>();
    all.add(contender("B", WpilibContender.Feed.SAME_INPUTS, template, kinematics));
    all.add(contender("B.03", WpilibContender.Feed.SAME_INPUTS, tighter, kinematics));
    all.add(
        contender(
            "B.003",
            WpilibContender.Feed.SAME_INPUTS,
            template.withState(0.003, 0.002),
            kinematics));
    team.forEach(
        (name, rules) -> all.add(contender(name, WpilibContender.Feed.TEAM, rules, kinematics)));
    all.add(contender("D", WpilibContender.Feed.RAW, template, kinematics));
    all.add(contender("O", WpilibContender.Feed.NONE, template, kinematics));
    return all;
  }

  private static WpilibContender contender(
      String name, WpilibContender.Feed feed, WpilibContender.Rules rules, RobotDefinition robot) {
    return new WpilibContender(name, feed, rules, FIELD, robot);
  }

  /** Plays a scenario with a seed. */
  static Played play(Scenario scenario, long seed) {
    return play(scenario, seed, false, CameraConditions.NOMINAL);
  }

  /**
   * Plays a scenario with a seed.
   *
   * @param keepLoops whether every loop's inputs are kept, to cost the estimators by
   */
  static Played play(Scenario scenario, long seed, boolean keepLoops) {
    return play(scenario, seed, keepLoops, CameraConditions.NOMINAL);
  }

  /**
   * Plays a scenario with a seed, its cameras erring as these conditions say on top of what the
   * scenario does.
   *
   * @param keepLoops whether every loop's inputs are kept, to cost the estimators by
   */
  static Played play(
      Scenario scenario, long seed, boolean keepLoops, CameraConditions cameraConditions) {
    List<EstimatorCost.Loop> kept = new ArrayList<>();
    Pose2d start = Pose2d.ZERO;
    EstimatorRig.Cameras cameras =
        new EstimatorRig.Cameras(FIELD, ExampleRobot.SIM.definition().cameras(), seed);
    cameras.flipShare(scenario.flipShare());
    List<WpilibContender> contenders = contenders();
    EstimatorScores.Marks marks = new EstimatorScores.Marks();
    Map<String, EstimatorScores.Track> tracks = new LinkedHashMap<>();
    tracks.put(OURS, new EstimatorScores.Track());
    for (WpilibContender contender : contenders) {
      tracks.put(contender.name(), new EstimatorScores.Track());
    }
    int samples = 0;
    int oursOffered = 0;
    int oursAccepted = 0;
    SensorErrors errors = cameraConditions.on(scenario.errors().apply(seed), seed);
    try (SimulatedRobot robot =
        new SimulatedRobot(cameraConditions.robot(), errors, cameras::watch)) {
      Stage stage = new Stage(robot, cameras, marks, contenders);
      int disabledLoops = RobotLoop.loops(DISABLED_SECONDS);
      int loops = disabledLoops + RobotLoop.loops(scenario.route().seconds());
      List<Timed> pending = new ArrayList<>(scenario.actions());
      for (int loop = 0; loop < loops; loop++) {
        stage.loop = loop;
        double seconds = (loop - disabledLoops) * RobotLoop.PERIOD_SECONDS;
        if (loop == disabledLoops) {
          stage.enable(true);
        }
        // What happens between loops happens before this one, once the robot has been placed.
        if (loop > 0) {
          pending.removeIf(
              timed -> {
                if (timed.seconds() <= seconds + 1e-9) {
                  timed.action().accept(stage);
                  return true;
                }
                return false;
              });
        }
        steer(robot, scenario.route(), seconds, stage.enabled, marks, loop);
        cameras.nextLoop(RobotLoop.PERIOD_SECONDS);
        robot.loop();

        // Every contender is given the loop as the robot's own localizer was: its readings, then a
        // placement if there was one, then its measurements.
        DrivetrainReadings readings = robot.readings();
        long now = RobotController.getTime();
        LoopContext context = new LoopContext(now, stage.enabled);
        for (WpilibContender contender : contenders) {
          contender.odometry(readings, context);
        }
        samples += readings.wheels().size();
        if (loop == 0) {
          // Placed on its start: the placement the robot's own localizer just got.
          start = robot.truePose();
          for (WpilibContender contender : contenders) {
            contender.place(start, now);
          }
        }
        ImuReadings imu = EstimatorRig.imu(readings);
        List<VisionDecision> decisions = robot.visionDecisions();
        List<VisionMeasurement> measurements = robot.visionMeasurements();
        List<WpilibContender.Image> images = cameras.drain();
        oursOffered += decisions.size();
        oursAccepted += (int) decisions.stream().filter(VisionDecision::accepted).count();
        if (keepLoops) {
          kept.add(new EstimatorCost.Loop(now, stage.enabled, readings, measurements, images));
        }
        Pose2d truth = robot.truePose();
        track(tracks, OURS).add(robot.pose(), truth, robot.estimate().covariance());
        for (WpilibContender contender : contenders) {
          contender.imu(imu.yawRateRadiansPerSecond(), imu.pitchRadians(), imu.rollRadians());
          if (contender.feed() == WpilibContender.Feed.SAME_INPUTS) {
            contender.measurements(measurements);
          } else {
            contender.images(images);
          }
          track(tracks, contender.name()).add(contender.pose(), truth);
        }
      }
      if (!pending.isEmpty()) {
        throw new IllegalStateException(
            scenario.name() + ": an action is timed after its route ends");
      }
      Map<String, EstimatorScores.Measures> measures = new LinkedHashMap<>();
      tracks.forEach((name, track) -> measures.put(name, EstimatorScores.measure(track, marks)));
      Map<String, Integer> added = new LinkedHashMap<>();
      Map<String, Integer> discarded = new LinkedHashMap<>();
      Map<String, Integer> offered = new LinkedHashMap<>();
      for (WpilibContender contender : contenders) {
        added.put(contender.name(), contender.added());
        discarded.put(contender.name(), contender.laterDiscarded());
        offered.put(contender.name(), contender.offered());
      }
      return new Played(
          measures,
          tracks,
          added,
          discarded,
          offered,
          oursOffered,
          oursAccepted,
          cameras.misreads(),
          (double) samples / loops,
          kept,
          start);
    }
  }

  private static EstimatorScores.Track track(
      Map<String, EstimatorScores.Track> tracks, String name) {
    return Objects.requireNonNull(tracks.get(name), name);
  }

  /**
   * Holds the driver's sticks for a loop: toward the leg's target as the driver sees the robot
   * (where it really is), at up to the leg's stick. A stop lets the sticks go, and its last loop is
   * marked for scoring.
   */
  private static void steer(
      SimulatedRobot robot,
      Route route,
      double seconds,
      boolean enabled,
      EstimatorScores.Marks marks,
      int loop) {
    if (seconds < 0) {
      robot.sticks(0, 0, 0);
      return;
    }
    double start = 0;
    for (Leg leg : route.legs()) {
      double end = start + leg.seconds();
      if (seconds < end - 1e-9) {
        boolean lastLoop = seconds + RobotLoop.PERIOD_SECONDS >= end - 1e-9;
        if (lastLoop && enabled && leg.rest() == Rest.NEAR_A_TAG) {
          marks.restsNear().add(loop);
        } else if (lastLoop && enabled && leg.rest() == Rest.FAR_FROM_TAGS) {
          marks.restsFar().add(loop);
        }
        steerFor(robot, leg);
        return;
      }
      start = end;
    }
    robot.sticks(0, 0, 0);
  }

  private static void steerFor(SimulatedRobot robot, Leg leg) {
    if (leg.stick() == 0) {
      robot.sticks(0, 0, 0);
      return;
    }
    Pose2d truth = robot.truePose();
    double x = leg.target().getX() - truth.getX();
    double y = leg.target().getY() - truth.getY();
    double meters = Math.hypot(x, y);
    double forward = 0;
    double left = 0;
    if (meters > ARRIVED_METERS) {
      // As fast as still stops there, braking gently, and no faster than the leg's stick.
      double speed = Math.sqrt(2 * BRAKING_METERS_PER_SEC_SQ * meters);
      double stick = Math.min(leg.stick(), stickFor(speed / TOP_METERS_PER_SECOND));
      forward = stick * x / meters;
      left = stick * y / meters;
    }
    double radians = leg.target().getRotation().minus(truth.getRotation()).getRadians();
    double turn = 0;
    if (Math.abs(radians) > ARRIVED_RADIANS) {
      double stick = Math.min(leg.stick(), stickFor(Math.abs(radians) / FULL_TURN_STICK_RADIANS));
      turn = Math.copySign(stick, radians);
    }
    robot.sticks(forward, left, turn);
  }

  /**
   * The stick that asks for a share of the robot's top speed: the drivetrain squares what is past
   * its deadband.
   */
  private static double stickFor(double share) {
    return STICK_DEADBAND + (1 - STICK_DEADBAND) * Math.sqrt(Math.min(1, share));
  }

  /** The driver lets the sticks go this near the target. */
  private static final double ARRIVED_METERS = 0.03;

  private static final double ARRIVED_RADIANS = Math.toRadians(1.5);

  /** How hard the driver brakes toward a target. */
  private static final double BRAKING_METERS_PER_SEC_SQ = 2.0;

  /** About the robot's top speed: what full stick asks for. */
  private static final double TOP_METERS_PER_SECOND = 4.5;

  /** The driver gives full turn stick this far from the target's heading. */
  private static final double FULL_TURN_STICK_RADIANS = 1.5;

  /** The drivetrain's stick deadband. */
  private static final double STICK_DEADBAND = 0.1;
}
