package frc.robot.subsystems.drive;

import frc.robot.lib.timing.RobotLoop;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisAccelerations;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.DrivetrainSplineTrajectoryGenerator;
import org.wpilib.math.trajectory.DrivetrainSplineTrajectoryParameterizer;
import org.wpilib.math.trajectory.HolonomicSample;
import org.wpilib.math.trajectory.HolonomicTrajectory;
import org.wpilib.math.trajectory.TrajectoryConfig;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.constraint.CentripetalAccelerationConstraint;

/**
 * The timed paths the drivetrain's moves follow, worked out as a move starts, from where the robot
 * is and how it's moving, under what the move may ask of it: a straight line, or a smooth path
 * through keyframes. Each is a WPILib {@link HolonomicTrajectory}, field-relative, every sample's
 * acceleration the change to the next, as the trajectory's own interpolation takes it.
 */
final class Trajectories {
  /**
   * What a move may ask of the robot.
   *
   * @param speed top speed, m/s
   * @param rate top turning rate, rad/s
   * @param acceleration how fast it speeds up, slows, and swerves in a curve, m/s²
   * @param angularAcceleration how fast it spins up, rad/s²
   */
  record Limits(double speed, double rate, double acceleration, double angularAcceleration) {}

  /**
   * A timed path, and when it passes each keyframe.
   *
   * @param trajectory the path, field-relative
   * @param passing for each keyframe, the seconds from the path's start it passes it; none for a
   *     line
   */
  record Plan(HolonomicTrajectory trajectory, double[] passing) {}

  /**
   * How far apart keyframes must be, meters: closer, a spline through them folds back on itself.
   */
  static final double APART_METERS = 0.05;

  /** How near a keyframe a path's sample must come to be where it passes it, meters. */
  private static final double PASSES_METERS = 0.001;

  private Trajectories() {}

  /**
   * Plans one short path, so the first move's plan, at the start of an auto, doesn't load WPILib's
   * spline and trajectory classes inside a robot loop. On a desktop, the first plan takes 65 ms,
   * the next 5 ms, and a plan with the code warm 0.3 ms.
   */
  static void warmUp() {
    through(
        Pose2d.ZERO,
        new ChassisVelocities(),
        List.of(
            Keyframe.at(new Pose2d(1, 0, Rotation2d.ZERO)),
            Keyframe.at(new Pose2d(2, 1, Rotation2d.CCW_90DEG))),
        new Limits(4, 6, 2, 9));
  }

  /**
   * In a straight line to a place, turning to a heading on the way, the nearer way round: one
   * trapezoid profile along the line, and one for the heading, each from the robot's speed that way
   * as it starts. To the place it starts at, it turns where it stands.
   */
  static Plan line(
      Pose2d start, ChassisVelocities moving, Translation2d to, Rotation2d facing, Limits limits) {
    TrapezoidProfile along =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(limits.speed, limits.acceleration));
    TrapezoidProfile turning =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(limits.rate, limits.angularAcceleration));
    Translation2d line = to.minus(start.getTranslation());
    Rotation2d direction = line.getAngle().orElse(Rotation2d.ZERO);
    double from = start.getRotation().getRadians();
    TrapezoidProfile.State leaving =
        new TrapezoidProfile.State(
            0, moving.vx * direction.getCos() + moving.vy * direction.getSin());
    TrapezoidProfile.State turned = new TrapezoidProfile.State(from, moving.omega);
    TrapezoidProfile.State there = new TrapezoidProfile.State(line.getNorm(), 0);
    TrapezoidProfile.State faced =
        new TrapezoidProfile.State(from + facing.minus(start.getRotation()).getRadians(), 0);
    double duration =
        Math.max(along.timeLeftUntil(leaving, there), turning.timeLeftUntil(turned, faced));
    List<Point> points = new ArrayList<>();
    int steps = (int) Math.ceil(duration / RobotLoop.PERIOD_SECONDS);
    for (int i = 0; i <= steps; i++) {
      double t = i * RobotLoop.PERIOD_SECONDS;
      TrapezoidProfile.State gone = along.calculate(t, leaving, there);
      TrapezoidProfile.State facingNow = turning.calculate(t, turned, faced);
      Translation2d at = start.getTranslation().plus(new Translation2d(gone.position, direction));
      Translation2d velocity = new Translation2d(gone.velocity, direction);
      points.add(
          new Point(
              t,
              at,
              facingNow.position,
              new ChassisVelocities(velocity.getX(), velocity.getY(), facingNow.velocity)));
    }
    return new Plan(timed(points), new double[0]);
  }

  /**
   * A smooth path through keyframes, stopping at the last: the shape a clamped cubic spline through
   * their places, leaving toward the first and arriving from the one before the last; its timing
   * WPILib's, under the top speed, and the acceleration both along the path and across it in
   * curves. The robot turns to each keyframe's heading from the moment it has passed, and faced,
   * the one before, the nearer way round, on a trapezoid profile; if it hasn't faced the last when
   * it gets there, the path waits there while it does.
   *
   * @throws IllegalArgumentException if no path goes through them: one too near the one before
   */
  static Plan through(
      Pose2d start, ChassisVelocities moving, List<Keyframe> keyframes, Limits limits) {
    Translation2d before = start.getTranslation();
    List<Translation2d> places = new ArrayList<>();
    for (Keyframe keyframe : keyframes) {
      Translation2d place = keyframe.pose().getTranslation();
      if (place.getDistance(before) < APART_METERS) {
        throw new IllegalArgumentException(
            String.format(
                Locale.ROOT,
                "%s is within %.0f cm of %s",
                keyframe,
                APART_METERS * 100,
                places.isEmpty() ? "where the robot starts" : "the keyframe before"));
      }
      places.add(place);
      before = place;
    }
    int last = places.size() - 1;
    Rotation2d leaving = places.get(0).minus(start.getTranslation()).getAngle().orElseThrow();
    Rotation2d arriving =
        places
            .get(last)
            .minus(last == 0 ? start.getTranslation() : places.get(last - 1))
            .getAngle()
            .orElseThrow();
    double already = moving.vx * leaving.getCos() + moving.vy * leaving.getSin();
    TrajectoryConfig config =
        new TrajectoryConfig(limits.speed, limits.acceleration)
            .addConstraint(new CentripetalAccelerationConstraint(limits.acceleration))
            .setStartVelocity(Math.clamp(already, 0, limits.speed));
    List<? extends HolonomicSample> path;
    try {
      path =
          DrivetrainSplineTrajectoryGenerator.generate(
                  new Pose2d(start.getTranslation(), leaving),
                  places.subList(0, last),
                  new Pose2d(places.get(last), arriving),
                  config)
              .getSamples();
    } catch (DrivetrainSplineTrajectoryParameterizer.TrajectoryGenerationException e) {
      throw new IllegalArgumentException("no path through its keyframes: " + e.getMessage(), e);
    }
    if (path.size() < 2) {
      throw new IllegalArgumentException("no path through its keyframes");
    }

    double[] passing = new double[places.size()];
    int from = 0;
    for (int k = 0; k <= last; k++) {
      from = passes(path, places.get(k), from);
      passing[k] = path.get(from).time;
    }
    passing[last] = path.get(path.size() - 1).time;

    // Each keyframe's heading, unwrapped: the nearer way round from the one before.
    double[] faced = new double[places.size()];
    double was = start.getRotation().getRadians();
    for (int k = 0; k <= last; k++) {
      was += keyframes.get(k).pose().getRotation().minus(new Rotation2d(was)).getRadians();
      faced[k] = was;
    }
    TrapezoidProfile turning =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(limits.rate, limits.angularAcceleration));
    Facing facing = new Facing(turning, passing, faced, start.getRotation().getRadians(), moving);
    List<Point> points = new ArrayList<>();
    for (HolonomicSample sample : path) {
      TrapezoidProfile.State now = facing.at(sample.time);
      points.add(
          new Point(
              sample.time,
              sample.pose.getTranslation(),
              now.position,
              new ChassisVelocities(sample.velocity.vx, sample.velocity.vy, now.velocity)));
    }
    double end = passing[last];
    double faces = facing.facedLast();
    for (double t = end + RobotLoop.PERIOD_SECONDS;
        t < faces + RobotLoop.PERIOD_SECONDS;
        t += RobotLoop.PERIOD_SECONDS) {
      TrapezoidProfile.State now = facing.at(t);
      points.add(
          new Point(t, places.get(last), now.position, new ChassisVelocities(0, 0, now.velocity)));
    }
    return new Plan(timed(points), passing);
  }

  /** The index of the first sample from {@code from} at a place, or the nearest after it. */
  private static int passes(List<? extends HolonomicSample> path, Translation2d place, int from) {
    int nearest = from;
    double best = Double.POSITIVE_INFINITY;
    for (int i = from; i < path.size(); i++) {
      double off = path.get(i).pose.getTranslation().getDistance(place);
      if (off <= PASSES_METERS) {
        return i;
      }
      if (off < best) {
        best = off;
        nearest = i;
      }
    }
    return nearest;
  }

  /**
   * The robot's heading along a path through keyframes: a trapezoid profile toward each keyframe's
   * heading in turn, the next begun once the path has passed a keyframe and the robot has faced it.
   */
  private static final class Facing {
    private final TrapezoidProfile turning;
    private final double[] passing;
    private final double[] faced;

    /** The keyframe it's turning toward, from when, and from what heading and rate. */
    private int toward;

    private double since;
    private TrapezoidProfile.State from;

    Facing(
        TrapezoidProfile turning,
        double[] passing,
        double[] faced,
        double start,
        ChassisVelocities moving) {
      this.turning = turning;
      this.passing = passing;
      this.faced = faced;
      from = new TrapezoidProfile.State(start, moving.omega);
    }

    /** Its heading and rate at a time, never earlier than the last asked. */
    TrapezoidProfile.State at(double t) {
      while (toward < faced.length - 1) {
        double facedAt = since + turning.timeLeftUntil(from, goal());
        if (t < passing[toward] || t < facedAt) {
          break;
        }
        since = Math.max(passing[toward], facedAt);
        from = goal();
        toward++;
      }
      return turning.calculate(t - since, from, goal());
    }

    /** When it faces the last keyframe's heading, asked once the path has ended. */
    double facedLast() {
      double t = passing[passing.length - 1];
      while (true) {
        at(t);
        t = since + turning.timeLeftUntil(from, goal());
        if (toward == faced.length - 1) {
          return t;
        }
      }
    }

    private TrapezoidProfile.State goal() {
      return new TrapezoidProfile.State(faced[toward], 0);
    }
  }

  /** A path's point: when, where, which way it faces (unwrapped), and its velocity. */
  private record Point(
      double time, Translation2d place, double facing, ChassisVelocities velocity) {}

  /** The points as a trajectory, each acceleration the change to the next: the last none. */
  private static HolonomicTrajectory timed(List<Point> points) {
    List<HolonomicSample> samples = new ArrayList<>(points.size());
    for (int i = 0; i < points.size(); i++) {
      Point point = points.get(i);
      ChassisAccelerations change = new ChassisAccelerations();
      if (i + 1 < points.size()) {
        Point next = points.get(i + 1);
        double dt = next.time - point.time;
        change =
            new ChassisAccelerations(
                (next.velocity.vx - point.velocity.vx) / dt,
                (next.velocity.vy - point.velocity.vy) / dt,
                (next.velocity.omega - point.velocity.omega) / dt);
      }
      samples.add(
          new HolonomicSample(
              point.time,
              new Pose2d(point.place, new Rotation2d(point.facing)),
              point.velocity,
              change));
    }
    return new HolonomicTrajectory(samples);
  }
}
