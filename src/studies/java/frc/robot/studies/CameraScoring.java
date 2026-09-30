package frc.robot.studies;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import frc.robot.RobotConfig;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.localization.CaptureContext;
import frc.robot.lib.localization.PoseCovariance;
import frc.robot.lib.localization.TagDetections;
import frc.robot.lib.localization.VisionMeasurement;
import frc.robot.lib.localization.VisionPipeline;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.SplittableRandom;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.IntConsumer;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Scores camera layouts all over the field. At each pose, each camera takes its images (with
 * PhotonVision's simulation, {@link CameraImageSim}), the robot's own vision pipeline turns each
 * into a measurement or throws it out, and each layout's measurements are combined as the estimator
 * combines them. What each layout's cameras saw, and how close the combination came to the truth,
 * is its {@link Outcome} there.
 *
 * <p>A camera that several layouts share (the same mount and sensor) takes its images once, and its
 * noise depends only on the seed, the pose, and the mount, so a layout scores the same whatever
 * else is tried beside it, and on any number of threads.
 */
public final class CameraScoring {
  private final RobotConfig robot;
  private final FieldModel field;
  private final List<Candidate> candidates;
  private final CameraStudyConfig config;
  private final VisionPipeline pipeline;

  /** Every distinct camera, over every layout: where it is, which way it looks, and its sensor. */
  private final List<CameraConfig> mounts = new ArrayList<>();

  /** Each layout's cameras, as their places in {@link #mounts}. */
  private final int[][] layouts;

  /** The distinct sensors, and each mount's among them. */
  private final List<CameraConfig.Sensor> sensors = new ArrayList<>();

  private final int[] sensorOf;
  private final long[] mountSeeds;

  /**
   * @param robot the robot the cameras go on: its body, and its vision pipeline's settings
   * @param field the field, whose tags the cameras see
   * @param candidates the layouts to score
   * @throws IllegalArgumentException if a layout has a camera outside the robot, naming the layout
   *     and the camera
   */
  public CameraScoring(
      RobotConfig robot, FieldModel field, List<Candidate> candidates, CameraStudyConfig config) {
    this.robot = robot;
    this.field = field;
    this.candidates = List.copyOf(candidates);
    this.config = config;
    pipeline = new VisionPipeline(field);
    Map<Mount, Integer> index = new LinkedHashMap<>();
    layouts = new int[candidates.size()][];
    for (int c = 0; c < candidates.size(); c++) {
      Candidate candidate = candidates.get(c);
      candidate.on(robot);
      List<CameraConfig> cameras = candidate.cameras();
      layouts[c] = new int[cameras.size()];
      for (int i = 0; i < cameras.size(); i++) {
        CameraConfig camera = cameras.get(i);
        Integer at = index.get(Mount.of(camera));
        if (at == null) {
          at = mounts.size();
          index.put(Mount.of(camera), at);
          mounts.add(camera);
        }
        layouts[c][i] = at;
      }
    }
    sensorOf = new int[mounts.size()];
    mountSeeds = new long[mounts.size()];
    for (int m = 0; m < mounts.size(); m++) {
      CameraConfig camera = mounts.get(m);
      if (!sensors.contains(camera.sensor())) {
        sensors.add(camera.sensor());
      }
      sensorOf[m] = sensors.indexOf(camera.sensor());
      mountSeeds[m] = Mount.of(camera).seed();
    }
  }

  /**
   * A camera as far as its images go: its mount, sensor, and noise. Two cameras with different
   * names but the same of these take the same images.
   */
  private record Mount(
      double forward,
      double left,
      double up,
      double roll,
      double tiltUp,
      double yaw,
      CameraConfig.Sensor sensor,
      double noiseScale) {
    static Mount of(CameraConfig camera) {
      return new Mount(
          camera.forward().in(Meters),
          camera.left().in(Meters),
          camera.up().in(Meters),
          camera.roll().in(Radians),
          camera.tiltUp().in(Radians),
          camera.yaw().in(Radians),
          camera.sensor(),
          camera.noiseScale());
    }

    /** A number from where the camera is and how it points: the same on every computer. */
    long seed() {
      return mix(0, forward, left, up, roll, tiltUp, yaw);
    }
  }

  /** How many distinct cameras the layouts have among them. */
  public int mounts() {
    return mounts.size();
  }

  /**
   * Every layout's outcome at every sample.
   *
   * @param progress told how many samples are done, after each one
   */
  public Result score(List<Sample> samples, IntConsumer progress) {
    Outcome[][][] outcomes = new Outcome[samples.size()][][];
    AtomicInteger next = new AtomicInteger();
    AtomicInteger done = new AtomicInteger();
    int threads = Math.min(config.threads(), Math.max(1, samples.size()));
    List<List<CameraImageSim>> cameras = new ArrayList<>();
    for (int w = 0; w < threads; w++) {
      List<CameraImageSim> own = new ArrayList<>(); // each thread its own: their noise isn't shared
      for (CameraConfig.Sensor sensor : sensors) {
        own.add(new CameraImageSim(field, sensor));
      }
      cameras.add(own);
    }
    ExecutorService pool = Executors.newFixedThreadPool(threads);
    try {
      List<Future<?>> workers = new ArrayList<>();
      for (List<CameraImageSim> own : cameras) {
        workers.add(
            pool.submit(
                () -> {
                  for (int i = next.getAndIncrement();
                      i < samples.size();
                      i = next.getAndIncrement()) {
                    outcomes[i] = at(samples.get(i).pose(), own);
                    progress.accept(done.incrementAndGet());
                  }
                  return null;
                }));
      }
      for (Future<?> worker : workers) {
        worker.get();
      }
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IllegalStateException("interrupted while scoring", e);
    } catch (ExecutionException e) {
      throw new IllegalStateException("scoring failed: " + e.getCause(), e.getCause());
    } finally {
      pool.shutdownNow();
    }
    return new Result(candidates, samples, outcomes, config.targetMeters());
  }

  /** Every layout's outcomes at one pose: one per set of images. */
  private Outcome[][] at(Pose2d truth, List<CameraImageSim> images) {
    Outcome[][] outcomes = new Outcome[candidates.size()][config.frames()];
    Pose3d robotPose = new Pose3d(truth);
    long poseSeed =
        mix(config.seed(), truth.getX(), truth.getY(), truth.getRotation().getRadians());
    int[] tags = new int[mounts.size()];
    VisionMeasurement[] measurements = new VisionMeasurement[mounts.size()];
    for (int frame = 0; frame < config.frames(); frame++) {
      long frameSeed = mix(poseSeed, frame);
      // One gyro for the whole robot, off by a little, the same for every camera.
      double gyroError =
          Math.toRadians(config.gyroErrorDegrees())
              * new SplittableRandom(frameSeed).nextGaussian();
      CaptureContext standingStill =
          new CaptureContext(
              truth.getRotation().plus(Rotation2d.fromRadians(gyroError)),
              Math.toRadians(config.gyroErrorDegrees()),
              0,
              0,
              0,
              0);
      for (int m = 0; m < mounts.size(); m++) {
        CameraConfig camera = mounts.get(m);
        TagDetections seen =
            images
                .get(sensorOf[m])
                .image(
                    robotPose.transformBy(camera.robotToCamera()),
                    camera.name(),
                    mix(frameSeed, mountSeeds[m]));
        tags[m] = seen.tags().size();
        measurements[m] =
            seen.tags().isEmpty()
                ? null
                : pipeline.process(seen, camera, standingStill, robot.vision()).measurement();
      }
      for (int c = 0; c < candidates.size(); c++) {
        outcomes[c][frame] = combine(layouts[c], tags, measurements, truth);
      }
    }
    return outcomes;
  }

  /**
   * One layout's images, combined as the estimator would combine them from no prior belief: each
   * measurement weighed by the inverse of its uncertainty, which is where a Kalman filter's
   * updates, one camera after another, end up.
   */
  static Outcome combine(
      int[] cameras, int[] tags, @Nullable VisionMeasurement[] measurements, Pose2d truth) {
    int most = 0;
    int seeing = 0;
    double weightX = 0;
    double weightY = 0;
    double weightHeading = 0;
    double errorX = 0;
    double errorY = 0;
    double errorHeading = 0;
    for (int m : cameras) {
      most = Math.max(most, tags[m]);
      seeing += tags[m] > 0 ? 1 : 0;
      VisionMeasurement measurement = measurements[m];
      if (measurement == null) {
        continue;
      }
      PoseCovariance spread = measurement.covariance();
      Pose2d pose = measurement.pose();
      weightX += 1 / spread.xx();
      weightY += 1 / spread.yy();
      errorX += (pose.getX() - truth.getX()) / spread.xx();
      errorY += (pose.getY() - truth.getY()) / spread.yy();
      if (spread.hasHeading()) {
        weightHeading += 1 / spread.heading();
        errorHeading +=
            pose.getRotation().minus(truth.getRotation()).getRadians() / spread.heading();
      }
    }
    return new Outcome(
        most,
        seeing,
        weightX > 0 ? Math.hypot(errorX / weightX, errorY / weightY) : Double.NaN,
        weightHeading > 0 ? errorHeading / weightHeading : Double.NaN);
  }

  /** A seed from a seed and numbers, the same on every computer. */
  static long mix(long seed, double... values) {
    long mixed = seed;
    for (double value : values) {
      mixed = mix(mixed, Double.doubleToLongBits(value));
    }
    return mixed;
  }

  /** A seed from a seed and a number: SplitMix64's mixing, so nearby numbers seed far apart. */
  static long mix(long seed, long value) {
    long h = seed + value + 0x9E3779B97F4A7C15L;
    h = (h ^ (h >>> 30)) * 0xBF58476D1CE4E5B9L;
    h = (h ^ (h >>> 27)) * 0x94D049BB133111EBL;
    return h ^ (h >>> 31);
  }

  /**
   * What every layout's cameras saw at every sample.
   *
   * @param outcomes for each sample, each layout's outcomes there: one per set of images
   */
  public record Result(
      List<Candidate> candidates,
      List<Sample> samples,
      Outcome[][][] outcomes,
      double targetMeters) {
    /** Every layout's score, best first. */
    public List<Score> ranking() {
      List<Score> scores = new ArrayList<>();
      for (Candidate candidate : candidates) {
        scores.add(Score.of(candidate, samples, of(candidate), targetMeters));
      }
      scores.sort(Score.RANKING);
      return scores;
    }

    /** One layout's outcomes: for each sample, one per set of images. */
    public Outcome[][] of(Candidate candidate) {
      int c = candidates.indexOf(candidate);
      if (c < 0) {
        throw new IllegalArgumentException("not scored: " + candidate.name());
      }
      Outcome[][] its = new Outcome[samples.size()][];
      for (int i = 0; i < samples.size(); i++) {
        its[i] = outcomes[i][c];
      }
      return its;
    }
  }
}
