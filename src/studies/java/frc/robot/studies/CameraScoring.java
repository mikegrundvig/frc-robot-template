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
import frc.robot.subsystems.vision.VisionSim;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.SplittableRandom;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.IntConsumer;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;

/**
 * Scores camera layouts all over the field. At each pose, each camera takes its images (with
 * PhotonVision's simulation, {@link CameraImageSim}), and the robot's own vision pipeline turns
 * each into a measurement or throws it out. Each layout's measurements are then combined as the
 * estimator combines them, and what its cameras saw, and how close the combination came to the
 * truth, is its {@link Outcome} there.
 *
 * <p>A camera that several layouts share (the same mount and sensor) takes its images once, and its
 * noise depends only on the seed, the pose, and the mount, so a layout scores the same whatever
 * else is tried beside it, and on any number of threads. What each camera saw is kept, not each
 * layout's outcome, so thousands of layouts made of a few hundred cameras fit in memory.
 */
public final class CameraScoring {
  private final RobotConfig robot;
  private final FieldModel field;
  private final List<Candidate> candidates;
  private final CameraStudyConfig config;
  private final VisionPipeline pipeline;

  /** Every distinct camera, over every layout: where it is, which way it looks, and its sensor. */
  private final List<CameraConfig> cameras = new ArrayList<>();

  /** Each layout's cameras, as their places in {@link #cameras}. */
  private final int[][] layouts;

  /** The distinct sensors, and each camera's among them. */
  private final List<CameraConfig.Sensor> sensors = new ArrayList<>();

  private final int[] sensorOf;
  private final long[] cameraSeeds;

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
    List<Long> seeds = new ArrayList<>();
    layouts = new int[candidates.size()][];
    for (int c = 0; c < candidates.size(); c++) {
      Candidate candidate = candidates.get(c);
      candidate.checkOn(robot);
      layouts[c] = new int[candidate.cameras().size()];
      for (int i = 0; i < layouts[c].length; i++) {
        CameraConfig camera = candidate.cameras().get(i);
        Mount mount = Mount.of(camera);
        Integer at = index.get(mount);
        if (at == null) {
          at = cameras.size();
          index.put(mount, at);
          cameras.add(camera);
          seeds.add(mount.seed());
        }
        layouts[c][i] = at;
      }
    }
    sensorOf = new int[cameras.size()];
    cameraSeeds = new long[cameras.size()];
    for (int m = 0; m < cameras.size(); m++) {
      CameraConfig.Sensor sensor = cameras.get(m).sensor();
      if (!sensors.contains(sensor)) {
        sensors.add(sensor);
      }
      sensorOf[m] = sensors.indexOf(sensor);
      cameraSeeds[m] = seeds.get(m);
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
      return Seeds.mix(0, forward, left, up, roll, tiltUp, yaw);
    }
  }

  /** Where the other robots' places come from, apart from the images' noise. */
  private static final long OTHER_ROBOTS = 0x6f7468657273L; // "others"

  /** How many distinct cameras the layouts have among them: each takes its images once. */
  public int distinctCameras() {
    return cameras.size();
  }

  /**
   * Every camera's images at every sample, then every layout's score from them.
   *
   * @param conditions how the cameras and the robot err, this time
   * @param progress told how many samples are done, after each one
   */
  public Result score(List<Sample> samples, Conditions conditions, IntConsumer progress) {
    Views[] views = new Views[samples.size()];
    // Where each camera really is, off its configured mount as these conditions have it.
    Transform3d[] mounts = new Transform3d[cameras.size()];
    for (int m = 0; m < cameras.size(); m++) {
      mounts[m] =
          VisionSim.mounted(cameras.get(m).robotToCamera(), conditions.mountError(cameraSeeds[m]));
    }
    AtomicInteger done = new AtomicInteger();
    int threads = Math.min(config.threads(), Math.max(1, samples.size()));
    inParallel(
        samples.size(),
        threads,
        () -> {
          // Each thread its own cameras: their noise isn't shared.
          List<CameraImageSim> images = new ArrayList<>();
          for (CameraConfig.Sensor sensor : sensors) {
            images.add(new CameraImageSim(field, sensor, conditions));
          }
          return i -> {
            views[i] = at(samples.get(i).pose(), images, conditions, mounts);
            progress.accept(done.incrementAndGet());
          };
        });
    Score[] scores = new Score[candidates.size()];
    // Each layout less one camera, scored once however many layouts share it.
    Map<List<Integer>, Score> lessOne = new ConcurrentHashMap<>();
    inParallel(
        candidates.size(),
        threads,
        () ->
            c -> {
              Score score = Score.of(candidates.get(c), samples, outcomes(views, layouts[c]));
              scores[c] = oneCameraLost(score, layouts[c], samples, views, lessOne);
            });
    return new Result(candidates, layouts, samples, views, List.of(scores));
  }

  /**
   * A layout's score, with how it does when it loses any one camera: the worst of its 95% errors
   * without each camera in turn, and the least of the field it still measures. A lone camera lost
   * leaves nothing.
   */
  private Score oneCameraLost(
      Score score,
      int[] layout,
      List<Sample> samples,
      Views[] views,
      Map<List<Integer>, Score> lessOne) {
    if (layout.length == 1) {
      return score.withOneCameraLost(Double.POSITIVE_INFINITY, 0);
    }
    double worst = 0;
    double least = 1;
    for (int lost = 0; lost < layout.length; lost++) {
      List<Integer> rest = new ArrayList<>();
      for (int i = 0; i < layout.length; i++) {
        if (i != lost) {
          rest.add(layout[i]);
        }
      }
      rest.sort(Comparator.naturalOrder());
      Score without =
          lessOne.computeIfAbsent(
              rest,
              cameras -> {
                List<CameraConfig> kept = cameras.stream().map(this.cameras::get).toList();
                int[] places = cameras.stream().mapToInt(Integer::intValue).toArray();
                return Score.of(
                    new Candidate("less one camera", kept), samples, outcomes(views, places));
              });
      worst = Math.max(worst, without.positionP95Meters());
      least = Math.min(least, without.measured());
    }
    return score.withOneCameraLost(worst, least);
  }

  /**
   * Runs work on every index from 0 to count, on several threads. Each thread's work is made here,
   * one thread at a time, so what it holds is its own; which thread does an index doesn't matter.
   */
  private static void inParallel(int count, int threads, Supplier<IntConsumer> work) {
    AtomicInteger next = new AtomicInteger();
    List<IntConsumer> each = new ArrayList<>();
    for (int w = 0; w < threads; w++) {
      each.add(work.get());
    }
    ExecutorService pool = Executors.newFixedThreadPool(threads);
    try {
      List<Future<?>> workers = new ArrayList<>();
      for (IntConsumer own : each) {
        workers.add(
            pool.submit(
                () -> {
                  for (int i = next.getAndIncrement(); i < count; i = next.getAndIncrement()) {
                    own.accept(i);
                  }
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
  }

  /**
   * What every camera saw at one pose, in each set of images: each camera where it really is, and
   * the pipeline solving with where it's configured, as on a robot.
   */
  private Views at(
      Pose2d truth, List<CameraImageSim> images, Conditions conditions, Transform3d[] mounts) {
    Views views = new Views(config.frames(), cameras.size());
    Pose3d robotPose = new Pose3d(truth);
    long poseSeed =
        Seeds.mix(conditions.seed(), truth.getX(), truth.getY(), truth.getRotation().getRadians());
    for (int frame = 0; frame < config.frames(); frame++) {
      long frameSeed = Seeds.mix(poseSeed, frame);
      // Other robots in the way, placed anew for each set of images, the same for every camera.
      SplittableRandom placing = new SplittableRandom(Seeds.mix(frameSeed, OTHER_ROBOTS));
      int others =
          config.otherRobotsLeast()
              + placing.nextInt(config.otherRobotsMost() - config.otherRobotsLeast() + 1);
      List<OtherRobots.Blocker> blockers =
          OtherRobots.around(truth, field, robot.body(), others, placing);
      // One gyro for the whole robot, off by a little, the same for every camera.
      double gyroError =
          Math.toRadians(conditions.gyroErrorDegrees())
              * new SplittableRandom(frameSeed).nextGaussian();
      CaptureContext standingStill =
          new CaptureContext(
              truth.getRotation().plus(Rotation2d.fromRadians(gyroError)),
              Math.toRadians(conditions.gyroErrorDegrees()),
              0,
              0,
              0,
              0);
      for (int m = 0; m < cameras.size(); m++) {
        CameraConfig camera = cameras.get(m);
        TagDetections seen =
            images
                .get(sensorOf[m])
                .image(
                    robotPose.transformBy(mounts[m]),
                    camera.name(),
                    Seeds.mix(frameSeed, cameraSeeds[m]),
                    blockers);
        VisionMeasurement measurement =
            seen.tags().isEmpty()
                ? null
                : pipeline
                    .process(seen, camera, standingStill, robot.vision(), VisionPipeline.Trust.FULL)
                    .measurement();
        views.set(frame, m, seen.tags().size(), measurement, truth);
      }
    }
    return views;
  }

  /** A layout's outcomes, from what its cameras saw: for each sample, one per set of images. */
  private static Outcome[][] outcomes(Views[] views, int[] layout) {
    Outcome[][] outcomes = new Outcome[views.length][];
    for (int i = 0; i < views.length; i++) {
      outcomes[i] = new Outcome[views[i].frames];
      for (int frame = 0; frame < views[i].frames; frame++) {
        outcomes[i][frame] = views[i].combine(frame, layout);
      }
    }
    return outcomes;
  }

  /**
   * One set of images from some cameras, combined. The measurements, each with its own uncertainty,
   * as the cameras that took them would give them.
   *
   * @param cameras which cameras the layout has, as places in the arrays
   * @param tags how many tags each camera saw
   * @param measurements each camera's measurement, or null if its image made none
   */
  static Outcome combine(
      int[] cameras, int[] tags, @Nullable VisionMeasurement[] measurements, Pose2d truth) {
    Views views = new Views(1, tags.length);
    for (int m = 0; m < tags.length; m++) {
      views.set(0, m, tags[m], measurements[m], truth);
    }
    return views.combine(0, cameras);
  }

  /**
   * What every camera saw in each set of images at one pose: how many tags, and how far its
   * measurement was from the truth, with the measurement's own uncertainty. Plain numbers, so a few
   * thousand poses of a few hundred cameras fit in memory.
   */
  private static final class Views {
    final int frames;
    final int cameras;
    final int[] tags;
    // Each measurement's miss along x and y and in heading (NaN for no measurement), and its
    // covariance: xx, xy, yy, and the heading's variance (infinite when it says nothing).
    final double[] missX;
    final double[] missY;
    final double[] missHeading;
    final double[] xx;
    final double[] xy;
    final double[] yy;
    final double[] headingVariance;

    Views(int frames, int cameras) {
      this.frames = frames;
      this.cameras = cameras;
      int size = frames * cameras;
      tags = new int[size];
      missX = new double[size];
      missY = new double[size];
      missHeading = new double[size];
      xx = new double[size];
      xy = new double[size];
      yy = new double[size];
      headingVariance = new double[size];
    }

    void set(
        int frame, int camera, int seen, @Nullable VisionMeasurement measurement, Pose2d truth) {
      int at = frame * cameras + camera;
      tags[at] = seen;
      if (measurement == null) {
        missX[at] = Double.NaN;
        return;
      }
      Pose2d pose = measurement.pose();
      PoseCovariance spread = measurement.covariance();
      missX[at] = pose.getX() - truth.getX();
      missY[at] = pose.getY() - truth.getY();
      missHeading[at] = pose.getRotation().minus(truth.getRotation()).getRadians();
      xx[at] = spread.xx();
      xy[at] = spread.xy();
      yy[at] = spread.yy();
      headingVariance[at] = spread.heading();
    }

    /**
     * One layout's images, combined as the estimator would combine them from no prior belief: in
     * information form, each measurement weighed by the inverse of its whole covariance (x and y
     * together, as the estimator's filter does), which is where a Kalman filter's updates, one
     * camera after another, end up. Heading is weighed on its own, as the estimator keeps it.
     */
    Outcome combine(int frame, int[] layout) {
      int most = 0;
      int seeing = 0;
      // The summed information (the inverse covariance, a 2 x 2 [a b; b d]), and information
      // times miss.
      double a = 0;
      double b = 0;
      double d = 0;
      double ax = 0;
      double ay = 0;
      double headingWeight = 0;
      double headingSum = 0;
      for (int camera : layout) {
        int at = frame * cameras + camera;
        most = Math.max(most, tags[at]);
        seeing += tags[at] > 0 ? 1 : 0;
        double determinant = xx[at] * yy[at] - xy[at] * xy[at];
        if (Double.isNaN(missX[at]) || !(determinant > 0)) {
          continue;
        }
        double ia = yy[at] / determinant;
        double ib = -xy[at] / determinant;
        double id = xx[at] / determinant;
        a += ia;
        b += ib;
        d += id;
        ax += ia * missX[at] + ib * missY[at];
        ay += ib * missX[at] + id * missY[at];
        if (Double.isFinite(headingVariance[at]) && headingVariance[at] > 0) {
          headingWeight += 1 / headingVariance[at];
          headingSum += missHeading[at] / headingVariance[at];
        }
      }
      double determinant = a * d - b * b;
      double position = Double.NaN;
      if (determinant > 0) {
        // The combined miss: the summed information's inverse times the summed information-miss.
        double x = (d * ax - b * ay) / determinant;
        double y = (a * ay - b * ax) / determinant;
        position = Math.hypot(x, y);
      }
      return new Outcome(
          most, seeing, position, headingWeight > 0 ? headingSum / headingWeight : Double.NaN);
    }
  }

  /** What every camera saw at every sample, and every layout's score. */
  public static final class Result {
    private final List<Candidate> candidates;
    private final int[][] layouts;
    private final List<Sample> samples;
    private final Views[] views;
    private final List<Score> ranking;

    private Result(
        List<Candidate> candidates,
        int[][] layouts,
        List<Sample> samples,
        Views[] views,
        List<Score> scores) {
      this.candidates = candidates;
      this.layouts = layouts;
      this.samples = List.copyOf(samples);
      this.views = views;
      List<Score> ranked = new ArrayList<>(scores);
      ranked.sort(Score.RANKING);
      ranking = List.copyOf(ranked);
    }

    /** Every layout's score, best first. */
    public List<Score> ranking() {
      return ranking;
    }

    public List<Sample> samples() {
      return samples;
    }

    /** One layout's outcomes: for each sample, one per set of images. */
    public Outcome[][] outcomes(Candidate candidate) {
      int c = candidates.indexOf(candidate);
      if (c < 0) {
        throw new IllegalArgumentException("not scored: " + candidate.name());
      }
      return CameraScoring.outcomes(views, layouts[c]);
    }
  }
}
