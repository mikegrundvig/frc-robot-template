package frc.robot.studies;

import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;

import com.michaelgrundvig.frc.runtime.subsystems.vision.VisionSim;
import com.michaelgrundvig.frc.simulation.Seeds;
import com.michaelgrundvig.frc.world.config.Checks;
import java.util.Locale;
import java.util.SplittableRandom;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * How the cameras and the robot err in one scoring of the field: what isn't known until the robot
 * is built and its cameras measured. The nominal conditions are the study's settings, which start
 * from the simulator's own. A robustness run also draws many conditions from the plausible ranges
 * below, and ranks layouts by how they do in the worst of them (see
 * runtime/docs/camera-placement.md, which gives each range's reason).
 *
 * @param seed where this scoring's random draws start
 * @param cornerNoiseMeanPixels how far each tag corner lands from where a perfect camera would put
 *     it, on average, in pixels
 * @param cornerNoiseStdDevPixels and that distance's spread
 * @param sightMeters tags farther than this aren't detected
 * @param gyroErrorDegrees how far off the gyro's heading typically is (one standard deviation)
 * @param mountErrorMillimeters how far each camera really sits from where it's configured, at most,
 *     along each axis: each camera's offset along each axis is drawn evenly within plus or minus
 *     this, fixed for these conditions, the same over the whole field and in every layout the
 *     camera is part of, as a mount's error is
 * @param mountErrorDegrees how far each camera is really turned from how it's configured, at most,
 *     about each axis
 */
public record Conditions(
    long seed,
    double cornerNoiseMeanPixels,
    double cornerNoiseStdDevPixels,
    double sightMeters,
    double gyroErrorDegrees,
    double mountErrorMillimeters,
    double mountErrorDegrees) {
  /** A range a robustness run draws a value from, evenly. */
  record Range(double low, double high) {
    double draw(SplittableRandom random) {
      return low + (high - low) * random.nextDouble();
    }

    @Override
    public String toString() {
      return String.format(Locale.ROOT, "%s to %s", shown(low), shown(high));
    }
  }

  // The plausible ranges, each explained in runtime/docs/camera-placement.md ("Robustness").
  static final Range CORNER_NOISE_PIXELS = new Range(0.1, 0.6);
  static final Range SIGHT_METERS = new Range(5, 7);
  static final Range GYRO_ERROR_DEGREES = new Range(0.1, 0.5);
  static final Range MOUNT_ERROR_MILLIMETERS = new Range(0, 5);
  static final Range MOUNT_ERROR_DEGREES = new Range(0, 1);

  public Conditions {
    Checks.on(Conditions.class)
        .field("cornerNoiseMeanPixels", cornerNoiseMeanPixels, nonNegative())
        .field("cornerNoiseStdDevPixels", cornerNoiseStdDevPixels, nonNegative())
        .field("sightMeters", sightMeters, positive())
        .field("gyroErrorDegrees", gyroErrorDegrees, nonNegative())
        .field("mountErrorMillimeters", mountErrorMillimeters, nonNegative())
        .field("mountErrorDegrees", mountErrorDegrees, nonNegative())
        .orThrow();
  }

  /**
   * The study's settings: by default, the simulator's own errors, and cameras mounted exactly.
   *
   * @param config the study's settings
   */
  static Conditions nominal(CameraStudyConfig config) {
    return new Conditions(
        config.seed(),
        config.cornerNoiseMeanPixels(),
        config.cornerNoiseStdDevPixels(),
        config.sightMeters(),
        config.gyroErrorDegrees(),
        config.mountErrorMillimeters(),
        config.mountErrorDegrees());
  }

  /**
   * Where the trials' seeds come from, apart from everything else the run's seed starts: so another
   * run's seed gives other trials, never the same ones shifted along.
   */
  private static final long TRIALS = 0x747269616c73L; // "trials"

  /**
   * One robustness trial's conditions, each drawn from its range. The corner noise's spread keeps
   * the simulator's proportion to its mean.
   *
   * @param config the study's settings, whose seed every trial's draws start from
   * @param trial which trial, from 0: each has its own draws, and its own seed for the images
   */
  static Conditions trial(CameraStudyConfig config, int trial) {
    long seed = Seeds.mix(Seeds.mix(config.seed(), TRIALS), trial);
    SplittableRandom random = new SplittableRandom(seed);
    double corners = CORNER_NOISE_PIXELS.draw(random);
    return new Conditions(
        seed,
        corners,
        corners * VisionSim.CORNER_NOISE_STD_DEV_PIXELS / VisionSim.CORNER_NOISE_MEAN_PIXELS,
        SIGHT_METERS.draw(random),
        GYRO_ERROR_DEGREES.draw(random),
        MOUNT_ERROR_MILLIMETERS.draw(random),
        MOUNT_ERROR_DEGREES.draw(random));
  }

  /**
   * How one camera is really mounted off its configured mount, in these conditions: a little along
   * and about each axis, drawn for the camera, so it's the same at every pose, as a real mount's
   * error is. In the robot's frame, as {@code SensorErrors} gives the simulator's.
   *
   * @param camera a number for the camera, the same whatever else is scored
   */
  Transform3d mountError(long camera) {
    return mountError(seed, camera, mountErrorMillimeters, mountErrorDegrees);
  }

  /**
   * How one camera is mounted off its configured mount, drawn evenly within these bounds.
   *
   * @param seed where the draw starts: a run's, already mixed with something of its own, since
   *     {@link Seeds#mix} of nearby seeds and camera numbers coincide
   * @param camera a number for the camera, the same whatever else is scored
   * @param millimeters the most it sits off along each axis, either way
   * @param degrees the most it is turned about each axis, either way
   */
  static Transform3d mountError(long seed, long camera, double millimeters, double degrees) {
    if (millimeters == 0 && degrees == 0) {
      return Transform3d.ZERO;
    }
    SplittableRandom random = new SplittableRandom(Seeds.mix(seed, camera));
    double meters = millimeters / 1000;
    double radians = Math.toRadians(degrees);
    return new Transform3d(
        within(random, meters),
        within(random, meters),
        within(random, meters),
        new Rotation3d(within(random, radians), within(random, radians), within(random, radians)));
  }

  private static double within(SplittableRandom random, double bound) {
    return bound * (2 * random.nextDouble() - 1);
  }

  /** The conditions in words, for the report. */
  String described() {
    return String.format(
        Locale.ROOT,
        "corners off %s px (spread %s), tags seen to %s m, the gyro off %s° typically, cameras"
            + " mounted %s",
        shown(cornerNoiseMeanPixels),
        shown(cornerNoiseStdDevPixels),
        shown(sightMeters),
        shown(gyroErrorDegrees),
        mountErrorMillimeters == 0 && mountErrorDegrees == 0
            ? "exactly as configured"
            : "off by up to "
                + shown(mountErrorMillimeters)
                + " mm and "
                + shown(mountErrorDegrees)
                + "°");
  }

  private static String shown(double value) {
    return String.format(Locale.ROOT, "%.2f", value).replaceAll("\\.?0+$", "");
  }
}
