package frc.robot.studies;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotConfigBuilder;
import com.michaelgrundvig.frc.runtime.subsystems.vision.VisionSim;
import com.michaelgrundvig.frc.simulation.Seeds;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import com.michaelgrundvig.frc.world.robot.CameraConfigSensorBuilder;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Transform3d;

/**
 * How the estimator study's simulated cameras err beyond what a scenario already does to them: the
 * simulator's own, kind errors by default ({@link #NOMINAL}), or the pessimistic end of the ranges
 * the camera placement study's robustness runs draw from ({@link #PESSIMISTIC}, {@code
 * runtime/docs/camera-placement.md}), to see whether a verdict hangs on the simulator's kindness.
 *
 * <p>The estimators are told the configured mounts, as on a real robot. A mount's error is drawn
 * once for a seed and kept for the whole run, so the robot's own estimator and every other see the
 * same images. It adds to the mount error a scenario gives the robot, and a camera knocked or
 * turned during a run moves from there.
 *
 * @param name how the study is asked for it
 * @param cornerNoise how far the tag corners land from a perfect camera's; null for the simulator's
 *     own, or whatever the scenario sets
 * @param sightMeters tags farther than this aren't detected, unless a camera's configuration says
 *     less; infinite to leave each camera's configuration be
 * @param mountMillimeters how far each camera sits off its configured mount, at most, along each
 *     axis: drawn evenly within plus or minus this
 * @param mountDegrees and how far it is turned off, at most, about each axis
 */
record CameraConditions(
    String name,
    SensorErrors.@Nullable CornerNoise cornerNoise,
    double sightMeters,
    double mountMillimeters,
    double mountDegrees) {
  /** Cameras as the scenarios build them: the simulator's own noise and sight, mounted as set. */
  static final CameraConditions NOMINAL =
      new CameraConditions("nominal", null, Double.POSITIVE_INFINITY, 0, 0);

  /**
   * The worst of each range the camera placement study draws from: corners 0.6 px off on average
   * (the simulator's spread in proportion), tags seen to 5 m, and each camera off its mount by up
   * to 5 mm along and 1° about each axis.
   */
  static final CameraConditions PESSIMISTIC =
      new CameraConditions(
          "pessimistic",
          SensorErrors.CornerNoise.builder()
              .meanPixels(Conditions.CORNER_NOISE_PIXELS.high())
              .stdDevPixels(
                  Conditions.CORNER_NOISE_PIXELS.high()
                      * VisionSim.CORNER_NOISE_STD_DEV_PIXELS
                      / VisionSim.CORNER_NOISE_MEAN_PIXELS)
              .build(),
          Conditions.SIGHT_METERS.low(),
          Conditions.MOUNT_ERROR_MILLIMETERS.high(),
          Conditions.MOUNT_ERROR_DEGREES.high());

  private static final List<CameraConditions> ALL = List.of(NOMINAL, PESSIMISTIC);

  /** Keeps a seed's mount draws apart from every other draw the study makes from the seed. */
  private static final long MOUNT_DRAWS = 0x63616d6d6f756e74L; // "cammount"

  /**
   * The conditions with this name.
   *
   * @throws IllegalArgumentException naming every choice, when none has this name
   */
  static CameraConditions named(String name) {
    return ALL.stream()
        .filter(each -> each.name.equals(name))
        .findFirst()
        .orElseThrow(
            () ->
                new IllegalArgumentException(
                    "--cameras must be "
                        + String.join(" or ", ALL.stream().map(CameraConditions::name).toList())
                        + ": "
                        + name));
  }

  /** The robot whose cameras see no farther than these conditions say. */
  RobotConfig robot() {
    RobotConfig robot = ExampleRobot.SIM;
    if (Double.isInfinite(sightMeters)) {
      return robot;
    }
    List<CameraConfig> cameras = new ArrayList<>();
    for (CameraConfig camera : robot.definition().cameras()) {
      CameraConfig.Sensor sensor = camera.sensor();
      double sight = Math.min(sightMeters, sensor.sightRange().in(Meters));
      cameras.add(
          CameraConfigBuilder.builder(camera)
              .sensor(
                  CameraConfigSensorBuilder.builder(sensor).sightRange(Meters.of(sight)).build())
              .build());
    }
    return RobotConfigBuilder.builder(robot)
        .definition(RobotDefinitionBuilder.builder(robot.definition()).cameras(cameras).build())
        .build();
  }

  /**
   * The errors with these conditions' on top: the corners as set, and each camera's mount off its
   * configured mount by a fixed draw for the seed, added to the mount error it already had.
   *
   * @param errors how the scenario builds the robot for the seed
   * @param seed the run's seed: the same draws for every estimator, whatever the scenario
   */
  SensorErrors on(SensorErrors errors, long seed) {
    SensorErrors result = cornerNoise == null ? errors : errors.withCornerNoise(cornerNoise);
    if (mountMillimeters == 0 && mountDegrees == 0) {
      return result;
    }
    List<CameraConfig> cameras = ExampleRobot.SIM.definition().cameras();
    for (int i = 0; i < cameras.size(); i++) {
      String camera = cameras.get(i).name();
      Transform3d drawn =
          Conditions.mountError(Seeds.mix(seed, MOUNT_DRAWS), i, mountMillimeters, mountDegrees);
      result = result.withCameraMountError(camera, add(errors.cameraMountError(camera), drawn));
    }
    return result;
  }

  /** Two mount errors in a row, as {@code VisionSim.mounted} puts one on a configured mount. */
  private static Transform3d add(Transform3d scenarios, Transform3d drawn) {
    return new Transform3d(
        scenarios.getTranslation().plus(drawn.getTranslation()),
        scenarios.getRotation().rotateBy(drawn.getRotation()));
  }
}
