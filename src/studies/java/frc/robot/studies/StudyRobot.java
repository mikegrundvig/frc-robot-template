package frc.robot.studies;

import com.michaelgrundvig.frc.localization.WpilibLocalizer;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotConfigBuilder;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Which robot the estimator study runs as A: the software its cameras run, and the localizer it
 * chose. The study's robot is the example robot on PhotonVision with the default localizer unless
 * asked otherwise.
 *
 * <p>Visor sends each tag of an image with its own pose and never a solution from all of them at
 * once, as PhotonVision's multi-tag one is: on Visor, no image has a heading but a single tag's.
 * WPILib's estimator reports no uncertainty, so a robot that chose it has every image solved from
 * its tags alone, never with the gyro's heading (runtime/docs/localizers.md).
 *
 * @param software what every camera runs
 * @param localizer what the robot works out where it is with
 */
record StudyRobot(Software software, Choice localizer) {
  /** What every camera runs. */
  enum Software {
    /** PhotonVision, which sends a multi-tag solution whenever an image has two tags or more. */
    PHOTONVISION,
    /** Visor, which sends each tag with its own pose and no multi-tag solution. */
    VISOR
  }

  /** The localizer the robot chose. */
  enum Choice {
    /** The library's own estimator, which a robot gets unless it chooses. */
    DEFAULT,
    /** WPILib's pose estimator, as a robot chooses it in one line. */
    WPILIB
  }

  /** The example robot as it is built: PhotonVision cameras and the default localizer. */
  static final StudyRobot AS_BUILT = new StudyRobot(Software.PHOTONVISION, Choice.DEFAULT);

  /**
   * The robot with this software and this localizer, by the study's option values.
   *
   * @throws IllegalArgumentException naming the choices, for a value that isn't one
   */
  static StudyRobot named(String software, String localizer) {
    return new StudyRobot(
        choice(Software.class, "--software", software),
        choice(Choice.class, "--localizer", localizer));
  }

  private static <E extends Enum<E>> E choice(Class<E> type, String option, String value) {
    List<String> names = new ArrayList<>();
    for (E each : type.getEnumConstants()) {
      String name = each.name().toLowerCase(Locale.ROOT);
      if (name.equals(value)) {
        return each;
      }
      names.add(name);
    }
    throw new IllegalArgumentException(
        option + " must be " + String.join(" or ", names) + ": " + value);
  }

  /** A robot's configuration, its cameras on this software and its localizer this one. */
  RobotConfig on(RobotConfig robot) {
    RobotConfigBuilder built = RobotConfigBuilder.builder(robot);
    if (software == Software.VISOR) {
      List<CameraConfig> cameras = new ArrayList<>();
      for (CameraConfig camera : robot.definition().cameras()) {
        cameras.add(camera.withSoftware(CameraConfig.Software.VISOR));
      }
      built.definition(RobotDefinitionBuilder.builder(robot.definition()).cameras(cameras).build());
    }
    if (localizer == Choice.WPILIB) {
      built.localizer(WpilibLocalizer::new);
    }
    return built.build();
  }

  /** How the report names it: "PhotonVision, the default localizer". */
  String describe() {
    String cameras = software == Software.VISOR ? "Visor" : "PhotonVision";
    String chosen =
        localizer == Choice.WPILIB ? "WPILib's pose estimator" : "the default localizer";
    return cameras + " cameras, " + chosen;
  }
}
