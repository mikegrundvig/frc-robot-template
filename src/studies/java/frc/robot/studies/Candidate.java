package frc.robot.studies;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import frc.robot.RobotConfig;
import java.util.List;

/**
 * A layout of cameras to try on the robot, each placed and pointed as its {@link CameraConfig}
 * says.
 *
 * @param name what the report calls it: "4 cameras: corners, splayed 30°, 16 in up, tilted 10°"
 * @param cameras the cameras, each named uniquely in the layout
 */
public record Candidate(String name, List<CameraConfig> cameras) {
  public Candidate {
    Checks.on(Candidate.class, name)
        .field("name", name, notBlank())
        .field("cameras", cameras, list -> list.isEmpty() ? "must have a camera" : null)
        .orThrow();
    cameras = List.copyOf(cameras);
  }

  /**
   * Checks these cameras on the robot, as the robot's definition checks its own: a camera outside
   * the robot, or two with one name, is refused.
   *
   * @throws IllegalArgumentException naming this layout and every camera that doesn't fit
   */
  public void checkOn(RobotConfig robot) {
    try {
      RobotDefinitionBuilder.builder(robot.definition()).cameras(cameras).build();
    } catch (IllegalArgumentException e) {
      throw new IllegalArgumentException(name + ": " + e.getMessage(), e);
    }
  }
}
