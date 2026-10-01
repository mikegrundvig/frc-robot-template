package frc.robot;

import com.michaelgrundvig.frc.world.bridges.RobotCatalog;
import com.michaelgrundvig.frc.world.bridges.RobotDescription;
import java.util.List;
import java.util.Optional;

/**
 * The robots in {@link Robots}, as the build's tool-config plugins hand them to the tool writers:
 * the AdvantageScope plugin (build.gradle's {@code advantageScope}) names this class, and its
 * writer builds each robot's configuration from it. Nothing else runs: no robot program, HAL, or
 * NetworkTables, so building a tool's files needs no simulator.
 */
public final class ToolRobots implements RobotCatalog {
  /** For the build, which makes one by name. */
  public ToolRobots() {}

  @Override
  public List<RobotDescription> all() {
    return Robots.all().stream().map(ToolRobots::describe).toList();
  }

  /**
   * A robot as the writers read it. The IMUs' mounts are known only along the floor (their forward
   * and left offsets) until a mount gives their height too, so none is drawn yet.
   */
  static RobotDescription describe(RobotConfig robot) {
    return new RobotDescription(
        robot.name(), robot.body(), robot.drivetrain(), robot.cameras(), Optional.empty());
  }
}
