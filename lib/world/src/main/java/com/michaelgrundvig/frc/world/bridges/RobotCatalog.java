package com.michaelgrundvig.frc.world.bridges;

import java.util.List;

/**
 * The team's robots, as the tool writers see them. The robot program implements it over its own
 * list of robots, and names that class to the build's tool-config plugins, which run a writer's
 * {@code main} on the compiled code: the writers live in the library, and the library never sees
 * the robot program. An implementation needs a public constructor that takes nothing.
 */
public interface RobotCatalog {
  /**
   * Every robot the team has: what a tool's files carry unless the build names some. Not the robot
   * this program would run as, which a robot decides by what it reads of itself (a serial number, a
   * file), and a build on a laptop can't.
   */
  List<RobotDescription> all();

  /**
   * A robot by its name.
   *
   * @throws IllegalArgumentException if no robot has that name, naming the robots there are
   */
  default RobotDescription named(String name) {
    List<RobotDescription> robots = all();
    for (RobotDescription robot : robots) {
      if (robot.name().equals(name)) {
        return robot;
      }
    }
    throw new IllegalArgumentException(
        "no robot named "
            + name
            + "; the robots are "
            + String.join(", ", robots.stream().map(RobotDescription::name).toList()));
  }
}
