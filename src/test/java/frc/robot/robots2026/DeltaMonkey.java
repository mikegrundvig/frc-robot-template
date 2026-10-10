package frc.robot.robots2026;

import frc.robot.DeltaControls;
import frc.robot.DeltaRobot;
import frc.robot.Monkey;

/**
 * A team's 2026 robot, Delta, with its controls, for the monkey to drive ({@code MonkeySimTest}).
 */
public final class DeltaMonkey {
  private DeltaMonkey() {}

  public static final Monkey.Robot ROBOT =
      Monkey.robot(Parts.withoutCameras(DeltaRobot.CONFIG), DeltaControls::new);
}
