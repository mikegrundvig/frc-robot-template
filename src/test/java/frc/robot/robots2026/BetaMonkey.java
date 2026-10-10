package frc.robot.robots2026;

import frc.robot.BetaControls;
import frc.robot.BetaRobot;
import frc.robot.Monkey;

/**
 * A team's 2026 robot, Beta, with their controls, for the monkey to drive ({@code MonkeySimTest}).
 */
public final class BetaMonkey {
  private BetaMonkey() {}

  public static final Monkey.Robot ROBOT =
      Monkey.robot(Parts.withoutCameras(BetaRobot.CONFIG), BetaControls::new);
}
