package frc.robot.robots2026;

import com.michaelgrundvig.frc.runtime.Monkey;
import frc.robot.BetaControls;
import frc.robot.BetaRobot;

/**
 * A team's 2026 robot, Beta, with their controls, for the monkey to drive ({@code MonkeySimTest}).
 */
public final class BetaMonkey {
  private BetaMonkey() {}

  public static final Monkey.Robot ROBOT =
      Monkey.robot(Parts.withoutCameras(BetaRobot.CONFIG), BetaControls::new);
}
