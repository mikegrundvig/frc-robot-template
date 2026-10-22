package frc.robot;

import com.michaelgrundvig.frc.runtime.RobotProgram;
import com.michaelgrundvig.frc.world.game.Rebuilt;

/** Where the robot program starts. */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotProgram.start(
        () -> ChassisRobot.CONFIG, // which robot this is
        Rebuilt.GAME, // the field it drives on
        Controls::new); // what the sticks and buttons do
  }
}
