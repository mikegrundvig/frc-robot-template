package frc.robot;

import com.michaelgrundvig.frc.pitwall.ConfigRules;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.localization.VisionConfig;

/**
 * The rules across this robot's mechanisms: checks that read more than one mechanism's
 * configuration, so no one record can make them. Two arms whose swings could collide are the usual
 * one; each arm's range is fine on its own, but not both reaching into the same space.
 *
 * <p>A rule names the values it reads, and runs whenever the pit changes one of them, against the
 * configurations as the change would leave them, so changing both arms together is one valid
 * change. The committed configuration is checked against every rule at startup, and in the tests
 * for every robot ({@code RobotRulesTest}), so a colliding configuration never reaches a robot.
 *
 * <p>The template's robot has none: its drivetrain and vision never constrain each other. Pitwall's
 * {@code ConfigRulesTest} works one through, an intake and a hood; a rule reads like this:
 *
 * <pre>{@code
 * rules.when(intake, "swing.from", "swing.to")
 *     .and(hood, "swing.from", "swing.to")
 *     .check("the intake and the hood", (i, h) -> ...); // why they collide, or null
 * }</pre>
 */
final class RobotRules {
  private RobotRules() {}

  /**
   * Registers this robot design's rules over its mechanisms' live configurations. A robot without a
   * mechanism registers none of the rules that read it.
   */
  static void register(
      ConfigRules rules, LiveConfig<SwerveConfig> drivetrain, LiveConfig<VisionConfig> vision) {
    // None yet: see above.
  }
}
