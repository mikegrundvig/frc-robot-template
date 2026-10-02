package frc.robot;

import static org.assertj.core.api.Assertions.assertThatCode;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.MetersPerSecond;

import com.michaelgrundvig.frc.pitwall.ConfigRules;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.junit.jupiter.api.Test;

/** Every robot's committed configuration keeps the rules across its mechanisms. */
class RobotRulesTest {
  @Test
  void everyRobotKeepsItsRulesAsCommitted() {
    for (RobotConfig robot : Robots.all()) {
      ConfigRules rules = new ConfigRules();
      RobotRules.register(
          rules,
          robot.definition(),
          Drive.tuning(robot.drivetrain()),
          Vision.tuning(robot.vision()));

      assertThatCode(rules::checkCommitted).as(robot.name()).doesNotThrowAnyException();
    }
  }

  /** What a committed configuration that breaks rules looks like: every rule it breaks, at once. */
  @Test
  void aCommittedConfigurationThatBreaksRulesFailsWithEveryOne() {
    var drivetrain = Drive.tuning(Robots.SIM.drivetrain());
    var vision = Vision.tuning(Robots.SIM.vision());
    ConfigRules rules = new ConfigRules();
    RobotRules.register(rules, Robots.SIM.definition(), drivetrain, vision);
    rules
        .when(drivetrain, "limits.maxLinearSpeed")
        .check(
            "a crawl",
            d -> d.limits().maxLinearSpeed().in(MetersPerSecond) <= 1 ? null : "faster than 1 m/s");
    rules
        .when(drivetrain, "gains.steerKp")
        .and(vision, "maxAmbiguity")
        .check("an unlikely pair", (d, v) -> "the steering gain and the ambiguity disagree");

    assertThatIllegalArgumentException()
        .isThrownBy(rules::checkCommitted)
        .withMessage(
            "a crawl: faster than 1 m/s\n"
                + "an unlikely pair: the steering gain and the ambiguity disagree");
  }
}
