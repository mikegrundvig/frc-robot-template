package frc.robot;

import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.noClasses;

import com.michaelgrundvig.frc.runtime.ProductionRules;
import com.tngtech.archunit.core.domain.JavaClasses;
import org.junit.jupiter.api.Test;

/**
 * The team's architecture rules, checked on every build: the rules that hold for every class of a
 * robot program ({@link ProductionRules}, shared with the runtime's own architecture test), on the
 * team's classes, and what keeps the team's code and the runtime apart. Each rule prevents a kind
 * of bug that robot code otherwise accumulates quietly until it bites at an event.
 */
class ArchitectureTest {
  private static final JavaClasses TEAM_CODE = ProductionCode.TEAM;

  /** The runtime is given what it needs from the team's code and never names any of it. */
  @Test
  void theRuntimeNeverReachesTheTeamsCode() {
    noClasses()
        .that()
        .resideInAPackage("com.michaelgrundvig.frc.runtime..")
        .should()
        .dependOnClassesThat()
        .resideInAPackage("frc.robot..")
        .because(
            "a runtime that names a team's class can't be built, tested or shared without that"
                + " team's robot: hand it what it needs, as RobotProgram.start does")
        .check(ProductionCode.CLASSES);
  }

  @Test
  void theAllianceIsReadWhenItsNeeded() {
    ProductionRules.ALLIANCE_READ_WHEN_NEEDED.check(TEAM_CODE);
  }

  @Test
  void vendorLibrariesStayBehindIoInterfaces() {
    ProductionRules.VENDOR_LIBRARIES_STAY_BEHIND_IO_INTERFACES.check(TEAM_CODE);
  }

  @Test
  void noStaticMutableState() {
    ProductionRules.NO_STATIC_MUTABLE_STATE.check(TEAM_CODE);
  }

  /**
   * The team has no configuration records of its own until it writes one (the chassis uses only the
   * libraries'), so this rule may match no class.
   */
  @Test
  void configurationIsBuiltByName() {
    ProductionRules.CONFIGURATION_IS_BUILT_BY_NAME.allowEmptyShould(true).check(TEAM_CODE);
  }

  /**
   * The team has no packages below {@code frc.robot} until it writes one, such as a game of its own
   * on kickoff day, so this rule and the next may match no class.
   */
  @Test
  void theSimulatorStaysOutOfTheRobotsCode() {
    noClasses()
        .that()
        .resideInAPackage("frc.robot.*..")
        .should()
        .dependOnClassesThat()
        .resideInAnyPackage(
            "com.michaelgrundvig.frc.runtime.sim..", "com.michaelgrundvig.frc.simulation..")
        .because(
            "robot code that reaches the simulator runs differently on the robot than in it: the"
                + " simulated world plugs in behind the IO interfaces, through the runtime's sim"
                + " package")
        .allowEmptyShould(true)
        .check(TEAM_CODE);
  }

  @Test
  void onlyTheTopLevelUsesTheTopLevel() {
    noClasses()
        .that()
        .resideInAPackage("frc.robot.*..")
        .should()
        .dependOnClassesThat()
        .resideInAPackage("frc.robot")
        .because(
            "a part that reaches up into Robot, RobotContainer or Robots can't be built or tested"
                + " without the whole robot: hand it what it needs")
        .allowEmptyShould(true)
        .check(TEAM_CODE);
  }
}
