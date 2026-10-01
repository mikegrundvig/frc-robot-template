package com.michaelgrundvig.frc.pitwall;

import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.classes;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.fields;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.noClasses;

import com.tngtech.archunit.base.DescribedPredicate;
import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.core.importer.ImportOption;
import java.util.Set;
import org.junit.jupiter.api.Test;

/** Pitwall's boundaries, checked on every build: what keeps it a library any robot can use. */
class ArchitectureTest {
  private static final JavaClasses PITWALL =
      new ClassFileImporter()
          .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
          .importPackages("com.michaelgrundvig.frc.pitwall");

  @Test
  void advantageKitIsOptional() {
    noClasses()
        .that()
        .haveNameNotMatching(".*\\.AdvantageKitLog(\\$.*)?")
        .should()
        .dependOnClassesThat()
        .resideInAPackage("org.littletonrobotics..")
        .because(
            "Pitwall works without AdvantageKit: only AdvantageKitLog touches it, and only once"
                + " Pitwall has found it on the classpath")
        .check(PITWALL);
  }

  @Test
  void theWebServerStaysInOnePlace() {
    noClasses()
        .that()
        .haveNameNotMatching(".*\\.PitwallServer(\\$.*)?")
        .should()
        .dependOnClassesThat()
        .resideInAPackage("com.sun.net.httpserver..")
        .because("the page and its API are one class; the rest of Pitwall runs in the robot loop")
        .check(PITWALL);
  }

  @Test
  void noHardwareVendorLibraries() {
    noClasses()
        .should()
        .dependOnClassesThat()
        .resideInAnyPackage("com.ctre..", "org.photonvision..", "com.revrobotics..")
        .because("Pitwall is for any robot, whatever its hardware")
        .check(PITWALL);
  }

  /**
   * The one way a gain changes is the page's, through the tee: it checks the change (limits, the
   * configuration's own checks, the robot's rules) and logs it for replay. An experiment may only
   * read a configuration, so none can step a value past those checks, or change one a replay
   * wouldn't. An allow-list, so a method LiveConfig gains later is refused until it's judged a
   * read.
   */
  @Test
  void experimentsOnlyReadAConfiguration() {
    Set<String> reads = Set.of("get", "value", "name");
    noClasses()
        .that()
        .resideInAPackage("com.michaelgrundvig.frc.pitwall.experiment..")
        .should()
        .callMethodWhere(
            DescribedPredicate.describe(
                "a method of LiveConfig other than " + reads,
                call ->
                    call.getTargetOwner().isEquivalentTo(LiveConfig.class)
                        && !reads.contains(call.getName())))
        .orShould()
        .dependOnClassesThat()
        .resideInAPackage("org.wpilib.tunable..")
        .because(
            "gains change only through the tee's checks, which log them for replay; an experiment"
                + " reads them")
        .check(PITWALL);
  }

  /**
   * A scene is data the robot program gives: Pitwall draws whatever its layers say, and knows
   * nothing of fields, robots, or what a robot is made of. An allow-list: the scene package may use
   * Java, WPILib's geometry (so a layer can hand over a pose as it has one), and Pitwall's push,
   * and nothing else of WPILib's or anyone's.
   */
  @Test
  void scenesKnowNothingOfFieldsOrRobots() {
    classes()
        .that()
        .resideInAPackage("com.michaelgrundvig.frc.pitwall.scene..")
        .should()
        .onlyDependOnClassesThat()
        .resideInAnyPackage(
            "java..",
            "org.jspecify..",
            "org.wpilib.math.geometry..",
            "com.michaelgrundvig.frc.pitwall.scene..",
            "com.michaelgrundvig.frc.pitwall.push..")
        .because(
            "Pitwall stays generic: a robot program contributes its world as data, and Pitwall"
                + " draws every shape itself")
        .check(PITWALL);
  }

  @Test
  void noStaticMutableState() {
    fields()
        .that()
        .areStatic()
        .should()
        .beFinal()
        .because("a library's static state leaks between robot programs and between tests")
        .check(PITWALL);
  }
}
