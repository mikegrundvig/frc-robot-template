package com.michaelgrundvig.frc.pitwall;

import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.noClasses;

import com.tngtech.archunit.base.DescribedPredicate;
import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.core.importer.ImportOption;
import java.util.List;
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
            "a robot program without AdvantageKit can't load a class that names it: only"
                + " AdvantageKitLog does, once Pitwall has found AdvantageKit on the classpath")
        .check(PITWALL);
  }

  @Test
  void noHardwareVendorLibraries() {
    noClasses()
        .should()
        .dependOnClassesThat()
        .resideInAnyPackage("com.ctre..", "org.photonvision..", "com.revrobotics..")
        .because(
            "a vendor's class in Pitwall can't load in a robot program without that vendor's"
                + " library: Pitwall is for any robot, whatever its hardware")
        .check(PITWALL);
  }

  /**
   * The one way a gain changes is the page's, through the tee: it checks the change (limits, then
   * the configuration's own checks) and logs it for replay. An experiment reads gains, and changes
   * none.
   */
  @Test
  void experimentsNeverChangeAConfiguration() {
    List<String> changes = List.of("set", "revert", "publishTunable");
    noClasses()
        .that()
        .resideInAPackage("com.michaelgrundvig.frc.pitwall.experiment..")
        .should()
        .callMethodWhere(
            DescribedPredicate.describe(
                "LiveConfig's " + changes,
                call ->
                    call.getTargetOwner().isEquivalentTo(LiveConfig.class)
                        && changes.contains(call.getName())))
        .orShould()
        .dependOnClassesThat()
        .resideInAPackage("org.wpilib.tunable..")
        .because(
            "a gain an experiment changed would skip the tee's checks, and a replay wouldn't see"
                + " it")
        .check(PITWALL);
  }
}
