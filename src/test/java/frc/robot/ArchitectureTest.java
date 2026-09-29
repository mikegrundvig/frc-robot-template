package frc.robot;

import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.classes;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.fields;

import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.core.importer.ImportOption;
import org.junit.jupiter.api.Test;

/**
 * Architecture rules, checked on every build. Each rule prevents a kind of bug that robot code
 * otherwise accumulates quietly until it bites at an event.
 */
class ArchitectureTest {
  private static final JavaClasses PRODUCTION_CODE =
      new ClassFileImporter()
          .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
          .importPackages("frc.robot");

  @Test
  void componentsNeverDependOnRobotCode() {
    classes()
        .that()
        .resideInAPackage("frc.robot.lib..")
        .should()
        .onlyDependOnClassesThat()
        .resideInAnyPackage("frc.robot.lib..", "java..", "org.wpilib..", "org.jspecify..")
        .because("components must work in any robot, and be testable without one")
        .allowEmptyShould(true)
        .check(PRODUCTION_CODE);
  }

  @Test
  void noStaticMutableState() {
    fields()
        .that()
        .areStatic()
        .should()
        .beFinal()
        .because(
            "static mutable state makes behavior depend on what ran before; pass state in through"
                + " constructors instead")
        .check(PRODUCTION_CODE);
  }
}
