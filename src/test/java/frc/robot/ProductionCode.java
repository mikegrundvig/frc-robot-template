package frc.robot;

import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.core.importer.ImportOption;

/**
 * The team's robot program's compiled classes (frc.robot), and with them the runtime's and the
 * libraries' it runs (the world model, localization, the simulation, Pitwall), as ArchUnit reads
 * them: imported once, the first time a test asks, and shared by every test that checks them. The
 * team's rules check the team's own ({@link #TEAM}); the runtime's rules check the runtime's in its
 * own project. The values register reads every configuration record in all of them.
 */
final class ProductionCode {
  private static final ImportOption NOT_TESTS =
      location -> !location.contains("/testFixtures/") && !location.contains("-test-fixtures");

  /** The team's classes alone. */
  static final JavaClasses TEAM =
      new ClassFileImporter()
          .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
          .withImportOption(NOT_TESTS)
          .importPackages("frc.robot");

  /** The team's, the runtime's and the libraries' classes, whole. */
  static final JavaClasses CLASSES =
      new ClassFileImporter()
          .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
          .withImportOption(NOT_TESTS)
          .importPackages(
              "frc.robot",
              "com.michaelgrundvig.frc.runtime",
              "com.michaelgrundvig.frc.world",
              "com.michaelgrundvig.frc.localization",
              "com.michaelgrundvig.frc.simulation",
              "com.michaelgrundvig.frc.pitwall");

  private ProductionCode() {}
}
