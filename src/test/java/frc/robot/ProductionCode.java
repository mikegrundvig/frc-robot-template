package frc.robot;

import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.core.importer.ImportOption;

/**
 * The robot program's compiled classes, and those of the libraries it runs (the world model,
 * localization, the simulation, Pitwall), as ArchUnit reads them: imported once, the first time a
 * test asks, and shared by every test that checks them. The rules that hold for every class (no
 * static mutable state, configuration built by name) check them here, once, and the values register
 * reads every configuration record.
 */
final class ProductionCode {
  static final JavaClasses CLASSES =
      new ClassFileImporter()
          .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
          // The simulation's test fixtures (its truth suite's known gaps) are tests' too.
          .withImportOption(
              location ->
                  !location.contains("/testFixtures/") && !location.contains("-test-fixtures"))
          .importPackages(
              "frc.robot",
              "com.michaelgrundvig.frc.world",
              "com.michaelgrundvig.frc.localization",
              "com.michaelgrundvig.frc.simulation",
              "com.michaelgrundvig.frc.pitwall");

  private ProductionCode() {}
}
