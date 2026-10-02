package frc.robot;

import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.core.importer.ImportOption;

/**
 * The robot program's compiled classes, and those of the libraries it runs (the world model,
 * localization, Pitwall), as ArchUnit reads them: imported once, the first time a test asks, and
 * shared by every test that checks them. The libraries' own architecture tests check their
 * boundaries; the robot's rules that hold for every class (no static mutable state, configuration
 * built by name) check them here again, and the values register reads every configuration record.
 */
final class ProductionCode {
  static final JavaClasses CLASSES =
      new ClassFileImporter()
          .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
          .importPackages(
              "frc.robot",
              "com.michaelgrundvig.frc.world",
              "com.michaelgrundvig.frc.localization",
              "com.michaelgrundvig.frc.pitwall");

  private ProductionCode() {}
}
