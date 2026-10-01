package frc.robot;

import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.core.importer.ImportOption;

/**
 * The robot program's compiled classes, and Pitwall's, as ArchUnit reads them: imported once, the
 * first time a test asks, and shared by every test that checks them.
 */
final class ProductionCode {
  static final JavaClasses CLASSES =
      new ClassFileImporter()
          .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
          .importPackages("frc.robot", "com.michaelgrundvig.frc.pitwall");

  private ProductionCode() {}
}
