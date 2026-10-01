package com.michaelgrundvig.frc.pitwall.docs;

import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.classes;

import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.core.importer.ImportOption;
import org.junit.jupiter.api.Test;

/** The processor's boundary, checked on every build: what lets it run in any robot's compiler. */
class ArchitectureTest {
  @Test
  void itNeedsNothingButJava() {
    classes()
        .should()
        .onlyDependOnClassesThat()
        .resideInAnyPackage(
            "com.michaelgrundvig.frc.pitwall.docs..", "java..", "javax..", "org.jspecify..")
        .because(
            "it runs inside the compiler of any robot program, and knows the records it documents"
                + " only by their annotation's name, given as text")
        .check(
            new ClassFileImporter()
                .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
                .importPackages("com.michaelgrundvig.frc.pitwall.docs"));
  }
}
