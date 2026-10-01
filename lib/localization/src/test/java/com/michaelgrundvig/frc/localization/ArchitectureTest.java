package com.michaelgrundvig.frc.localization;

import static com.tngtech.archunit.core.domain.JavaClass.Predicates.resideInAnyPackage;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.classes;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.fields;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.noClasses;
import static com.tngtech.archunit.library.dependencies.SlicesRuleDefinition.slices;

import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.tngtech.archunit.base.DescribedPredicate;
import com.tngtech.archunit.core.domain.JavaClass;
import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.domain.JavaConstructorCall;
import com.tngtech.archunit.core.domain.JavaField;
import com.tngtech.archunit.core.domain.JavaModifier;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.core.importer.ImportOption;
import com.tngtech.archunit.lang.ArchCondition;
import com.tngtech.archunit.lang.ConditionEvents;
import com.tngtech.archunit.lang.SimpleConditionEvent;
import java.time.Instant;
import java.time.LocalDate;
import java.time.LocalDateTime;
import org.junit.jupiter.api.Test;

/**
 * Localization's boundaries, checked on every build. Gradle keeps the robot program, the simulator,
 * and Pitwall out of reach, since this project depends only on the world model; these rules keep
 * what Gradle can't see: no clock and no shared state inside, so the estimate is a function of its
 * inputs alone, and a replay reproduces it.
 */
class ArchitectureTest {
  private static final JavaClasses LOCALIZATION =
      new ClassFileImporter()
          .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
          .importPackages("com.michaelgrundvig.frc.localization");

  /**
   * WPILib's math brings the rest of its utilities along (its logs, its own clock), which Gradle
   * can't keep out: localization uses its math and units, and the world model, and nothing more.
   */
  @Test
  void localizationSeesOnlyTheWorldModelAndWpilibsMath() {
    classes()
        .should()
        .onlyDependOnClassesThat(
            resideInAnyPackage(
                "java..",
                "org.wpilib.math..",
                "org.wpilib.units..",
                "org.jspecify..",
                // The annotations on generated configuration builders; nothing at run time.
                "io.soabase.recordbuilder..",
                "com.michaelgrundvig.frc.world..",
                "com.michaelgrundvig.frc.localization.."))
        .because(
            "localization runs every loop on the robot from the field and the robot's definition"
                + " alone, and its inputs come in through its methods")
        .check(LOCALIZATION);
  }

  @Test
  void localizationNeverReadsTheClock() {
    noClasses()
        .should()
        .callMethod(System.class, "nanoTime")
        .orShould()
        .callMethod(System.class, "currentTimeMillis")
        .orShould()
        .callMethod(Instant.class, "now")
        .orShould()
        .callMethod(LocalDate.class, "now")
        .orShould()
        .callMethod(LocalDateTime.class, "now")
        .because(
            "an estimator that reads the clock decides differently in a replay than it did on the"
                + " robot; take the time as an argument, stamped when the data was captured")
        .check(LOCALIZATION);
  }

  /** Each package its own slice: the estimator's, and the calibration math's. */
  @Test
  void itsPackagesHaveNoCycles() {
    slices()
        .matching("com.michaelgrundvig.frc.(**)")
        .should()
        .beFreeOfCycles()
        .because("a package that needs another which needs it back is really one")
        .check(LOCALIZATION);
  }

  @Test
  void noStaticMutableState() {
    fields()
        .that()
        .areStatic()
        .should()
        .beFinal()
        .because(
            "static mutable state makes behavior depend on what ran before: the simulator's"
                + " parallel runs and a replay would each see the others'")
        .check(LOCALIZATION);
  }

  @Test
  void staticArraysArePrivate() {
    fields()
        .that()
        .areStatic()
        .and(arrays())
        // The compiler's own tables for switching on an enum aren't ours to keep private.
        .and()
        .doNotHaveModifier(JavaModifier.SYNTHETIC)
        .should()
        .bePrivate()
        .because(
            "a final array's elements can still be changed by anyone who can see it: share a"
                + " List.of instead")
        .allowEmptyShould(true) // none today: the rule is for the first
        .check(LOCALIZATION);
  }

  @Test
  void configurationIsBuiltByName() {
    classes()
        .that()
        .areAnnotatedWith(ConfigRecord.class)
        .should(onlyBeConstructedByTheirBuilder())
        .because(
            "a constructor takes its values by position, so two can swap without a compile error;"
                + " the builder names every value")
        .check(LOCALIZATION);
  }

  private static DescribedPredicate<JavaField> arrays() {
    return DescribedPredicate.describe("are arrays", field -> field.getRawType().isArray());
  }

  /** A record {@code a.b.Outer$Name} is built by {@code a.b.OuterNameBuilder}, and nothing else. */
  private static ArchCondition<JavaClass> onlyBeConstructedByTheirBuilder() {
    return new ArchCondition<>("only be constructed by their generated builder") {
      @Override
      public void check(JavaClass record, ConditionEvents events) {
        String builder = record.getName().replace("$", "") + "Builder";
        for (JavaConstructorCall call : record.getConstructorCallsToSelf()) {
          String caller = call.getOriginOwner().getName();
          if (!caller.equals(builder) && !caller.startsWith(builder + "$")) {
            events.add(SimpleConditionEvent.violated(call, call.getDescription()));
          }
        }
      }
    };
  }
}
