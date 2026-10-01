package com.michaelgrundvig.frc.world;

import static com.tngtech.archunit.core.domain.JavaClass.Predicates.belongToAnyOf;
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
import org.wpilib.driverstation.Alliance;

/**
 * The world model's boundaries, checked on every build. Gradle keeps the robot program, the
 * simulator, localization, and Pitwall out of reach, since this project doesn't depend on them;
 * these rules keep what Gradle can't see: its three packages apart, and the model free of clocks
 * and shared state, so everything that reads it, on the robot, in the simulator, and in a replay,
 * reads the same thing.
 */
class ArchitectureTest {
  private static final JavaClasses WORLD =
      new ClassFileImporter()
          .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
          .importPackages("com.michaelgrundvig.frc.world");

  /** What any of the model may use: the JDK, WPILib's math and units, and annotations. */
  private static final String[] PLAIN_JAVA_AND_MATH = {
    "java..",
    "org.wpilib.math..",
    "org.wpilib.units..",
    "org.jspecify..",
    // The annotations on generated configuration builders; nothing at run time.
    "io.soabase.recordbuilder.."
  };

  /** The validation kit: how a record checks its values. Every description builds on it. */
  private static final String KIT = "com.michaelgrundvig.frc.world.config..";

  /** The field: its tags, places, zones, and terrain. */
  private static final String FIELD = "com.michaelgrundvig.frc.world.field..";

  /** The robot's definition: what the robot is, from its frame to its sensors' mounts. */
  private static final String DEFINITION = "com.michaelgrundvig.frc.world.robot..";

  @Test
  void theKitSeesOnlyItself() {
    classes()
        .that()
        .resideInAPackage(KIT)
        .should()
        .onlyDependOnClassesThat(
            resideInAnyPackage(PLAIN_JAVA_AND_MATH).or(resideInAnyPackage(KIT)))
        .because(
            "every description builds on the validation kit, so it can't need one of them; and it"
                + " names no hardware, which only the robot's wiring does")
        .check(WORLD);
  }

  @Test
  void theFieldSeesOnlyTheKit() {
    classes()
        .that()
        .resideInAPackage(FIELD)
        .should()
        .onlyDependOnClassesThat(
            resideInAnyPackage(PLAIN_JAVA_AND_MATH)
                .or(resideInAnyPackage(KIT, FIELD, "org.wpilib.fields.."))
                // Which alliance a place is for: a plain value from WPILib's Driver Station, the
                // one thing of WPILib's main library the model may name.
                .or(belongToAnyOf(Alliance.class)))
        .because(
            "the field is the same whatever robot plays on it, so it knows nothing of the robot's"
                + " definition, as the definition knows nothing of the field")
        .check(WORLD);
  }

  @Test
  void theRobotsDefinitionSeesOnlyTheKit() {
    classes()
        .that()
        .resideInAPackage(DEFINITION)
        .should()
        .onlyDependOnClassesThat(
            resideInAnyPackage(PLAIN_JAVA_AND_MATH).or(resideInAnyPackage(KIT, DEFINITION)))
        .because(
            "what the robot is doesn't depend on the field it plays on, nor on what reads it:"
                + " localization, the simulator, and the views all read the definition")
        .check(WORLD);
  }

  @Test
  void theModelNeverReadsTheClock() {
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
            "a description that reads the clock reads differently in a replay than it did on the"
                + " robot; take the time as an argument")
        .check(WORLD);
  }

  @Test
  void itsPackagesHaveNoCycles() {
    slices()
        .matching("com.michaelgrundvig.frc.world.(*)..")
        .should()
        .beFreeOfCycles()
        .because("a package that needs another which needs it back is really one")
        .check(WORLD);
  }

  @Test
  void noStaticMutableState() {
    fields()
        .that()
        .areStatic()
        .should()
        .beFinal()
        .because(
            "static mutable state makes behavior depend on what ran before, and the robot, the"
                + " simulator's parallel runs, and a replay all share the model")
        .check(WORLD);
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
        .check(WORLD);
  }

  @Test
  void configurationIsBuiltByName() {
    classes()
        .that()
        .areAnnotatedWith(ConfigRecord.class)
        .should(onlyBeConstructedByTheirBuilder())
        .because(
            "a constructor takes its values by position, so two can swap without a compile error;"
                + " the builder names every value (Body.builder().frameLength(...)...)")
        .check(WORLD);
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
