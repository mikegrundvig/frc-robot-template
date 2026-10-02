package com.michaelgrundvig.frc.simulation;

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
import com.tngtech.archunit.core.domain.JavaMethodCall;
import com.tngtech.archunit.core.domain.JavaModifier;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.core.importer.ImportOption;
import com.tngtech.archunit.lang.ArchCondition;
import com.tngtech.archunit.lang.ConditionEvents;
import com.tngtech.archunit.lang.SimpleConditionEvent;
import java.security.SecureRandom;
import java.time.Clock;
import java.time.Instant;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.Collection;
import java.util.Map;
import java.util.Random;
import java.util.Timer;
import java.util.UUID;
import org.jspecify.annotations.NullMarked;
import org.junit.jupiter.api.Test;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.EdgeCounterFilter;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.util.MathSharedStore;

/**
 * The simulation's boundaries, checked on every build. Gradle keeps the robot program,
 * localization, and Pitwall out of reach, since this project depends only on the world model; these
 * rules keep what Gradle can't see: no hardware layer, no clock, no unseeded randomness, no
 * threads, and no shared state inside, so a simulated run is a function of its configuration, its
 * seed, and its inputs alone, and worlds run side by side without touching.
 */
class ArchitectureTest {
  private static final JavaClasses SIMULATION =
      new ClassFileImporter()
          .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
          .withImportOption(ArchitectureTest::notATestFixture)
          .importPackages("com.michaelgrundvig.frc.simulation");

  /** Whether a class file isn't the tests' shared fixtures (src/testFixtures), which are tests'. */
  static boolean notATestFixture(com.tngtech.archunit.core.importer.Location location) {
    return !location.contains("/testFixtures/") && !location.contains("-test-fixtures");
  }

  /**
   * WPILib's math brings the rest of its utilities along (its logs, its own clock), which Gradle
   * can't keep out: the simulation uses its math and units, and the world model, and nothing more.
   * No hardware layer (WPILib's HAL, its simulated devices), so a simulated world needs no native
   * libraries and runs on any thread; no robot program, localization, or Pitwall, which see it only
   * through the robot's adapters.
   */
  @Test
  void theSimulationSeesOnlyTheWorldModelAndWpilibsMath() {
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
                "com.michaelgrundvig.frc.simulation.."))
        .because(
            "the simulated world is built from the field and the robot's definition alone, and the"
                + " robot program reaches it only through its adapters")
        .check(SIMULATION);
  }

  @Test
  void theSimulationNeverReadsTheClock() {
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
        // WPILib's math reads the robot's clock in a few places: its shared timestamp, the filters
        // that time what they see, and its pose estimators' update without a time.
        .orShould()
        .callMethod(MathSharedStore.class, "getTimestamp")
        .orShould()
        .callMethodWhere(wpilibPoseEstimatorUpdateWithoutATime())
        .orShould()
        .dependOnClassesThat()
        .belongToAnyOf(Debouncer.class, SlewRateLimiter.class, EdgeCounterFilter.class)
        .orShould()
        .dependOnClassesThat()
        .belongToAnyOf(Clock.class)
        .because(
            "a simulation that reads the clock runs differently each time, and differently beside"
                + " others; time is what its steps add up to")
        .check(SIMULATION);
  }

  /** Each package its own slice. */
  @Test
  void itsPackagesHaveNoCycles() {
    slices()
        .matching("com.michaelgrundvig.frc.(**)")
        .should()
        .beFreeOfCycles()
        .because("a package that needs another which needs it back is really one")
        .check(SIMULATION);
  }

  /**
   * NullAway trusts a library's {@code @Nullable} only where it treats the code as annotated.
   * Marking each package {@code @NullMarked} (in its package-info) carries that with the jar, to
   * the robot program and every project that reads this one; a package without it would let a null
   * through unannounced.
   */
  @Test
  void everyPackageIsNullMarked() {
    classes().should(resideInANullMarkedPackage()).check(SIMULATION);
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
        .check(SIMULATION);
  }

  /**
   * Randomness from nowhere differs every run: a replay, a test, or the simulator's seeded runs
   * would each draw differently. Anything random takes its seed, or its draws, as an argument.
   */
  @Test
  void theSimulationDrawsNothingUnseeded() {
    noClasses()
        .should()
        .callMethod(Math.class, "random")
        .orShould()
        .callMethod(StrictMath.class, "random")
        .orShould()
        .callConstructor(Random.class)
        .orShould()
        .callConstructor(SecureRandom.class)
        .orShould()
        .callMethod(UUID.class, "randomUUID")
        .because(
            "a draw nobody seeded differs in every run, so a study can't repeat it; draw from the"
                + " world's seed (Seeds)")
        .check(SIMULATION);
  }

  /**
   * What the program's environment says (its variables, its system properties) is the machine's,
   * not the match's, and a replay on another machine reads something else.
   */
  @Test
  void theSimulationReadsNoEnvironment() {
    noClasses()
        .should()
        .callMethod(System.class, "getenv")
        .orShould()
        .callMethod(System.class, "getenv", String.class)
        .orShould()
        .callMethod(System.class, "getProperty", String.class)
        .orShould()
        .callMethod(System.class, "getProperty", String.class, String.class)
        .orShould()
        .callMethod(System.class, "getProperties")
        .orShould()
        .callMethod(Integer.class, "getInteger", String.class)
        .orShould()
        .callMethod(Long.class, "getLong", String.class)
        .orShould()
        .callMethod(Boolean.class, "getBoolean", String.class)
        .because(
            "the environment is the machine's, and a run elsewhere reads another; take what it"
                + " would say as configuration")
        .check(SIMULATION);
  }

  /**
   * A world steps on its caller's thread, in a fixed order: a thread of its own, or anything that
   * hands work to one, would make the order depend on the scheduler. Worlds run side by side on
   * threads the caller starts.
   */
  @Test
  void theSimulationStartsNoThreads() {
    noClasses()
        .should()
        .dependOnClassesThat()
        .belongToAnyOf(Thread.class, ThreadLocal.class, Timer.class)
        .orShould()
        .dependOnClassesThat()
        .resideInAPackage("java.util.concurrent..")
        .because(
            "a call made on another thread lands in an order the scheduler picks, which a run can't"
                + " repeat; the caller steps the world, in its order")
        .check(SIMULATION);
  }

  /**
   * A static collection anyone can change is shared state, as a static field that isn't final is:
   * share an immutable one ({@code List.of}, {@code Map.copyOf}) instead.
   */
  @Test
  void theSimulationKeepsNoStaticMutableCollections() {
    fields()
        .that()
        .areStatic()
        .should(notHaveAMutableCollectionType())
        .allowEmptyShould(true)
        .check(SIMULATION);
    noClasses()
        .should()
        .callConstructorWhere(aStaticInitializerMakingAMutableCollection())
        .because(
            "a static collection built from a mutable one stays mutable to whoever can reach it;"
                + " build it with List.of, Set.of or Map.of, or copy it with copyOf")
        .check(SIMULATION);
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
        .check(SIMULATION);
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
        .check(SIMULATION);
  }

  /** Typed as a collection or map class that can be changed, rather than an interface. */
  private static ArchCondition<JavaField> notHaveAMutableCollectionType() {
    return new ArchCondition<>("not have a mutable collection class as their type") {
      @Override
      public void check(JavaField field, ConditionEvents events) {
        JavaClass type = field.getRawType();
        if (isCollection(type) && !isInterface(type)) {
          events.add(
              SimpleConditionEvent.violated(
                  field,
                  field.getFullName() + " is a " + type.getName() + ", which can be changed"));
        }
      }
    };
  }

  /** A static initializer making a collection or map that can be changed. */
  private static DescribedPredicate<JavaConstructorCall>
      aStaticInitializerMakingAMutableCollection() {
    return DescribedPredicate.describe(
        "a static initializer making a mutable collection",
        call ->
            call.getOrigin().getName().equals("<clinit>") && isCollection(call.getTargetOwner()));
  }

  /**
   * Whether a class is a collection or a map. The JDK's own aren't imported (archunit.properties
   * leaves what the project references unresolved), so their supertypes are asked of the class
   * itself.
   */
  private static boolean isCollection(JavaClass type) {
    if (!type.getPackageName().startsWith("java.")) {
      return type.isAssignableTo(Collection.class) || type.isAssignableTo(Map.class);
    }
    Class<?> loaded = type.reflect();
    return Collection.class.isAssignableFrom(loaded) || Map.class.isAssignableFrom(loaded);
  }

  /** Whether a class is an interface, asked of the JDK's own classes themselves, as above. */
  private static boolean isInterface(JavaClass type) {
    return type.getPackageName().startsWith("java.")
        ? type.reflect().isInterface()
        : type.isInterface();
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

  private static ArchCondition<JavaClass> resideInANullMarkedPackage() {
    return new ArchCondition<>("reside in a package marked @NullMarked") {
      @Override
      public void check(JavaClass type, ConditionEvents events) {
        if (!type.getPackage().isAnnotatedWith(NullMarked.class)) {
          events.add(
              SimpleConditionEvent.violated(
                  type,
                  type.getName()
                      + ": its package isn't @NullMarked; mark it in its package-info.java, as the"
                      + " others are"));
        }
      }
    };
  }

  /**
   * A WPILib pose estimator's {@code update}, which stamps what it's given with the clock's time.
   */
  private static DescribedPredicate<JavaMethodCall> wpilibPoseEstimatorUpdateWithoutATime() {
    return DescribedPredicate.describe(
        "a WPILib pose estimator's update, which reads the clock (updateWithTime doesn't)",
        call ->
            call.getName().equals("update")
                && call.getTargetOwner().getPackageName().equals("org.wpilib.math.estimator")
                && call.getTargetOwner().getSimpleName().contains("PoseEstimator"));
  }
}
