package com.michaelgrundvig.frc.world;

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
 * The world model's boundaries, checked on every build. Gradle keeps the robot program, the
 * simulator, localization, and Pitwall out of reach, since this project doesn't depend on them;
 * these rules keep what Gradle can't see: its packages apart, and the model free of clocks and
 * shared state, so everything that reads it, on the robot, in the simulator, and in a replay, reads
 * the same thing.
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

  /** The writers that give other tools what the model says, such as AdvantageScope's assets. */
  private static final String BRIDGES = "com.michaelgrundvig.frc.world.bridges..";

  /**
   * The whole model, whatever package it gains (the writers for other tools, in bridges, too):
   * Gradle keeps the robot program and the other libraries out of reach, and WPILib's main library
   * isn't on its classpath, but the math brings WPILib's utilities along (their logs, their own
   * clock).
   */
  @Test
  void theModelSeesOnlyWpilibsMathAndFieldLayouts() {
    classes()
        .should()
        .onlyDependOnClassesThat(
            resideInAnyPackage(PLAIN_JAVA_AND_MATH)
                .or(resideInAnyPackage("com.michaelgrundvig.frc.world..", "org.wpilib.fields..")))
        .because(
            "everything reads the model, on the robot, in the simulator, and in a replay, so it"
                + " holds no hardware, robot state, NetworkTables, logs, or clock")
        .check(WORLD);
  }

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
                .or(resideInAnyPackage(KIT, FIELD, "org.wpilib.fields..")))
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

  /**
   * The writers read the model; the model never reads them. Each writer is one tool's view of the
   * field and the robot, so the field, the definition, and the kit they're built with depend on
   * none.
   */
  @Test
  void theFieldAndTheDefinitionNeverSeeTheWriters() {
    noClasses()
        .that()
        .resideInAnyPackage(KIT, FIELD, DEFINITION)
        .should()
        .dependOnClassesThat()
        .resideInAPackage(BRIDGES)
        .because(
            "a writer is one tool's view of the field and the robot, which mean the same without"
                + " any tool; the views read the model, and the model reads none of them")
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

  /**
   * NullAway trusts a library's {@code @Nullable} only where it treats the code as annotated.
   * Marking each package {@code @NullMarked} (in its package-info) carries that with the jar, to
   * the robot program and every project that reads this one; a package without it would let a null
   * through unannounced.
   */
  @Test
  void everyPackageIsNullMarked() {
    classes().should(resideInANullMarkedPackage()).check(WORLD);
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

  /**
   * Randomness from nowhere differs every run: a replay, a test, or the simulator's seeded runs
   * would each draw differently. Anything random takes its seed, or its draws, as an argument.
   */
  @Test
  void theModelDrawsNothingUnseeded() {
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
            "a draw nobody seeded differs in every run, so a replay can't reproduce it; take the"
                + " seed, or the draws, as an argument")
        .check(WORLD);
  }

  /**
   * What the program's environment says (its variables, its system properties) is the machine's,
   * not the match's, and a replay on another machine reads something else.
   */
  @Test
  void theModelReadsNoEnvironment() {
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
            "the environment is the machine's, and a replay elsewhere reads another; take what it"
                + " would say as configuration")
        .check(WORLD);
  }

  /**
   * Everything runs on the robot loop's thread, in a fixed order: a thread of its own, or anything
   * that hands work to one, would make the order depend on the scheduler.
   */
  @Test
  void theModelStartsNoThreads() {
    noClasses()
        .should()
        .dependOnClassesThat()
        .belongToAnyOf(Thread.class, ThreadLocal.class, Timer.class)
        .orShould()
        .dependOnClassesThat()
        .resideInAPackage("java.util.concurrent..")
        .because(
            "a call made on another thread lands in an order the scheduler picks, which a replay"
                + " can't reproduce; the robot loop calls everything, in its order")
        .check(WORLD);
  }

  /**
   * A static collection anyone can change is shared state, as a static field that isn't final is:
   * share an immutable one ({@code List.of}, {@code Map.copyOf}) instead.
   */
  @Test
  void theModelKeepsNoStaticMutableCollections() {
    fields()
        .that()
        .areStatic()
        .should(notHaveAMutableCollectionType())
        .allowEmptyShould(true)
        .check(WORLD);
    noClasses()
        .should()
        .callConstructorWhere(aStaticInitializerMakingAMutableCollection())
        .because(
            "a static collection built from a mutable one stays mutable to whoever can reach it;"
                + " build it with List.of, Set.of or Map.of, or copy it with copyOf")
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
