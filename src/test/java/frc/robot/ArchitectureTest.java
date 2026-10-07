package frc.robot;

import static com.tngtech.archunit.core.domain.JavaClass.Predicates.belongToAnyOf;
import static com.tngtech.archunit.core.domain.JavaClass.Predicates.resideInAnyPackage;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.classes;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.fields;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.noClasses;
import static com.tngtech.archunit.library.dependencies.SlicesRuleDefinition.slices;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.tngtech.archunit.base.DescribedPredicate;
import com.tngtech.archunit.core.domain.JavaClass;
import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.domain.JavaCodeUnit;
import com.tngtech.archunit.core.domain.JavaConstructorCall;
import com.tngtech.archunit.core.domain.JavaField;
import com.tngtech.archunit.core.domain.JavaMethodCall;
import com.tngtech.archunit.core.domain.JavaModifier;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.lang.ArchCondition;
import com.tngtech.archunit.lang.ArchRule;
import com.tngtech.archunit.lang.ConditionEvents;
import com.tngtech.archunit.lang.SimpleConditionEvent;
import java.lang.reflect.RecordComponent;
import java.time.Instant;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.EdgeCounterFilter;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.util.MathSharedStore;

/**
 * Architecture rules, checked on every build. Each rule prevents a kind of bug that robot code
 * otherwise accumulates quietly until it bites at an event.
 */
class ArchitectureTest {
  private static final JavaClasses PRODUCTION_CODE = ProductionCode.CLASSES;

  /** Hardware vendors' libraries: each belongs behind an IO interface. */
  private static final String[] VENDOR_LIBRARIES = {
    "com.ctre..", "org.photonvision..", "com.revrobotics..", "com.studica.."
  };

  /**
   * The world's model: the field and the robot as configured, the validation kit they're built
   * with, and the physical values both are made of (ADR 0015). Not its bridges, which write it out
   * for other tools.
   */
  private static final String[] WORLD_MODEL = {
    "com.michaelgrundvig.frc.world.config..",
    "com.michaelgrundvig.frc.world.physical..",
    "com.michaelgrundvig.frc.world.field..",
    "com.michaelgrundvig.frc.world.robot.."
  };

  /** Localization (ADR 0015). */
  private static final String[] LOCALIZATION = {"com.michaelgrundvig.frc.localization.."};

  /**
   * The robot program's own components (its loop timing, its wiring), which only it reads, beside
   * the libraries in lib/ (the world model, localization), whose boundaries Gradle keeps: neither
   * depends on the robot program, so they can't reach it. The Phoenix helper has a rule of its own
   * (below).
   */
  @Test
  void componentsNeverDependOnRobotCode() {
    classes()
        .that()
        .resideInAPackage("frc.robot.lib..")
        .and()
        .resideOutsideOfPackage("frc.robot.lib.phoenix..")
        .and()
        .resideOutsideOfPackage("frc.robot.lib.rev..")
        .should()
        .onlyDependOnClassesThat(
            resideInAnyPackage(
                    "frc.robot.lib..",
                    "java..",
                    "org.wpilib.math..",
                    "org.wpilib.units..",
                    "org.jspecify..",
                    // The annotations on generated configuration builders; nothing at run time.
                    "io.soabase.recordbuilder..",
                    // The validation kit, which every configuration record is built with.
                    "com.michaelgrundvig.frc.world.config..",
                    // The robot's model, its wiring and control among it: plain records.
                    "com.michaelgrundvig.frc.world.robot..")
                // A plain value from elsewhere in WPILib: Systemcore's CAN ports, by its names.
                .or(belongToAnyOf(CANPort.class)))
        .because(
            "components must work in any robot and be testable without one: no hardware, robot"
                + " state, NetworkTables, telemetry, tunables, commands, or simulation inside")
        .check(PRODUCTION_CODE);
  }

  /**
   * The Phoenix helper turns the robot's definition, wiring and control into Phoenix's settings, so
   * it reads the model and Phoenix's configuration classes, which other components may not. Never a
   * device: it builds settings for the IO layer to apply, so it stays pure functions, tested
   * without hardware.
   */
  @Test
  void thePhoenixHelperBuildsSettingsAndNeverTouchesADevice() {
    classes()
        .that()
        .resideInAPackage("frc.robot.lib.phoenix..")
        .should()
        .onlyDependOnClassesThat(
            resideInAnyPackage(
                    "frc.robot.lib..",
                    "java..",
                    "org.wpilib.math..",
                    "org.wpilib.units..",
                    "org.jspecify..",
                    "io.soabase.recordbuilder..",
                    "com.michaelgrundvig.frc.world.config..",
                    "com.michaelgrundvig.frc.world.physical..",
                    "com.michaelgrundvig.frc.world.robot..",
                    // Phoenix's settings: plain values, which the IO layer applies.
                    "com.ctre.phoenix6.configs..",
                    "com.ctre.phoenix6.signals..")
                // A bus's name, a follower's request, and a differential's constants: values too.
                .or(
                    belongToAnyOf(
                        CANBus.class,
                        Follower.class,
                        DifferentialMotorConstants.class,
                        CANPort.class)))
        .because(
            "the helper is pure functions from the robot's definition, wiring and control to"
                + " Phoenix's settings: devices, control requests other than a follower's, and the"
                + " robot's state stay in the IO layer that applies them")
        .check(PRODUCTION_CODE);
  }

  /**
   * The REV helper says the program's plan for a joint ({@code ControllerPlan}) in a SPARK's terms,
   * so it reads the model, the wiring and REV's configuration classes: never Phoenix's settings,
   * and never a device, as the Phoenix helper never is.
   */
  @Test
  void theRevHelperBuildsSettingsAndNeverTouchesADevice() {
    classes()
        .that()
        .resideInAPackage("frc.robot.lib.rev..")
        .should()
        .onlyDependOnClassesThat(
            resideInAnyPackage(
                    "frc.robot.lib.rev..",
                    "frc.robot.lib.wiring..",
                    "java..",
                    "org.wpilib.math..",
                    "org.wpilib.units..",
                    "org.jspecify..",
                    "com.michaelgrundvig.frc.world.robot..",
                    // REV's settings: plain values, which the IO layer applies.
                    "com.revrobotics.spark.config..")
                .or(belongToAnyOf(ClosedLoopSlot.class, FeedbackSensor.class)))
        .because(
            "the helper is pure functions from the program's plan for a joint to a SPARK's"
                + " settings, with no vendor's settings between: devices and the robot's state stay"
                + " in the IO layer")
        .check(PRODUCTION_CODE);
  }

  @Test
  void componentsNeverReadTheClock() {
    noClasses()
        .that()
        .resideInAPackage("frc.robot.lib..")
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
        .because(
            "a component that reads the clock decides differently in a replay than it did on the"
                + " robot; take the time as an argument, stamped when the data was captured")
        .check(PRODUCTION_CODE);
  }

  /**
   * The alliance isn't known until the Driver Station connects, often after the program starts. A
   * class that reads it as it loads, or an object as it's built, keeps that guess all match: the
   * spectrum survey found teams whose red alliance aimed at blue's goal, patched by hand-editing
   * the fallback before each event.
   */
  static final ArchRule ALLIANCE_READ_WHEN_NEEDED =
      classes()
          .should(neverReadTheAllianceAsTheyreBuilt())
          .because(
              "the alliance may not be known yet: read it when it's needed, as"
                  + " RobotContainer.periodic does each loop, and hand it down");

  @Test
  void theAllianceIsReadWhenItsNeeded() {
    ALLIANCE_READ_WHEN_NEEDED.check(PRODUCTION_CODE);
  }

  /**
   * The rule catches a class that reads the alliance as it loads, and one that does as it's built.
   */
  @Test
  void theAllianceRuleCatchesAGuessKeptFromTheStart() {
    JavaClasses guessing =
        new ClassFileImporter().importClasses(GuessesAtLoad.class, GuessesWhenBuilt.class);

    assertThatThrownBy(() -> ALLIANCE_READ_WHEN_NEEDED.check(guessing))
        .hasMessageContaining("GuessesAtLoad")
        .hasMessageContaining("GuessesWhenBuilt");
  }

  /** Keeps the alliance from when the class loaded: a guess before the Driver Station connects. */
  static final class GuessesAtLoad {
    static final boolean RED =
        MatchState.getAlliance().map(alliance -> alliance == Alliance.RED).orElse(false);
  }

  /** Keeps the alliance from when it was built. */
  static final class GuessesWhenBuilt {
    final boolean red =
        MatchState.getAlliance().map(alliance -> alliance == Alliance.RED).orElse(false);
  }

  private static ArchCondition<JavaClass> neverReadTheAllianceAsTheyreBuilt() {
    return new ArchCondition<>("never read the alliance as they load or are built") {
      @Override
      public void check(JavaClass item, ConditionEvents events) {
        List<JavaCodeUnit> building = new ArrayList<>(item.getConstructors());
        item.getStaticInitializer().ifPresent(building::add);
        for (JavaCodeUnit unit : building) {
          for (JavaMethodCall call : unit.getMethodCallsFromSelf()) {
            if (call.getTargetOwner().isEquivalentTo(MatchState.class)
                && call.getName().equals("getAlliance")) {
              events.add(SimpleConditionEvent.violated(item, call.getDescription()));
            }
          }
        }
      }
    };
  }

  @Test
  void componentsHaveNoCycles() {
    slices()
        .matching("frc.robot.lib.(*)..")
        .should()
        .beFreeOfCycles()
        .because("a component package that needs another which needs it back is really one")
        .check(PRODUCTION_CODE);
  }

  @Test
  void vendorLibrariesStayBehindIoInterfaces() {
    noClasses()
        .that()
        .haveNameNotMatching(".*\\.(\\w+IO\\w+|Phoenix\\w*|Rev\\w*|\\w+Sim)(\\$.*)?")
        .should()
        .dependOnClassesThat()
        .resideInAnyPackage(VENDOR_LIBRARIES)
        .because(
            "hardware behind an IO interface is what lets the robot's code run in the simulator and"
                + " replay a log (AdvantageKit's IO pattern): name an implementation for what it"
                + " reads and the hardware it reads it from, like ModuleIOTalonFX; a vendor's own"
                + " helpers take the vendor's name, like PhoenixOdometryThread")
        .check(PRODUCTION_CODE);
  }

  @Test
  void subsystemsStandAlone() {
    slices()
        .matching("frc.robot.subsystems.(*)..")
        .should()
        .notDependOnEachOther()
        .because(
            "each subsystem works on its own, with what it needs handed in; the robot's container"
                + " wires them together")
        .check(PRODUCTION_CODE);
  }

  @Test
  void viewsReadTheModelAndLocalizationAndDrawThroughPitwallsScenes() {
    classes()
        .that()
        .resideInAPackage("frc.robot.views..")
        .should()
        .onlyDependOnClassesThat(
            resideInAnyPackage(
                Stream.of(
                        Stream.of(
                            "frc.robot.views..",
                            "com.michaelgrundvig.frc.pitwall.scene..",
                            "java..",
                            "org.wpilib.math..",
                            "org.wpilib.units..",
                            "org.jspecify.."),
                        Stream.of(WORLD_MODEL),
                        Stream.of(LOCALIZATION))
                    .flatMap(packages -> packages)
                    .toArray(String[]::new)))
        .because(
            "a view draws what the world's model and localization say, through Pitwall's scenes"
                + " alone: no mechanism, hardware, robot state or simulator, and none of the rest"
                + " of Pitwall, so it moves with the libraries it reads")
        .check(PRODUCTION_CODE);
  }

  @Test
  void theSimulatorStaysOutOfTheRobotsCode() {
    noClasses()
        .that()
        .resideInAnyPackage(
            "frc.robot.lib..",
            "frc.robot.subsystems..",
            "frc.robot.hardware..",
            "frc.robot.behavior..",
            "frc.robot.views..",
            "frc.robot.commands..",
            "frc.robot.dashboard..",
            "frc.robot.game..")
        .should()
        .dependOnClassesThat()
        .resideInAnyPackage("frc.robot.sim..", "com.michaelgrundvig.frc.simulation..")
        .because(
            "the robot's code runs the same on the robot as in the simulator: the simulated world"
                + " plugs in behind the IO interfaces, through its adapters in frc.robot.sim, chosen"
                + " at the top level")
        .check(PRODUCTION_CODE);
  }

  @Test
  void onlyTheTopLevelUsesTheTopLevel() {
    noClasses()
        .that()
        .resideInAnyPackage(
            "frc.robot.lib..",
            "frc.robot.subsystems..",
            "frc.robot.hardware..",
            "frc.robot.behavior..",
            "frc.robot.commands..",
            "frc.robot.dashboard..",
            "frc.robot.game..",
            "frc.robot.sim..",
            "frc.robot.views..")
        .should()
        .dependOnClassesThat()
        .resideInAPackage("frc.robot")
        .because(
            "the top level (Robot, RobotContainer, the robots' configurations) wires the rest"
                + " together; what it wires never reaches back up into it")
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
        .check(PRODUCTION_CODE);
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
        .check(PRODUCTION_CODE);
  }

  /**
   * A configuration record's builder hands on a list, a set or a map as it's given, so the record
   * copies each it holds: one changed after the record is built would change the record.
   */
  static final ArchRule CONFIGURATION_COPIES_WHAT_IT_HOLDS =
      classes()
          .that()
          .areAnnotatedWith(ConfigRecord.class)
          .should(copyEachCollectionTheyHold())
          .because(
              "its builder hands a list on as it's given, so a list changed after the record is"
                  + " built would change the record: copy each in the record's constructor, names"
                  + " = List.copyOf(names)");

  @Test
  void configurationCopiesTheListsItHolds() {
    CONFIGURATION_COPIES_WHAT_IT_HOLDS.check(PRODUCTION_CODE);
  }

  /** The rule catches a record that holds a list as it's given, and passes one that copies it. */
  @Test
  void theCopyRuleCatchesARecordHoldingTheListItsGiven() {
    JavaClasses records =
        new ClassFileImporter().importClasses(HoldsWhatItsGiven.class, CopiesWhatItHolds.class);

    assertThatThrownBy(() -> CONFIGURATION_COPIES_WHAT_IT_HOLDS.check(records))
        .hasMessageContaining("HoldsWhatItsGiven holds parts and copies 0 of them")
        .hasMessageNotContaining("CopiesWhatItHolds");
  }

  /** Holds the list it's given: changing that list changes it. */
  @ConfigRecord
  record HoldsWhatItsGiven(String name, List<String> parts) {}

  /** Copies the list it's given. */
  @ConfigRecord
  record CopiesWhatItHolds(String name, List<String> parts) {
    CopiesWhatItHolds {
      parts = List.copyOf(parts);
    }
  }

  /**
   * Each list, set or map a record holds copied in its canonical constructor: as many copies there
   * ({@code List.copyOf}, {@code Set.copyOf}, {@code Map.copyOf}, or a stream's {@code toList}) as
   * it holds.
   */
  private static ArchCondition<JavaClass> copyEachCollectionTheyHold() {
    return new ArchCondition<>("copy each list, set or map they hold") {
      @Override
      public void check(JavaClass record, ConditionEvents events) {
        RecordComponent[] components = record.reflect().getRecordComponents();
        if (components == null) {
          return;
        }
        List<String> held =
            Arrays.stream(components)
                .filter(
                    component ->
                        Collection.class.isAssignableFrom(component.getType())
                            || Map.class.isAssignableFrom(component.getType()))
                .map(RecordComponent::getName)
                .toList();
        if (held.isEmpty()) {
          return;
        }
        List<String> canonical =
            Arrays.stream(components).map(component -> component.getType().getName()).toList();
        long copies =
            record.getConstructors().stream()
                .filter(
                    constructor ->
                        constructor.getRawParameterTypes().stream()
                            .map(JavaClass::getName)
                            .toList()
                            .equals(canonical))
                .flatMap(constructor -> constructor.getMethodCallsFromSelf().stream())
                .filter(ArchitectureTest::copies)
                .count();
        if (copies < held.size()) {
          events.add(
              SimpleConditionEvent.violated(
                  record,
                  record.getName()
                      + " holds "
                      + String.join(", ", held)
                      + " and copies "
                      + copies
                      + " of them in its constructor"));
        }
      }
    };
  }

  /** A call that copies a list, a set or a map into one that can't change. */
  private static boolean copies(JavaMethodCall call) {
    String owner = call.getTargetOwner().getName();
    return (call.getName().equals("copyOf")
            && List.of("java.util.List", "java.util.Set", "java.util.Map").contains(owner))
        || (call.getName().equals("toList") && owner.equals("java.util.stream.Stream"));
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
