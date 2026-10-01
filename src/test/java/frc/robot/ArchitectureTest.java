package frc.robot;

import static com.tngtech.archunit.core.domain.JavaClass.Predicates.belongToAnyOf;
import static com.tngtech.archunit.core.domain.JavaClass.Predicates.resideInAnyPackage;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.classes;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.fields;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.noClasses;
import static com.tngtech.archunit.library.dependencies.SlicesRuleDefinition.slices;

import com.tngtech.archunit.base.DescribedPredicate;
import com.tngtech.archunit.core.domain.JavaClass;
import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.domain.JavaConstructorCall;
import com.tngtech.archunit.core.domain.JavaField;
import com.tngtech.archunit.core.domain.JavaModifier;
import com.tngtech.archunit.lang.ArchCondition;
import com.tngtech.archunit.lang.ConditionEvents;
import com.tngtech.archunit.lang.SimpleConditionEvent;
import frc.robot.lib.config.ConfigRecord;
import java.time.Instant;
import java.time.LocalDate;
import java.time.LocalDateTime;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.Alliance;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.imu.OnboardIMU;

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

  /** What any component may use: the JDK, WPILib's math and units, and annotations. */
  private static final String[] PLAIN_JAVA_AND_MATH = {
    "java..",
    "org.wpilib.math..",
    "org.wpilib.units..",
    "org.jspecify..",
    // The annotations on generated configuration builders; nothing at run time.
    "io.soabase.recordbuilder.."
  };

  /** The validation kit: how a record checks its values. Every description builds on it. */
  private static final String KIT = "frc.robot.lib.config..";

  /** The field: its tags, places, zones, and terrain. */
  private static final String FIELD = "frc.robot.lib.field..";

  /** The robot's definition: what the robot is, from its frame to its sensors' mounts. */
  private static final String DEFINITION = "frc.robot.lib.robot..";

  @Test
  void componentsNeverDependOnRobotCode() {
    classes()
        .that()
        .resideInAPackage("frc.robot.lib..")
        .should()
        .onlyDependOnClassesThat(
            resideInAnyPackage(PLAIN_JAVA_AND_MATH)
                .or(resideInAnyPackage("frc.robot.lib..", "org.wpilib.fields.."))
                // Plain values from elsewhere in WPILib.
                .or(
                    belongToAnyOf(
                        Alliance.class, CANPort.class, OnboardIMU.MountOrientation.class)))
        .because(
            "components must work in any robot and be testable without one: no hardware, robot"
                + " state, NetworkTables, telemetry, tunables, commands, or simulation inside")
        .check(PRODUCTION_CODE);
  }

  @Test
  void componentsNeverSeeTheRobotsMechanismsOrTheSimulator() {
    noClasses()
        .that()
        .resideInAPackage("frc.robot.lib..")
        .should()
        .dependOnClassesThat()
        .resideInAnyPackage(
            "com.michaelgrundvig.frc.pitwall..",
            "frc.robot.sim..",
            "frc.robot.subsystems..",
            "frc.robot.commands..",
            "org.littletonrobotics..")
        .orShould()
        .dependOnClassesThat()
        .resideInAnyPackage(VENDOR_LIBRARIES)
        .because(
            "the robot program, the simulator, and Pitwall's views all use the components, so a"
                + " component can't need any of them: Pitwall, the simulator, a mechanism's IO, or"
                + " a vendor's library would tie it to one")
        .check(PRODUCTION_CODE);
  }

  /**
   * The kit, the field, and the robot's definition describe the world. Localization, the simulator,
   * and the views read them, so they read nothing back, and hold no hardware.
   */
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
        .check(PRODUCTION_CODE);
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
                // Which alliance a place is for: a plain value from WPILib's Driver Station.
                .or(belongToAnyOf(Alliance.class)))
        .because(
            "the field is the same whatever robot plays on it, so it knows nothing of the robot's"
                + " definition, as the definition knows nothing of the field")
        .check(PRODUCTION_CODE);
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
        .check(PRODUCTION_CODE);
  }

  @Test
  void localizationSeesOnlyTheWorldModel() {
    classes()
        .that()
        .resideInAnyPackage("frc.robot.lib.localization..", "frc.robot.lib.calibration..")
        .should()
        .onlyDependOnClassesThat(
            resideInAnyPackage(PLAIN_JAVA_AND_MATH)
                .or(
                    resideInAnyPackage(
                        KIT,
                        FIELD,
                        DEFINITION,
                        "frc.robot.lib.localization..",
                        "frc.robot.lib.calibration..")))
        .because(
            "localization runs every loop on the robot from the field and the robot's definition"
                + " alone: it needs nothing of the robot's loop, so its tests run in milliseconds")
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
        .because(
            "a component that reads the clock decides differently in a replay than it did on the"
                + " robot; take the time as an argument, stamped when the data was captured")
        .check(PRODUCTION_CODE);
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
        .haveNameNotMatching(".*\\.(\\w+IO\\w+|Phoenix\\w*|\\w+Sim)(\\$.*)?")
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
  void theSimulatorStaysOutOfTheRobotsCode() {
    noClasses()
        .that()
        .resideInAnyPackage("frc.robot.lib..", "frc.robot.subsystems..")
        .should()
        .dependOnClassesThat()
        .resideInAPackage("frc.robot.sim..")
        .because(
            "the robot's code runs the same on the robot as in the simulator: the simulated world"
                + " plugs in behind the IO interfaces, chosen at the top level")
        .check(PRODUCTION_CODE);
  }

  @Test
  void onlyTheTopLevelUsesTheTopLevel() {
    noClasses()
        .that()
        .resideInAnyPackage(
            "frc.robot.lib..",
            "frc.robot.subsystems..",
            "frc.robot.commands..",
            "frc.robot.dashboard..",
            "frc.robot.game..",
            "frc.robot.sim..")
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
