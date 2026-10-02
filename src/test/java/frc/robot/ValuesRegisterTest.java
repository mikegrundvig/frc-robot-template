package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.localization.LocalizationGate;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import frc.robot.ValuesRegister.CodeRecord;
import frc.robot.ValuesRegister.Committed;
import frc.robot.ValuesRegister.Component;
import frc.robot.ValuesRegister.Knob;
import frc.robot.ValuesRegister.Switch;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/**
 * The values register (docs/values.md) against the code: every configuration value has a row saying
 * what kind it is and where its number came from, and no row names a value that's gone. When this
 * fails, each problem says what to add to the register or remove from it.
 */
class ValuesRegisterTest {
  private static final Path REGISTER =
      Path.of(System.getProperty("frc.projectDir", "."), "docs", "values.md");

  /** The code's records, scanned the first time a test asks, and only then. */
  private static final class Scanned {
    static final List<CodeRecord> RECORDS = ValuesRegister.scan(ProductionCode.CLASSES);
  }

  /**
   * Each mechanism's live tuning, from the container's one list of them: the values it tunes and
   * their limits, by the tuning's name.
   */
  private static Map<String, List<Knob>> tuning() {
    Map<String, List<Knob>> tuning = new LinkedHashMap<>();
    for (LiveConfig<?> live : RobotContainer.tunings(Robots.current()).all()) {
      tuning.put(
          live.name(),
          live.knobs().stream()
              .map(k -> new Knob(k.path(), k.limits().min(), k.limits().max()))
              .toList());
    }
    return tuning;
  }

  /**
   * The values the template ships, which the register's starting values must show: every record's
   * DEFAULTS, the scoring gate, the simulator's season-worn sensors, and the template's robot.
   */
  private static Map<String, Committed> committed() {
    Map<String, Committed> committed =
        new LinkedHashMap<>(ValuesRegister.defaults(Scanned.RECORDS));
    RobotConfig robot = Robots.SIM;
    committed.put(
        "LocalizationGate", new Committed("LocalizationGate.SCORING", LocalizationGate.SCORING));
    committed.put("SensorErrors", new Committed("SensorErrors.TYPICAL", SensorErrors.TYPICAL));
    committed.put(
        "SensorErrors.GyroErrors",
        new Committed("SensorErrors.TYPICAL's gyro", SensorErrors.TYPICAL.gyro()));
    committed.put(
        "SensorErrors.AccelErrors",
        new Committed(
            "SensorErrors.TYPICAL's accelerometer", SensorErrors.TYPICAL.accelerometer()));
    RobotDefinition definition = robot.definition();
    committed.put("Body", new Committed("Robots.SIM's body", definition.body()));
    committed.put(
        "DrivetrainGeometry", new Committed("Robots.SIM's drivetrain", definition.drivetrain()));
    committed.put(
        "DrivetrainGeometry.Modules",
        new Committed("Robots.SIM's modules", definition.drivetrain().modules()));
    committed.put(
        "Motor",
        new Committed("Robots.SIM's drive motor", definition.drivetrain().modules().driveMotor()));
    committed.put(
        "Gearing",
        new Committed(
            "Robots.SIM's drive gearing", definition.drivetrain().modules().driveGearing()));
    committed.put(
        "DrivetrainControl.Limits",
        new Committed("Robots.SIM's limits", robot.drivetrain().limits()));
    committed.put(
        "DrivetrainControl.Gains", new Committed("Robots.SIM's gains", robot.drivetrain().gains()));
    committed.put(
        "CameraConfig.Sensor",
        new Committed("Robots.SIM's camera sensor", definition.cameras().get(0).sensor()));
    return committed;
  }

  private static ValuesRegister register() throws IOException {
    return ValuesRegister.parse(Files.readString(REGISTER));
  }

  @Test
  void theRegisterHasARowForEveryConfigurationValueAndNoOther() throws IOException {
    assertThat(register().problems(Scanned.RECORDS, tuning())).isEmpty();
  }

  @Test
  void theLegendsListTheKindsAndSourcesTheCheckAccepts() throws IOException {
    assertThat(register().legendProblems()).isEmpty();
  }

  @Test
  void theStartingValuesAreTheOnesTheTemplateShips() throws IOException {
    Map<String, Committed> committed = committed();

    assertThat(committed)
        .containsKeys("VisionConfig", "EstimatorConfig", "TerrainCrossing.Settings");
    assertThat(register().startingValueProblems(committed)).isEmpty();
  }

  @Test
  void theScanFindsEveryConfigurationRecordAndWhatEachValueHolds() {
    assertThat(Scanned.RECORDS)
        .extracting(CodeRecord::name)
        .contains(
            "RobotConfig", "Body", "DrivetrainControl.Limits", "ImuCrossCheck.Settings", "Motor")
        .contains("ImuReadings") // listed in the register as not configuration
        .doesNotHaveDuplicates();
    assertThat(record("RobotConfig").components())
        .contains(
            new Component("definition", "RobotDefinition"),
            new Component("drivetrainWiring", "SwerveWiring")); // in an Optional
    assertThat(record("RobotDefinition").components())
        .contains(
            new Component("name", null),
            new Component("body", "Body"),
            new Component("cameras", "CameraConfig")); // in a List
    assertThat(record("SensorErrors").components())
        .contains(
            new Component("cameraMountErrors", null), // a map of transforms, no record
            new Component("gyro", "SensorErrors.GyroErrors"));
  }

  @Test
  void nestedRecordsAreNamedWithTheirOuterRecord() {
    assertThat(ValuesRegister.registerName("frc.robot.subsystems.drive.DrivetrainControl$Limits"))
        .isEqualTo("DrivetrainControl.Limits");
    assertThat(record("SensorErrors.GyroErrors").className())
        .isEqualTo("com.michaelgrundvig.frc.simulation.SensorErrors$GyroErrors");
  }

  private static CodeRecord record(String name) {
    return Scanned.RECORDS.stream().filter(r -> r.name().equals(name)).findFirst().orElseThrow();
  }

  // An arm, for the checks' own tests: a length, gains in a record of their own, and readings
  // that aren't configuration.
  private static final List<CodeRecord> ARM =
      List.of(
          new CodeRecord(
              "Arm",
              "frc.robot.Arm",
              List.of(new Component("length", null), new Component("gains", "Arm.Gains"))),
          new CodeRecord(
              "Arm.Gains",
              "frc.robot.Arm$Gains",
              List.of(new Component("kP", null), new Component("kD", null))),
          new CodeRecord(
              "ArmReadings", "frc.robot.ArmReadings", List.of(new Component("angle", null))));

  private static final Map<String, List<Knob>> ARM_TUNING =
      Map.of("arm", List.of(new Knob("gains.kP", 0, 10), new Knob("gains.kD", 0, 1)));

  /** The arm's gains as it ships them. */
  record ArmGains(double kP, double kD) {}

  private static final String ARM_REGISTER =
      """
      # Values

      | Kind | Meaning |
      |---|---|
      | chosen | A legend, which the check reads apart. |

      ## The arm

      ### `Arm`

      | Value | Kind | Template's starting value | Source | How to measure it, or why |
      |---|---|---|---|---|
      | `length` | as built | 0.8 m | datasheet (its drawing) | From CAD. |

      ### `Arm.Gains`

      | Value | Kind | Template's starting value | Source | How to measure it, or why |
      |---|---|---|---|---|
      | `kP` | chosen | 2 | simulation | Why: it holds. |
      | `kD` | chosen | 0.1 | top team (254) | Why: it damps. |

      ## Tuning limits

      ### `arm` tuning

      | Knob | Limits | Source | Why |
      |---|---|---|---|
      | `gains.kP` | 0 to 10 | guess | Room to tune. |
      | `gains.kD` | 0 to 1 | guess | Room to tune. |

      ## Records that aren't configuration

      | Record | Why it isn't configuration |
      |---|---|
      | `ArmReadings` | What the arm read in one loop. |
      """;

  private static List<String> problems(String register) {
    return ValuesRegister.parse(register).problems(ARM, ARM_TUNING);
  }

  @Test
  void aRegisterThatMatchesTheCodeHasNoProblems() {
    assertThat(problems(ARM_REGISTER)).isEmpty();
  }

  @Test
  void aMissingValueOrRecordSaysWhatToAdd() {
    String register =
        ARM_REGISTER
            .replace(
                "### `Arm`\n\n"
                    + "| Value | Kind | Template's starting value | Source | How to measure it, or"
                    + " why |\n"
                    + "|---|---|---|---|---|\n"
                    + "| `length` | as built | 0.8 m | datasheet (its drawing) | From CAD. |\n\n",
                "")
            .replace("### `Arm.Gains`", "### `Arm.Gainz`");

    assertThat(problems(register))
        .containsExactly(
            "docs/values.md line 13: remove the section for Arm.Gainz: no configuration record is"
                + " named Arm.Gainz",
            "Add a section for Arm (frc.robot.Arm) to docs/values.md: a heading ### `Arm` and a"
                + " table of its values with a row for each: length. Or, if it carries readings"
                + " rather than configuration, list it with the records that aren't configuration",
            "Add a section for Arm.Gains (frc.robot.Arm$Gains) to docs/values.md: a heading ###"
                + " `Arm.Gains` and a table of its values with a row for each: kP, kD. Or, if it"
                + " carries readings rather than configuration, list it with the records that"
                + " aren't configuration");
  }

  @Test
  void aSectionWithoutItsTableSaysSoAtItsHeading() {
    String register =
        ARM_REGISTER.replace(
            "| `length` | as built | 0.8 m | datasheet (its drawing) | From CAD. |\n", "");

    assertThat(problems(register))
        .containsExactly(
            "docs/values.md line 9: the section for Arm has no table of its values: add one whose"
                + " header starts with Value, with a row for each: length");
  }

  @Test
  void aTableUnderARecordsHeadingMustBeItsValues() {
    String register =
        ARM_REGISTER.replace(
            "| Value | Kind | Template's starting value | Source | How to measure it, or why |\n"
                + "|---|---|---|---|---|\n"
                + "| `length` |",
            "| Name | Kind | Template's starting value | Source | How to measure it, or why |\n"
                + "|---|---|---|---|---|\n"
                + "| `length` |");

    assertThat(problems(register))
        .containsExactly(
            "docs/values.md line 11: the table under ### `Arm` must start its header with Value (a"
                + " record's values) or Knob (a tuning's knobs), not \"Name\"",
            "docs/values.md line 9: the section for Arm has no table of its values: add one whose"
                + " header starts with Value, with a row for each: length");
  }

  @Test
  void aValueMissingFromItsRecordsTableSaysWhichAndWhere() {
    String register =
        ARM_REGISTER.replace("| `kD` | chosen | 0.1 | top team (254) | Why: it damps. |\n", "");

    assertThat(problems(register))
        .containsExactly(
            "Add Arm.Gains.kD to docs/values.md: a row | `kD` | kind | starting value | source |"
                + " how | in Arm.Gains's table");
  }

  @Test
  void aRowForAValueThatsGoneSaysToRemoveIt() {
    String register =
        ARM_REGISTER.replace("| `kD` |", "| `kI` | chosen | 0 | guess | Why: none. |\n| `kD` |");

    assertThat(problems(register))
        .containsExactly(
            "docs/values.md line 20: remove Arm.Gains.kI: Arm.Gains has no value named kI");
  }

  @Test
  void aValueThatHoldsAConfigurationRecordHasItsSectionNotARow() {
    String register =
        ARM_REGISTER.replace(
            "| `length` |", "| `gains` | chosen | | none | See below. |\n| `length` |");

    assertThat(problems(register))
        .containsExactly(
            "docs/values.md line 13: remove Arm.gains: it holds Arm.Gains, which has its own"
                + " section");
  }

  @Test
  void kindsAndSourcesComeFromTheirLists() {
    String register =
        ARM_REGISTER
            .replace("| `kP` | chosen | 2 | simulation |", "| `kP` | tuned | 2 | simulation |")
            .replace("| `kD` | chosen | 0.1 | top team (254) |", "| `kD` | chosen | 0.1 | 254 |")
            .replace("| `gains.kP` | 0 to 10 | guess |", "| `gains.kP` | 0 to 10 | |");

    assertThat(problems(register))
        .containsExactly(
            "docs/values.md line 19: Arm.Gains.kP's kind is \"tuned\": it must be one of"
                + " calibrated, characterized, chosen, as built",
            "docs/values.md line 20: Arm.Gains.kD's source is \"254\": it must start with one of"
                + " measured, 2026 code, top team, datasheet, rules, math, simulation, guess,"
                + " unknown, none, with any detail after it in parentheses",
            "docs/values.md line 28: the arm tuning's gains.kP's source is \"\": it must start"
                + " with one of measured, 2026 code, top team, datasheet, rules, math, simulation,"
                + " guess, unknown, none, with any detail after it in parentheses");
  }

  @Test
  void theLegendsMustListWhatTheCheckAccepts() {
    ValuesRegister register = ValuesRegister.parse(ARM_REGISTER);

    assertThat(register.legendProblems())
        .containsExactly(
            "docs/values.md's legend lists the kinds [chosen], but the test accepts [calibrated,"
                + " characterized, chosen, as built]: make them the same (ValuesRegister.KINDS)",
            "docs/values.md's legend lists the sources [], but the test accepts [measured, 2026"
                + " code, top team, datasheet, rules, math, simulation, guess, unknown, none]: make"
                + " them the same (ValuesRegister.SOURCES)");
  }

  @Test
  void aRecordIsConfigurationOrListedAsNotNeverBoth() {
    String register =
        ARM_REGISTER.replace(
            "| `ArmReadings` | What the arm read in one loop. |",
            "| `ArmReadings` | What the arm read in one loop. |\n| `Arm` | Not really. |\n"
                + "| `OldReadings` | Long gone. |");

    assertThat(problems(register))
        .containsExactly(
            "docs/values.md line 37: remove OldReadings from the records that aren't"
                + " configuration: no record marked @ConfigRecord is named OldReadings",
            "docs/values.md line 13: Arm has values here, but line 36 lists it as a record that"
                + " isn't configuration: keep one");
  }

  @Test
  void aRecordARobotsConfigurationHoldsCantBeListedAsNotConfiguration() {
    List<CodeRecord> robot =
        List.of(
            new CodeRecord(
                "RobotConfig", "frc.robot.RobotConfig", List.of(new Component("arm", "Arm"))),
            ARM.get(0),
            ARM.get(1),
            ARM.get(2));
    String register =
        ARM_REGISTER.replace(
            "| `ArmReadings` | What the arm read in one loop. |",
            "| `ArmReadings` | What the arm read in one loop. |\n| `Arm.Gains` | Not really. |");

    assertThat(ValuesRegister.parse(register).recordProblems(robot))
        .contains(
            "docs/values.md line 36: Arm.Gains can't be listed as not configuration: a robot's"
                + " configuration holds it (RobotConfig.arm → Arm.gains)");
  }

  @Test
  void aRecordOfReadingsNotListedAsSuchIsAskedFor() {
    String register =
        ARM_REGISTER.replace("| `ArmReadings` | What the arm read in one loop. |\n", "");

    assertThat(problems(register))
        .singleElement()
        .asString()
        .startsWith("Add a section for ArmReadings (frc.robot.ArmReadings)");
  }

  @Test
  void everyKnobOfEveryTuningHasARow() {
    String register =
        ARM_REGISTER
            .replace("| `gains.kD` | 0 to 1 | guess | Room to tune. |\n", "")
            .replace("| `gains.kP` |", "| `gains.kI` | 0 to 1 | guess | Gone. |\n| `gains.kP` |");
    Map<String, List<Knob>> tuning =
        Map.of(
            "arm",
            List.of(new Knob("gains.kP", 0, 10), new Knob("gains.kD", 0, 1)),
            "wrist",
            List.of(new Knob("gains.kP", 0, 5)));

    assertThat(ValuesRegister.parse(register).problems(ARM, tuning))
        .containsExactly(
            "Add the arm tuning's gains.kD to docs/values.md: a row | `gains.kD` | limits |"
                + " source | why | in its table",
            "docs/values.md line 28: remove the arm tuning's gains.kI: it tunes no value by that"
                + " path",
            "Add a section for the wrist tuning to docs/values.md: a heading ### `wrist` tuning"
                + " and a table of its knobs with a row for each: gains.kP");
  }

  @Test
  void aStaleStartingValueSaysWhatItShouldBe() {
    Map<String, Committed> shipped =
        Map.of("Arm.Gains", new Committed("ArmGains.DEFAULTS", new ArmGains(2.5, 0.1)));

    assertThat(ValuesRegister.parse(ARM_REGISTER).startingValueProblems(shipped))
        .containsExactly(
            "docs/values.md line 19: Arm.Gains.kP's starting value is \"2\", but"
                + " ArmGains.DEFAULTS has 2.5: update the cell");
  }

  @Test
  void aCellShowsAValueToItsOwnPrecision() {
    assertThat(ValuesRegister.shows("5° (0.087 rad)", Math.toRadians(5))).isTrue();
    assertThat(ValuesRegister.shows("2.54 V/(m/s)", 2.5376)).isTrue();
    assertThat(ValuesRegister.shows("0.985, 0.99", 0.99)).isTrue();
    assertThat(ValuesRegister.shows("-0.3", -0.3)).isTrue();
    assertThat(ValuesRegister.shows("2.5", 2.56)).isFalse();
    assertThat(ValuesRegister.shows("none", 0)).isFalse();
  }

  @Test
  void staleKnobLimitsSayWhatTheyShouldBe() {
    Map<String, List<Knob>> tuning =
        Map.of("arm", List.of(new Knob("gains.kP", 0, 20), new Knob("gains.kD", 0, 1)));

    assertThat(ValuesRegister.parse(ARM_REGISTER).problems(ARM, tuning))
        .containsExactly(
            "docs/values.md line 28: the arm tuning's gains.kP has the limits \"0 to 10\", but"
                + " it's tuned from 0 to 20: update the cell");
  }

  private static final String ARM_SWITCHES =
      """

      ## Switches

      | Switch | Limits | Template's starting value | Source | Why |
      |---|---|---|---|---|
      | `Arm/holdSeconds` | 0 to 5 s | 0 s | guess | Holds the arm. |
      | `Arm/<camera>/enabled` | on or off | on | none | Leaves a camera out. |
      """;

  private static final Map<String, Switch> ARM_PUBLISHED =
      Map.of(
          "Arm/holdSeconds",
          new Switch("Arm/holdSeconds", 0.0, 0, 5),
          "Arm/<camera>/enabled",
          new Switch(
              "Arm/<camera>/enabled", true, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY));

  @Test
  void everySwitchTheRobotPublishesHasARowShowingItsLimitsAndStart() {
    assertThat(ValuesRegister.parse(ARM_REGISTER + ARM_SWITCHES).switchProblems(ARM_PUBLISHED))
        .isEmpty();
  }

  @Test
  void aSwitchMissingStaleOrChangedSaysWhatToDo() {
    String register =
        (ARM_REGISTER + ARM_SWITCHES)
            .replace(
                "| `Arm/holdSeconds` | 0 to 5 s | 0 s |", "| `Arm/holdSeconds` | 0 to 3 s | 1 s |")
            .replace(
                "| `Arm/<camera>/enabled` | on or off | on | none | Leaves a camera out. |",
                "| `Arm/oldSwitch` | on or off | off | none | Gone. |");

    assertThat(ValuesRegister.parse(register).switchProblems(ARM_PUBLISHED))
        .containsExactly(
            "Add the switch Arm/<camera>/enabled to docs/values.md: a row | `Arm/<camera>/enabled`"
                + " | limits | starting value | source | why | in the table of switches",
            "docs/values.md line 41: the switch Arm/holdSeconds has the limits \"0 to 3 s\", but it"
                + " may be set from 0 to 5: update the cell",
            "docs/values.md line 41: the switch Arm/holdSeconds starts at \"1 s\", but the robot"
                + " starts it at 0: update the cell",
            "docs/values.md line 42: remove the switch Arm/oldSwitch: the robot publishes no such"
                + " switch");
  }

  @Test
  void aValueListedTwiceSaysWhere() {
    String register =
        ARM_REGISTER.replace(
            "| `kD` | chosen | 0.1 | top team (254) | Why: it damps. |",
            "| `kD` | chosen | 0.1 | top team (254) | Why: it damps. |\n"
                + "| `kD` | chosen | 0.2 | guess | Why: again. |");

    assertThat(problems(register))
        .containsExactly(
            "docs/values.md lines 20 and 21: Arm.Gains.kD is listed twice; remove one");
  }

  @Test
  void tablesTheCheckDoesntReadAndCodeExamplesAreLeftAlone() {
    String register =
        ARM_REGISTER
            + """

            ## An example

            ```markdown
            | Value | Kind | Template's starting value | Source | How |
            |---|---|---|---|---|
            | `example` | chosen | 1 | guess | Not a real row. |
            ```

            | Where | Number |
            |---|---|
            | `Arm` | 3 |
            """;

    assertThat(problems(register)).isEmpty();
  }

  @Test
  void tildeFencesAndCommentsAreLeftAloneToo() {
    String register =
        ARM_REGISTER.replace(
            "| `length` | as built | 0.8 m | datasheet (its drawing) | From CAD. |\n",
            """
            | `length` | as built | 0.8 m | datasheet (its drawing) | From CAD. |

            ~~~
            | `fenced` | chosen | 1 | guess | Not a row. |
            ~~~

            <!-- an old row, kept for now:
            | `commented` | chosen | 1 | guess | Not a row either. |
            -->
            """);

    assertThat(problems(register)).isEmpty();
  }

  @Test
  void anEscapedPipeIsPartOfItsCell() {
    assertThat(ValuesRegister.cells("| `kP` | chosen | 2 \\| 3 | guess | Why: a \\| b. |"))
        .containsExactly("`kP`", "chosen", "2 | 3", "guess", "Why: a | b.");

    String register =
        ARM_REGISTER.replace(
            "| `kP` | chosen | 2 | simulation | Why: it holds. |",
            "| `kP` | chosen | 2 \\| 3 | simulation | Why: either \\| both. |");
    assertThat(problems(register)).isEmpty();
  }

  @Test
  void aTableOfValuesNeedsItsRecordsHeadingAndItsColumns() {
    String register =
        """
        ## No record named

        | Value | Kind | Template's starting value | How |
        |---|---|---|---|
        | `length` | as built | 0.8 m | From CAD. |
        Length, not a row.
        """;

    assertThat(ValuesRegister.parse(register).problems(List.of(), Map.of()))
        .containsExactly(
            "docs/values.md line 3: a table of values must follow a ### heading that names its"
                + " record in backticks",
            "docs/values.md line 3: a table of values needs a Source column");
  }

  @Test
  void aRowStartsWithANameInBackticks() {
    String register = ARM_REGISTER.replace("| `length` |", "| length |");

    assertThat(problems(register))
        .contains(
            "docs/values.md line 13: a row must start with a name in backticks, like"
                + " `frameLength`, not \"length\"");
  }
}
