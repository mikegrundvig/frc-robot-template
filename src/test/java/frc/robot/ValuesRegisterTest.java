package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.LiveConfig;
import frc.robot.ValuesRegister.CodeRecord;
import frc.robot.ValuesRegister.Component;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
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

  private static final List<CodeRecord> RECORDS = ValuesRegister.scan();

  /** Each mechanism's live tuning: the paths of the values it tunes, by the tuning's name. */
  private static Map<String, List<String>> tuning() {
    RobotConfig robot = Robots.current();
    Map<String, List<String>> tuning = new LinkedHashMap<>();
    for (LiveConfig<?> live :
        List.of(Drive.tuning(robot.drivetrain()), Vision.tuning(robot.vision()))) {
      tuning.put(live.name(), live.knobs().stream().map(LiveConfig.Knob::path).toList());
    }
    return tuning;
  }

  @Test
  void theRegisterHasARowForEveryConfigurationValueAndNoOther() throws IOException {
    ValuesRegister register = ValuesRegister.parse(Files.readString(REGISTER));

    assertThat(register.problems(RECORDS, tuning())).isEmpty();
  }

  @Test
  void theScanFindsEveryConfigurationRecordAndWhatEachValueHolds() {
    assertThat(RECORDS)
        .extracting(CodeRecord::name)
        .contains("RobotConfig", "Body", "SwerveConfig.Limits", "ImuCrossCheck.Settings")
        .contains("ImuReadings") // listed in the register as not configuration
        .doesNotHaveDuplicates();
    assertThat(record("RobotConfig").components())
        .contains(
            new Component("name", null),
            new Component("body", "Body"),
            new Component("drivetrainWiring", "SwerveWiring"), // in an Optional
            new Component("cameras", "CameraConfig")); // in a List
    assertThat(record("SensorErrors").components())
        .contains(
            new Component("cameraMountErrors", null), // a map of transforms, no record
            new Component("gyro", "SensorErrors.GyroErrors"));
  }

  @Test
  void nestedRecordsAreNamedWithTheirOuterRecord() {
    assertThat(ValuesRegister.registerName("frc.robot.lib.config.SwerveConfig$Limits"))
        .isEqualTo("SwerveConfig.Limits");
    assertThat(record("SensorErrors.GyroErrors").className())
        .isEqualTo("frc.robot.sim.SensorErrors$GyroErrors");
  }

  private static CodeRecord record(String name) {
    return RECORDS.stream().filter(r -> r.name().equals(name)).findFirst().orElseThrow();
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

  private static final Map<String, List<String>> ARM_TUNING =
      Map.of("arm", List.of("gains.kP", "gains.kD"));

  private static final String ARM_REGISTER =
      """
      # Values

      | Kind | Meaning |
      |---|---|
      | chosen | Not a table the check reads. |

      ## The arm

      ### `Arm`

      | Value | Kind | Today | Source | How to measure it, or why |
      |---|---|---|---|---|
      | `length` | as built | 0.8 m | datasheet (its drawing) | From CAD. |

      ### `Arm.Gains`

      | Value | Kind | Today | Source | How to measure it, or why |
      |---|---|---|---|---|
      | `kP` | chosen | 2 | simulation | Why: it holds. |
      | `kD` | chosen | 0.1 | top team (254) | Why: it damps. |

      ## Tuning limits

      ### `arm` tuning

      | Knob | Limits | Source | Why |
      |---|---|---|---|
      | `gains.kP` | 0 to 10 | unknown | Room to tune. |
      | `gains.kD` | 0 to 1 | unknown | Room to tune. |

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
            .replace("| `length` | as built | 0.8 m | datasheet (its drawing) | From CAD. |\n", "")
            .replace("### `Arm.Gains`", "### `Arm.Gainz`");

    assertThat(problems(register))
        .containsExactly(
            "docs/values.md line 18: remove the section for Arm.Gainz: no configuration record is"
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
  void aValueMissingFromItsRecordsTableSaysWhichAndWhere() {
    String register =
        ARM_REGISTER.replace("| `kD` | chosen | 0.1 | top team (254) | Why: it damps. |\n", "");

    assertThat(problems(register))
        .containsExactly(
            "Add Arm.Gains.kD to docs/values.md: a row | `kD` | kind | today | source | how | in"
                + " Arm.Gains's table");
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
            .replace("| `gains.kP` | 0 to 10 | unknown |", "| `gains.kP` | 0 to 10 | |");

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
            .replace("| `gains.kD` | 0 to 1 | unknown | Room to tune. |\n", "")
            .replace("| `gains.kP` |", "| `gains.kI` | 0 to 1 | unknown | Gone. |\n| `gains.kP` |");
    Map<String, List<String>> tuning =
        Map.of("arm", List.of("gains.kP", "gains.kD"), "wrist", List.of("gains.kP"));

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
            | Value | Kind | Today | Source | How |
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
  void aTableOfValuesNeedsItsRecordsHeadingAndItsColumns() {
    String register =
        """
        ## No record named

        | Value | Kind | Today | How |
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
