package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.io.CleanupMode;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.datalog.DataLogReader;
import org.wpilib.datalog.DataLogRecord;

/**
 * AdvantageKit replay, end to end, with Pitwall in the loop. The real robot program records a
 * session in the simulator ({@link ReplaySessionRobot}): the Driver Station set from the
 * simulator's controls and a joystick driven from the page, a tuning change from Pitwall's page, a
 * WPILib tunable changed through Pitwall's tee, an action started from the page, which drives the
 * robot, and an experiment on the steering, scored from its trace. Then it replays its own log.
 * Every output the session recorded must come out of the replay the same, at the same moments,
 * because every one of those changes was logged as an input.
 *
 * <p>Each run is its own process: AdvantageKit's logger starts once per program, and a replay ends
 * the program when the log runs out.
 */
@EndToEndTest
class ReplayEndToEndTest {
  /** Where the robot programs' coverage goes, beside the test's; build.gradle reports it. */
  static final String COVERAGE = "end-to-end.exec";

  private static final List<String> OUTPUTS =
      List.of(
          "Session/Gain",
          "Session/Operator",
          "Session/SteerKp",
          "Session/Action",
          "Session/Experiment",
          "Session/ExperimentRise",
          "Drive/Pose",
          "Field/Alliance",
          "Field/Placement",
          "Field/Zones/Trench");

  @TempDir(cleanup = CleanupMode.ON_SUCCESS) // a failed run keeps its logs to look at
  Path dir;

  /** An output in a log: its name without the table, and its type. */
  record Entry(String name, String type) {}

  /** One logged value and when. Values are numbers, text, or raw bytes as hex. */
  record Sample(long timestamp, Object value) {}

  @Test
  @Timeout(value = 4, unit = TimeUnit.MINUTES) // two runs of the program, each allowed 90 s
  void aReplayReproducesTheSessionIncludingEveryChangeMadeThroughPitwall() throws Exception {
    run("record", Map.of("frc.simMode", "SIM", "pitwall.port", "0"), Map.of());
    Path recorded = onlyLog();
    run("replay", Map.of("frc.simMode", "REPLAY"), Map.of("AKIT_LOG_PATH", recorded.toString()));
    Path replayed =
        recorded.resolveSibling(
            recorded.getFileName().toString().replace(".wpilog", "_sim.wpilog"));

    Map<String, List<Sample>> original = outputs(recorded, "RealOutputs/");
    Map<String, List<Sample>> replay = outputs(replayed, "ReplayOutputs/");

    // The session happened: each change took effect while recording.
    assertThat(values(original, "Session/Gain")).contains(1.0, 2.5);
    assertThat(values(original, "Session/Operator")).as("the page's stick").contains(0.0, 0.5);
    assertThat(values(original, "Session/SteerKp")).contains(16.0, 18.5);
    assertThat(values(original, "Session/Action")).contains("READY", "RUNNING", "ENDED");
    assertThat(values(original, "Session/Experiment")).contains("NONE", "RUNNING", "ENDED");
    assertThat(values(original, "Session/ExperimentRise"))
        .as("the steering's rise, scored from its trace")
        .anySatisfy(rise -> assertThat((Double) rise).isBetween(0.02, 0.3));
    assertThat(values(original, "Field/Alliance")).contains("Red 1");
    assertThat(values(original, "Field/Placement")).contains("Ready on Left: 0 cm, 0° off");
    assertThat(values(original, "Field/Zones/Trench")).as("drove under it").contains(true);

    // AdvantageKit's reader reads each loop up to the next loop's timestamp, so a replay runs every
    // loop of the log except the last.
    long lastReplayed =
        replay.values().stream().flatMap(List::stream).mapToLong(Sample::timestamp).max().orElse(0);
    long notReplayed =
        original.values().stream()
            .flatMap(List::stream)
            .mapToLong(Sample::timestamp)
            .filter(time -> time > lastReplayed)
            .distinct()
            .count();
    assertThat(notReplayed).as("recorded loops the replay didn't run").isLessThanOrEqualTo(1);

    // AdvantageKit writes an output when it changes, so each series is the output's changes: the
    // replay must change it at the same moments, to the same values.
    for (String output : OUTPUTS) {
      List<Sample> expected =
          original.getOrDefault(output, List.of()).stream()
              .filter(sample -> sample.timestamp() <= lastReplayed)
              .toList();
      assertThat(expected).as(output + " recorded").isNotEmpty();
      List<Sample> actual = replay.getOrDefault(output, List.of());
      assertThat(actual)
          .as(output + " replayed: " + firstDifference(expected, actual))
          .isEqualTo(expected);
    }
  }

  /** Runs the session robot in its own process, in the test's temporary directory. */
  private void run(String name, Map<String, String> properties, Map<String, String> environment)
      throws IOException, InterruptedException {
    List<String> command = new ArrayList<>();
    command.add(ProcessHandle.current().info().command().orElseThrow());
    for (String argument : ManagementFactory.getRuntimeMXBean().getInputArguments()) {
      if (!argument.startsWith("-javaagent")) {
        command.add(argument);
      } else if (argument.contains("jacoco")) {
        command.add(coverageOfItsOwn(argument));
      }
    }
    properties.forEach((key, value) -> command.add("-D" + key + "=" + value));
    command.add("-cp");
    command.add(System.getProperty("java.class.path"));
    command.add(ReplaySessionRobot.class.getName());

    Path output = dir.resolve(name + ".out");
    ProcessBuilder builder =
        new ProcessBuilder(command).directory(dir.toFile()).redirectErrorStream(true);
    builder.redirectOutput(output.toFile());
    builder.environment().remove("HALSIM_EXTENSIONS"); // replay refuses simulation extensions
    builder.environment().putAll(environment);
    Process process = builder.start();
    boolean finished = process.waitFor(90, TimeUnit.SECONDS);
    if (!finished) {
      process.destroyForcibly();
    }
    assertThat(finished && process.exitValue() == 0)
        .as(name + " run finished cleanly; its output:\n" + tail(output))
        .isTrue();
  }

  /**
   * The test's coverage agent (JaCoCo's), for a robot program: it writes beside the test's own
   * coverage, to {@value #COVERAGE}, which the coverage report counts too.
   */
  private static String coverageOfItsOwn(String agent) {
    Matcher destination = Pattern.compile("destfile=([^,]*)").matcher(agent);
    if (!destination.find()) {
      return agent;
    }
    Path own = Path.of(destination.group(1)).toAbsolutePath().normalize().resolveSibling(COVERAGE);
    return destination.replaceFirst(Matcher.quoteReplacement("destfile=" + own));
  }

  private Path onlyLog() throws IOException {
    try (Stream<Path> logs = Files.list(dir.resolve("logs"))) {
      List<Path> found = logs.filter(p -> p.toString().endsWith(".wpilog")).toList();
      assertThat(found).as("the recorded log").hasSize(1);
      return found.get(0);
    }
  }

  /** Every output under a log's output table, by name without the table. */
  private static Map<String, List<Sample>> outputs(Path log, String table) throws IOException {
    DataLogReader reader = new DataLogReader(ByteBuffer.wrap(Files.readAllBytes(log)));
    assertThat(reader.isValid()).as("a valid log: " + log).isTrue();
    Map<Integer, Entry> entries = new LinkedHashMap<>();
    Map<String, List<Sample>> samples = new LinkedHashMap<>();
    for (DataLogRecord record : reader) {
      if (record.isStart()) {
        var start = record.getStartData();
        String name = start.name.startsWith("/") ? start.name.substring(1) : start.name;
        if (name.startsWith(table)) {
          entries.put(start.entry, new Entry(name.substring(table.length()), start.type));
        }
        continue;
      }
      if (record.isControl()) {
        continue;
      }
      Entry entry = entries.get(record.getEntry());
      if (entry == null) {
        continue;
      }
      Object value =
          switch (entry.type()) {
            case "double" -> record.getDouble();
            case "string" -> record.getString();
            case "boolean" -> record.getBoolean();
            default -> HexFormat.of().formatHex(record.getRaw());
          };
      samples
          .computeIfAbsent(entry.name(), n -> new ArrayList<>())
          .add(new Sample(record.getTimestamp(), value));
    }
    return samples;
  }

  private static String firstDifference(List<Sample> expected, List<Sample> actual) {
    for (int i = 0; i < Math.min(expected.size(), actual.size()); i++) {
      if (!expected.get(i).equals(actual.get(i))) {
        return "sample "
            + i
            + " of "
            + expected.size()
            + " recorded, "
            + actual.size()
            + " replayed: "
            + expected.get(i)
            + " vs "
            + actual.get(i);
      }
    }
    return expected.size() + " recorded, " + actual.size() + " replayed";
  }

  private static List<Object> values(Map<String, List<Sample>> outputs, String name) {
    return outputs.getOrDefault(name, List.of()).stream().map(Sample::value).distinct().toList();
  }

  private static String tail(Path output) {
    try {
      List<String> lines = Files.readAllLines(output);
      return String.join("\n", lines.subList(Math.max(0, lines.size() - 40), lines.size()));
    } catch (IOException e) {
      return "(no output: " + e + ")";
    }
  }
}
