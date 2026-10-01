package com.michaelgrundvig.frc.coprocessor.tools;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

/**
 * The recipe hash covers what builds the agent, and nothing outside it may change the agent: its
 * build applies only files the recipe covers, and the robot's build files (which the recipe leaves
 * out, so a GradleRIO update doesn't ask for a new image) configure no other project.
 */
class RecipeBoundaryTest {
  private static final Path PROJECT = Path.of(System.getProperty("frc.projectDir", "../.."));

  private static final Pattern APPLY_FROM =
      Pattern.compile("apply from:\\s*(?:rootProject\\.file\\()?\"([^\"]+)\"");

  /** What reaches into another project's configuration. */
  private static final Pattern CROSS_PROJECT =
      Pattern.compile(
          "\\b(allprojects|subprojects|configure\\s*\\(|beforeProject|afterProject"
              + "|beforeEvaluate|afterEvaluate|evaluationDependsOn)\\b");

  private static List<String> code(Path file) throws IOException {
    List<String> lines = new ArrayList<>();
    for (String line : Files.readAllLines(file, StandardCharsets.UTF_8)) {
      String code = line.strip();
      if (!code.startsWith("//") && !code.isEmpty()) {
        lines.add(code);
      }
    }
    return lines;
  }

  @Test
  void theAgentsBuildAppliesOnlyFilesTheRecipeCovers() throws IOException {
    for (String build :
        List.of("coprocessor/agent/build.gradle", "coprocessor/common/build.gradle")) {
      for (String line : code(PROJECT.resolve(build))) {
        Matcher applied = APPLY_FROM.matcher(line);
        if (applied.find()) {
          assertThat(RecipeHash.PATHS)
              .as("%s applies %s, which the recipe hash doesn't cover", build, applied.group(1))
              .contains(applied.group(1));
        }
      }
    }
  }

  @Test
  void theRobotsBuildFilesConfigureNoOtherProject() throws IOException {
    List<Path> files =
        new ArrayList<>(
            List.of(PROJECT.resolve("build.gradle"), PROJECT.resolve("settings.gradle")));
    try (Stream<Path> gradle = Files.list(PROJECT.resolve("gradle"))) {
      gradle
          .filter(file -> file.getFileName().toString().endsWith(".gradle"))
          .filter(file -> !RecipeHash.PATHS.contains("gradle/" + file.getFileName()))
          .forEach(files::add);
    }
    for (Path file : files) {
      for (String line : code(file)) {
        assertThat(CROSS_PROJECT.matcher(line).find())
            .as("%s reaches into other projects: %s", PROJECT.relativize(file), line)
            .isFalse();
        if (line.contains(":coprocessor-")) {
          assertThat(line)
              .as("%s touches a coprocessor project: %s", PROJECT.relativize(file), line)
              .matches(
                  "include \"coprocessor-(common|agent)\""
                      + "|project\\(\":coprocessor-(common|agent)\"\\)\\.projectDir = .*"
                      + "|(implementation|coprocessorTools) project\\(\":coprocessor-common\"\\)"
                      + "|dependsOn \":coprocessor-agent:runtimeLibraries\"");
        }
      }
    }
  }
}
