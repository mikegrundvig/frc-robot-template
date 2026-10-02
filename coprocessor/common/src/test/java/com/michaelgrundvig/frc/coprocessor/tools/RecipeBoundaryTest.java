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
 * The recipe hash covers what makes the agent's jar, and nothing outside it may change the jar: the
 * jar's rules are in {@code coprocessor/agent/jar.gradle}, which the recipe covers, and not in the
 * agent's build file, which it leaves out; the coprocessor projects' builds apply only files the
 * recipe covers, and the checks every project applies; and the robot's build files (which the
 * recipe leaves out too, so a GradleRIO update doesn't ask for a new image) configure no other
 * project.
 */
class RecipeBoundaryTest {
  private static final Path PROJECT = Path.of(System.getProperty("frc.projectDir", "../.."));

  /**
   * Applied by every project, the coprocessor projects too, and left out of the recipe: checks
   * (static analysis, formatting, tests, coverage) that change nothing a project compiles to.
   */
  private static final String CHECKS = "gradle/quality.gradle";

  /** What configures a jar. */
  private static final Pattern JAR =
      Pattern.compile("tasks\\.named\\(\"jar\"\\)|tasks\\.jar\\b|^jar\\s*\\{|withType\\(Jar\\)");

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
        if (applied.find() && !applied.group(1).equals(CHECKS)) {
          assertThat(RecipeHash.PATHS)
              .as("%s applies %s, which the recipe hash doesn't cover", build, applied.group(1))
              .contains(applied.group(1));
        }
      }
    }
  }

  @Test
  void theAgentsJarIsMadeOnlyByFilesTheRecipeCovers() throws IOException {
    assertThat(RecipeHash.PATHS).contains("coprocessor/agent/jar.gradle");
    for (String build :
        List.of("coprocessor/agent/build.gradle", "coprocessor/common/build.gradle", CHECKS)) {
      for (String line : code(PROJECT.resolve(build))) {
        assertThat(JAR.matcher(line).find())
            .as(
                "%s configures a jar, which belongs in coprocessor/agent/jar.gradle: %s",
                build, line)
            .isFalse();
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
          .filter(file -> !("gradle/" + file.getFileName()).equals(CHECKS))
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
