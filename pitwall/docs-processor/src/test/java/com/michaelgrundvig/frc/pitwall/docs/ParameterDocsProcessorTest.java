package com.michaelgrundvig.frc.pitwall.docs;

import static java.nio.charset.StandardCharsets.UTF_8;
import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.io.InputStream;
import java.net.URI;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;
import java.util.stream.Stream;
import javax.tools.DiagnosticCollector;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.SimpleJavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.StandardLocation;
import javax.tools.ToolProvider;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** The processor, run in the JDK's own compiler on small documented records. */
class ParameterDocsProcessorTest {
  @TempDir Path output;

  @Test
  void aRecordsJavadocBecomesItsDocumentation() throws IOException {
    var written =
        documented(
            """
            package robot;

            /**
             * An arm's control gains. Tuned on a running robot.
             *
             * <p>Commit what works.
             *
             * @param kP volts per radian of error
             * @param kD volts per radian
             *     per second
             */
            public record Gains(double kP, double kD) {}
            """);

    assertThat(written)
        .containsOnlyKeys("robot.Gains.json")
        .containsEntry(
            "robot.Gains.json",
            """
            {
              "summary": "An arm's control gains.",
              "text": "An arm's control gains. Tuned on a running robot.\\n\\nCommit what works.",
              "params": {
                "kP": "volts per radian of error",
                "kD": "volts per radian per second"
              }
            }
            """);
  }

  /** Nested records are compiled from source too, so their doc comments are there to read. */
  @Test
  void nestedRecordsAreDocumentedUnderTheirBinaryNames() throws IOException {
    var written =
        documented(
            """
            package robot;

            /**
             * A drivetrain.
             *
             * @param gains how it's controlled
             */
            public record Drive(Drive.Gains gains) {
              /**
               * How it's controlled.
               *
               * @param kP volts per meter per second of error
               */
              public record Gains(double kP) {}

              /** Not a record, but it holds one. */
              public static final class Holder {
                /** Deep inside. */
                public record Deep(double x) {}
              }
            }
            """);

    assertThat(written)
        .containsOnlyKeys(
            "robot.Drive.json", "robot.Drive$Gains.json", "robot.Drive$Holder$Deep.json");
    assertThat(written.get("robot.Drive$Gains.json"))
        .contains("\"summary\": \"How it's controlled.\"")
        .contains("\"kP\": \"volts per meter per second of error\"");
    assertThat(written.get("robot.Drive$Holder$Deep.json")).contains("\"params\": {}");
  }

  /** The page asks for what's missing, so a missing @param is left out, never made up. */
  @Test
  void onlyTheComponentsDocumentedHaveAMeaning() throws IOException {
    var written =
        documented(
            """
            package robot;

            /**
             * Limits.
             *
             * @param <T> what it limits
             * @param current amps
             * @param voltage
             * @param gone a value it no longer has
             * @see Object
             */
            public record Limits<T>(double current, double voltage, double speed, T what) {}
            """);

    assertThat(written.get("robot.Limits.json"))
        .endsWith(
            """
              "params": {
                "current": "amps"
              }
            }
            """);
  }

  @Test
  void markupBecomesPlainText() throws IOException {
    var written =
        documented(
            """
            package robot;

            /**
             * Limits {@code current} through the {@link Motor#stator stator}, see {@link #speed}.
             *
             * <p>Two kinds:
             * <ul>
             *   <li>the stator's, which caps <em>torque</em>
             *   <li>the supply's, which keeps the battery &amp; the radio up
             * </ul>
             * Both in amps: {@code Amps.of(40)}.
             *
             * @param current the most, {@code > 0}, through each {@link robot.Motor}
             * @param speed "fast", in m/s
             */
            public record Limits(double current, double speed) {}

            record Motor(double stator) {}
            """);

    assertThat(written.get("robot.Limits.json"))
        .contains("\"summary\": \"Limits `current` through the stator, see speed.\"")
        .contains(
            "\"text\": \"Limits `current` through the stator, see speed.\\n\\nTwo kinds:\\n"
                + "- the stator's, which caps torque\\n"
                + "- the supply's, which keeps the battery & the radio up\\n\\n"
                + "Both in amps: `Amps.of(40)`.\"")
        .contains("\"current\": \"the most, `> 0`, through each Motor\"")
        .contains("\"speed\": \"\\\"fast\\\", in m/s\"");
  }

  /** A Markdown comment, written with three slashes, is read like a traditional one. */
  @Test
  void aMarkdownCommentIsDocumentedToo() throws IOException {
    var written =
        documented(
            """
            package robot;

            /// Gains for [Drive], set with `kP(16)`.
            ///
            /// Tuned live.
            ///
            /// @param kP volts per [meter][Units#Meters] of error
            public record Gains(double kP) {}
            """);

    assertThat(written.get("robot.Gains.json"))
        .isEqualTo(
            """
            {
              "summary": "Gains for Drive, set with `kP(16)`.",
              "text": "Gains for Drive, set with `kP(16)`.\\n\\nTuned live.",
              "params": {
                "kP": "volts per meter of error"
              }
            }
            """);
  }

  /**
   * What JSON gives a meaning is escaped; everything else goes as it is, in UTF-8, but control
   * characters, which are dropped.
   */
  @Test
  void theTextIsEscapedAsJsonRequires() throws IOException {
    var written =
        documented(
            """
            package robot;

            /**
             * Symbols.
             *
             * @param heading the "zero" heading, in ° from C:\\field
             * @param bell rings \\u0007 once
             */
            public record Symbols(double heading, double bell) {}
            """);

    assertThat(written.get("robot.Symbols.json"))
        .contains("\"heading\": \"the \\\"zero\\\" heading, in ° from C:\\\\field\"")
        .contains("\"bell\": \"rings once\"");
    assertThat(ParameterDocsProcessor.quoted("a \"b\" \\c\nd\u0007"))
        .as("JSON's own escapes, and any control character")
        .isEqualTo("\"a \\\"b\\\" \\\\c\\nd\\u0007\"");
  }

  @Test
  void theOptionDocumentsOnlyTheRecordsItsAnnotationMarks() throws IOException {
    String sources =
        """
        package robot;

        import java.lang.annotation.ElementType;
        import java.lang.annotation.Retention;
        import java.lang.annotation.RetentionPolicy;
        import java.lang.annotation.Target;

        /** Marks configuration. */
        @Retention(RetentionPolicy.CLASS)
        @Target(ElementType.TYPE)
        @interface Config {}

        /** Marked. */
        @Config
        public record Marked(double a) {
          /** Marked too. */
          @Config
          public record Inner(double b) {}

          /** Not marked. */
          public record Unmarked(double c) {}
        }

        /** Marked, but not a record. */
        @Config
        final class NotARecord {}

        /** Not marked. */
        record Plain(double d) {}
        """;

    assertThat(documented(sources, "-Apitwall.docs.records=robot.Config"))
        .containsOnlyKeys("robot.Marked.json", "robot.Marked$Inner.json");
    assertThat(documented(sources))
        .as("without the option, every record")
        .containsOnlyKeys(
            "robot.Marked.json",
            "robot.Marked$Inner.json",
            "robot.Marked$Unmarked.json",
            "robot.Plain.json");
  }

  /**
   * A compile with nothing marked (Gradle recompiling one changed file, say) still knows the
   * option: no warning, which the robot's {@code -Werror} would make a failure, and no files.
   */
  @Test
  void theOptionWithNothingMarkedIsQuiet() throws IOException {
    var written =
        documented(
            """
            package robot;

            /** Marks configuration. */
            public @interface Config {}

            /** Not marked. */
            record Plain(double d) {}
            """,
            "-Apitwall.docs.records=robot.Config");

    assertThat(written).isEmpty();
  }

  @Test
  void anOptionNamingNoAnnotationFailsSayingWhereToFixIt() throws IOException {
    String source =
        """
        package robot;

        /** Not an annotation. */
        public record Plain(double d) {}
        """;

    Map<String, String> wrongs =
        Map.of(
            "robot.Missing", "names nothing the compiler can find",
            "robot.Plain", "names a record, not an annotation");
    for (Map.Entry<String, String> wrong : wrongs.entrySet()) {
      String name = wrong.getKey();
      Compiled compiled = compile(source, "-Apitwall.docs.records=" + name);

      assertThat(compiled.ok()).as(name).isFalse();
      assertThat(compiled.messages()).as(name).hasSize(1);
      assertThat(compiled.messages().get(0))
          .startsWith(
              "ERROR: -Apitwall.docs.records="
                  + name
                  + " "
                  + wrong.getValue()
                  + ". Fix the name in build.gradle");
      assertThat(compiled.written()).as(name).isEmpty();
    }
  }

  @Test
  void anUndocumentedRecordGetsNoFile() throws IOException {
    var written =
        documented(
            """
            package robot;

            public record Bare(double x) {
              /* Not a doc comment. */
              public record Commented(double y) {}

              /** */
              public record Empty(double z) {}
            }
            """);

    assertThat(written).isEmpty();
  }

  /**
   * Gradle finds it as an isolating processor, and the compiler finds it at all, by these files.
   */
  @Test
  void itIsRegisteredWithTheCompilerAndWithGradle() throws IOException {
    assertThat(resource("META-INF/services/javax.annotation.processing.Processor"))
        .isEqualTo(ParameterDocsProcessor.class.getName() + "\n");
    assertThat(resource("META-INF/gradle/incremental.annotation.processors"))
        .isEqualTo(ParameterDocsProcessor.class.getName() + ",isolating\n");
  }

  /**
   * What compiling a source with the processor gave.
   *
   * @param ok whether it compiled
   * @param messages the compiler's messages, each as {@code "ERROR: ..."}
   * @param written what the processor wrote, each file by its name
   */
  private record Compiled(boolean ok, List<String> messages, Map<String, String> written) {}

  /**
   * Compiles the source with the processor alone, as strictly as the robot program compiles (any
   * warning fails it), and returns what the processor wrote, checking that nothing went wrong.
   */
  private Map<String, String> documented(String source, String... options) throws IOException {
    Compiled compiled = compile(source, options);
    assertThat(compiled.ok()).as(compiled.messages().toString()).isTrue();
    assertThat(compiled.messages()).isEmpty();
    return compiled.written();
  }

  private Compiled compile(String source, String... options) throws IOException {
    JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
    var diagnostics = new DiagnosticCollector<JavaFileObject>();
    Path classes = Files.createTempDirectory(output, "classes");
    boolean ok;
    try (StandardJavaFileManager files =
        compiler.getStandardFileManager(diagnostics, Locale.ROOT, UTF_8)) {
      files.setLocationFromPaths(StandardLocation.CLASS_OUTPUT, List.of(classes));
      List<String> arguments = new ArrayList<>(List.of("-proc:only", "-Werror"));
      arguments.addAll(List.of(options));
      var task = compiler.getTask(null, files, diagnostics, arguments, null, List.of(file(source)));
      task.setProcessors(List.of(new ParameterDocsProcessor()));
      ok = task.call();
    }
    List<String> messages =
        diagnostics.getDiagnostics().stream()
            .map(message -> message.getKind() + ": " + message.getMessage(Locale.ROOT))
            .toList();
    Map<String, String> written = new TreeMap<>();
    Path folder = classes.resolve(ParameterDocsProcessor.FOLDER);
    if (Files.isDirectory(folder)) {
      try (Stream<Path> paths = Files.list(folder)) {
        for (Path path : paths.toList()) {
          written.put(path.getFileName().toString(), Files.readString(path));
        }
      }
    }
    return new Compiled(ok, messages, written);
  }

  /** A source file in memory, named for its first public type as the compiler requires. */
  private static JavaFileObject file(String source) {
    String name = source.replaceAll("(?s).*?public (record|final class|@interface) (\\w+).*", "$2");
    return new SimpleJavaFileObject(
        URI.create("string:///robot/" + name + ".java"), JavaFileObject.Kind.SOURCE) {
      @Override
      public CharSequence getCharContent(boolean ignoreEncodingErrors) {
        return source;
      }
    };
  }

  private static String resource(String name) throws IOException {
    try (InputStream in = ParameterDocsProcessor.class.getClassLoader().getResourceAsStream(name)) {
      assertThat(in).as(name).isNotNull();
      return new String(in.readAllBytes(), UTF_8);
    }
  }
}
