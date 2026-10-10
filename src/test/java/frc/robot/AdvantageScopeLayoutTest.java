package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.bridges.AdvantageScopeAssets;
import frc.robot.AdvantageScopeLayoutCheck.Source;
import io.avaje.jsonb.Jsonb;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

/**
 * The AdvantageScope layout (layouts/advantagescope.json, AdvantageScope's File > Export Layout)
 * against the code: its format is the release the template pairs with, and every robot model it
 * names is one the build generates. That every value it reads is logged, of the type it says, is
 * checked against a real run's log ({@link AdvantageScopeLayoutEndToEndTest}).
 */
class AdvantageScopeLayoutTest {
  static final Path LAYOUT =
      Path.of(System.getProperty("frc.projectDir", "."), "layouts", "advantagescope.json");

  /** Where the build writes AdvantageScope's assets. */
  static final Path ASSETS = Path.of(System.getProperty("frc.projectDir", "."), "ascope_assets");

  /**
   * The robot models the build generated, by the names AdvantageScope lists them under: each
   * folder's config.json that the build wrote (it says so beside it, in GENERATED.md). The build
   * writes them before the tests run.
   */
  @SuppressWarnings("unchecked")
  static Set<String> models() throws IOException {
    Set<String> models = new TreeSet<>();
    try (Stream<Path> folders = Files.list(ASSETS)) {
      for (Path folder : folders.filter(f -> Files.exists(f.resolve("GENERATED.md"))).toList()) {
        Map<String, Object> config =
            (Map<String, Object>)
                Jsonb.instance()
                    .type(Object.class)
                    .fromJson(Files.readString(folder.resolve("config.json")));
        models.add(String.valueOf(config.get("name")));
      }
    }
    return models;
  }

  @Test
  void theLayoutIsForTheReleaseTheAssetsAreWrittenFor() throws IOException {
    String layout = Files.readString(LAYOUT);

    assertThat("v" + AdvantageScopeLayoutCheck.version(layout))
        .isEqualTo(AdvantageScopeAssets.ADVANTAGESCOPE);
  }

  @Test
  void everySourceIsAKindItsTabShowsAndEveryModelIsGenerated() throws IOException {
    List<Source> sources = AdvantageScopeLayoutCheck.sources(Files.readString(LAYOUT));
    // As if everything it reads were logged as it says: here only the layout itself is checked.
    Map<String, String> logged = new LinkedHashMap<>();
    sources.forEach(source -> logged.put(source.key(), source.type()));

    assertThat(sources).as("sources found in the layout").hasSizeGreaterThan(20);
    assertThat(sources)
        .extracting(Source::tabType)
        .contains("3D Field", "2D Field", "Swerve", "Line Graph");
    assertThat(models()).as("the models the build generated").isNotEmpty();
    assertThat(AdvantageScopeLayoutCheck.problems(sources, logged, models())).isEmpty();
  }

  @Test
  void theCheckNamesWhatIsWrong() {
    List<Source> sources =
        List.of(
            new Source(
                "3D", "robot", "/RealOutputs/Pose3d", "Pose3d", Map.of("model", "Old"), "3D Field"),
            new Source("3D", "vision", "/RealOutputs/Tags", "Pose3d[]", Map.of(), "3D Field"),
            new Source("3D", "trajectory", "/RealOutputs/Path", "Pose2d[]", Map.of(), "3D Field"),
            new Source("3D", "component", "/RealOutputs/Wheels", "Pose3d[]", Map.of(), "3D Field"),
            new Source("3D", "robo", "/RealOutputs/Pose3d", "Pose3d", Map.of(), "3D Field"),
            new Source("Graph", "smooth", "/RealOutputs/Mode", "String", Map.of(), "Line Graph"));
    Map<String, String> logged =
        Map.of(
            "/RealOutputs/Pose3d", "Pose3d",
            "/RealOutputs/Tags", "Pose2d[]",
            "/RealOutputs/Path", "Pose2d[]",
            "/RealOutputs/Wheels", "Pose3d[]",
            "/RealOutputs/Mode", "String");

    List<String> problems =
        AdvantageScopeLayoutCheck.problems(sources, logged, Set.of("sim (schematic)"));

    assertThat(problems)
        .containsExactly(
            "3D, robot /RealOutputs/Pose3d: the model Old isn't one the build generates (sim"
                + " (schematic))",
            "3D, vision /RealOutputs/Tags: /RealOutputs/Tags is logged as Pose2d[], not Pose3d[]",
            "3D, component /RealOutputs/Wheels: hangs from a robot, but follows none, so nothing"
                + " shows it",
            "3D, robo /RealOutputs/Pose3d: a 3D Field tab has no source of kind robo",
            "Graph, smooth /RealOutputs/Mode: a smooth can't show a String");
  }

  @Test
  void aLogsTypesAreNamedAsAdvantageScopeNamesThem() {
    assertThat(AdvantageScopeLayoutCheck.advantageScopeType("struct:Pose3d[]"))
        .isEqualTo("Pose3d[]");
    assertThat(AdvantageScopeLayoutCheck.advantageScopeType("double")).isEqualTo("Number");
    assertThat(AdvantageScopeLayoutCheck.advantageScopeType("int64")).isEqualTo("Number");
    assertThat(AdvantageScopeLayoutCheck.advantageScopeType("string")).isEqualTo("String");
    assertThat(AdvantageScopeLayoutCheck.advantageScopeType("string[]")).isEqualTo("StringArray");
    assertThat(AdvantageScopeLayoutCheck.advantageScopeType("boolean")).isEqualTo("Boolean");
    assertThat(AdvantageScopeLayoutCheck.advantageScopeType("double[]")).isEqualTo("NumberArray");
    assertThat(AdvantageScopeLayoutCheck.advantageScopeType("boolean[]")).isEqualTo("BooleanArray");
    assertThat(AdvantageScopeLayoutCheck.advantageScopeType("raw")).isEqualTo("Raw");
  }
}
