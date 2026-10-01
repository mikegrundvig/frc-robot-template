package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.fields.Fields;

/** The fields the study runs on: WPILib's, or a layout file published before WPILib has it. */
class TagLayoutsTest {
  @TempDir Path folder;

  @Test
  void wpilibsFieldsByNameWithTheSymmetryTheirTagsKeep() {
    FieldModel rebuilt = TagLayouts.named("FRC_2026_REBUILT_WELDED");
    FieldModel crescendo = TagLayouts.named("FRC_2024_CRESCENDO");

    assertThat(rebuilt.tags()).hasSize(32);
    assertThat(rebuilt.symmetry()).isEqualTo(Symmetry.ROTATED);
    assertThat(crescendo.symmetry()).isEqualTo(Symmetry.MIRRORED);
    assertThatThrownBy(() -> TagLayouts.named("FRC_2027_UNKNOWN"))
        .hasMessageContaining("no field named FRC_2027_UNKNOWN")
        .hasMessageContaining("FRC_2026_REBUILT_WELDED");
  }

  @Test
  void aLayoutFileInWpilibsForm() throws IOException {
    Path file = folder.resolve("rebuilt.json");
    Files.writeString(file, Fields.FRC_2026_REBUILT_WELDED.loadField().getJson());

    FieldModel field = TagLayouts.read(file);

    assertThat(field.tags()).isEqualTo(TagLayouts.named("FRC_2026_REBUILT_WELDED").tags());
  }

  /** Two tags facing each other across a 16 x 8 m field: a mirrored field, as FIRST writes it. */
  @Test
  void aLayoutFileInFirstsForm() throws IOException {
    Path file = folder.resolve("2027-field.json");
    Files.writeString(
        file,
        """
        {
          "tags": [
            {"ID": 1, "pose": {"translation": {"x": 0.5, "y": 2, "z": 1},
              "rotation": {"quaternion": {"W": 1, "X": 0, "Y": 0, "Z": 0}}}},
            {"ID": 2, "pose": {"translation": {"x": 15.5, "y": 2, "z": 1},
              "rotation": {"quaternion": {"W": 0, "X": 0, "Y": 0, "Z": 1}}}}
          ],
          "field": {"length": 16, "width": 8}
        }
        """);

    FieldModel field = TagLayouts.read(file);

    assertThat(field.name()).isEqualTo("2027-field.json");
    assertThat(field.length()).isEqualTo(16);
    assertThat(field.width()).isEqualTo(8);
    assertThat(field.tags()).containsOnlyKeys(1, 2);
    assertThat(field.tag(2).orElseThrow().getRotation().getZ()).isCloseTo(Math.PI, within(1e-9));
    assertThat(field.symmetry()).isEqualTo(Symmetry.MIRRORED);
  }

  @Test
  void aFileThatIsntALayoutSaysSo() throws IOException {
    Path notJson = Files.writeString(folder.resolve("a.json"), "tags: none");
    Path notALayout = Files.writeString(folder.resolve("b.json"), "{\"robots\": []}");
    Path badTag =
        Files.writeString(folder.resolve("c.json"), "{\"tags\": [{\"ID\": 1}], \"field\": {}}");

    assertThatThrownBy(() -> TagLayouts.read(notJson)).hasMessageContaining("isn't JSON");
    assertThatThrownBy(() -> TagLayouts.read(notALayout))
        .hasMessageContaining("isn't a tag layout");
    assertThatThrownBy(() -> TagLayouts.read(badTag)).hasMessageContaining("a tag's pose");
  }
}
