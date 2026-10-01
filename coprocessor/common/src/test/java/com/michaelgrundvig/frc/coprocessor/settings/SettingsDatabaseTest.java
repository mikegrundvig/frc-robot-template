package com.michaelgrundvig.frc.coprocessor.settings;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.coprocessor.json.JsonValue;
import java.nio.file.Path;
import java.sql.Connection;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** Settings out of PhotonVision's database and back in, through files, losing nothing. */
class SettingsDatabaseTest {
  @TempDir Path dir;

  private Settings read(Path file) throws SQLException {
    try (Connection connection = PhotonVisionDatabase.open(file)) {
      return SettingsDatabase.read(connection);
    }
  }

  @Test
  void everyRowOfEveryTableIsRead() throws SQLException {
    Settings settings = read(PhotonVisionDatabase.configured(dir));
    assertThat(settings.userVersion()).isEqualTo(2);
    assertThat(settings.rows())
        .extracting(row -> row.table() + "/" + row.key())
        .containsExactly(
            "cameras/3e5a7c41-9d2b-4f60-8a1e-5b7c9d0e2f13",
            "cameras/front right",
            "global/fieldLayout",
            "global/hardwareConfig",
            "global/hardwareSettings",
            "global/networkConfig",
            "global/neuralNetworkProperties");
    SettingsRow camera = settings.rows("cameras").get(1);
    assertThat(camera.columns().members().keySet())
        .containsExactly("config_json", "drivermode_json", "pipeline_jsons", "otherpaths_json");
    assertThat(camera.columns().get("drivermode_json")).isEqualTo(JsonValue.NULL);
    assertThat(camera.columns().objectOrEmpty("config_json").string("nickname", ""))
        .isEqualTo("front-right");
    assertThat(settings.row("global", "hardwareSettings")).isPresent();
    assertThat(settings.row("global", "nope")).isEmpty();
  }

  @Test
  void aDatabaseToRowsToFilesToRowsToADatabaseLosesNothing() throws SQLException {
    Settings original = read(PhotonVisionDatabase.configured(dir));
    Settings fromFiles = SettingsFiles.parse(SettingsFiles.render(original));
    assertThat(fromFiles).isEqualTo(original);

    Path copy = PhotonVisionDatabase.empty(dir.resolve("copy"));
    try (Connection connection = PhotonVisionDatabase.open(copy)) {
      SettingsDatabase.write(connection, fromFiles);
    }
    Settings again = read(copy);
    assertThat(again).isEqualTo(original);
    assertThat(again.hash()).isEqualTo(original.hash());
  }

  @Test
  void writingReplacesWhatTheDatabaseHad() throws SQLException {
    Path file = PhotonVisionDatabase.configured(dir);
    Settings one =
        new Settings(
            2,
            List.of(
                SettingsRow.fromText(
                    "global",
                    "hardwareSettings",
                    Map.of("contents", "{\"ledBrightnessPercentage\":5}"))));
    try (Connection connection = PhotonVisionDatabase.open(file)) {
      SettingsDatabase.write(connection, one);
    }
    assertThat(read(file)).isEqualTo(one);
  }

  @Test
  void textThatIsntJsonIsKeptAsText() throws SQLException {
    Path file = PhotonVisionDatabase.empty(dir);
    try (Connection connection = PhotonVisionDatabase.open(file);
        Statement statement = connection.createStatement()) {
      statement.executeUpdate(
          "INSERT INTO global (filename, contents) VALUES ('broken', '{\"half\":')");
    }
    Settings settings = read(file);
    SettingsRow row = settings.rows().get(0);
    assertThat(row.columns().get("contents" + SettingsRow.TEXT_SUFFIX))
        .isEqualTo(JsonValue.of("{\"half\":"));
    assertThat(row.columnText()).containsEntry("contents", "{\"half\":");

    Path copy = PhotonVisionDatabase.empty(dir.resolve("copy"));
    try (Connection connection = PhotonVisionDatabase.open(copy)) {
      SettingsDatabase.write(connection, SettingsFiles.parse(SettingsFiles.render(settings)));
    }
    assertThat(read(copy)).isEqualTo(settings);
  }

  @Test
  void settingsForAnotherSchemaAreRefusedAndChangeNothing() throws SQLException {
    Path file = PhotonVisionDatabase.configured(dir);
    Settings before = read(file);
    Settings otherVersion = new Settings(3, before.rows());
    Settings otherTable =
        new Settings(2, List.of(SettingsRow.fromText("pipelines", "a", Map.of("json", "{}"))));
    Settings otherColumn =
        new Settings(2, List.of(SettingsRow.fromText("global", "a", Map.of("data", "{}"))));
    try (Connection connection = PhotonVisionDatabase.open(file)) {
      assertThatThrownBy(() -> SettingsDatabase.write(connection, otherVersion))
          .isInstanceOf(SettingsException.class)
          .hasMessageContaining("schema version 3 and the database is at 2");
      assertThatThrownBy(() -> SettingsDatabase.write(connection, otherTable))
          .hasMessageContaining("no table pipelines");
      assertThatThrownBy(() -> SettingsDatabase.write(connection, otherColumn))
          .hasMessageContaining("table global has no column data");
    }
    assertThat(read(file)).isEqualTo(before);
  }

  @Test
  void aTableWithoutASingleKeyIsRefused() throws SQLException {
    Path file = dir.resolve("odd.sqlite");
    try (Connection connection = PhotonVisionDatabase.open(file);
        Statement statement = connection.createStatement()) {
      statement.executeUpdate("CREATE TABLE odd (a TEXT, b TEXT)");
      assertThatThrownBy(() -> SettingsDatabase.read(connection))
          .isInstanceOf(SettingsException.class)
          .hasMessage("table odd has no single primary key");
    }
  }

  @Test
  void identifiersAreQuoted() {
    assertThat(SettingsDatabase.quote("a\"b")).isEqualTo("\"a\"\"b\"");
  }
}
