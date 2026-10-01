package com.michaelgrundvig.frc.coprocessor.agent;

import com.michaelgrundvig.frc.coprocessor.json.Json;
import com.michaelgrundvig.frc.coprocessor.json.JsonException;
import com.michaelgrundvig.frc.coprocessor.json.JsonValue;
import com.michaelgrundvig.frc.coprocessor.settings.SettingsDatabase;
import com.michaelgrundvig.frc.coprocessor.settings.SettingsText;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.BasicFileAttributes;
import java.sql.Connection;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * PhotonVision's live settings, read from its database read-only. Read again only when the database
 * file changes (its size or time), and kept as text ({@link SettingsText}), parsed a row at a time;
 * the hash and where each camera is expected are worked out once per change.
 */
final class SettingsSource {
  /** The largest database the agent reads; PhotonVision's is a few megabytes with calibrations. */
  static final long MAX_DATABASE = 64L * 1024 * 1024;

  private final Host host;
  private final String database;
  private @Nullable Snapshot snapshot;

  /**
   * The settings as of one version of the database file.
   *
   * @param version the file's size and modification time
   * @param text the rows' text
   * @param hash their hash
   * @param usbPaths where each camera is expected (its by-path), by camera name (nickname)
   */
  record Snapshot(String version, SettingsText text, String hash, Map<String, String> usbPaths) {}

  SettingsSource(Host host, String database) {
    this.host = host;
    this.database = database;
  }

  /** The settings now; empty when PhotonVision has no database yet. */
  synchronized Optional<Snapshot> read() throws IOException {
    Path file = host.path(database);
    if (!Files.isRegularFile(file)) {
      snapshot = null;
      return Optional.empty();
    }
    BasicFileAttributes attributes = Files.readAttributes(file, BasicFileAttributes.class);
    if (attributes.size() > MAX_DATABASE) {
      throw new IOException(
          database + " is " + attributes.size() / (1024 * 1024) + " MB, more than the agent reads");
    }
    String version = attributes.size() + "@" + attributes.lastModifiedTime().toMillis();
    Snapshot known = snapshot;
    if (known != null && known.version().equals(version)) {
      return Optional.of(known);
    }
    SettingsText text;
    try (Connection connection = Sqlite.openReadOnly(file)) {
      text = SettingsDatabase.readText(connection);
    } catch (SQLException e) {
      throw new IOException(database + ": " + e.getMessage(), e);
    }
    Snapshot read = new Snapshot(version, text, text.hash(), usbPaths(text));
    snapshot = read;
    return Optional.of(read);
  }

  /**
   * Where each camera is expected, as PhotonVision matches it: its matched camera's unique path,
   * the first by-path entry (sorted) among its other paths, or failing those its path.
   */
  static Map<String, String> usbPaths(SettingsText text) {
    Map<String, String> paths = new LinkedHashMap<>();
    for (SettingsText.Row row : text.rows()) {
      if (!row.table().equals("cameras")) {
        continue;
      }
      String config = row.columns().get("config_json");
      if (config == null) {
        continue;
      }
      JsonValue.Obj camera;
      try {
        camera = Json.parse(config).asObject("config_json");
      } catch (JsonException e) {
        continue;
      }
      String nickname = camera.string("nickname", "");
      JsonValue.Obj matched = camera.objectOrEmpty("matchedCameraInfo");
      String path = matched.string("uniquePath", "");
      if (path.isEmpty()) {
        List<String> others = new ArrayList<>();
        for (JsonValue other : matched.items("otherPaths")) {
          if (other instanceof JsonValue.Str string && string.value().contains("/by-path/")) {
            others.add(string.value());
          }
        }
        Collections.sort(others);
        path = others.isEmpty() ? matched.string("path", "") : others.get(0);
      }
      if (!nickname.isEmpty()) {
        paths.put(nickname, path);
      }
    }
    return paths;
  }
}
