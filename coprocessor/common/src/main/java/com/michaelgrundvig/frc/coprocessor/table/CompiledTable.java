package com.michaelgrundvig.frc.coprocessor.table;

import com.michaelgrundvig.frc.coprocessor.api.Stamp;
import com.michaelgrundvig.frc.coprocessor.json.Json;
import com.michaelgrundvig.frc.coprocessor.json.JsonValue;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.jspecify.annotations.Nullable;

/**
 * The coprocessor table as the build compiles it into the robot program: the table, plus what the
 * build knows that the robot checks each coprocessor against (the PhotonVision version, the image
 * recipe's hash, and each computer's committed settings hash). The build writes it from {@code
 * coprocessors/coprocessors.yaml} into the jar as {@link #RESOURCE}.
 *
 * @param table the table
 * @param photonvisionVersion the PhotonVision version PhotonLib was built for (the vendordep's)
 * @param recipeHash the hash of the image recipe this build would build
 * @param settingsHashes each computer's committed settings hash, by name; empty for a computer with
 *     none committed
 */
public record CompiledTable(
    Table table,
    String photonvisionVersion,
    String recipeHash,
    Map<String, String> settingsHashes) {
  /** Where the build puts the compiled table in the robot program's jar. */
  public static final String RESOURCE = "/coprocessor/table.json";

  public CompiledTable {
    settingsHashes = Collections.unmodifiableMap(new LinkedHashMap<>(settingsHashes));
  }

  /** A computer's committed settings hash; empty when none are committed. */
  public String settingsHash(Computer computer) {
    return settingsHashes.getOrDefault(computer.name(), "");
  }

  /**
   * How a computer's stamp differs from what this build expects of it: its name, team, address,
   * PhotonVision version, and recipe hash, each as a sentence naming both values. Empty when they
   * all match. The settings hash is left out: settings change while someone calibrates, which isn't
   * a mismatch.
   */
  public List<String> mismatches(Computer computer, Stamp stamp) {
    List<String> mismatches = new ArrayList<>();
    compare(mismatches, "name", computer.name(), stamp.name());
    compare(mismatches, "team", Integer.toString(table.team()), Integer.toString(stamp.team()));
    compare(mismatches, "address", table.ip(computer), stamp.address());
    compare(mismatches, "PhotonVision", photonvisionVersion, stamp.photonvisionVersion());
    compare(mismatches, "recipe hash", recipeHash, stamp.recipeHash());
    return mismatches;
  }

  private static void compare(List<String> mismatches, String what, String built, String found) {
    if (!built.equals(found)) {
      mismatches.add(
          what + " is \"" + found + "\" on the coprocessor and \"" + built + "\" in this build");
    }
  }

  /** The compiled table as JSON. */
  public JsonValue.Obj toJson() {
    JsonValue.Obj.Builder hashes = JsonValue.Obj.builder();
    settingsHashes.forEach(hashes::put);
    return JsonValue.Obj.builder()
        .put("table", table.toJson())
        .put("photonvisionVersion", photonvisionVersion)
        .put("recipeHash", recipeHash)
        .put("settingsHashes", hashes.build())
        .build();
  }

  /** A compiled table from its JSON; the table is checked as any table is. */
  public static CompiledTable fromJson(JsonValue json) {
    JsonValue.Obj o = json.asObject("compiled table");
    Map<String, String> hashes = new LinkedHashMap<>();
    JsonValue.Obj hashesJson = o.objectOrEmpty("settingsHashes");
    for (String name : hashesJson.members().keySet()) {
      hashes.put(name, hashesJson.string(name, ""));
    }
    return new CompiledTable(
        Table.fromJson(o.objectOrEmpty("table")),
        o.string("photonvisionVersion", ""),
        o.string("recipeHash", ""),
        hashes);
  }

  /** A compiled table from JSON text. */
  public static CompiledTable parse(String json) {
    return fromJson(Json.parse(json));
  }

  /** The compiled table in the robot program's jar. */
  public static CompiledTable load() {
    try (InputStream in = CompiledTable.class.getResourceAsStream(RESOURCE)) {
      return read(in);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  /** The compiled table from a resource's stream; null (no such resource) says how it's made. */
  static CompiledTable read(@Nullable InputStream in) throws IOException {
    if (in == null) {
      throw new IllegalStateException(
          RESOURCE
              + " isn't in the program: the build makes it from "
              + Table.PATH
              + " (the coprocessorTable task)");
    }
    return parse(new String(in.readAllBytes(), StandardCharsets.UTF_8));
  }
}
