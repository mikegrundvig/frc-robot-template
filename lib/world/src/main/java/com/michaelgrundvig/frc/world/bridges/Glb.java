package com.michaelgrundvig.frc.world.bridges;

import io.avaje.jsonb.Jsonb;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Writes meshes as a binary glTF 2.0 file (a {@code .glb}), by the Khronos specification, with no
 * glTF library: a 12-byte header, a JSON chunk describing the scene (written by avaje-jsonb), and a
 * binary chunk holding every corner's position and normal as little-endian 32-bit floats.
 *
 * <p>Each mesh is one node and one primitive of triangles (no index buffer: every corner is its
 * own, since faces are flat), with its own material, a flat color.
 *
 * <p>glTF's axes are the specification's: +y up, and an asset's front faces +z. So a point's robot
 * coordinates (forward, left, up) are written as glTF's (left, up, forward), and any glTF viewer
 * shows the robot upright. AdvantageScope turns it back with the rotations in the robot's
 * config.json: 90° about x, then 90° about z ({@link AdvantageScopeAssets#GLTF_TO_ROBOT}).
 */
final class Glb {
  /** "glTF", the file's first four bytes, read little-endian. */
  static final int MAGIC = 0x46546C67;

  static final int VERSION = 2;

  /** "JSON", a chunk's type. */
  static final int JSON_CHUNK = 0x4E4F534A;

  /** "BIN" and a zero byte, a chunk's type. */
  static final int BINARY_CHUNK = 0x004E4942;

  // glTF's codes, from the specification.
  private static final int FLOAT = 5126;
  private static final int ARRAY_BUFFER = 34962;
  private static final int TRIANGLES = 4;

  private Glb() {}

  /**
   * A whole {@code .glb} file.
   *
   * @param generator what made it, and that it's generated: glTF's place for saying so
   */
  static byte[] write(String generator, List<Mesh> meshes) {
    List<Map<String, Object>> nodes = new ArrayList<>();
    List<Map<String, Object>> gltfMeshes = new ArrayList<>();
    List<Map<String, Object>> materials = new ArrayList<>();
    List<Map<String, Object>> accessors = new ArrayList<>();
    List<Map<String, Object>> views = new ArrayList<>();
    List<Mesh.Material> materialOrder = new ArrayList<>();
    List<float[]> chunks = new ArrayList<>();
    int offset = 0;
    for (Mesh mesh : meshes) {
      if (mesh.vertexCount() == 0) {
        continue;
      }
      float[] positions = toGltf(mesh.positions());
      float[] normals = toGltf(mesh.normals());
      int material = materialOrder.indexOf(mesh.material());
      if (material < 0) {
        material = materialOrder.size();
        materialOrder.add(mesh.material());
        materials.add(material(mesh.material()));
      }
      Map<String, Object> attributes = new LinkedHashMap<>();
      attributes.put("POSITION", accessors.size());
      accessors.add(positionAccessor(views.size(), mesh.vertexCount(), positions));
      views.add(view(offset, positions.length));
      offset += 4 * positions.length;
      chunks.add(positions);
      attributes.put("NORMAL", accessors.size());
      accessors.add(accessor(views.size(), mesh.vertexCount()));
      views.add(view(offset, normals.length));
      offset += 4 * normals.length;
      chunks.add(normals);

      Map<String, Object> primitive = new LinkedHashMap<>();
      primitive.put("attributes", attributes);
      primitive.put("material", material);
      primitive.put("mode", TRIANGLES);
      Map<String, Object> gltfMesh = new LinkedHashMap<>();
      gltfMesh.put("name", mesh.name());
      gltfMesh.put("primitives", List.of(primitive));
      Map<String, Object> node = new LinkedHashMap<>();
      node.put("name", mesh.name());
      node.put("mesh", gltfMeshes.size());
      gltfMeshes.add(gltfMesh);
      nodes.add(node);
    }

    Map<String, Object> gltf = new LinkedHashMap<>();
    Map<String, Object> asset = new LinkedHashMap<>();
    asset.put("version", "2.0");
    asset.put("generator", generator);
    gltf.put("asset", asset);
    gltf.put("scene", 0);
    List<Integer> roots = new ArrayList<>();
    for (int i = 0; i < nodes.size(); i++) {
      roots.add(i);
    }
    gltf.put("scenes", List.of(Map.of("nodes", roots)));
    gltf.put("nodes", nodes);
    if (!gltfMeshes.isEmpty()) {
      gltf.put("meshes", gltfMeshes);
      gltf.put("materials", materials);
      gltf.put("accessors", accessors);
      gltf.put("bufferViews", views);
      gltf.put("buffers", List.of(Map.of("byteLength", offset)));
    }
    String json = Jsonb.instance().type(Object.class).toJson(gltf);
    return pack(json.getBytes(StandardCharsets.UTF_8), chunks, offset);
  }

  /** A robot-frame point or direction (forward, left, up) as glTF's (left, up, forward). */
  private static float[] toGltf(float[] robot) {
    float[] gltf = new float[robot.length];
    for (int i = 0; i < robot.length; i += 3) {
      gltf[i] = robot[i + 1];
      gltf[i + 1] = robot[i + 2];
      gltf[i + 2] = robot[i];
    }
    return gltf;
  }

  private static Map<String, Object> material(Mesh.Material material) {
    Map<String, Object> pbr = new LinkedHashMap<>();
    pbr.put(
        "baseColorFactor",
        List.of(material.red(), material.green(), material.blue(), material.alpha()));
    pbr.put("metallicFactor", 0.0);
    pbr.put("roughnessFactor", 1.0);
    Map<String, Object> gltf = new LinkedHashMap<>();
    gltf.put("name", material.name());
    gltf.put("pbrMetallicRoughness", pbr);
    if (material.alpha() < 1) {
      gltf.put("alphaMode", "BLEND");
    }
    gltf.put("doubleSided", true);
    return gltf;
  }

  /** Three floats a corner. */
  private static Map<String, Object> accessor(int view, int count) {
    Map<String, Object> accessor = new LinkedHashMap<>();
    accessor.put("bufferView", view);
    accessor.put("componentType", FLOAT);
    accessor.put("count", count);
    accessor.put("type", "VEC3");
    return accessor;
  }

  /**
   * Positions carry their bounds, as the specification requires, each the exact float the data
   * holds: a viewer checks them against the data.
   */
  private static Map<String, Object> positionAccessor(int view, int count, float[] positions) {
    List<Double> min = new ArrayList<>();
    List<Double> max = new ArrayList<>();
    for (int axis = 0; axis < 3; axis++) {
      float low = Float.POSITIVE_INFINITY;
      float high = Float.NEGATIVE_INFINITY;
      for (int i = axis; i < positions.length; i += 3) {
        low = Math.min(low, positions[i]);
        high = Math.max(high, positions[i]);
      }
      min.add((double) low);
      max.add((double) high);
    }
    Map<String, Object> accessor = accessor(view, count);
    accessor.put("min", min);
    accessor.put("max", max);
    return accessor;
  }

  private static Map<String, Object> view(int offset, int floats) {
    Map<String, Object> view = new LinkedHashMap<>();
    view.put("buffer", 0);
    view.put("byteOffset", offset);
    view.put("byteLength", 4 * floats);
    view.put("target", ARRAY_BUFFER);
    return view;
  }

  /**
   * The header and the two chunks, each chunk padded to four bytes: the JSON with spaces, the
   * binary data with zeros, as the specification asks.
   */
  private static byte[] pack(byte[] json, List<float[]> chunks, int binaryLength) {
    int jsonLength = padded(json.length);
    int binaryPadded = padded(binaryLength);
    boolean hasBinary = binaryLength > 0;
    int total = 12 + 8 + jsonLength + (hasBinary ? 8 + binaryPadded : 0);
    ByteBuffer out = ByteBuffer.allocate(total).order(ByteOrder.LITTLE_ENDIAN);
    out.putInt(MAGIC).putInt(VERSION).putInt(total);
    out.putInt(jsonLength).putInt(JSON_CHUNK).put(json);
    for (int i = json.length; i < jsonLength; i++) {
      out.put((byte) ' ');
    }
    if (hasBinary) {
      out.putInt(binaryPadded).putInt(BINARY_CHUNK);
      for (float[] chunk : chunks) {
        for (float value : chunk) {
          out.putFloat(value);
        }
      }
      // ByteBuffer.allocate gave zeros for the padding.
    }
    return out.array();
  }

  private static int padded(int length) {
    return (length + 3) & ~3;
  }
}
