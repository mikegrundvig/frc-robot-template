package com.michaelgrundvig.frc.world.bridges;

import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.Arrays;
import java.util.List;

/**
 * Flat-shaded triangles in one color, built from simple solids: boxes, pyramids, wheels, arrows.
 * Every triangle carries its own corners and its face's normal, so nothing is smoothed: a
 * schematic, not a rendering. Points are in the robot's frame (x forward, y left, z up, meters).
 *
 * <p>Each solid is convex, and its faces are turned to face away from a point inside it, so every
 * normal points out however a face's corners were listed.
 */
final class Mesh {
  /**
   * How a mesh is colored: red, green and blue from 0 to 1, and how opaque, from 0 to 1. A glTF
   * viewer draws one under 1 see-through; AdvantageScope draws any under 0.75 at its own fixed
   * opacity.
   */
  record Material(String name, double red, double green, double blue, double alpha) {}

  private final String name;
  private final Material material;
  private float[] positions = new float[3 * 36];
  private float[] normals = new float[3 * 36];
  private int vertices;

  Mesh(String name, Material material) {
    this.name = name;
    this.material = material;
  }

  String name() {
    return name;
  }

  Material material() {
    return material;
  }

  /** How many corners: three for each triangle. */
  int vertexCount() {
    return vertices;
  }

  /** Each corner's x, y and z in turn, in the robot's frame. */
  float[] positions() {
    return Arrays.copyOf(positions, 3 * vertices);
  }

  /** Each corner's face normal, as positions lists them. */
  float[] normals() {
    return Arrays.copyOf(normals, 3 * vertices);
  }

  /** A box, square to its frame, between two opposite corners given in the frame. */
  Mesh box(Frame frame, Vector3 low, Vector3 high) {
    Vector3[] c = new Vector3[8];
    for (int i = 0; i < 8; i++) {
      c[i] =
          frame.place(
              new Vector3(
                  (i & 1) == 0 ? low.x() : high.x(),
                  (i & 2) == 0 ? low.y() : high.y(),
                  (i & 4) == 0 ? low.z() : high.z()));
    }
    Vector3 inside = frame.place(low.plus(high).times(0.5));
    face(inside, c[0], c[2], c[3], c[1]); // bottom
    face(inside, c[4], c[5], c[7], c[6]); // top
    face(inside, c[0], c[1], c[5], c[4]); // the side at low y
    face(inside, c[2], c[6], c[7], c[3]); // high y
    face(inside, c[0], c[4], c[6], c[2]); // low x
    face(inside, c[1], c[3], c[7], c[5]); // high x
    return this;
  }

  /**
   * A pyramid with a four-cornered base: a camera's view out to its sight range, or an arrow's
   * head. Points are given in the frame, the base's corners in order around it.
   */
  Mesh pyramid(Frame frame, Vector3 apex, List<Vector3> base) {
    Vector3 tip = frame.place(apex);
    Vector3[] corners = base.stream().map(frame::place).toArray(Vector3[]::new);
    Vector3 inside = tip;
    for (Vector3 corner : corners) {
      inside = inside.plus(corner);
    }
    inside = inside.times(1.0 / (corners.length + 1));
    for (int i = 0; i < corners.length; i++) {
      face(inside, tip, corners[i], corners[(i + 1) % corners.length]);
    }
    face(inside, corners);
    return this;
  }

  /**
   * A wheel: a cylinder around its frame's y axis, centered on the frame's origin.
   *
   * @param halfWidth half the wheel's width, along y
   * @param sides how many flat faces stand in for the round tread
   */
  Mesh wheel(Frame frame, double radius, double halfWidth, int sides) {
    Vector3[] left = new Vector3[sides];
    Vector3[] right = new Vector3[sides];
    for (int i = 0; i < sides; i++) {
      double around = 2 * StrictMath.PI * i / sides;
      double x = radius * StrictMath.cos(around);
      double z = radius * StrictMath.sin(around);
      left[i] = frame.place(new Vector3(x, halfWidth, z));
      right[i] = frame.place(new Vector3(x, -halfWidth, z));
    }
    Vector3 inside = frame.place(Vector3.ZERO);
    for (int i = 0; i < sides; i++) {
      int next = (i + 1) % sides;
      face(inside, left[i], left[next], right[next], right[i]);
    }
    face(inside, left);
    face(inside, right);
    return this;
  }

  /**
   * A flat arrow along its frame's x, between two heights: a shaft, then a head whose tip is at
   * {@code tip}.
   *
   * @param tail where the shaft starts, along x
   * @param tip where the head ends
   * @param headLength how long the head is, along x
   * @param shaftWidth how wide the shaft is, along y
   * @param headWidth how wide the head is at its base
   * @param low the arrow's underside, along z
   * @param high its top
   */
  Mesh arrow(
      Frame frame,
      double tail,
      double tip,
      double headLength,
      double shaftWidth,
      double headWidth,
      double low,
      double high) {
    double neck = tip - headLength;
    box(frame, new Vector3(tail, -shaftWidth / 2, low), new Vector3(neck, shaftWidth / 2, high));
    Vector3[] bottom = {
      frame.place(new Vector3(neck, -headWidth / 2, low)),
      frame.place(new Vector3(tip, 0, low)),
      frame.place(new Vector3(neck, headWidth / 2, low))
    };
    Vector3[] top = {
      frame.place(new Vector3(neck, -headWidth / 2, high)),
      frame.place(new Vector3(tip, 0, high)),
      frame.place(new Vector3(neck, headWidth / 2, high))
    };
    Vector3 inside = frame.place(new Vector3(neck + headLength / 3, 0, (low + high) / 2));
    face(inside, bottom);
    face(inside, top);
    for (int i = 0; i < 3; i++) {
      int next = (i + 1) % 3;
      face(inside, bottom[i], bottom[next], top[next], top[i]);
    }
    return this;
  }

  /**
   * A flat, convex face, cut into triangles from its first corner, each turned to face away from a
   * point inside the solid.
   */
  private void face(Vector3 inside, Vector3... corners) {
    for (int i = 1; i + 1 < corners.length; i++) {
      Vector3 a = corners[0];
      Vector3 b = corners[i];
      Vector3 c = corners[i + 1];
      Vector3 normal = b.minus(a).cross(c.minus(a));
      if (normal.length() == 0) {
        continue; // no area to draw
      }
      if (normal.dot(a.minus(inside)) < 0) {
        triangle(a, c, b, normal.times(-1).unit());
      } else {
        triangle(a, b, c, normal.unit());
      }
    }
  }

  private void triangle(Vector3 a, Vector3 b, Vector3 c, Vector3 normal) {
    if (3 * (vertices + 3) > positions.length) {
      positions = Arrays.copyOf(positions, 2 * positions.length);
      normals = Arrays.copyOf(normals, 2 * normals.length);
    }
    for (Vector3 corner : new Vector3[] {a, b, c}) {
      int at = 3 * vertices++;
      positions[at] = (float) corner.x();
      positions[at + 1] = (float) corner.y();
      positions[at + 2] = (float) corner.z();
      normals[at] = (float) normal.x();
      normals[at + 1] = (float) normal.y();
      normals[at + 2] = (float) normal.z();
    }
  }
}
