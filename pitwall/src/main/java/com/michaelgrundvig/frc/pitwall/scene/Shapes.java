package com.michaelgrundvig.frc.pitwall.scene;

/**
 * Where a layer adds its shapes: a scene being described ({@link SceneBuilder}), whose shapes stay
 * until the layer is described again, or a layer's shapes of the moment ({@link
 * SceneState#shapes()}), which replace the last of them whole. Each method adds the shape and
 * returns it to set up:
 *
 * {@snippet :
 * scene.box("chassis", "robot").size(0.7, 0.6, 0.2).at(0, 0, 0.1).style(Style.ACCENT);
 * }
 *
 * <p>Each shape names its id, unique in its scene, and the frame it's in, which must be one the
 * scene has. See {@link Shape} for each kind.
 */
public interface Shapes {
  /** A box: {@link Shape.Box}. */
  Shape.Box box(String id, String frame);

  /** A cylinder, disc, or ring: {@link Shape.Cylinder}. */
  Shape.Cylinder cylinder(String id, String frame);

  /** A straight line: {@link Shape.Line}. */
  Shape.Line line(String id, String frame);

  /** Points joined by lines: {@link Shape.Polyline}. */
  Shape.Polyline polyline(String id, String frame);

  /** A flat, filled polygon: {@link Shape.Polygon}. */
  Shape.Polygon polygon(String id, String frame);

  /** Triangles between vertices: {@link Shape.Mesh}. */
  Shape.Mesh mesh(String id, String frame);

  /** An arrow: {@link Shape.Arrow}. */
  Shape.Arrow arrow(String id, String frame);

  /** What a camera sees: {@link Shape.Frustum}. */
  Shape.Frustum frustum(String id, String frame);

  /** A few words: {@link Shape.Label}. */
  Shape.Label label(String id, String frame);
}
