package com.michaelgrundvig.frc.pitwall.scene;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Transform3d;

/**
 * Something drawn in a scene: a box, a cylinder, a line, a polyline, a polygon, a mesh, an arrow, a
 * camera's frustum, or a label. Each is in a frame, and moves with it; each has a {@link Style}.
 * Lengths are meters and angles radians, measured in its frame; the page shows them to the
 * millimeter and the tenth of a degree.
 *
 * <p>A shape is made by a {@link Shapes} ({@code scene.box("frame", "robot")}) and set up by its
 * methods, each returning it, until the layer's {@code describe} or {@code update} returns. Then
 * Pitwall reads it once; changing it later changes nothing.
 *
 * <p>A shape with {@link #inspect} can be clicked on the page, which shows what the supplier says.
 *
 * @param <S> the kind of shape, so each method returns it
 */
public abstract sealed class Shape<S extends Shape<S>>
    permits Shape.Box,
        Shape.Cylinder,
        Shape.Line,
        Shape.Polyline,
        Shape.Polygon,
        Shape.Mesh,
        Shape.Arrow,
        Shape.Frustum,
        Shape.Label {
  private final String id;
  private final String kind;
  private final String frame;
  private Style style = Style.MUTED;
  private @Nullable Supplier<Inspection> inspection;
  private List<String> reveals = List.of();
  private double @Nullable [] at;

  Shape(String id, String kind, String frame) {
    this.id = Wire.name("A shape's id", id);
    this.kind = kind;
    this.frame = Wire.name("A frame", frame);
  }

  /** Its id: unique in its scene, it names it to the page and when it's clicked. */
  public String id() {
    return id;
  }

  /** The frame it's in. */
  public String frame() {
    return frame;
  }

  /** How it's drawn: {@link Style#MUTED} unless it's given another. */
  public S style(Style style) {
    this.style = style;
    return self();
  }

  /**
   * Makes it clickable, saying this when it's clicked. The supplier is called on a web server
   * thread, never the robot loop, any time while the shape is drawn and for two seconds after a
   * shape added in {@code update} is gone; so it reads only what can't change under it, such as a
   * record the loop made and keeps no more.
   */
  public S inspect(Supplier<Inspection> details) {
    inspection = details;
    return self();
  }

  /**
   * Shapes the page shows while this one is clicked (its details open), even in a layer switched
   * off: a camera's full view, say, kept off the drawing until its camera is clicked. Each is a
   * shape of the scene's description, or of the same moment.
   */
  public S reveal(String... shapeIds) {
    List<String> ids = new ArrayList<>(shapeIds.length);
    for (String shape : shapeIds) {
      ids.add(Wire.name("A shape's id", shape));
    }
    reveals = List.copyOf(ids);
    return self();
  }

  /** Puts it this far from its frame's origin, unturned. */
  public S at(double x, double y, double z) {
    return at(x, y, z, 0, 0, 0);
  }

  /** Puts it this far from its frame's origin, turned (roll, pitch, yaw, as WPILib turns). */
  public S at(double x, double y, double z, double roll, double pitch, double yaw) {
    at = new double[] {x, y, z, roll, pitch, yaw};
    return self();
  }

  /** Puts it where a transform from its frame's origin says. */
  public S at(Transform3d where) {
    return at(
        where.getX(),
        where.getY(),
        where.getZ(),
        where.getRotation().getX(),
        where.getRotation().getY(),
        where.getRotation().getZ());
  }

  /** What's clicked shows, or null when it can't be clicked. */
  @Nullable Supplier<Inspection> inspection() {
    return inspection;
  }

  /** The shapes the page shows while this one is clicked. */
  List<String> reveals() {
    return reveals;
  }

  /** The value names its size reads, which the scene must declare: none for most shapes. */
  List<String> values() {
    return List.of();
  }

  /** The frames it reaches besides its own: a line between two frames. */
  List<String> reaches() {
    return List.of();
  }

  /** It, as the page reads it. */
  final Map<String, Object> wire() {
    Map<String, Object> wire = new LinkedHashMap<>();
    wire.put("id", id);
    wire.put("kind", kind);
    wire.put("frame", frame);
    wire.put("style", style.wire());
    if (inspection != null) {
      wire.put("inspect", true);
    }
    if (!reveals.isEmpty()) {
      wire.put("reveals", reveals);
    }
    double[] where = at;
    if (where != null) {
      wire.put("at", Wire.pose(where[0], where[1], where[2], where[3], where[4], where[5]));
    }
    describe(wire);
    return wire;
  }

  /** Adds what this kind of shape has to its wire form. */
  abstract void describe(Map<String, Object> wire);

  @SuppressWarnings("unchecked") // each subclass is its own S
  private S self() {
    return (S) this;
  }

  private static List<List<Number>> points(List<double[]> points) {
    List<List<Number>> wire = new ArrayList<>(points.size());
    for (double[] point : points) {
      wire.add(Wire.point(point[0], point[1], point[2]));
    }
    return wire;
  }

  private static double positive(String what, double value) {
    if (!(value > 0) || Double.isInfinite(value)) {
      throw new IllegalArgumentException(what + " must be positive: " + value);
    }
    return value;
  }

  private static double nonNegative(String what, double value) {
    if (!(value >= 0) || Double.isInfinite(value)) {
      throw new IllegalArgumentException(what + " must be zero or more: " + value);
    }
    return value;
  }

  /** A box, centered where it's put: filled faintly, with its edges drawn. */
  public static final class Box extends Shape<Box> {
    private double length = 1;
    private double width = 1;
    private double height = 1;
    private boolean outline;

    Box(String id, String frame) {
      super(id, "box", frame);
    }

    /** Its size: along x, y, and z. */
    public Box size(double length, double width, double height) {
      this.length = positive("A box's length", length);
      this.width = positive("A box's width", width);
      this.height = positive("A box's height", height);
      return this;
    }

    /** Only its edges, not filled. */
    public Box outline() {
      outline = true;
      return this;
    }

    @Override
    void describe(Map<String, Object> wire) {
      wire.put("size", List.of(Wire.meters(length), Wire.meters(width), Wire.meters(height)));
      if (outline) {
        wire.put("outline", true);
      }
    }
  }

  /**
   * A cylinder along its z, centered where it's put: a wheel, a post. Its cross-section may be an
   * ellipse, and its radii may follow two of the scene's values (an uncertainty ellipse). With no
   * length it's a flat disc, or with {@link #outline} a ring.
   */
  public static final class Cylinder extends Shape<Cylinder> {
    private double radiusX = 1;
    private double radiusY = 1;
    private @Nullable String valueX;
    private @Nullable String valueY;
    private double length;
    private boolean outline;

    Cylinder(String id, String frame) {
      super(id, "cylinder", frame);
    }

    /** A round cross-section. */
    public Cylinder radius(double radius) {
      return radii(radius, radius);
    }

    /** An elliptical cross-section: its radii along x and y. */
    public Cylinder radii(double x, double y) {
      radiusX = positive("A cylinder's radius", x);
      radiusY = positive("A cylinder's radius", y);
      valueX = null;
      valueY = null;
      return this;
    }

    /**
     * Radii that follow two of the scene's values ({@link SceneBuilder#value}), set in {@code
     * update} ({@link SceneState#value}): an ellipse that grows and shrinks without the shape being
     * sent again.
     */
    public Cylinder radii(String x, String y) {
      valueX = Wire.name("A value", x);
      valueY = Wire.name("A value", y);
      return this;
    }

    /** How long it is along its z: zero, the default, for a disc or a ring. */
    public Cylinder length(double length) {
      this.length = nonNegative("A cylinder's length", length);
      return this;
    }

    /** Only its edges, not filled. */
    public Cylinder outline() {
      outline = true;
      return this;
    }

    @Override
    List<String> values() {
      String x = valueX;
      String y = valueY;
      return x != null && y != null ? List.of(x, y) : List.of();
    }

    @Override
    void describe(Map<String, Object> wire) {
      String x = valueX;
      String y = valueY;
      if (x != null && y != null) {
        wire.put("radii", List.of(x, y));
      } else if (radiusX == radiusY) {
        wire.put("radius", Wire.meters(radiusX));
      } else {
        wire.put("radii", List.of(Wire.meters(radiusX), Wire.meters(radiusY)));
      }
      wire.put("length", Wire.meters(length));
      if (outline) {
        wire.put("outline", true);
      }
    }
  }

  /**
   * A straight line: between two points in its frame, or between the origins of two frames, which
   * then follows both as they move (a camera's sight line to a tag).
   */
  public static final class Line extends Shape<Line> {
    private double[] from = {0, 0, 0};
    private double[] to = {0, 0, 0};
    private @Nullable String fromFrame;
    private @Nullable String toFrame;

    Line(String id, String frame) {
      super(id, "line", frame);
    }

    /** Where it starts, in its frame. */
    public Line from(double x, double y, double z) {
      from = new double[] {x, y, z};
      return this;
    }

    /** Where it ends, in its frame. */
    public Line to(double x, double y, double z) {
      to = new double[] {x, y, z};
      return this;
    }

    /** From one frame's origin to another's, wherever they are. */
    public Line between(String fromFrame, String toFrame) {
      this.fromFrame = Wire.name("A frame", fromFrame);
      this.toFrame = Wire.name("A frame", toFrame);
      return this;
    }

    @Override
    List<String> reaches() {
      String a = fromFrame;
      String b = toFrame;
      return a != null && b != null ? List.of(a, b) : List.of();
    }

    @Override
    void describe(Map<String, Object> wire) {
      String a = fromFrame;
      String b = toFrame;
      if (a != null && b != null) {
        wire.put("between", List.of(a, b));
      } else {
        wire.put("from", Wire.point(from[0], from[1], from[2]));
        wire.put("to", Wire.point(to[0], to[1], to[2]));
      }
    }
  }

  /** Points joined by straight lines, closed into a loop if asked: a field's outline. */
  public static final class Polyline extends Shape<Polyline> {
    private final List<double[]> points = new ArrayList<>();
    private boolean closed;

    Polyline(String id, String frame) {
      super(id, "polyline", frame);
    }

    /** The next point. */
    public Polyline point(double x, double y, double z) {
      points.add(new double[] {x, y, z});
      return this;
    }

    /** Joins the last point back to the first. */
    public Polyline closed() {
      closed = true;
      return this;
    }

    @Override
    void describe(Map<String, Object> wire) {
      if (points.size() < 2) {
        throw new IllegalArgumentException(id() + ": a polyline needs two points or more");
      }
      wire.put("points", points(points));
      if (closed) {
        wire.put("closed", true);
      }
    }
  }

  /** A flat polygon, filled faintly with its edges drawn: its points all in one plane. */
  public static final class Polygon extends Shape<Polygon> {
    private final List<double[]> points = new ArrayList<>();
    private boolean outline;

    Polygon(String id, String frame) {
      super(id, "polygon", frame);
    }

    /** The next corner. */
    public Polygon point(double x, double y, double z) {
      points.add(new double[] {x, y, z});
      return this;
    }

    /** Only its edges, not filled. */
    public Polygon outline() {
      outline = true;
      return this;
    }

    @Override
    void describe(Map<String, Object> wire) {
      if (points.size() < 3) {
        throw new IllegalArgumentException(id() + ": a polygon needs three corners or more");
      }
      wire.put("points", points(points));
      if (outline) {
        wire.put("outline", true);
      }
    }
  }

  /** Triangles between vertices: any solid the other shapes don't make, such as a ramp. */
  public static final class Mesh extends Shape<Mesh> {
    private final List<double[]> vertices = new ArrayList<>();
    private final List<List<Integer>> triangles = new ArrayList<>();
    private boolean outline;

    Mesh(String id, String frame) {
      super(id, "mesh", frame);
    }

    /** The next vertex; the first is 0. */
    public Mesh vertex(double x, double y, double z) {
      vertices.add(new double[] {x, y, z});
      return this;
    }

    /** A triangle between three vertices, counterclockwise seen from outside. */
    public Mesh triangle(int a, int b, int c) {
      triangles.add(List.of(a, b, c));
      return this;
    }

    /** Only its edges, not filled. */
    public Mesh outline() {
      outline = true;
      return this;
    }

    @Override
    void describe(Map<String, Object> wire) {
      for (List<Integer> triangle : triangles) {
        for (int vertex : triangle) {
          if (vertex < 0 || vertex >= vertices.size()) {
            throw new IllegalArgumentException(
                id() + ": a triangle names vertex " + vertex + " of " + vertices.size());
          }
        }
      }
      if (triangles.isEmpty()) {
        throw new IllegalArgumentException(id() + ": a mesh needs a triangle or more");
      }
      wire.put("vertices", points(vertices));
      wire.put("triangles", List.copyOf(triangles));
      if (outline) {
        wire.put("outline", true);
      }
    }
  }

  /**
   * An arrow from one point toward another: which way the robot faces, a camera looks, a wheel
   * drives. Its length may follow one of the scene's values instead (a wheel's speed), pointing
   * back the other way when the value is negative.
   */
  public static final class Arrow extends Shape<Arrow> {
    private double[] from = {0, 0, 0};
    private double[] to = {1, 0, 0};
    private @Nullable String length;

    Arrow(String id, String frame) {
      super(id, "arrow", frame);
    }

    /** Where it starts, in its frame. */
    public Arrow from(double x, double y, double z) {
      from = new double[] {x, y, z};
      return this;
    }

    /** Where it points, in its frame: its tip, unless its length follows a value. */
    public Arrow to(double x, double y, double z) {
      to = new double[] {x, y, z};
      return this;
    }

    /**
     * A length that follows one of the scene's values ({@link SceneBuilder#value}), in meters, set
     * in {@code update}: it still points from {@link #from} toward {@link #to}.
     */
    public Arrow length(String value) {
      length = Wire.name("A value", value);
      return this;
    }

    @Override
    List<String> values() {
      String value = length;
      return value != null ? List.of(value) : List.of();
    }

    @Override
    void describe(Map<String, Object> wire) {
      if (from[0] == to[0] && from[1] == to[1] && from[2] == to[2]) {
        throw new IllegalArgumentException(id() + ": an arrow points from one point to another");
      }
      wire.put("from", Wire.point(from[0], from[1], from[2]));
      wire.put("to", Wire.point(to[0], to[1], to[2]));
      String value = length;
      if (value != null) {
        wire.put("length", value);
      }
    }
  }

  /**
   * What a camera sees: a pyramid along its frame's x (WPILib's camera frame: x forward, y left, z
   * up), as wide and tall as its field of view, from a near range out to a far one.
   */
  public static final class Frustum extends Shape<Frustum> {
    private double horizontal = Math.toRadians(70);
    private double vertical = Math.toRadians(50);
    private double near;
    private double far = 1;

    Frustum(String id, String frame) {
      super(id, "frustum", frame);
    }

    /** The field of view, side to side and top to bottom, in radians. */
    public Frustum fov(double horizontal, double vertical) {
      if (!(horizontal > 0 && horizontal < Math.PI && vertical > 0 && vertical < Math.PI)) {
        throw new IllegalArgumentException(
            id()
                + ": a field of view is between 0 and 180 degrees: "
                + horizontal
                + ", "
                + vertical);
      }
      this.horizontal = horizontal;
      this.vertical = vertical;
      return this;
    }

    /** From how near to how far it's drawn, in meters. */
    public Frustum range(double near, double far) {
      this.near = nonNegative("A frustum's near range", near);
      this.far = positive("A frustum's far range", far);
      if (!(far > near)) {
        throw new IllegalArgumentException(id() + ": a frustum's far range is past its near");
      }
      return this;
    }

    @Override
    void describe(Map<String, Object> wire) {
      wire.put("fov", List.of(Wire.degrees(horizontal), Wire.degrees(vertical)));
      wire.put("range", List.of(Wire.meters(near), Wire.meters(far)));
    }
  }

  /** A few words that stay readable at any zoom: a tag's number, a camera's name. */
  public static final class Label extends Shape<Label> {
    private String text = "";

    Label(String id, String frame) {
      super(id, "label", frame);
    }

    /** What it says: up to 40 characters. */
    public Label text(String text) {
      if (text.isBlank() || text.length() > 40) {
        throw new IllegalArgumentException(id() + ": a label says 1 to 40 characters: " + text);
      }
      this.text = text;
      return this;
    }

    @Override
    void describe(Map<String, Object> wire) {
      if (text.isEmpty()) {
        throw new IllegalArgumentException(id() + ": a label needs its text");
      }
      wire.put("text", text);
    }
  }
}
