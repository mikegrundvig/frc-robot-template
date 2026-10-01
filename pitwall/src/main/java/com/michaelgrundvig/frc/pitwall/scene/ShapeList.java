package com.michaelgrundvig.frc.pitwall.scene;

import java.util.ArrayList;
import java.util.List;

/** Shapes as a layer adds them, in order. */
class ShapeList implements Shapes {
  private final List<Shape<?>> shapes = new ArrayList<>();

  /** Every shape added, in order. */
  List<Shape<?>> shapes() {
    return shapes;
  }

  private <S extends Shape<S>> S add(S shape) {
    shapes.add(shape);
    return shape;
  }

  @Override
  public Shape.Box box(String id, String frame) {
    return add(new Shape.Box(id, frame));
  }

  @Override
  public Shape.Cylinder cylinder(String id, String frame) {
    return add(new Shape.Cylinder(id, frame));
  }

  @Override
  public Shape.Line line(String id, String frame) {
    return add(new Shape.Line(id, frame));
  }

  @Override
  public Shape.Polyline polyline(String id, String frame) {
    return add(new Shape.Polyline(id, frame));
  }

  @Override
  public Shape.Polygon polygon(String id, String frame) {
    return add(new Shape.Polygon(id, frame));
  }

  @Override
  public Shape.Mesh mesh(String id, String frame) {
    return add(new Shape.Mesh(id, frame));
  }

  @Override
  public Shape.Arrow arrow(String id, String frame) {
    return add(new Shape.Arrow(id, frame));
  }

  @Override
  public Shape.Frustum frustum(String id, String frame) {
    return add(new Shape.Frustum(id, frame));
  }

  @Override
  public Shape.Label label(String id, String frame) {
    return add(new Shape.Label(id, frame));
  }
}
