package com.michaelgrundvig.frc.pitwall.scene;

import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.Nullable;

/**
 * A view of the world the page draws in 3D, from data the robot program gives it: named frames,
 * shapes in them, grouped into {@link Layer}s the page can switch on and off. Pitwall knows nothing
 * of fields or robots: a scene is whatever its layers say.
 *
 * {@snippet :
 * scenes.add(
 *     Scene.named("world", "World")
 *         .layer(field)
 *         .layer(robotLayer)
 *         .layer(rays, Scene.Visibility.OFF)
 *         .focus("robot"));
 * }
 *
 * <p>The page offers a view of it once one is added, and only then: Pitwall without scenes shows no
 * such view. Its description goes to the page once, when it opens, and again only when a layer is
 * described again; what moves goes only to the pages that show it, at most 20 times a second, and
 * only what changed. See {@link Scenes}.
 */
public final class Scene {
  /** Whether a layer is on when the page opens. */
  public enum Visibility {
    ON,
    OFF
  }

  /** A layer, and whether it starts on. */
  record Entry(Layer layer, Visibility visibility) {}

  private final String id;
  private final String title;
  private final List<Entry> layers = new ArrayList<>();
  private @Nullable String focus;

  private Scene(String id, String title) {
    this.id = Wire.name("A scene's id", id);
    if (title.isBlank()) {
      throw new IllegalArgumentException("scene " + id + " needs a title");
    }
    this.title = title;
  }

  /**
   * A scene with no layers yet.
   *
   * @param id its name: the page's address for it, and what a page names to be sent it
   * @param title what the page calls it
   */
  public static Scene named(String id, String title) {
    return new Scene(id, title);
  }

  /** Adds a layer, on when the page opens. */
  public Scene layer(Layer layer) {
    return layer(layer, Visibility.ON);
  }

  /** Adds a layer, on or off when the page opens. */
  public Scene layer(Layer layer, Visibility visibility) {
    Wire.name("A layer's id", layer.id());
    for (Entry entry : layers) {
      if (entry.layer().id().equals(layer.id())) {
        throw new IllegalArgumentException("scene " + id + " has two layers named " + layer.id());
      }
    }
    layers.add(new Entry(layer, visibility));
    return this;
  }

  /**
   * The frame the page's Follow keeps in the middle of the view as it moves: the robot. Without
   * one, the page offers no Follow.
   */
  public Scene focus(String frame) {
    focus = Wire.name("A frame", frame);
    return this;
  }

  /** The frame the page can follow, or null for none. */
  @Nullable String focus() {
    return focus;
  }

  /** Its id. */
  public String id() {
    return id;
  }

  /** What the page calls it. */
  public String title() {
    return title;
  }

  /** Its layers, in order, each with whether it starts on. */
  List<Entry> layers() {
    return List.copyOf(layers);
  }
}
