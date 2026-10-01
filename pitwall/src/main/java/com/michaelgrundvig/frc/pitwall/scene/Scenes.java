package com.michaelgrundvig.frc.pitwall.scene;

import com.michaelgrundvig.frc.pitwall.push.Feed;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Consumer;
import java.util.function.LongSupplier;

/**
 * The scenes the page draws: {@code pitwall.scenes().add(Scene.named("world", "World")...)}.
 *
 * <p>What it costs the robot, and the page's link:
 *
 * <ul>
 *   <li><b>The description</b> (frames and shapes) is built when a scene is added, checked whole,
 *       and goes to each page once, in its snapshot; again only when a layer's configuration
 *       changes. It's the {@code scenes} section of the page's state: {@code {id: {"title",
 *       "focus", "layers": {id: {"title", "shown", "frames", "shapes"}}}}}, the focus being the
 *       frame the page may follow, if the scene names one.
 *   <li><b>What moves</b> goes only to the pages that show the scene, as JSON merge patches at
 *       {@code scenes.<id>.state}: {@code {"frames": {name: pose}, "values": {name: number},
 *       "styles": {shape: style}, "shapes": {layer: [shape, ...]}}}. Each layer updates every loop
 *       while a page shows the scene, and never while none does; a patch goes at most 20 times a
 *       second, holding only what moved by what the page can show. A robot standing still sends
 *       nothing.
 *   <li><b>A click</b> asks {@code GET /api/scenes/<id>/inspect/<shape>}, answered on the web
 *       server's thread from what the loop published: a shape of the moment can still be inspected
 *       for two seconds after it's gone.
 * </ul>
 *
 * <p>A pose goes as {@code [x, y, yaw]} for one level on the floor, else {@code [x, y, z, roll,
 * pitch, yaw]}: meters to the millimeter, degrees to the tenth.
 */
public final class Scenes {
  private final LongSupplier clock;
  private final Consumer<String> say;
  private final List<SceneFeed> feeds = new CopyOnWriteArrayList<>();

  /** The descriptions as last handed to the page, rebuilt when one changes. */
  private Map<String, Object> section = Map.of();

  /**
   * @param clock the robot's clock, in nanoseconds: patches are timed on it
   * @param say where a layer that throws is reported, once for each thing it throws
   */
  public Scenes(LongSupplier clock, Consumer<String> say) {
    this.clock = clock;
    this.say = say;
  }

  /**
   * Adds a scene, describing each of its layers now.
   *
   * @return this, to add another
   * @throws IllegalArgumentException if a scene has this one's id, or its layers' descriptions
   *     don't fit together (a shape in a frame the scene doesn't have, two shapes with one id): a
   *     mistake to fix before the robot runs, so it stops the program at once
   */
  public synchronized Scenes add(Scene scene) {
    for (SceneFeed feed : feeds) {
      if (feed.view().equals(scene.id())) {
        throw new IllegalArgumentException("two scenes have the id " + scene.id());
      }
    }
    feeds.add(new SceneFeed(scene, clock, say));
    return this;
  }

  /** Whether any scene has been added. Safe from any thread. */
  public boolean isEmpty() {
    return feeds.isEmpty();
  }

  /**
   * Every scene's description, by id, for the page's {@code scenes} section: each layer described
   * again first if its configuration changed. Call it in the robot loop, while a page is open.
   */
  public synchronized Map<String, Object> descriptions() {
    boolean changed = section.size() != feeds.size();
    for (SceneFeed feed : feeds) {
      Map<String, Object> before = feed.description();
      feed.redescribeChanged();
      changed |= feed.description() != before;
    }
    if (changed) {
      Map<String, Object> all = new LinkedHashMap<>();
      for (SceneFeed feed : feeds) {
        all.put(feed.view(), feed.description());
      }
      section = Collections.unmodifiableMap(all);
    }
    return section;
  }

  /** What moves in each scene, for the pages that show it. */
  public List<? extends Feed> feeds() {
    return feeds;
  }

  /**
   * What a click on a shape says. Safe from any thread: it reads only what the loop published.
   *
   * @return empty when there's no such scene, or nothing to say about the shape (a shape of the
   *     moment gone for more than two seconds, say)
   */
  public Optional<Inspection> inspect(String scene, String shapeId) {
    for (SceneFeed feed : feeds) {
      if (feed.view().equals(scene)) {
        return feed.inspect(shapeId);
      }
    }
    return Optional.empty();
  }

  /** Whether a scene has this id. Safe from any thread. */
  public boolean has(String scene) {
    return feeds.stream().anyMatch(feed -> feed.view().equals(scene));
  }
}
