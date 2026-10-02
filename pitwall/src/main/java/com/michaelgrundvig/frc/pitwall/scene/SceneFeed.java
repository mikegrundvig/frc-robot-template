package com.michaelgrundvig.frc.pitwall.scene;

import com.michaelgrundvig.frc.pitwall.push.Feed;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;

/**
 * One scene as it runs: its description, built from its layers and checked whole, and its moving
 * parts, which its layers set each loop while a page shows it and which go to those pages as a
 * {@link Feed}, at most 20 times a second and only what changed.
 *
 * <p>The robot loop's, but for {@link #inspect}, which a web server thread calls: it reads only the
 * suppliers the loop publishes whole.
 */
final class SceneFeed implements Feed, SceneState {
  /** The least time between two patches: at most 20 a second. */
  static final long INTERVAL_NANOS = 50_000_000L;

  /** How long a shape of the moment can still be inspected after it's gone. */
  static final long KEPT_NANOS = 2_000_000_000L;

  /** A move this much less than a step still counts as the step: floating point's slack. */
  private static final double SLACK = 1 - 1e-9;

  /** A frame's pose: as set now, and as last sent, and how far it must move to be sent again. */
  private static final class Pose {
    final String name;
    final double[] now = new double[6];
    final double[] sent = new double[6];
    SceneBuilder.Deadband deadband = SceneBuilder.Deadband.RESOLUTION;
    boolean set;
    boolean everSent;

    Pose(String name) {
      this.name = name;
    }
  }

  /** A value: as set now, and as last sent, and the least change worth sending. */
  private static final class Value {
    final String name;
    double resolution;
    double now;
    double sent;
    boolean set;
    boolean everSent;

    Value(String name, double resolution) {
      this.name = name;
      this.resolution = resolution;
    }
  }

  /** An inspection a shape of the moment offered, and when the shape went. */
  private record Held(Supplier<Inspection> inspection, long goneNanos) {}

  /** One layer as it runs. */
  private static final class Running {
    final Layer layer;
    Object configuration;
    SceneBuilder described = new SceneBuilder();
    Map<String, Supplier<Inspection>> inspections = Map.of();

    /** Its shapes of the moment, if {@code update} started them over; null if it didn't. */
    @Nullable ShapeList moment;

    List<Map<String, Object>> shapes = List.of();
    Map<String, Supplier<Inspection>> shapeInspections = Map.of();
    List<Map<String, Object>> shapesSent = List.of();
    String lastError = "";

    Running(Layer layer) {
      this.layer = layer;
      configuration = layer.configuration();
    }
  }

  private final Scene scene;
  private final LongSupplier clock;
  private final Consumer<String> say;
  private final List<Running> layers = new ArrayList<>();
  private Map<String, Object> description = Map.of();
  private Map<String, Pose> frames = new LinkedHashMap<>();
  private Map<String, Value> values = new LinkedHashMap<>();

  /** The frames and values again, in order, to go through each loop without an iterator. */
  private List<Pose> poseList = List.of();

  /** Every frame's name, the root's too: what a shape of the moment may be in. */
  private Set<String> frameNames = Set.of(SceneBuilder.ROOT);

  /** Every described shape's id: what a shape of the moment may reveal, besides its fellows. */
  private Set<String> describedShapes = Set.of();

  private List<Value> valueList = List.of();
  private final Map<String, String> styles = new LinkedHashMap<>();
  private Map<String, String> stylesSent = Map.of();

  /** Whether {@link #styles} may differ from what was sent: only then are they compared. */
  private boolean restyled;

  private long nextNanos = Long.MIN_VALUE;
  private @Nullable Running updating;

  /** Shapes of the moment that have gone, still inspectable for {@link #KEPT_NANOS}. */
  private final Map<String, Held> gone = new HashMap<>();

  /** When the first of {@link #gone} is to be forgotten. */
  private long forgetNanos = Long.MAX_VALUE;

  private boolean inspectionsChanged;
  private volatile Map<String, Supplier<Inspection>> inspections = Map.of();

  /**
   * Describes every layer of the scene.
   *
   * @param clock the robot's clock, in nanoseconds
   * @param say where a layer that throws is reported, once for each thing it throws
   * @throws IllegalArgumentException if the layers' descriptions don't fit together: a frame with
   *     no parent, two shapes with one id, a shape in a frame the scene doesn't have
   */
  SceneFeed(Scene scene, LongSupplier clock, Consumer<String> say) {
    this.scene = scene;
    this.clock = clock;
    this.say = say;
    for (Scene.Entry entry : scene.layers()) {
      Running running = new Running(entry.layer());
      running.described = describe(entry.layer());
      layers.add(running);
    }
    adopt(check(layers.stream().map(running -> running.described).toList()));
  }

  /** The scene's id: the view a page names to be sent it. */
  @Override
  public String view() {
    return scene.id();
  }

  @Override
  public List<String> at() {
    return List.of("scenes", scene.id(), "state");
  }

  /** The scene as the page draws it before anything moves: {@code {"title", "layers"}}. */
  Map<String, Object> description() {
    return description;
  }

  /**
   * Describes again each layer whose configuration changed. A description that doesn't fit, or a
   * configuration that can't be read or compared, is reported, and the description before stands: a
   * layer that throws costs only itself. Called in the robot loop while a page is open.
   */
  void redescribeChanged() {
    for (Running running : layers) {
      try {
        Object now = running.layer.configuration();
        if (Objects.equals(now, running.configuration)) {
          continue;
        }
        running.configuration = now;
        SceneBuilder described = describe(running.layer);
        List<SceneBuilder> all = new ArrayList<>();
        for (Running each : layers) {
          all.add(each == running ? described : each.described);
        }
        Checked checked = check(all);
        running.described = described;
        adopt(checked);
      } catch (RuntimeException e) {
        report(running, "couldn't be described again", e);
      }
    }
  }

  @Override
  public Map<String, @Nullable Object> patch() {
    long now = clock.getAsLong();
    for (int i = 0; i < layers.size(); i++) {
      Running running = layers.get(i);
      updating = running;
      try {
        running.layer.update(this);
        endMoment(running, now);
      } catch (RuntimeException e) {
        running.moment = null;
        report(running, "couldn't update", e);
      } finally {
        updating = null;
      }
    }
    keepInspections(now);
    if (now < nextNanos) {
      return Map.of();
    }
    Map<String, @Nullable Object> patch = changes();
    if (!patch.isEmpty()) {
      nextNanos = now + INTERVAL_NANOS;
    }
    return patch;
  }

  @Override
  public Map<String, Object> sent() {
    Map<String, Object> framesSent = new LinkedHashMap<>();
    frames.forEach(
        (name, pose) -> {
          if (pose.everSent) {
            framesSent.put(name, wire(pose.sent));
          }
        });
    Map<String, Object> valuesSent = new LinkedHashMap<>();
    values.forEach(
        (name, value) -> {
          if (value.everSent) {
            valuesSent.put(name, Wire.toResolution(value.sent, value.resolution));
          }
        });
    Map<String, Object> shapesSent = new LinkedHashMap<>();
    for (Running running : layers) {
      if (!running.shapesSent.isEmpty()) {
        shapesSent.put(running.layer.id(), running.shapesSent);
      }
    }
    Map<String, Object> sent = new LinkedHashMap<>();
    sent.put("frames", Collections.unmodifiableMap(framesSent));
    sent.put("values", Collections.unmodifiableMap(valuesSent));
    sent.put("styles", stylesSent);
    sent.put("shapes", Collections.unmodifiableMap(shapesSent));
    return Collections.unmodifiableMap(sent);
  }

  /**
   * What a click on a shape says: the shape's own inspection if it has one (a shape of the moment's
   * for two seconds after it's gone), else the first layer with something to say. From any thread.
   */
  Optional<Inspection> inspect(String shapeId) {
    Supplier<Inspection> own = inspections.get(shapeId);
    if (own != null) {
      return Optional.of(own.get());
    }
    for (Running running : layers) {
      Optional<Inspection> said = running.layer.inspect(shapeId);
      if (said.isPresent()) {
        return said;
      }
    }
    return Optional.empty();
  }

  // ---- SceneState, for the layer updating ----

  @Override
  public long nowNanos() {
    return clock.getAsLong();
  }

  @Override
  public void pose(
      String frame, double x, double y, double z, double roll, double pitch, double yaw) {
    Pose pose = frames.get(frame);
    if (pose == null) {
      throw new IllegalArgumentException("scene " + scene.id() + " has no frame named " + frame);
    }
    if (!(Double.isFinite(x)
        && Double.isFinite(y)
        && Double.isFinite(z)
        && Double.isFinite(roll)
        && Double.isFinite(pitch)
        && Double.isFinite(yaw))) {
      return;
    }
    pose.now[0] = x;
    pose.now[1] = y;
    pose.now[2] = z;
    pose.now[3] = roll;
    pose.now[4] = pitch;
    pose.now[5] = yaw;
    pose.set = true;
  }

  @Override
  public void value(String name, double value) {
    Value held = values.get(name);
    if (held == null) {
      throw new IllegalArgumentException("scene " + scene.id() + " has no value named " + name);
    }
    if (Double.isFinite(value)) {
      held.now = value;
      held.set = true;
    }
  }

  @Override
  public void style(String shape, @Nullable Style style) {
    if (style == null) {
      restyled |= styles.remove(shape) != null;
    } else {
      String wire = style.wire();
      restyled |= !wire.equals(styles.put(Wire.name("A shape's id", shape), wire));
    }
  }

  @Override
  public Shapes shapes() {
    Running running = updating;
    if (running == null) {
      throw new IllegalStateException("a layer's shapes of the moment are set in its update");
    }
    ShapeList moment = new ShapeList();
    running.moment = moment;
    return moment;
  }

  // ---- Describing ----

  private static SceneBuilder describe(Layer layer) {
    SceneBuilder builder = new SceneBuilder();
    layer.describe(builder);
    return builder;
  }

  /** The scene's description checked whole, ready to adopt. */
  private record Checked(
      List<Map<String, Object>> layers,
      List<Map<String, Supplier<Inspection>>> inspections,
      Set<String> frames,
      Map<String, Double> values,
      Map<String, SceneBuilder.Deadband> deadbands,
      Set<String> shapes) {}

  /**
   * Checks the layers' descriptions fit together, and puts them in the page's form.
   *
   * @throws IllegalArgumentException saying what doesn't fit
   */
  private Checked check(List<SceneBuilder> described) {
    Map<String, SceneBuilder.Frame> allFrames = new LinkedHashMap<>();
    Map<String, Double> allValues = new LinkedHashMap<>();
    for (SceneBuilder builder : described) {
      builder
          .frames()
          .forEach(
              (name, frame) -> {
                SceneBuilder.Frame before = allFrames.putIfAbsent(name, frame);
                if (before != null && !before.equals(frame)) {
                  throw new IllegalArgumentException(
                      "scene " + scene.id() + ": two layers place frame " + name + " differently");
                }
              });
      builder
          .values()
          .forEach(
              (name, resolution) -> {
                Double before = allValues.putIfAbsent(name, resolution);
                if (before != null && !before.equals(resolution)) {
                  throw new IllegalArgumentException(
                      "scene "
                          + scene.id()
                          + ": two layers declare value "
                          + name
                          + " differently");
                }
              });
    }
    for (String name : allFrames.keySet()) {
      Set<String> seen = new HashSet<>();
      String at = name;
      while (!at.equals(SceneBuilder.ROOT)) {
        if (!seen.add(at)) {
          throw new IllegalArgumentException(
              "scene " + scene.id() + ": frame " + name + " is its own ancestor");
        }
        SceneBuilder.Frame frame = allFrames.get(at);
        if (frame == null) {
          throw new IllegalArgumentException(
              "scene " + scene.id() + ": frame " + name + "'s parent " + at + " isn't a frame");
        }
        at = frame.parent();
      }
    }
    Set<String> frameNames = new LinkedHashSet<>(allFrames.keySet());
    frameNames.add(SceneBuilder.ROOT);
    String focus = scene.focus();
    if (focus != null && !frameNames.contains(focus)) {
      throw new IllegalArgumentException(
          "scene " + scene.id() + " follows frame " + focus + ", which it doesn't have");
    }
    Set<String> ids = new HashSet<>();
    List<Map<String, Object>> wires = new ArrayList<>();
    List<Map<String, Supplier<Inspection>>> inspections = new ArrayList<>();
    List<Scene.Entry> entries = scene.layers();
    for (int i = 0; i < described.size(); i++) {
      SceneBuilder builder = described.get(i);
      Scene.Entry entry = entries.get(i);
      Map<String, Supplier<Inspection>> inspected = new LinkedHashMap<>();
      List<Object> shapes = new ArrayList<>();
      for (Shape<?> shape : builder.shapes()) {
        if (!ids.add(shape.id())) {
          throw new IllegalArgumentException(
              "scene " + scene.id() + " has two shapes with the id " + shape.id());
        }
        shapes.add(fitted(shape, frameNames, allValues.keySet()));
        Supplier<Inspection> inspection = shape.inspection();
        if (inspection != null) {
          inspected.put(shape.id(), inspection);
        }
      }
      Map<String, Object> framesWire = new LinkedHashMap<>();
      builder.frames().forEach((name, frame) -> framesWire.put(name, frame.wire()));
      Map<String, Object> wire = new LinkedHashMap<>();
      wire.put("title", entry.layer().title());
      wire.put("shown", entry.visibility() == Scene.Visibility.ON);
      wire.put("frames", Collections.unmodifiableMap(framesWire));
      wire.put("shapes", List.copyOf(shapes));
      wires.add(Collections.unmodifiableMap(wire));
      inspections.add(Map.copyOf(inspected));
    }
    for (SceneBuilder builder : described) {
      for (Shape<?> shape : builder.shapes()) {
        checkReveals(shape, ids);
      }
    }
    Map<String, SceneBuilder.Deadband> deadbands = new LinkedHashMap<>();
    for (SceneBuilder builder : described) {
      deadbands.putAll(builder.deadbands());
    }
    return new Checked(wires, inspections, frameNames, allValues, deadbands, Set.copyOf(ids));
  }

  /**
   * Checks the shapes a shape reveals are there.
   *
   * @throws IllegalArgumentException naming the first that isn't
   */
  private void checkReveals(Shape<?> shape, Set<String> shapes) {
    for (String revealed : shape.reveals()) {
      if (!shapes.contains(revealed)) {
        throw new IllegalArgumentException(
            "scene "
                + scene.id()
                + ": shape "
                + shape.id()
                + " reveals shape "
                + revealed
                + ", which the scene doesn't have");
      }
    }
  }

  /**
   * A shape in the page's form, once it's checked against the scene's frames and values.
   *
   * @throws IllegalArgumentException if it's in a frame the scene doesn't have, or follows a value
   *     it doesn't
   */
  private Map<String, Object> fitted(
      Shape<?> shape, Set<String> frameNames, Set<String> valueNames) {
    List<String> reached = new ArrayList<>(shape.reaches());
    reached.add(shape.frame());
    for (String frame : reached) {
      if (!frameNames.contains(frame)) {
        throw new IllegalArgumentException(
            "scene "
                + scene.id()
                + ": shape "
                + shape.id()
                + " is in frame "
                + frame
                + ", which the scene doesn't have");
      }
    }
    for (String value : shape.values()) {
      if (!valueNames.contains(value)) {
        throw new IllegalArgumentException(
            "scene "
                + scene.id()
                + ": shape "
                + shape.id()
                + " follows value "
                + value
                + ", which the scene doesn't declare");
      }
    }
    return Collections.unmodifiableMap(shape.wire());
  }

  /** Takes a checked description as the scene's: what the page draws, and what can move. */
  private void adopt(Checked checked) {
    Map<String, Object> layersWire = new LinkedHashMap<>();
    for (int i = 0; i < layers.size(); i++) {
      Running running = layers.get(i);
      layersWire.put(running.layer.id(), checked.layers().get(i));
      running.inspections = checked.inspections().get(i);
    }
    Map<String, Object> wire = new LinkedHashMap<>();
    wire.put("title", scene.title());
    String focus = scene.focus();
    if (focus != null) {
      wire.put("focus", focus);
    }
    wire.put("layers", Collections.unmodifiableMap(layersWire));
    description = Collections.unmodifiableMap(wire);
    Map<String, Pose> poses = new LinkedHashMap<>();
    for (String name : checked.frames()) {
      if (!name.equals(SceneBuilder.ROOT)) {
        Pose pose = frames.getOrDefault(name, new Pose(name));
        pose.deadband = checked.deadbands().getOrDefault(name, SceneBuilder.Deadband.RESOLUTION);
        poses.put(name, pose);
      }
    }
    frames = poses;
    poseList = List.copyOf(poses.values());
    frameNames = Set.copyOf(checked.frames());
    describedShapes = checked.shapes();
    Map<String, Value> held = new LinkedHashMap<>();
    checked
        .values()
        .forEach(
            (name, resolution) -> {
              Value value = values.getOrDefault(name, new Value(name, resolution));
              value.resolution = resolution;
              held.put(name, value);
            });
    values = held;
    valueList = List.copyOf(held.values());
    publishInspections();
  }

  // ---- Updating ----

  /**
   * Takes a layer's shapes of the moment, if its update started them over. An inspectable shape
   * that went (or now says something else) stays inspectable, as it was, for {@link #KEPT_NANOS}.
   */
  private void endMoment(Running running, long now) {
    ShapeList moment = running.moment;
    if (moment == null) {
      return;
    }
    running.moment = null;
    Map<String, Map<String, Object>> byId = new LinkedHashMap<>();
    Map<String, Supplier<Inspection>> inspected = new LinkedHashMap<>();
    for (Shape<?> shape : moment.shapes()) {
      byId.put(shape.id(), fitted(shape, frameNames, values.keySet()));
      Supplier<Inspection> inspection = shape.inspection();
      if (inspection != null) {
        inspected.put(shape.id(), inspection);
      } else {
        inspected.remove(shape.id());
      }
    }
    for (Shape<?> shape : moment.shapes()) {
      if (!shape.reveals().isEmpty()) {
        Set<String> there = new HashSet<>(describedShapes);
        there.addAll(byId.keySet());
        checkReveals(shape, there);
      }
    }
    for (Map.Entry<String, Supplier<Inspection>> before : running.shapeInspections.entrySet()) {
      if (inspected.get(before.getKey()) != before.getValue()) {
        gone.put(before.getKey(), new Held(before.getValue(), now));
        forgetNanos = Math.min(forgetNanos, now + KEPT_NANOS);
      }
    }
    running.shapes = List.copyOf(byId.values());
    running.shapeInspections = inspected;
    inspectionsChanged = true;
  }

  /**
   * Forgets the shapes of the moment gone for more than {@link #KEPT_NANOS}, and publishes what can
   * be inspected whenever that changes. Each loop, so it costs nothing while nothing does.
   */
  private void keepInspections(long now) {
    if (now >= forgetNanos) {
      forgetNanos = Long.MAX_VALUE;
      for (Iterator<Held> held = gone.values().iterator(); held.hasNext(); ) {
        long forget = held.next().goneNanos() + KEPT_NANOS;
        if (now >= forget) {
          held.remove();
          inspectionsChanged = true;
        } else {
          forgetNanos = Math.min(forgetNanos, forget);
        }
      }
    }
    if (inspectionsChanged) {
      publishInspections();
    }
  }

  /**
   * Publishes what can be inspected now, whole, for the web server's threads to read: what's
   * described, what has gone lately, and what's there of the moment.
   */
  private void publishInspections() {
    inspectionsChanged = false;
    Map<String, Supplier<Inspection>> all = new HashMap<>();
    for (Running running : layers) {
      all.putAll(running.inspections);
    }
    gone.forEach((id, held) -> all.put(id, held.inspection()));
    for (Running running : layers) {
      all.putAll(running.shapeInspections);
    }
    inspections = Map.copyOf(all);
  }

  /**
   * What changed by what the page can show since last sent, remembered as sent. Nothing changed,
   * nothing made: it's asked every loop the robot stands still.
   */
  private Map<String, @Nullable Object> changes() {
    Map<String, Object> movedFrames = null;
    for (int i = 0; i < poseList.size(); i++) {
      Pose pose = poseList.get(i);
      if (pose.set && (!pose.everSent || moved(pose))) {
        System.arraycopy(pose.now, 0, pose.sent, 0, 6);
        pose.everSent = true;
        if (movedFrames == null) {
          movedFrames = new LinkedHashMap<>();
        }
        movedFrames.put(pose.name, wire(pose.sent));
      }
    }
    Map<String, Object> movedValues = null;
    for (int i = 0; i < valueList.size(); i++) {
      Value value = valueList.get(i);
      if (value.set
          && (!value.everSent || Math.abs(value.now - value.sent) >= value.resolution * SLACK)) {
        value.sent = value.now;
        value.everSent = true;
        if (movedValues == null) {
          movedValues = new LinkedHashMap<>();
        }
        movedValues.put(value.name, Wire.toResolution(value.sent, value.resolution));
      }
    }
    Map<String, @Nullable Object> restyledShapes = restyle();
    Map<String, Object> reshaped = null;
    for (int i = 0; i < layers.size(); i++) {
      Running running = layers.get(i);
      if (running.shapes != running.shapesSent && !running.shapes.equals(running.shapesSent)) {
        if (reshaped == null) {
          reshaped = new LinkedHashMap<>();
        }
        reshaped.put(running.layer.id(), running.shapes);
      }
      running.shapesSent = running.shapes;
    }
    if (movedFrames == null && movedValues == null && restyledShapes == null && reshaped == null) {
      return Map.of();
    }
    Map<String, @Nullable Object> patch = new LinkedHashMap<>();
    if (movedFrames != null) {
      patch.put("frames", Collections.unmodifiableMap(movedFrames));
    }
    if (movedValues != null) {
      patch.put("values", Collections.unmodifiableMap(movedValues));
    }
    if (restyledShapes != null) {
      patch.put("styles", Collections.unmodifiableMap(restyledShapes));
    }
    if (reshaped != null) {
      patch.put("shapes", Collections.unmodifiableMap(reshaped));
    }
    return patch;
  }

  /**
   * The shapes whose style differs from what was sent (null for one back as described), remembered
   * as sent; null when none does. Compared only when a layer set a style other than it had.
   */
  private @Nullable Map<String, @Nullable Object> restyle() {
    if (!restyled) {
      return null;
    }
    restyled = false;
    Map<String, @Nullable Object> changed = new LinkedHashMap<>();
    styles.forEach(
        (shape, style) -> {
          if (!style.equals(stylesSent.get(shape))) {
            changed.put(shape, style);
          }
        });
    for (String shape : stylesSent.keySet()) {
      if (!styles.containsKey(shape)) {
        changed.put(shape, null);
      }
    }
    stylesSent = Collections.unmodifiableMap(new LinkedHashMap<>(styles));
    return changed.isEmpty() ? null : changed;
  }

  /**
   * Whether a pose has moved its deadband since last sent: by default a step, a millimeter or a
   * tenth of a degree.
   */
  private static boolean moved(Pose pose) {
    for (int i = 0; i < 3; i++) {
      if (Math.abs(pose.now[i] - pose.sent[i]) >= pose.deadband.meters() * SLACK) {
        return true;
      }
    }
    for (int i = 3; i < 6; i++) {
      if (Wire.angleBetween(pose.now[i], pose.sent[i]) >= pose.deadband.radians() * SLACK) {
        return true;
      }
    }
    return false;
  }

  private static List<Number> wire(double[] pose) {
    return Wire.pose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
  }

  /** Says once what a layer threw, until it throws something else. */
  private void report(Running running, String what, RuntimeException e) {
    String message =
        "Pitwall: scene " + scene.id() + ", layer " + running.layer.id() + " " + what + ": " + e;
    if (!message.equals(running.lastError)) {
      running.lastError = message;
      say.accept(message);
    }
  }
}
