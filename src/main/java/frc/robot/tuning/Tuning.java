package frc.robot.tuning;

import frc.robot.RobotConfig;
import frc.robot.lib.config.ConfigTuner;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import org.jspecify.annotations.Nullable;

/**
 * Live tuning of the robot's configuration. The values marked {@code @Tunable} can be changed from
 * the tuning page while the robot runs. Nothing is saved on the robot: what differs from the
 * committed configuration is listed, ready to write into {@code Robots.java}.
 *
 * <p>Only the robot loop changes anything. The web server's threads queue requests and read a
 * snapshot ({@link #view()}); each loop, {@link #periodic} applies the queued requests and logs
 * them as AdvantageKit inputs, so the configuration changes only between loops, and a replay makes
 * the same changes at the same moments.
 *
 * <p>Changes need tuning switched on, and are refused while the field system is connected.
 */
public final class Tuning {
  /**
   * What the tuning page shows. Safe to read from any thread.
   *
   * @param robot the robot's name
   * @param enabled whether tuning is switched on
   * @param fieldConnected whether the field system is connected, which locks tuning off
   * @param values every tunable value
   * @param changes the tuned values that differ from the committed configuration
   */
  public record View(
      String robot,
      boolean enabled,
      boolean fieldConnected,
      List<Value> values,
      List<ConfigTuner.Change> changes) {
    /** Whether any tuned value differs from the committed configuration. */
    public boolean hasChanges() {
      return !changes.isEmpty();
    }

    /** The paths of the tuned values that differ, for a warning. */
    public String changedPaths() {
      return changes.stream().map(ConfigTuner.Change::path).collect(Collectors.joining(", "));
    }
  }

  /**
   * One tunable value.
   *
   * @param path where it is in the configuration
   * @param unit the unit it is tuned in; empty for a plain number
   * @param committed its committed value
   * @param current its value now
   * @param refused why the last change to it was refused, if it was
   */
  public record Value(
      String path, String unit, double committed, double current, @Nullable String refused) {}

  /** A request from the page, waiting for the robot loop. */
  private record Request(TuningInputs.Kind kind, String path, double value) {}

  private final ConfigTuner<RobotConfig> tuner;
  private final Consumer<TuningInputs> logInputs;
  private final Consumer<View> onChange;
  private final Set<String> paths;
  private final ConcurrentLinkedQueue<Request> queue = new ConcurrentLinkedQueue<>();
  private final TuningInputs inputs = new TuningInputs();
  private final Map<String, String> refusals = new HashMap<>();
  private boolean enabled;
  private boolean fieldConnected;
  private volatile View view;

  /**
   * @param committed the robot's committed configuration
   * @param enabled whether tuning starts switched on
   * @param logInputs logs the requests one loop applies, or in replay reads them from the log:
   *     {@code inputs -> Logger.processInputs("Tuning", inputs)}
   * @param onChange told, in the robot loop, each time what the page shows changes
   */
  public Tuning(
      RobotConfig committed,
      boolean enabled,
      Consumer<TuningInputs> logInputs,
      Consumer<View> onChange) {
    this.tuner = new ConfigTuner<>(committed);
    this.enabled = enabled;
    this.logInputs = logInputs;
    this.onChange = onChange;
    this.paths =
        tuner.knobs().stream().map(ConfigTuner.Knob::path).collect(Collectors.toUnmodifiableSet());
    this.view = snapshot();
  }

  /** The configuration with every tuned value applied. Read it in the robot loop. */
  public RobotConfig config() {
    return tuner.current();
  }

  /** What the tuning page shows. Safe to call from any thread. */
  public View view() {
    return view;
  }

  /** Whether a path names a tunable value. Safe to call from any thread. */
  public boolean isTunable(String path) {
    return paths.contains(path);
  }

  /** Asks the robot loop to change a value. Safe to call from any thread. */
  public void requestSet(String path, double value) {
    queue.add(new Request(TuningInputs.Kind.SET, path, value));
  }

  /** Asks the robot loop to return every value to its committed value. */
  public void requestReset() {
    queue.add(new Request(TuningInputs.Kind.RESET, "", 0));
  }

  /** Asks the robot loop to switch tuning on or off. */
  public void requestEnabled(boolean on) {
    queue.add(new Request(on ? TuningInputs.Kind.ENABLE : TuningInputs.Kind.DISABLE, "", 0));
  }

  /**
   * Applies the requests queued since the last loop. Call it once per loop, before anything reads
   * {@link #config()}.
   *
   * @param fieldConnected whether the field system is connected
   */
  public void periodic(boolean fieldConnected) {
    inputs.enabled = enabled;
    drainInto(inputs);
    logInputs.accept(inputs); // in replay, this replaces them with the logged ones

    boolean before = enabled;
    enabled = inputs.enabled && !fieldConnected;
    boolean changed = enabled != before || fieldConnected != this.fieldConnected;
    this.fieldConnected = fieldConnected;
    for (int i = 0; i < inputs.kinds.length; i++) {
      apply(TuningInputs.Kind.valueOf(inputs.kinds[i]), inputs.paths[i], inputs.values[i]);
      changed = true;
    }
    if (changed) {
      view = snapshot();
      onChange.accept(view);
    }
  }

  /**
   * The tuned values that differ from the committed configuration, as lines to print or log, each
   * with the builder call that commits it. Empty when nothing differs.
   */
  public List<String> report() {
    View now = view;
    if (now.changes().isEmpty()) {
      return List.of();
    }
    List<String> lines = new ArrayList<>();
    lines.add(
        "Tuned values differ from the committed configuration of "
            + now.robot()
            + ". To keep them, write these into Robots.java:");
    for (ConfigTuner.Change change : now.changes()) {
      lines.add(
          "  "
              + change.path()
              + ": "
              + change.committed()
              + " -> "
              + change.tuned()
              + "   "
              + change.builderCall());
    }
    return lines;
  }

  private void drainInto(TuningInputs inputs) {
    if (queue.isEmpty()) {
      inputs.clearRequests();
      return;
    }
    List<Request> requests = new ArrayList<>();
    for (Request request = queue.poll(); request != null; request = queue.poll()) {
      requests.add(request);
    }
    inputs.kinds = requests.stream().map(r -> r.kind().name()).toArray(String[]::new);
    inputs.paths = requests.stream().map(Request::path).toArray(String[]::new);
    inputs.values = requests.stream().mapToDouble(Request::value).toArray();
  }

  private void apply(TuningInputs.Kind kind, String path, double value) {
    switch (kind) {
      case ENABLE -> enabled = !fieldConnected;
      case DISABLE -> enabled = false;
      case RESET -> {
        if (enabled) {
          tuner.reset();
          refusals.clear();
        }
      }
      case SET -> set(path, value);
    }
  }

  private void set(String path, double value) {
    if (fieldConnected) {
      refusals.put(path, "locked while the field is connected");
    } else if (!enabled) {
      refusals.put(path, "tuning is off");
    } else {
      try {
        tuner.set(path, value);
        refusals.remove(path);
      } catch (IllegalArgumentException refused) {
        refusals.put(path, String.valueOf(refused.getMessage()));
      }
    }
  }

  private View snapshot() {
    List<Value> values =
        tuner.knobs().stream()
            .map(
                knob ->
                    new Value(
                        knob.path(),
                        knob.unit(),
                        tuner.committedValue(knob.path()),
                        tuner.value(knob.path()),
                        refusals.get(knob.path())))
            .toList();
    return new View(tuner.current().name(), enabled, fieldConnected, values, tuner.changes());
  }
}
