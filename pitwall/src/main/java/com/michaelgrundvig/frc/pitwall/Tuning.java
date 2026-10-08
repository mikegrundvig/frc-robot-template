package com.michaelgrundvig.frc.pitwall;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.jspecify.annotations.Nullable;

/**
 * Live tuning's switch and what follows from it. The values themselves are WPILib tunables, changed
 * through the tee, most of them the values of {@link LiveConfig live configurations}; this is the
 * rest: whether tuning is on, the field's lock, putting every configuration back as committed, and
 * saying what differs.
 *
 * <p>Nothing is saved on the robot: what differs from the committed configurations is listed as the
 * builder calls that commit it. Changes need tuning switched on, and are refused while the field
 * system is connected. When the field system connects, every configuration returns to committed: a
 * match runs the committed configuration.
 *
 * <p>Only the robot loop changes anything. Requests, from the web server's threads, wait in a
 * queue; each loop, {@link #periodic} applies them in order and logs them, so a replay makes the
 * same changes at the same moments. Each request's future completes with what became of it.
 */
public final class Tuning {
  /**
   * The state of tuning, as the page shows it. Safe to read from any thread.
   *
   * @param name what is being tuned: the robot's name
   * @param enabled whether tuning is switched on
   * @param fieldConnected whether the field system is connected, which locks tuning off
   * @param configs every live configuration
   */
  public record View(
      String name, boolean enabled, boolean fieldConnected, List<ConfigView> configs) {
    /** Whether any tuned value differs from its committed one. */
    public boolean hasChanges() {
      return configs.stream().anyMatch(config -> !config.changes().isEmpty());
    }

    /** The tuned values that differ, as {@code "drivetrain gains.steerKp"}, for a warning. */
    public String changedPaths() {
      return configs.stream()
          .flatMap(config -> config.changes().stream().map(c -> config.name() + " " + c.path()))
          .collect(Collectors.joining(", "));
    }
  }

  /**
   * A live configuration, as the page shows it.
   *
   * @param name what it configures
   * @param values its tunable values
   * @param changes what differs from committed, with the builder calls that commit it
   */
  public record ConfigView(String name, List<Value> values, List<LiveConfig.Change> changes) {}

  /**
   * One tunable value of a configuration.
   *
   * @param path the WPILib tunable it is published as, such as {@code /Drive/config/gains/steerKp}
   * @param field where it is in the configuration, such as {@code gains.steerKp}
   * @param unit the unit it is tuned in; empty for a plain number
   * @param limits the range it may be tuned in
   * @param committed its committed value
   * @param current its value now
   * @param record the display name of the record that declares it, such as {@code
   *     DrivetrainControl.Gains}
   * @param group the path of that record in the configuration, such as {@code gains}; empty at the
   *     root
   * @param meaning what it means, its {@code @param} there; empty when there's none
   */
  public record Value(
      String path,
      String field,
      String unit,
      Limits limits,
      double committed,
      double current,
      String record,
      String group,
      String meaning) {}

  /** A request from the page, waiting for the robot loop. */
  private record Request(TuningInputs.Kind kind, CompletableFuture<Outcome> outcome) {}

  private final String name;
  private final Supplier<List<LiveConfig<?>>> configs;
  private final PathFinder paths;
  private final Consumer<TuningInputs> logInputs;
  private final Consumer<View> onChange;
  private final ConcurrentLinkedQueue<Request> queue = new ConcurrentLinkedQueue<>();
  private final TuningInputs inputs = new TuningInputs();
  private volatile boolean enabled;
  private volatile boolean fieldConnected;

  /**
   * What the last loop told {@code onChange}: whether tuning was on, the field, and each config.
   */
  private boolean toldEnabled;

  private boolean toldFieldConnected;

  /** Each configuration as last told: a change makes a new record, so identity tells. */
  private List<Object> told;

  /** Where a configuration's value is published as a WPILib tunable. */
  @FunctionalInterface
  interface PathFinder {
    @Nullable String pathOf(LiveConfig<?> config, String field);
  }

  /**
   * @param name what is being tuned: the robot's name, for the page and the report
   * @param configs the live configurations, as published
   * @param paths where each configuration value is published
   * @param enabled whether tuning starts switched on
   * @param logInputs logs the requests one loop applies, or in a replay replaces them with the
   *     logged ones (with AdvantageKit, through {@code Logger.processInputs})
   * @param onChange told, in the robot loop, each time the state changes
   */
  Tuning(
      String name,
      Supplier<List<LiveConfig<?>>> configs,
      PathFinder paths,
      boolean enabled,
      Consumer<TuningInputs> logInputs,
      Consumer<View> onChange) {
    this.name = name;
    this.configs = configs;
    this.paths = paths;
    this.enabled = enabled;
    this.logInputs = logInputs;
    this.onChange = onChange;
    this.toldEnabled = enabled;
    this.told = configs.get().stream().map(config -> (Object) config.get()).toList();
  }

  /** The state of tuning, as it is now. Safe to call from any thread. */
  public View view() {
    return snapshot();
  }

  /** What is being tuned: the robot's name. */
  public String name() {
    return name;
  }

  /** Whether tuning is switched on. Safe to call from any thread. */
  public boolean isEnabled() {
    return enabled;
  }

  /** Whether the field system is connected, which locks tuning off. Safe from any thread. */
  public boolean isFieldConnected() {
    return fieldConnected;
  }

  /**
   * Why changes to tunables are refused right now, or null while they're allowed. Safe to call from
   * any thread.
   */
  public @Nullable String whyLocked() {
    if (fieldConnected) {
      return "the field system is connected";
    }
    return enabled ? null : "tuning is off";
  }

  /** Asks the robot loop to return every configuration value to its committed one. */
  public CompletableFuture<Outcome> requestReset() {
    return enqueue(TuningInputs.Kind.RESET);
  }

  /** Asks the robot loop to switch tuning on or off. */
  public CompletableFuture<Outcome> requestEnabled(boolean on) {
    return enqueue(on ? TuningInputs.Kind.ENABLE : TuningInputs.Kind.DISABLE);
  }

  /**
   * Applies the requests queued since the last loop. Call it once per loop, before anything reads a
   * configuration.
   *
   * @param fieldConnected whether the field system is connected
   */
  public void periodic(boolean fieldConnected) {
    inputs.enabled = enabled;
    List<CompletableFuture<Outcome>> outcomes = drainInto(inputs);
    logInputs.accept(inputs); // in replay, this replaces them with the logged ones

    enabled = inputs.enabled && !fieldConnected;
    if (fieldConnected && !this.fieldConnected) {
      // A match runs the committed configuration: what was tuned in the pit goes back, and the
      // report printed before (see Pitwall) keeps the numbers for committing.
      configs.get().forEach(LiveConfig::revert);
    }
    this.fieldConnected = fieldConnected;
    List<Outcome> results = new ArrayList<>();
    for (String kind : inputs.kinds) {
      results.add(apply(TuningInputs.Kind.valueOf(kind)));
    }
    List<Object> current = configs.get().stream().map(config -> (Object) config.get()).toList();
    if (enabled != toldEnabled
        || fieldConnected != toldFieldConnected
        || !sameObjects(current, told)) {
      toldEnabled = enabled;
      toldFieldConnected = fieldConnected;
      told = current;
      onChange.accept(snapshot());
    }
    // Only now, so whoever is waiting sees the state their request left. In replay, no one is.
    for (int i = 0; i < outcomes.size() && i < results.size(); i++) {
      outcomes.get(i).complete(results.get(i));
    }
  }

  /**
   * The tuned values that differ from the committed configurations, as lines to print or log, each
   * with the builder call that commits it. Empty when nothing differs.
   */
  public List<String> report() {
    List<String> lines = new ArrayList<>();
    for (LiveConfig<?> config : configs.get()) {
      lines.addAll(config.report());
    }
    return lines;
  }

  private CompletableFuture<Outcome> enqueue(TuningInputs.Kind kind) {
    CompletableFuture<Outcome> outcome = new CompletableFuture<>();
    queue.add(new Request(kind, outcome));
    return outcome;
  }

  /** Moves the queued requests into the inputs, and returns their futures in request order. */
  private List<CompletableFuture<Outcome>> drainInto(TuningInputs inputs) {
    if (queue.isEmpty()) {
      inputs.clearRequests();
      return List.of();
    }
    List<Request> requests = new ArrayList<>();
    for (Request request = queue.poll(); request != null; request = queue.poll()) {
      requests.add(request);
    }
    inputs.kinds = requests.stream().map(request -> request.kind().name()).toArray(String[]::new);
    return requests.stream().map(Request::outcome).toList();
  }

  private Outcome apply(TuningInputs.Kind kind) {
    return switch (kind) {
      case ENABLE -> {
        if (fieldConnected) {
          yield locked("the field system is connected");
        }
        enabled = true;
        yield Outcome.applied();
      }
      case DISABLE -> {
        enabled = false;
        yield Outcome.applied();
      }
      case RESET -> {
        if (fieldConnected) {
          yield locked("the field system is connected");
        }
        if (!enabled) {
          yield locked("tuning is off");
        }
        configs.get().forEach(LiveConfig::revert);
        yield Outcome.applied();
      }
    };
  }

  private static boolean sameObjects(List<Object> a, List<Object> b) {
    if (a.size() != b.size()) {
      return false;
    }
    for (int i = 0; i < a.size(); i++) {
      if (a.get(i) != b.get(i)) {
        return false;
      }
    }
    return true;
  }

  private static Outcome locked(String why) {
    return new Outcome(Outcome.Result.LOCKED, "Tuning is locked: " + why);
  }

  private View snapshot() {
    List<ConfigView> views = new ArrayList<>();
    for (LiveConfig<?> config : configs.get()) {
      List<Value> values = new ArrayList<>();
      LiveConfig.Documentation docs = config.documentation(); // read when it was published
      for (LiveConfig.Knob knob : config.knobs()) {
        String path = paths.pathOf(config, knob.path());
        if (path != null) {
          LiveConfig.Documentation.Value doc = docs.value(knob.path());
          values.add(
              new Value(
                  path,
                  knob.path(),
                  knob.unit(),
                  knob.limits(),
                  config.committedValue(knob.path()),
                  config.value(knob.path()),
                  doc.record(),
                  doc.group(),
                  doc.meaning()));
        }
      }
      views.add(new ConfigView(config.name(), List.copyOf(values), config.changes()));
    }
    return new View(name, enabled, fieldConnected, List.copyOf(views));
  }
}
