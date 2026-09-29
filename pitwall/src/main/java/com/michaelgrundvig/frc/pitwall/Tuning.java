package com.michaelgrundvig.frc.pitwall;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;
import java.util.stream.Collectors;

/**
 * Live tuning of a configuration: the values marked {@link Tunable} can be changed while the robot
 * runs. Nothing is saved on the robot: what differs from the committed configuration is listed as
 * the builder calls that commit it.
 *
 * <p>Only the robot loop changes anything. Requests, from the web server's threads, wait in a
 * queue; each loop, {@link #periodic} applies them in order, each one all or nothing, and logs
 * them, so the configuration changes only between loops and a replay makes the same changes at the
 * same moments. Each request's future completes with what became of it.
 *
 * <p>Changes need tuning switched on, and are refused while the field system is connected.
 *
 * @param <T> the configuration record at the root
 */
public final class Tuning<T extends Record> {
  /**
   * The state of tuning, as the page shows it. Safe to read from any thread.
   *
   * @param name what is being tuned, such as the robot's name
   * @param enabled whether tuning is switched on
   * @param fieldConnected whether the field system is connected, which locks tuning off
   * @param values every tunable value
   * @param changes the tuned values that differ from the committed configuration
   */
  public record View(
      String name,
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
   */
  public record Value(String path, String unit, double committed, double current) {}

  /**
   * What became of a request.
   *
   * @param result whether it was applied, refused by the configuration's checks, or locked out
   * @param detail why it wasn't applied; empty when it was
   */
  public record Outcome(Result result, String detail) {
    /** Whether a request was applied, and if not, why not. */
    public enum Result {
      /** Applied, all of it. */
      APPLIED,
      /** Refused by the configuration's checks; nothing changed. */
      REFUSED,
      /** Refused because tuning is off or the field system is connected; nothing changed. */
      LOCKED
    }

    static Outcome applied() {
      return new Outcome(Result.APPLIED, "");
    }
  }

  /** A request from the page, waiting for the robot loop. */
  private record Request(
      TuningInputs.Kind kind, Map<String, Double> values, CompletableFuture<Outcome> outcome) {}

  private final String name;
  private final ConfigTuner<T> tuner;
  private final Consumer<TuningInputs> logInputs;
  private final Consumer<View> onChange;
  private final Set<String> paths;
  private final ConcurrentLinkedQueue<Request> queue = new ConcurrentLinkedQueue<>();
  private final TuningInputs inputs = new TuningInputs();
  private boolean enabled;
  private boolean fieldConnected;
  private volatile View view;

  /**
   * @param name what is being tuned, such as the robot's name; the page and the report show it
   * @param committed the committed configuration
   * @param enabled whether tuning starts switched on
   * @param logInputs logs the requests one loop applies, or in a replay replaces them with the
   *     logged ones (with AdvantageKit, through {@code Logger.processInputs})
   * @param onChange told, in the robot loop, each time the state changes
   */
  public Tuning(
      String name,
      T committed,
      boolean enabled,
      Consumer<TuningInputs> logInputs,
      Consumer<View> onChange) {
    this.name = name;
    this.tuner = new ConfigTuner<>(committed);
    this.enabled = enabled;
    this.logInputs = logInputs;
    this.onChange = onChange;
    this.paths =
        tuner.knobs().stream().map(ConfigTuner.Knob::path).collect(Collectors.toUnmodifiableSet());
    this.view = snapshot();
  }

  /** The configuration with every tuned value applied. Read it in the robot loop. */
  public T config() {
    return tuner.current();
  }

  /** The state of tuning. Safe to call from any thread. */
  public View view() {
    return view;
  }

  /** Whether a path names a tunable value. Safe to call from any thread. */
  public boolean isTunable(String path) {
    return paths.contains(path);
  }

  /**
   * Asks the robot loop to change several values at once, all or none. Safe to call from any
   * thread.
   *
   * @param values new values by path, each in its value's unit
   * @return completes, in the robot loop, with what became of the request
   */
  public CompletableFuture<Outcome> requestChanges(Map<String, Double> values) {
    return enqueue(TuningInputs.Kind.SET, new LinkedHashMap<>(values));
  }

  /** Asks the robot loop to return every value to its committed value. */
  public CompletableFuture<Outcome> requestReset() {
    return enqueue(TuningInputs.Kind.RESET, Map.of());
  }

  /** Asks the robot loop to switch tuning on or off. */
  public CompletableFuture<Outcome> requestEnabled(boolean on) {
    return enqueue(on ? TuningInputs.Kind.ENABLE : TuningInputs.Kind.DISABLE, Map.of());
  }

  /**
   * Applies the requests queued since the last loop. Call it once per loop, before anything reads
   * {@link #config()}.
   *
   * @param fieldConnected whether the field system is connected
   */
  public void periodic(boolean fieldConnected) {
    inputs.enabled = enabled;
    List<CompletableFuture<Outcome>> outcomes = drainInto(inputs);
    logInputs.accept(inputs); // in replay, this replaces them with the logged ones

    boolean before = enabled;
    enabled = inputs.enabled && !fieldConnected;
    boolean changed = enabled != before || fieldConnected != this.fieldConnected;
    this.fieldConnected = fieldConnected;
    // Entries with the same request number are one request, applied all or none.
    List<Outcome> results = new ArrayList<>();
    for (int start = 0, end; start < inputs.kinds.length; start = end) {
      int request = inputs.requests[start];
      end = start + 1;
      while (end < inputs.kinds.length && inputs.requests[end] == request) {
        end++;
      }
      results.add(apply(start, end));
      changed = true;
    }
    if (changed) {
      view = snapshot();
      onChange.accept(view);
    }
    // Only now, so whoever is waiting sees the state their request left. In replay, no one is.
    for (int i = 0; i < outcomes.size() && i < results.size(); i++) {
      outcomes.get(i).complete(results.get(i));
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
            + now.name()
            + ". To keep them, write these builder calls into it:");
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

  private CompletableFuture<Outcome> enqueue(TuningInputs.Kind kind, Map<String, Double> values) {
    CompletableFuture<Outcome> outcome = new CompletableFuture<>();
    queue.add(new Request(kind, values, outcome));
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
    List<String> kinds = new ArrayList<>();
    List<String> paths = new ArrayList<>();
    List<Double> values = new ArrayList<>();
    List<Integer> numbers = new ArrayList<>();
    for (int number = 0; number < requests.size(); number++) {
      Request request = requests.get(number);
      if (request.values().isEmpty()) {
        kinds.add(request.kind().name());
        paths.add("");
        values.add(0.0);
        numbers.add(number);
      }
      for (Map.Entry<String, Double> entry : request.values().entrySet()) {
        kinds.add(request.kind().name());
        paths.add(entry.getKey());
        values.add(entry.getValue());
        numbers.add(number);
      }
    }
    inputs.kinds = kinds.toArray(String[]::new);
    inputs.paths = paths.toArray(String[]::new);
    inputs.values = values.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.requests = numbers.stream().mapToInt(Integer::intValue).toArray();
    return requests.stream().map(Request::outcome).toList();
  }

  /** Applies one request: the entries from {@code start} to just before {@code end}. */
  private Outcome apply(int start, int end) {
    return switch (TuningInputs.Kind.valueOf(inputs.kinds[start])) {
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
      case RESET -> change(tuner::reset);
      case SET -> {
        Map<String, Double> values = new LinkedHashMap<>();
        for (int i = start; i < end; i++) {
          if (!inputs.paths[i].isEmpty()) {
            values.put(inputs.paths[i], inputs.values[i]);
          }
        }
        yield change(() -> tuner.set(values));
      }
    };
  }

  /** Makes a change to the configuration, if tuning allows it, and says what became of it. */
  private Outcome change(Runnable change) {
    if (fieldConnected) {
      return locked("the field system is connected");
    }
    if (!enabled) {
      return locked("tuning is off");
    }
    try {
      change.run();
      return Outcome.applied();
    } catch (RuntimeException refused) {
      // Whatever a configuration's checks throw refuses the change: a request from the page must
      // never stop the robot loop.
      String why = refused.getMessage();
      return new Outcome(Outcome.Result.REFUSED, why != null ? why : refused.toString());
    }
  }

  private static Outcome locked(String why) {
    return new Outcome(Outcome.Result.LOCKED, "Tuning is locked: " + why);
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
                        tuner.value(knob.path())))
            .toList();
    return new View(name, enabled, fieldConnected, values, tuner.changes());
  }
}
