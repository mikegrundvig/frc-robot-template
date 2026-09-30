package com.michaelgrundvig.frc.pitwall;

import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableBackend;
import org.wpilib.tunable.TunableBase;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableFloat;
import org.wpilib.tunable.TunableInt;
import org.wpilib.tunable.TunableLong;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.tunable.util.PathUtil;

/**
 * Pitwall's tee into WPILib's tunables: a backend between every tunable in the robot program and
 * the backend that publishes them (NetworkTables, on a robot), so dashboards work as before while
 * Pitwall:
 *
 * <ul>
 *   <li>applies a change from a dashboard or the page only while tuning is allowed (on, and the
 *       field not connected); a refused change snaps back on the dashboard;
 *   <li>logs every change it applies, so a replay makes the same changes at the same moments;
 *   <li>lists every tunable for the page, which can change the simple ones (numbers, booleans).
 * </ul>
 *
 * <p>A simple tunable (a number or a boolean) reaches the inner backend as a proxy that reads the
 * real value and sends writes through here. Unit-aware tunables ({@code
 * Tunable.create(Amps.of(30))}) are covered too: WPILib's registry unwraps them and publishes their
 * number in base units. Other kinds (structs, strings, arrays) pass straight through: published as
 * usual, but not locked or replayed. Pitwall's own configuration tunables pass straight through as
 * well, because their writes already go through {@link Tuning}.
 *
 * <p>WPILib calls {@link #update} once per loop, after {@code robotPeriodic()}; dashboard writes
 * and the page's requests are applied there, and so are the logged ones in a replay.
 */
public final class TunableTee implements TunableBackend {
  /**
   * One tunable, as the page shows it.
   *
   * @param path where it is, such as {@code Shooter/kP}
   * @param type {@code double}, {@code boolean}, and so on
   * @param value its value: a number, a boolean, or text
   * @param editable whether the page can change it
   */
  public record Listed(String path, String type, Object value, boolean editable) {}

  private record Entry(TunableBase real, @Nullable TunableBase proxy, String type) {}

  private record PageRequest(Map<String, Object> values, CompletableFuture<Outcome> outcome) {}

  private final TunableBackend inner;
  private final Supplier<@Nullable String> whyLocked;
  private final Consumer<TunableInputs> logInputs;
  private final boolean replaying;
  private final Map<String, Entry> entries = new LinkedHashMap<>();
  private final Set<TunableBase> own = Collections.newSetFromMap(new IdentityHashMap<>());
  private final ConcurrentLinkedQueue<PageRequest> queue = new ConcurrentLinkedQueue<>();
  private final TunableInputs inputs = new TunableInputs();
  private final List<String> appliedPaths = new ArrayList<>();
  private final List<String> appliedValues = new ArrayList<>();
  private volatile List<Listed> view = List.of();

  /**
   * @param inner the backend that publishes the tunables, such as WPILib's NetworkTables backend
   * @param whyLocked why changes are refused right now, or null while they're allowed
   * @param logInputs logs the changes one update applied, or in a replay replaces them with the
   *     logged ones
   * @param replaying whether this is a replay, where only logged changes are applied
   */
  public TunableTee(
      TunableBackend inner,
      Supplier<@Nullable String> whyLocked,
      Consumer<TunableInputs> logInputs,
      boolean replaying) {
    this.inner = inner;
    this.whyLocked = whyLocked;
    this.logInputs = logInputs;
    this.replaying = replaying;
  }

  /**
   * Passes a tunable straight through, unlocked and unlogged here, because its writes are already
   * checked and logged elsewhere (Pitwall's own configuration tunables). Call it before publishing.
   */
  public void passThrough(TunableBase tunable) {
    synchronized (entries) {
      own.add(tunable);
    }
  }

  /** Every tunable, as of the last update, except Pitwall's own. Safe to call from any thread. */
  public List<Listed> view() {
    return view;
  }

  /** Whether the page can change the tunable at a path. Safe to call from any thread. */
  public boolean isEditable(String path) {
    String key = PathUtil.normalizeName(path);
    for (Listed listed : view) {
      if (listed.path().equals(key)) {
        return listed.editable();
      }
    }
    return false;
  }

  /**
   * Asks the next update to change several tunables at once, all or none. Safe to call from any
   * thread.
   *
   * @param values new values by path: a number, or a boolean for a boolean tunable
   * @return completes, in the robot loop, with what became of the request
   */
  public CompletableFuture<Outcome> requestChanges(Map<String, Object> values) {
    CompletableFuture<Outcome> outcome = new CompletableFuture<>();
    Map<String, Object> byPath = new LinkedHashMap<>();
    values.forEach((path, value) -> byPath.put(PathUtil.normalizeName(path), value));
    queue.add(new PageRequest(byPath, outcome));
    return outcome;
  }

  @Override
  public boolean publish(String path, TunableBase tunable) {
    String key = PathUtil.normalizeName(path);
    TunableBase proxy;
    synchronized (entries) {
      proxy = own.contains(tunable) ? null : proxy(key, tunable);
    }
    boolean published = inner.publish(key, proxy != null ? proxy : tunable);
    if (published) {
      synchronized (entries) {
        entries.put(key, new Entry(tunable, proxy, typeOf(tunable)));
      }
    }
    return published;
  }

  @Override
  public boolean publishComplex(String path, ComplexTunable tunable) {
    return inner.publishComplex(path, tunable);
  }

  @Override
  public void remove(String path) {
    String key = PathUtil.normalizeName(path);
    inner.remove(key);
    synchronized (entries) {
      entries.remove(key);
    }
  }

  @Override
  public List<PublishedTunable> removePrefix(String prefix) {
    List<PublishedTunable> removed = inner.removePrefix(prefix);
    List<PublishedTunable> reals = new ArrayList<>(removed.size());
    synchronized (entries) {
      for (PublishedTunable published : removed) {
        Entry entry = entries.remove(published.path());
        // Hand back the real tunables, not the proxies, so the next backend publishes those.
        reals.add(
            entry != null && published.tunable() != null
                ? new PublishedTunable(published.path(), entry.real(), null)
                : published);
      }
    }
    return reals;
  }

  @Override
  public void markDirty(TunableBase tunable) {
    inner.markDirty(tunable);
  }

  @Override
  public void update() {
    appliedPaths.clear();
    appliedValues.clear();
    List<PageRequest> requests = new ArrayList<>();
    for (PageRequest request = queue.poll(); request != null; request = queue.poll()) {
      requests.add(request);
    }
    List<Outcome> outcomes = new ArrayList<>();
    for (PageRequest request : requests) {
      outcomes.add(applyFromPage(request.values()));
    }
    inner.update(); // dashboard writes arrive through the proxies: see write()

    inputs.paths = appliedPaths.toArray(String[]::new);
    inputs.values = appliedValues.toArray(String[]::new);
    logInputs.accept(inputs); // in replay, this replaces them with the logged ones
    if (replaying) {
      for (int i = 0; i < inputs.paths.length && i < inputs.values.length; i++) {
        applyLogged(inputs.paths[i], inputs.values[i]);
      }
    }
    view = snapshot();
    for (int i = 0; i < outcomes.size(); i++) {
      requests.get(i).outcome().complete(outcomes.get(i));
    }
  }

  @Override
  public void close() throws Exception {
    inner.close();
  }

  /** A dashboard's write, through a proxy: applied if tuning allows it, then logged. */
  private void write(String path, TunableBase real, String text, Runnable apply) {
    if (replaying || whyLocked.get() != null) {
      return; // refused: the dashboard shows the real value again at the next update
    }
    apply.run();
    applied(path, real, text);
  }

  private void applied(String path, TunableBase real, String text) {
    appliedPaths.add(path);
    appliedValues.add(text);
    TunableConfig config = real.getConfig();
    Runnable onTune = config != null ? config.getOnTune() : null;
    if (onTune != null) {
      TunableRegistry.runAfterUpdate(onTune);
    }
  }

  private Outcome applyFromPage(Map<String, Object> values) {
    String why = whyLocked.get();
    if (why != null) {
      return new Outcome(Outcome.Result.LOCKED, "Tuning is locked: " + why);
    }
    List<Runnable> changes = new ArrayList<>();
    for (Map.Entry<String, Object> value : values.entrySet()) {
      Entry entry;
      synchronized (entries) {
        entry = entries.get(value.getKey());
      }
      if (entry == null || entry.proxy() == null || !isMutable(entry.real())) {
        return new Outcome(
            Outcome.Result.REFUSED, "There is no tunable the page can change at " + value.getKey());
      }
      Runnable change = change(value.getKey(), entry, value.getValue());
      if (change == null) {
        return new Outcome(
            Outcome.Result.REFUSED, value.getKey() + " takes a " + entry.type() + " value");
      }
      changes.add(change);
    }
    changes.forEach(Runnable::run);
    return Outcome.applied();
  }

  /** The change a page's value makes to a tunable, or null when the value doesn't fit its type. */
  private @Nullable Runnable change(String path, Entry entry, Object value) {
    TunableBase real = entry.real();
    if (value instanceof Boolean on) {
      return real instanceof TunableBoolean t
          ? () -> {
            t.set(on);
            applied(path, t, Boolean.toString(on));
          }
          : null;
    }
    if (!(value instanceof Number number)) {
      return null;
    }
    double d = number.doubleValue();
    return switch (real) {
      case TunableDouble t ->
          () -> {
            t.set(d);
            applied(path, t, Double.toString(d));
          };
      case TunableFloat t ->
          () -> {
            t.set((float) d);
            applied(path, t, Float.toString((float) d));
          };
      case TunableInt t
          when d == Math.rint(d) && d >= Integer.MIN_VALUE && d <= Integer.MAX_VALUE ->
          () -> {
            t.set((int) d);
            applied(path, t, Integer.toString((int) d));
          };
      case TunableLong t when d == Math.rint(d) ->
          () -> {
            t.set(number.longValue());
            applied(path, t, Long.toString(number.longValue()));
          };
      default -> null;
    };
  }

  /** Applies a logged change during a replay, as the robot applied it. */
  private void applyLogged(String path, String text) {
    Entry entry;
    synchronized (entries) {
      entry = entries.get(path);
    }
    if (entry == null) {
      TunableRegistry.reportWarning("Pitwall: a logged change is for an unknown tunable, " + path);
      return;
    }
    try {
      switch (entry.real()) {
        case TunableDouble t -> t.set(Double.parseDouble(text));
        case TunableFloat t -> t.set(Float.parseFloat(text));
        case TunableInt t -> t.set(Integer.parseInt(text));
        case TunableLong t -> t.set(Long.parseLong(text));
        case TunableBoolean t -> t.set(Boolean.parseBoolean(text));
        default -> {
          return;
        }
      }
    } catch (NumberFormatException e) {
      TunableRegistry.reportWarning("Pitwall: a logged change to " + path + " is unreadable");
      return;
    }
    TunableConfig config = entry.real().getConfig();
    Runnable onTune = config != null ? config.getOnTune() : null;
    if (onTune != null) {
      TunableRegistry.runAfterUpdate(onTune);
    }
  }

  /**
   * A stand-in the inner backend publishes for a simple tunable: it reads the real value (polled
   * every update, so local changes publish), and sends writes through {@link #write}. The real
   * tunable's onTune runs only when a write is applied.
   */
  private @Nullable TunableBase proxy(String path, TunableBase real) {
    TunableConfig config = real.getConfig();
    TunableConfig proxyConfig =
        (config != null ? config : new TunableConfig())
            .withOnTune(null)
            .withPolling(TunableConfig.Polling.ALWAYS_GET);
    return switch (real) {
      case TunableDouble t ->
          TunableDouble.createConfig(
              t::get, v -> write(path, t, Double.toString(v), () -> t.set(v)), proxyConfig);
      case TunableFloat t ->
          TunableFloat.createConfig(
              t::get, v -> write(path, t, Float.toString(v), () -> t.set(v)), proxyConfig);
      case TunableInt t ->
          TunableInt.createConfig(
              t::get, v -> write(path, t, Integer.toString(v), () -> t.set(v)), proxyConfig);
      case TunableLong t ->
          TunableLong.createConfig(
              t::get, v -> write(path, t, Long.toString(v), () -> t.set(v)), proxyConfig);
      case TunableBoolean t ->
          TunableBoolean.createConfig(
              t::get, v -> write(path, t, Boolean.toString(v), () -> t.set(v)), proxyConfig);
      default -> null;
    };
  }

  private static boolean isMutable(TunableBase tunable) {
    TunableConfig config = tunable.getConfig();
    return config == null || config.isMutable();
  }

  private static String typeOf(TunableBase tunable) {
    return switch (tunable) {
      case TunableDouble _ -> "double";
      case TunableFloat _ -> "float";
      case TunableInt _ -> "int";
      case TunableLong _ -> "long";
      case TunableBoolean _ -> "boolean";
      default -> tunable.getClass().getSimpleName();
    };
  }

  private List<Listed> snapshot() {
    List<Listed> listed = new ArrayList<>();
    synchronized (entries) {
      for (Map.Entry<String, Entry> item : entries.entrySet()) {
        Entry entry = item.getValue();
        if (own.contains(entry.real())) {
          continue;
        }
        listed.add(
            new Listed(
                item.getKey(),
                entry.type(),
                valueOf(entry.real()),
                entry.proxy() != null && isMutable(entry.real())));
      }
    }
    return List.copyOf(listed);
  }

  private static Object valueOf(TunableBase tunable) {
    return switch (tunable) {
      case TunableDouble t -> t.get();
      case TunableFloat t -> t.get();
      case TunableInt t -> t.get();
      case TunableLong t -> t.get();
      case TunableBoolean t -> t.get();
      case Tunable<?> t -> String.valueOf(t.get());
      default -> "";
    };
  }
}
