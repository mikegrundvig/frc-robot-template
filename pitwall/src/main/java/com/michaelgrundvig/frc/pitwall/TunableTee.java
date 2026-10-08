package com.michaelgrundvig.frc.pitwall;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
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
 *       field not connected), and only within the tunable's {@link Limits}; a refused change snaps
 *       back on the dashboard, and says why on the page;
 *   <li>logs every change it applies, so a replay makes the same changes at the same moments;
 *   <li>lists every tunable for the page, which can change the simple ones (numbers, booleans).
 * </ul>
 *
 * <p>A simple tunable (a number or a boolean) reaches the inner backend as a proxy that reads the
 * real value and sends writes through here. Unit-aware tunables ({@code
 * Tunable.create(Amps.of(30))}) are covered too: WPILib's registry unwraps them and publishes their
 * number in base units. Other kinds (structs, strings, arrays) pass straight through: published as
 * usual, but not locked or replayed.
 *
 * <p>A {@link LiveConfig}'s values are checked whole: before a change to one applies, the
 * configuration says whether it would take it (its limits, then its records' own checks), and a
 * page's changes to several of its values apply together, as one rebuild. Nothing applies unless
 * everything passes, and a refusal gives every reason. {@link Tuning} puts them back as committed
 * when the field connects.
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
   * @param limits the range a number may be tuned in
   */
  public record Listed(String path, String type, Object value, boolean editable, Limits limits) {}

  /**
   * @param config the live configuration this is a value of, or null
   * @param field its path in that configuration
   */
  private record Entry(
      TunableBase real,
      @Nullable TunableBase proxy,
      String type,
      Object startup,
      Limits limits,
      @Nullable LiveConfig<?> config,
      String field) {}

  private record PageRequest(Map<String, Object> values, CompletableFuture<Outcome> outcome) {}

  /**
   * One tunable's change from the page, or one configuration's, and how to take it back if a later
   * one fails.
   *
   * @param name the tunable's path, or the configuration's name
   * @param config the configuration it changes; empty for a tunable of none
   * @param paths the tunables it changes
   */
  private record Change(
      String name, String config, List<String> paths, Runnable apply, Runnable undo) {}

  private final TunableBackend inner;
  private final Supplier<@Nullable String> whyLocked;
  private final Consumer<TunableInputs> logInputs;
  private final boolean replaying;
  private final Map<String, Entry> entries = new LinkedHashMap<>();
  private final ConcurrentLinkedQueue<PageRequest> queue = new ConcurrentLinkedQueue<>();
  private final TunableInputs inputs = new TunableInputs();
  private final List<String> appliedPaths = new ArrayList<>();
  private final List<String> appliedValues = new ArrayList<>();

  /** How many refused dashboard writes {@link #refusals()} keeps. */
  private static final int REFUSALS_KEPT = 10;

  private final ArrayDeque<String> refusals = new ArrayDeque<>();

  /** The live configurations published through here, in the order they were. */
  private final List<LiveConfig<?>> configs = new ArrayList<>();

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
   * Every tunable, as of the last update, except the values of live configurations (see {@link
   * #configs()}). Safe to call from any thread.
   */
  public List<Listed> view() {
    return view;
  }

  /**
   * The simple tunables (numbers, true or false) whose value differs from when they were published,
   * such as a camera switched off in the pit, each as {@code "path: value (was startup)"}.
   * Configuration values aren't listed: they return to their committed values when the field
   * connects. Call it in the robot loop.
   */
  public List<String> changedSinceStartup() {
    List<String> changed = new ArrayList<>();
    synchronized (entries) {
      for (Map.Entry<String, Entry> item : entries.entrySet()) {
        Entry entry = item.getValue();
        if (entry.proxy() == null || entry.config() != null) {
          continue;
        }
        Object now = valueOf(entry.real());
        if (!now.equals(entry.startup())) {
          changed.add(item.getKey() + ": " + now + " (was " + entry.startup() + ")");
        }
      }
    }
    return changed;
  }

  /**
   * The latest dashboard writes that were refused, newest last, each as {@code "path = value:
   * why"}. A dashboard can't show why a value snapped back; the page can. Safe to call from any
   * thread.
   */
  public List<String> refusals() {
    synchronized (refusals) {
      return List.copyOf(refusals);
    }
  }

  /**
   * Where a configuration's value is published, or null if it isn't. Safe to call from any thread.
   */
  public @Nullable String pathOf(LiveConfig<?> config, String field) {
    synchronized (entries) {
      for (Map.Entry<String, Entry> item : entries.entrySet()) {
        if (item.getValue().config() == config && item.getValue().field().equals(field)) {
          return item.getKey();
        }
      }
    }
    return null;
  }

  /** The live configurations published through here. Safe to call from any thread. */
  public List<LiveConfig<?>> configs() {
    synchronized (configs) {
      return List.copyOf(configs);
    }
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
      proxy = proxy(key, tunable);
    }
    boolean published = inner.publish(key, proxy != null ? proxy : tunable);
    if (published) {
      synchronized (entries) {
        LiveConfig<?> config = null;
        String field = "";
        synchronized (configs) {
          for (LiveConfig<?> candidate : configs) {
            String known = candidate.pathOf(tunable);
            if (known != null) {
              config = candidate;
              field = known;
            }
          }
        }
        entries.put(
            key,
            new Entry(
                tunable,
                proxy,
                typeOf(tunable),
                valueOf(tunable),
                Limits.of(tunable.getConfig()),
                config,
                field));
      }
    }
    return published;
  }

  @Override
  public boolean publishComplex(String path, ComplexTunable tunable) {
    // Known before its values are published (the inner backend publishes them from here), so each
    // is recognized as the configuration's.
    if (tunable instanceof LiveConfig<?> config) {
      synchronized (configs) {
        if (configs.stream().noneMatch(known -> known == config)) {
          configs.add(config);
        }
      }
    }
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
      applyLogged(inputs.paths, inputs.values);
    }
    // The same list while nothing changed, so the page's comparison stops at it.
    List<Listed> now = snapshot();
    if (!now.equals(view)) {
      view = now;
    }
    for (int i = 0; i < outcomes.size(); i++) {
      requests.get(i).outcome().complete(outcomes.get(i));
    }
  }

  @Override
  public void close() throws Exception {
    inner.close();
  }

  /**
   * A dashboard's write, through a proxy: applied if tuning allows it and the value is one the
   * tunable can take, within its limits, then logged. A refused write (tuning locked, a value that
   * isn't a number, one outside the limits, one the tunable's own setter throws on) is recorded
   * with its reason; the dashboard shows the real value again at the next update.
   *
   * @param number the value as a number, for the limits; NaN for a true-or-false
   */
  private void write(
      String path, TunableBase real, String text, double number, boolean valid, Runnable apply) {
    if (replaying) {
      return;
    }
    String locked = whyLocked.get();
    if (locked != null) {
      refused(path, text, "tuning is locked: " + locked);
      return;
    }
    if (!valid) {
      refused(path, text, "not a number");
      return;
    }
    Entry entry;
    synchronized (entries) {
      entry = entries.get(path);
    }
    String outside = Double.isNaN(number) || entry == null ? null : entry.limits().whyNot(number);
    LiveConfig<?> config = entry != null ? entry.config() : null;
    if (outside == null && entry != null && config != null) {
      outside = config.whyNot(Map.of(entry.field(), number));
    }
    if (outside != null) {
      refused(path, text, outside);
      return;
    }
    try {
      apply.run();
    } catch (RuntimeException e) {
      String why = e.getMessage();
      refused(path, text, why != null ? why : e.toString());
      return;
    }
    applied(path, real, text);
  }

  private void refused(String path, String text, String why) {
    String refusal = path + " = " + text + ": " + why;
    TunableRegistry.reportWarning("Pitwall refused " + refusal);
    synchronized (refusals) {
      refusals.addLast(refusal);
      while (refusals.size() > REFUSALS_KEPT) {
        refusals.removeFirst();
      }
    }
  }

  /** A configuration's value was set: logged like any tunable's (its reaction is the config's). */
  private void appliedToConfig(@Nullable String path, String text) {
    if (path != null) {
      appliedPaths.add(path);
      appliedValues.add(text);
    }
  }

  private void applied(String path, TunableBase real, String text) {
    appliedPaths.add(path);
    appliedValues.add(text);
    runOnTune(path, real);
  }

  /**
   * Runs a tunable's own reaction to a change after the update, where WPILib runs it with nothing
   * around it: a reaction that throws is reported, and never stops the robot loop.
   */
  private static void runOnTune(String path, TunableBase real) {
    TunableConfig config = real.getConfig();
    Runnable onTune = config != null ? config.getOnTune() : null;
    if (onTune != null) {
      TunableRegistry.runAfterUpdate(
          () -> {
            try {
              onTune.run();
            } catch (RuntimeException e) {
              TunableRegistry.reportWarning("Pitwall: reacting to " + path + " failed: " + e);
            }
          });
    }
  }

  /**
   * The page's changes, all or none. Everything is checked before anything applies: each value (a
   * tunable the page can change, its type, and its limits), then each configuration it changes,
   * rebuilt with all of its new values. A refusal gives every reason, one per line, each tied to
   * the values it concerns. Then each is applied, and if a tunable's own setter throws, the ones
   * already applied are put back.
   */
  private Outcome applyFromPage(Map<String, Object> values) {
    String why = whyLocked.get();
    if (why != null) {
      return new Outcome(Outcome.Result.LOCKED, "Tuning is locked: " + why);
    }
    List<Outcome.Reason> problems = new ArrayList<>();
    List<Change> changes = new ArrayList<>();
    // A configuration's values, gathered to be checked and applied together.
    Map<LiveConfig<?>, Map<String, Double>> configured = new IdentityHashMap<>();
    Map<LiveConfig<?>, Map<String, String>> configuredPaths = new IdentityHashMap<>();
    for (Map.Entry<String, Object> value : values.entrySet()) {
      String path = value.getKey();
      Entry entry;
      synchronized (entries) {
        entry = entries.get(path);
      }
      if (entry == null || entry.proxy() == null || !isMutable(entry.real())) {
        String text = "There is no tunable the page can change at " + path;
        problems.add(Outcome.Reason.of(Outcome.Layer.VALUE, path, "", text, text));
        continue;
      }
      LiveConfig<?> config = entry.config();
      String configName = config != null ? config.name() : "";
      Change change = change(path, entry, value.getValue());
      if (change == null) {
        String text = path + " takes a " + entry.type() + " value";
        problems.add(Outcome.Reason.of(Outcome.Layer.VALUE, path, configName, text, text));
        continue;
      }
      if (value.getValue() instanceof Number number) {
        String outside = entry.limits().whyNot(number.doubleValue());
        if (outside != null) {
          problems.add(
              Outcome.Reason.of(
                  Outcome.Layer.LIMIT, path, configName, outside, path + ": " + outside));
          continue;
        }
      }
      if (config != null && value.getValue() instanceof Number number) {
        configured
            .computeIfAbsent(config, c -> new LinkedHashMap<>())
            .put(entry.field(), number.doubleValue());
        configuredPaths
            .computeIfAbsent(config, c -> new LinkedHashMap<>())
            .put(entry.field(), path);
        continue;
      }
      changes.add(change);
    }
    if (!problems.isEmpty()) {
      return Outcome.refused(problems);
    }
    for (Map.Entry<LiveConfig<?>, Map<String, Double>> group : configured.entrySet()) {
      LiveConfig<?> config = group.getKey();
      List<LiveConfig.Problem> refused = new ArrayList<>();
      Record candidate = config.candidate(group.getValue(), refused);
      if (candidate == null) {
        for (LiveConfig.Problem problem : refused) {
          problems.add(
              new Outcome.Reason(
                  problem.layer(),
                  problem.record(),
                  pathsOf(config, problem.fields()),
                  config.name(),
                  problem.group(),
                  problem.text(),
                  problem.line()));
        }
        continue;
      }
      Record old = config.get();
      Map<String, Double> oldValues = new LinkedHashMap<>();
      group.getValue().keySet().forEach(field -> oldValues.put(field, config.value(field)));
      Map<String, String> paths = Objects.requireNonNull(configuredPaths.get(config));
      changes.add(
          new Change(
              config.name(),
              config.name(),
              List.copyOf(paths.values()),
              () -> {
                config.accept(candidate);
                group
                    .getValue()
                    .forEach((field, v) -> appliedToConfig(paths.get(field), Double.toString(v)));
              },
              () -> {
                config.accept(old);
                oldValues.forEach(
                    (field, v) -> appliedToConfig(paths.get(field), Double.toString(v)));
              }));
    }
    if (!problems.isEmpty()) {
      return Outcome.refused(problems);
    }
    List<Change> done = new ArrayList<>();
    for (Change change : changes) {
      try {
        change.apply().run();
        done.add(change);
      } catch (RuntimeException e) {
        for (Change applied : done.reversed()) {
          try {
            applied.undo().run();
          } catch (RuntimeException alsoFailed) {
            TunableRegistry.reportWarning(
                "Pitwall: couldn't put " + applied.name() + " back: " + alsoFailed);
          }
        }
        String reason = e.getMessage() != null ? e.getMessage() : e.toString();
        return Outcome.refused(
            List.of(
                new Outcome.Reason(
                    Outcome.Layer.TUNABLE,
                    "",
                    change.paths(),
                    change.config(),
                    "",
                    reason,
                    change.name() + " refused the value: " + reason)));
      }
    }
    return Outcome.applied();
  }

  /** Where a configuration's values are published, those that are, in the order given. */
  private List<String> pathsOf(LiveConfig<?> config, List<String> fields) {
    List<String> paths = new ArrayList<>();
    for (String field : fields) {
      String path = pathOf(config, field);
      if (path != null) {
        paths.add(path);
      }
    }
    return paths;
  }

  /**
   * The change a page's value makes to a tunable, or null when the value doesn't fit its type: a
   * number that isn't one (NaN, infinity) fits none, and a whole number must fit its type's range.
   */
  private @Nullable Change change(String path, Entry entry, Object value) {
    TunableBase real = entry.real();
    if (value instanceof Boolean on) {
      if (!(real instanceof TunableBoolean t)) {
        return null;
      }
      boolean old = t.get();
      return change(path, t, () -> t.set(on), Boolean.toString(on), () -> t.set(old), old);
    }
    if (!(value instanceof Number number)) {
      return null;
    }
    double d = number.doubleValue();
    if (!Double.isFinite(d)) {
      return null;
    }
    return switch (real) {
      case TunableDouble t -> {
        double old = t.get();
        yield change(path, t, () -> t.set(d), Double.toString(d), () -> t.set(old), old);
      }
      case TunableFloat t when Math.abs(d) <= Float.MAX_VALUE -> {
        float old = t.get();
        float f = (float) d;
        yield change(path, t, () -> t.set(f), Float.toString(f), () -> t.set(old), old);
      }
      case TunableInt t
          when d == Math.rint(d) && d >= Integer.MIN_VALUE && d <= Integer.MAX_VALUE -> {
        int old = t.get();
        int i = (int) d;
        yield change(path, t, () -> t.set(i), Integer.toString(i), () -> t.set(old), old);
      }
      // Exactly representable longs only: 2^63 and beyond would silently become Long.MAX_VALUE.
      case TunableLong t when d == Math.rint(d) && d >= -0x1p63 && d < 0x1p63 -> {
        long old = t.get();
        long l = number.longValue();
        yield change(path, t, () -> t.set(l), Long.toString(l), () -> t.set(old), old);
      }
      default -> null;
    };
  }

  private Change change(
      String path, TunableBase real, Runnable set, String text, Runnable restore, Object old) {
    return new Change(
        path,
        "",
        List.of(path),
        () -> {
          set.run();
          applied(path, real, text);
        },
        () -> {
          restore.run();
          applied(path, real, String.valueOf(old));
        });
  }

  /**
   * Applies one update's logged changes during a replay, as the robot applied them: a
   * configuration's values together, through its records' checks, the rest one by one.
   */
  private void applyLogged(String[] paths, String[] texts) {
    Map<LiveConfig<?>, Map<String, Double>> configured = new IdentityHashMap<>();
    for (int i = 0; i < paths.length && i < texts.length; i++) {
      Entry entry;
      synchronized (entries) {
        entry = entries.get(paths[i]);
      }
      LiveConfig<?> config = entry != null ? entry.config() : null;
      if (entry == null || config == null) {
        applyLogged(paths[i], texts[i]);
        continue;
      }
      try {
        configured
            .computeIfAbsent(config, c -> new LinkedHashMap<>())
            .put(entry.field(), Double.parseDouble(texts[i]));
      } catch (NumberFormatException e) {
        TunableRegistry.reportWarning("Pitwall: a logged change to " + paths[i] + " is unreadable");
      }
    }
    configured.forEach(
        (config, values) -> {
          try {
            config.accept(config.candidate(values));
          } catch (IllegalArgumentException refused) {
            TunableRegistry.reportWarning(
                "Pitwall: " + config.name() + " refused a logged change: " + refused.getMessage());
          }
        });
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
    } catch (RuntimeException e) {
      TunableRegistry.reportWarning("Pitwall: " + path + " refused a logged change: " + e);
      return;
    }
    runOnTune(path, entry.real());
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
              t::get,
              v -> write(path, t, Double.toString(v), v, Double.isFinite(v), () -> t.set(v)),
              proxyConfig);
      case TunableFloat t ->
          TunableFloat.createConfig(
              t::get,
              v -> write(path, t, Float.toString(v), v, Float.isFinite(v), () -> t.set(v)),
              proxyConfig);
      case TunableInt t ->
          TunableInt.createConfig(
              t::get,
              v -> write(path, t, Integer.toString(v), v, true, () -> t.set(v)),
              proxyConfig);
      case TunableLong t ->
          TunableLong.createConfig(
              t::get,
              v -> write(path, t, Long.toString(v), (double) v, true, () -> t.set(v)),
              proxyConfig);
      case TunableBoolean t ->
          TunableBoolean.createConfig(
              t::get,
              v -> write(path, t, Boolean.toString(v), Double.NaN, true, () -> t.set(v)),
              proxyConfig);
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
        if (entry.config() != null) {
          continue;
        }
        listed.add(
            new Listed(
                item.getKey(),
                entry.type(),
                valueOf(entry.real()),
                entry.proxy() != null && isMutable(entry.real()),
                entry.limits()));
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
