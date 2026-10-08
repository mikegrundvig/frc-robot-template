package com.michaelgrundvig.frc.pitwall;

import com.michaelgrundvig.frc.pitwall.experiment.Experiment;
import com.michaelgrundvig.frc.pitwall.experiment.Experiments;
import com.michaelgrundvig.frc.pitwall.experiment.Rig;
import com.michaelgrundvig.frc.pitwall.experiment.Run;
import com.michaelgrundvig.frc.pitwall.experiment.StepResponse;
import com.michaelgrundvig.frc.pitwall.experiment.Trace;
import com.michaelgrundvig.frc.pitwall.scene.Scenes;
import com.michaelgrundvig.frc.pitwall.simulator.Simulator;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.UUID;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;

/**
 * Everything the page shows and changes (the status, actions, tuning, every tunable, and the
 * experiments), and the page's state section by section, for the event stream.
 *
 * <p>{@link #sections()} is the whole state as the page holds it, each section a map from keys to
 * values, so that {@link com.michaelgrundvig.frc.pitwall.push.Delta} can tell what changed from one
 * loop to the next. It's built in the robot loop, and only while a page has its stream open; the
 * tuning's sections are rebuilt only when a configuration or tuning's lock changed.
 */
public final class Page {
  private final Status status;
  private final Actions actions;
  private final Tuning tuning;
  private final TunableTee tunables;
  private final Experiments experiments;
  private final Supplier<String> mode;
  private final @Nullable Simulator simulator;
  private final Devices devices;
  private final Scenes scenes;

  /** Tells this run of the robot program from the last: the page keeps runs across restarts. */
  private final String program = UUID.randomUUID().toString();

  /** What the tuning sections were built from, and the sections, rebuilt when that changes. */
  private List<Object> builtFrom = List.of();

  private @Nullable String builtLock = "never built";
  private Map<String, Object> tunableSection = Map.of();
  private Map<String, Object> valueSection = Map.of();
  private Map<String, Object> changeSection = Map.of();

  /** The configurations the documentation section was built for, and the section. */
  private List<LiveConfig<?>> documented = List.of();

  private Map<String, Object> configSection = Map.of();

  /** The experiments the rig sections were built from, and whether every gain's tunable was. */
  private @Nullable List<Experiment> rigsBuiltFor;

  private boolean pathsFound;
  private Map<String, Object> rigSection = Map.of();
  private Map<String, Object> experimentSection = Map.of();

  /**
   * The sections that change now and then, each with what it was built from: built again only when
   * that changes, else handed over as the same map, where the comparison with what was sent stops.
   */
  private final Built robotSection = new Built();

  private final Built actionSection = new Built();
  private final Built tuningSection = new Built();
  private final Built wpilibSection = new Built();
  private final Built runSection = new Built();

  /** A section and what it was built from. */
  private static final class Built {
    private @Nullable Object from;
    private Map<String, Object> section = Map.of();

    /** The section, built again only when what it's built from differs (by equals). */
    Map<String, Object> from(Object now, Supplier<Map<String, Object>> build) {
      if (!now.equals(from)) {
        section = build.get();
        from = now;
      }
      return section;
    }
  }

  /** A page that doesn't know the robot's mode, and has no simulator: a test's, say. */
  public Page(
      Status status, Actions actions, Tuning tuning, TunableTee tunables, Experiments experiments) {
    this(
        status,
        actions,
        tuning,
        tunables,
        experiments,
        () -> "",
        null,
        new Devices(System::nanoTime),
        new Scenes(System::nanoTime, System.out::println));
  }

  /**
   * A robot's page: no simulator.
   *
   * @param mode the robot's mode as the Driver Station sets it, such as "Utility, enabled": read in
   *     the robot loop, only while a page is open
   */
  public Page(
      Status status,
      Actions actions,
      Tuning tuning,
      TunableTee tunables,
      Experiments experiments,
      Supplier<String> mode) {
    this(
        status,
        actions,
        tuning,
        tunables,
        experiments,
        mode,
        null,
        new Devices(System::nanoTime),
        new Scenes(System::nanoTime, System.out::println));
  }

  /**
   * @param mode the robot's mode as the Driver Station sets it, such as "Utility, enabled": read in
   *     the robot loop, only while a page is open
   * @param simulator the simulator's controls, in the simulator; null on a robot, where the page
   *     has none
   * @param devices the devices the robot reports on, for the Pit view's cards
   * @param scenes the scenes the page draws
   */
  Page(
      Status status,
      Actions actions,
      Tuning tuning,
      TunableTee tunables,
      Experiments experiments,
      Supplier<String> mode,
      @Nullable Simulator simulator,
      Devices devices,
      Scenes scenes) {
    this.status = status;
    this.actions = actions;
    this.tuning = tuning;
    this.tunables = tunables;
    this.experiments = experiments;
    this.mode = mode;
    this.simulator = simulator;
    this.devices = devices;
    this.scenes = scenes;
  }

  public Status status() {
    return status;
  }

  public Actions actions() {
    return actions;
  }

  public Tuning tuning() {
    return tuning;
  }

  public TunableTee tunables() {
    return tunables;
  }

  public Experiments experiments() {
    return experiments;
  }

  /** The devices the robot reports on: the Pit view's cards. */
  public Devices devices() {
    return devices;
  }

  /** The scenes the page draws: its World view. */
  public Scenes scenes() {
    return scenes;
  }

  /** The simulator's controls; null on a robot. */
  public @Nullable Simulator simulator() {
    return simulator;
  }

  /**
   * The page's state now, section by section. Call it from the robot loop.
   *
   * <ul>
   *   <li>{@code robot}: its name, the code, simulator or real, this program's id, its mode (empty
   *       when unknown), and why actions and experiments can't start now (empty while they can);
   *   <li>{@code status}, {@code units}: each item's value, and the unit of each number;
   *   <li>{@code actions}: each action's phase, detail, why it can't start now (empty while it
   *       can), and when it may run ({@code ENABLED_IN_UTILITY} or {@code DISABLED});
   *   <li>{@code tuning}: whether it's on, the field, and refused dashboard changes;
   *   <li>{@code configs}: each configuration's documentation, as {@code /api/configs} gives it;
   *   <li>{@code tunables}, {@code values}, {@code changes}: the configurations' tunable values, by
   *       the WPILib tunable each is published as, as {@code /api/tunables}, {@code /api/values},
   *       and {@code /api/changes} give them;
   *   <li>{@code wpilibTunables}: every other WPILib tunable, by path;
   *   <li>{@code rigs}, {@code experiments}: what can be run;
   *   <li>{@code run}: the run in progress, or the latest to end;
   *   <li>{@code devices}: each device's card, as of its last read: see {@link Devices#view};
   *   <li>{@code scenes}: each scene's description, by id; what moves in one goes only to the pages
   *       that show it: see {@link Scenes};
   *   <li>in the simulator only, {@code simulator} (the Driver Station, the clock, and the
   *       joysticks the page drives), {@code components} (what can fail, and how), and {@code
   *       scenarios}: see {@link Simulator#view}.
   * </ul>
   *
   * <p>A section that hasn't changed since the last call is the same map as then.
   */
  public Map<String, Map<String, Object>> sections() {
    List<LiveConfig<?>> configs = tunables.configs();
    if (!sameObjects(configs, documented)) {
      // Only when a configuration is published: their documentation never changes.
      configSection = configs(configs);
      documented = configs;
    }
    rebuildTuningIfChanged(configs);
    Map<String, Map<String, Object>> sections = new LinkedHashMap<>();
    sections.put(
        "robot",
        robotSection.from(
            Arrays.asList(
                mode.get(),
                Objects.requireNonNullElse(actions.whyLocked(), ""),
                Objects.requireNonNullElse(experiments.whyLocked(), "")),
            this::robot));
    sections.put("status", status.values());
    sections.put("units", status.units());
    List<Actions.State> actionStates = actions.view();
    sections.put("actions", actionSection.from(actionStates, () -> actionStates(actionStates)));
    sections.put(
        "tuning",
        tuningSection.from(
            Arrays.asList(
                tuning.name(), tuning.isEnabled(), tuning.isFieldConnected(), tunables.refusals()),
            this::session));
    sections.put("configs", configSection);
    sections.put("tunables", tunableSection);
    sections.put("values", valueSection);
    sections.put("changes", changeSection);
    List<TunableTee.Listed> listed = tunables.view();
    sections.put("wpilibTunables", wpilibSection.from(listed, () -> wpilibTunables(listed)));
    rebuildRigsIfChanged();
    sections.put("rigs", rigSection);
    sections.put("experiments", experimentSection);
    Run run = experiments.run();
    sections.put(
        "run",
        run == null
            ? Map.of()
            : runSection.from(
                Arrays.asList(
                    run,
                    run.startNanos(),
                    run.state(),
                    run.gainsChanged(),
                    run.controller(),
                    rigsBuiltFor,
                    pathsFound),
                () -> run(run)));
    sections.put("devices", devices.view());
    sections.put("scenes", scenes.descriptions());
    if (simulator != null) {
      sections.putAll(simulator.view());
    }
    return sections;
  }

  Map<String, Object> robot() {
    Experiments.Robot robot = experiments.robot();
    Map<String, Object> shown = new LinkedHashMap<>();
    shown.put("name", robot.name());
    shown.put("simulated", robot.simulated());
    shown.put("code", robot.code());
    shown.put("program", program);
    shown.put("mode", mode.get());
    // Why Run is off, beside it: the gate's own reason, as the robot would refuse with it.
    shown.put("actionsLocked", Objects.requireNonNullElse(actions.whyLocked(), ""));
    shown.put("experimentsLocked", Objects.requireNonNullElse(experiments.whyLocked(), ""));
    return shown;
  }

  /** Tuning's state: {@code {"name", "enabled", "fieldConnected", "refused"}}. */
  Map<String, Object> session() {
    Map<String, Object> session = new LinkedHashMap<>();
    session.put("name", tuning.name());
    session.put("enabled", tuning.isEnabled());
    session.put("fieldConnected", tuning.isFieldConnected());
    session.put("refused", tunables.refusals());
    return session;
  }

  /**
   * Each configuration's documentation, by name: what its root record's Javadoc says, then each
   * record its values are in, with what the record holding it says it is: {@code {"record", "doc":
   * {"summary", "text"}, "groups": [{"path", "record", "doc", "meaning"}]}}. Each {@code record} is
   * a display name ({@code DrivetrainControl.Gains}). It was read when the configuration was
   * published.
   */
  static Map<String, Object> configs(List<LiveConfig<?>> configs) {
    Map<String, Object> described = new LinkedHashMap<>();
    for (LiveConfig<?> config : configs) {
      LiveConfig.Documentation docs = config.documentation();
      List<Object> groups = new ArrayList<>();
      for (LiveConfig.Documentation.Group group : docs.groups()) {
        Map<String, Object> shown = new LinkedHashMap<>();
        shown.put("path", group.path());
        shown.put("record", group.record());
        shown.put("doc", doc(group.doc()));
        shown.put("meaning", group.meaning());
        groups.add(shown);
      }
      Map<String, Object> shown = new LinkedHashMap<>();
      shown.put("record", docs.record());
      shown.put("doc", doc(docs.doc()));
      shown.put("groups", groups);
      described.put(config.name(), shown);
    }
    return described;
  }

  /**
   * Each configuration value, by its tunable: {@code {"config", "field", "unit", "limits",
   * "committed", "record", "group", "meaning"}}, the last three the display name of the record that
   * declares it, that record's path ({@code /api/configs}' group), and what its {@code @param} says
   * the value means.
   */
  static Map<String, Object> tunables(Tuning.View view) {
    Map<String, Object> described = new LinkedHashMap<>();
    for (Tuning.ConfigView config : view.configs()) {
      for (Tuning.Value value : config.values()) {
        Map<String, Object> description = new LinkedHashMap<>();
        description.put("config", config.name());
        description.put("field", value.field());
        description.put("unit", value.unit());
        description.put(
            "limits", value.limits().equals(Limits.NONE) ? "" : value.limits().toString());
        ends(description, value.limits());
        description.put("committed", value.committed());
        description.put("record", value.record());
        description.put("group", value.group());
        description.put("meaning", value.meaning());
        described.put(value.path(), description);
      }
    }
    return described;
  }

  /** Each configuration value now, by its tunable. */
  static Map<String, Object> values(Tuning.View view) {
    Map<String, Object> values = new LinkedHashMap<>();
    for (Tuning.ConfigView config : view.configs()) {
      for (Tuning.Value value : config.values()) {
        values.put(value.path(), value.current());
      }
    }
    return values;
  }

  /** What differs from committed, by tunable, with the builder call that commits it. */
  static Map<String, Object> changes(Tuning.View view) {
    Map<String, Object> changes = new LinkedHashMap<>();
    for (Tuning.ConfigView config : view.configs()) {
      Map<String, String> paths = new LinkedHashMap<>();
      config.values().forEach(value -> paths.put(value.field(), value.path()));
      for (LiveConfig.Change change : config.changes()) {
        Map<String, Object> shown = new LinkedHashMap<>();
        shown.put("config", config.name());
        shown.put("field", change.path());
        shown.put("committed", change.committed());
        shown.put("tuned", change.tuned());
        shown.put("builderCall", change.builderCall());
        changes.put(Objects.requireNonNullElse(paths.get(change.path()), change.path()), shown);
      }
    }
    return changes;
  }

  /** What can be run: {@code {"rigs": {...}, "experiments": {...}}}. */
  Map<String, Object> experimentsView() {
    Map<String, Object> shown = new LinkedHashMap<>();
    shown.put("rigs", rigs(gainPaths()));
    shown.put("experiments", experimentList());
    return shown;
  }

  /**
   * A run as the page records it: which experiment on which rig, with what gains (by the tunable
   * each is published as), where it stands, its signals, and its scores once it has ended.
   */
  Map<String, Object> run(Run run) {
    Experiment experiment = run.experiment();
    Map<String, Object> shown = new LinkedHashMap<>();
    shown.put("id", run.id());
    shown.put("experiment", experiment.name());
    shown.put("rig", experiment.rig().name());
    shown.put("settings", experiment.settings());
    Map<String, Object> gains = new LinkedHashMap<>();
    Map<String, String> paths = gainPaths().getOrDefault(experiment.rig().name(), Map.of());
    run.gains().forEach((field, value) -> gains.put(paths.getOrDefault(field, field), value));
    shown.put("gains", gains);
    shown.put("startNanos", run.startNanos());
    shown.put("controller", run.controller());
    shown.put("gainsChanged", run.gainsChanged());
    Run.State state = run.state();
    shown.put("phase", state.phase().name());
    shown.put("detail", state.detail());
    shown.put("signals", signals(run.trace().signals()));
    Run.Scores scores = state.scores();
    if (scores != null) {
      shown.put("scores", scores(scores));
    }
    return shown;
  }

  private void rebuildTuningIfChanged(List<LiveConfig<?>> configs) {
    List<Object> from = new ArrayList<>();
    for (LiveConfig<?> config : configs) {
      from.add(config.get());
    }
    String lock = tuning.whyLocked();
    if (sameObjects(from, builtFrom) && Objects.equals(lock, builtLock)) {
      return;
    }
    Tuning.View view = tuning.view();
    tunableSection = tunables(view);
    valueSection = values(view);
    changeSection = changes(view);
    builtFrom = from;
    builtLock = lock;
  }

  private static Map<String, Object> actionStates(List<Actions.State> states) {
    Map<String, Object> shown = new LinkedHashMap<>();
    for (Actions.State state : states) {
      Map<String, Object> each = pair("phase", state.phase().name(), "detail", state.detail());
      each.put("locked", state.locked());
      each.put("when", state.when().name());
      shown.put(state.name(), each);
    }
    return shown;
  }

  private static Map<String, Object> wpilibTunables(List<TunableTee.Listed> listed) {
    Map<String, Object> shown = new LinkedHashMap<>();
    for (TunableTee.Listed tunable : listed) {
      Map<String, Object> described = new LinkedHashMap<>();
      described.put("type", tunable.type());
      described.put("value", tunable.value());
      described.put("editable", tunable.editable());
      described.put(
          "limits", tunable.limits().equals(Limits.NONE) ? "" : tunable.limits().toString());
      ends(described, tunable.limits());
      shown.put(tunable.path(), described);
    }
    return shown;
  }

  /**
   * The rigs' and experiments' sections, rebuilt only when an experiment is added, or a gain's
   * tunable is published at last: both happen as the robot starts.
   */
  private void rebuildRigsIfChanged() {
    List<Experiment> now = experiments.experiments();
    if (now == rigsBuiltFor && pathsFound) {
      return;
    }
    Map<String, Map<String, String>> paths = gainPaths();
    rigSection = rigs(paths);
    experimentSection = experimentList();
    rigsBuiltFor = now;
    pathsFound =
        paths.values().stream()
            .allMatch(found -> found.values().stream().allMatch(p -> p.startsWith("/")));
  }

  /** Each rig's gains, by field, as the WPILib tunables they're published as; the field if not. */
  private Map<String, Map<String, String>> gainPaths() {
    Map<String, Map<String, String>> paths = new LinkedHashMap<>();
    for (Rig rig : experiments.rigs()) {
      Map<String, String> gains = new LinkedHashMap<>();
      for (String gain : rig.gains()) {
        gains.put(gain, Objects.requireNonNullElse(tunables.pathOf(rig.config(), gain), gain));
      }
      paths.put(rig.name(), gains);
    }
    return paths;
  }

  private Map<String, Object> rigs(Map<String, Map<String, String>> paths) {
    Map<String, Object> shown = new LinkedHashMap<>();
    for (Rig rig : experiments.rigs()) {
      Map<String, Object> described = new LinkedHashMap<>();
      described.put("config", rig.config().name());
      described.put("gains", List.copyOf(paths.getOrDefault(rig.name(), Map.of()).values()));
      described.put("limits", rig.limits());
      described.put("unit", rig.measurementUnit().symbol());
      described.put("measurements", rig.measurements().stream().map(Trace.Signal::name).toList());
      described.put("signals", signals(rig.signals()));
      shown.put(rig.name(), described);
    }
    return shown;
  }

  /** Each experiment's rig, its settings, and how long it runs, for the page to count down. */
  private Map<String, Object> experimentList() {
    Map<String, Object> shown = new LinkedHashMap<>();
    for (Experiment experiment : experiments.experiments()) {
      Map<String, Object> described =
          pair("rig", experiment.rig().name(), "settings", experiment.settings());
      described.put("seconds", experiment.profile().durationSeconds());
      shown.put(experiment.name(), described);
    }
    return shown;
  }

  private static List<Object> signals(List<Trace.Signal> signals) {
    List<Object> shown = new ArrayList<>();
    for (Trace.Signal signal : signals) {
      shown.add(pair("name", signal.name(), "unit", signal.unit().symbol()));
    }
    return shown;
  }

  /**
   * A run's scores. A score that couldn't be had (it never settled) is NaN, which goes as the text
   * "NaN": in a merge patch, null would mean removed.
   */
  private static Map<String, Object> scores(Run.Scores scores) {
    Map<String, Object> steps = new LinkedHashMap<>();
    scores
        .steps()
        .forEach(
            (measurement, jumps) -> {
              List<Object> shown = new ArrayList<>();
              for (Optional<StepResponse> jump : jumps) {
                shown.add(step(jump));
              }
              steps.put(measurement, shown);
            });
    Map<String, Object> peaks = new LinkedHashMap<>();
    scores.peaks().forEach((signal, peak) -> peaks.put(signal, orNaN(peak)));
    return pair("steps", steps, "peaks", peaks);
  }

  private static Map<String, Object> step(Optional<StepResponse> jump) {
    Map<String, Object> shown = new LinkedHashMap<>();
    shown.put("rise", jump.map(s -> orNaN(s.riseSeconds())).orElse(Double.NaN));
    shown.put("overshoot", jump.map(StepResponse::overshoot).orElse(Double.NaN));
    shown.put("settling", jump.map(s -> orNaN(s.settlingSeconds())).orElse(Double.NaN));
    shown.put("error", jump.map(StepResponse::steadyStateError).orElse(Double.NaN));
    shown.put("samples", jump.map(StepResponse::samples).orElse(0));
    return shown;
  }

  /** A record's documentation as the page reads it: its summary and its text. */
  private static Map<String, Object> doc(ParameterDocs doc) {
    return pair("summary", doc.summary(), "text", doc.text());
  }

  /**
   * A tuning limit's ends as numbers, {@code "min"} and {@code "max"}, beside its text, so the page
   * can draw them and check an edit before sending it; an open end has none.
   */
  static void ends(Map<String, Object> shown, Limits limits) {
    if (Double.isFinite(limits.min())) {
      shown.put("min", limits.min());
    }
    if (Double.isFinite(limits.max())) {
      shown.put("max", limits.max());
    }
  }

  /** Two values by name, in this order, as JSON writes them. */
  private static Map<String, Object> pair(String a, Object first, String b, Object second) {
    Map<String, Object> pair = new LinkedHashMap<>();
    pair.put(a, first);
    pair.put(b, second);
    return pair;
  }

  private static double orNaN(OptionalDouble value) {
    return value.isPresent() ? value.getAsDouble() : Double.NaN;
  }

  private static boolean sameObjects(List<?> a, List<?> b) {
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
}
