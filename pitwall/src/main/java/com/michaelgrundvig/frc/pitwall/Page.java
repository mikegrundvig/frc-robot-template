package com.michaelgrundvig.frc.pitwall;

import com.michaelgrundvig.frc.pitwall.experiment.Experiment;
import com.michaelgrundvig.frc.pitwall.experiment.Experiments;
import com.michaelgrundvig.frc.pitwall.experiment.Rig;
import com.michaelgrundvig.frc.pitwall.experiment.Run;
import com.michaelgrundvig.frc.pitwall.experiment.StepResponse;
import com.michaelgrundvig.frc.pitwall.experiment.Trace;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.UUID;
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

  /** Tells this run of the robot program from the last: the page keeps runs across restarts. */
  private final String program = UUID.randomUUID().toString();

  /** What the tuning sections were built from, and the sections, rebuilt when that changes. */
  private List<Object> builtFrom = List.of();

  private @Nullable String builtLock = "never built";
  private Map<String, Object> tunableSection = Map.of();
  private Map<String, Object> valueSection = Map.of();
  private Map<String, Object> changeSection = Map.of();

  public Page(
      Status status, Actions actions, Tuning tuning, TunableTee tunables, Experiments experiments) {
    this.status = status;
    this.actions = actions;
    this.tuning = tuning;
    this.tunables = tunables;
    this.experiments = experiments;
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

  /**
   * The page's state now, section by section. Call it from the robot loop.
   *
   * <ul>
   *   <li>{@code robot}: its name, the code, simulator or real, and this program's id;
   *   <li>{@code status}, {@code units}: each item's value, and the unit of each number;
   *   <li>{@code actions}: each action's phase and detail;
   *   <li>{@code tuning}: whether it's on, the field, and refused dashboard changes;
   *   <li>{@code tunables}, {@code values}, {@code changes}: the configurations' tunable values, by
   *       the WPILib tunable each is published as, as {@code /api/tunables}, {@code /api/values},
   *       and {@code /api/changes} give them;
   *   <li>{@code wpilibTunables}: every other WPILib tunable, by path;
   *   <li>{@code rigs}, {@code experiments}: what can be run;
   *   <li>{@code run}: the run in progress, or the latest to end.
   * </ul>
   */
  public Map<String, Map<String, Object>> sections() {
    rebuildTuningIfChanged();
    Map<String, Map<String, Object>> sections = new LinkedHashMap<>();
    sections.put("robot", robot());
    sections.put("status", status.values());
    sections.put("units", status.units());
    sections.put("actions", actionStates());
    sections.put("tuning", session());
    sections.put("tunables", tunableSection);
    sections.put("values", valueSection);
    sections.put("changes", changeSection);
    sections.put("wpilibTunables", wpilibTunables());
    sections.put("rigs", rigs());
    sections.put("experiments", experimentList());
    Run run = experiments.run();
    sections.put("run", run == null ? Map.of() : run(run));
    return sections;
  }

  Map<String, Object> robot() {
    Experiments.Robot robot = experiments.robot();
    Map<String, Object> shown = new LinkedHashMap<>();
    shown.put("name", robot.name());
    shown.put("simulated", robot.simulated());
    shown.put("code", robot.code());
    shown.put("program", program);
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

  /** Each configuration value, by its tunable: {@code {"config", "field", "unit", ...}}. */
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
        description.put("committed", value.committed());
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
    shown.put("rigs", rigs());
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
    run.gains().forEach((field, value) -> gains.put(pathOf(experiment.rig(), field), value));
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

  private void rebuildTuningIfChanged() {
    List<Object> from = new ArrayList<>();
    for (LiveConfig<?> config : tunables.configs()) {
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

  private Map<String, Object> actionStates() {
    Map<String, Object> shown = new LinkedHashMap<>();
    for (Actions.State state : actions.view()) {
      shown.put(state.name(), pair("phase", state.phase().name(), "detail", state.detail()));
    }
    return shown;
  }

  private Map<String, Object> wpilibTunables() {
    Map<String, Object> shown = new LinkedHashMap<>();
    for (TunableTee.Listed tunable : tunables.view()) {
      Map<String, Object> described = new LinkedHashMap<>();
      described.put("type", tunable.type());
      described.put("value", tunable.value());
      described.put("editable", tunable.editable());
      described.put(
          "limits", tunable.limits().equals(Limits.NONE) ? "" : tunable.limits().toString());
      shown.put(tunable.path(), described);
    }
    return shown;
  }

  private Map<String, Object> rigs() {
    Map<String, Object> shown = new LinkedHashMap<>();
    for (Rig rig : experiments.rigs()) {
      Map<String, Object> described = new LinkedHashMap<>();
      described.put("config", rig.config().name());
      described.put("gains", rig.gains().stream().map(gain -> pathOf(rig, gain)).toList());
      described.put("limits", rig.limits());
      described.put("unit", rig.measurementUnit().symbol());
      described.put("measurements", rig.measurements().stream().map(Trace.Signal::name).toList());
      described.put("signals", signals(rig.signals()));
      shown.put(rig.name(), described);
    }
    return shown;
  }

  private Map<String, Object> experimentList() {
    Map<String, Object> shown = new LinkedHashMap<>();
    for (Experiment experiment : experiments.experiments()) {
      shown.put(
          experiment.name(),
          pair("rig", experiment.rig().name(), "settings", experiment.settings()));
    }
    return shown;
  }

  /** Where a rig's gain is published as a WPILib tunable; its path in the configuration if not. */
  private String pathOf(Rig rig, String gain) {
    return Objects.requireNonNullElse(tunables.pathOf(rig.config(), gain), gain);
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
}
