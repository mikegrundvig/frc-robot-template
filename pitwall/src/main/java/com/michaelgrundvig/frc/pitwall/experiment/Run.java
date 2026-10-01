package com.michaelgrundvig.frc.pitwall.experiment;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.NeedsNameBuilderStage;

/**
 * One run of an experiment: the gains it started with, what it recorded, and, once it ends, its
 * scores. The robot keeps only the latest; the page keeps the rest.
 *
 * <p>Each loop the run commands its profile's setpoint through the rig and records the rig's
 * signals, until the profile ends. Then it scores each jump in the setpoint, and the peak of every
 * other signal. The loop's thread runs it; any other thread may read its trace and its {@link
 * #state()} at the same time.
 */
public final class Run {
  /** Where a run stands. */
  public enum Phase {
    RUNNING,
    /** Ran to the end of its profile, and is scored. */
    ENDED,
    /** Stopped early: by the page, the deadman, the robot leaving utility mode, or a command. */
    CANCELED
  }

  /**
   * How a run went.
   *
   * @param steps for each measurement, how it answered each jump, in order; empty where the jump's
   *     window held too few samples to say
   * @param peaks for each other signal (voltage, current), its largest magnitude during the run
   */
  public record Scores(
      Map<String, List<Optional<StepResponse>>> steps, Map<String, OptionalDouble> peaks) {}

  /**
   * Where a run stands, as one value so another thread reads it whole.
   *
   * @param detail a sentence about it: why it was canceled, or that a signal couldn't be read
   * @param scores once it has ended; null until then, and for a canceled run
   */
  public record State(Phase phase, String detail, @Nullable Scores scores) {}

  private final int id;
  private final Experiment experiment;
  private final String controller;
  private final Map<String, Double> gains;
  private final Trace trace;
  private volatile long startNanos;
  private volatile State state = new State(Phase.RUNNING, "Running", null);
  private int steps;
  private @Nullable String unread;

  /** The configuration its gains were last read from; a new one may hold new gains. */
  private Record checked;

  /** Its gains as they changed while it ran, by field: empty unless they did. */
  private volatile Map<String, Double> changedGains = Map.of();

  /**
   * A run, before its first loop. Its gains are read now, and its trace made with room for the
   * whole run.
   *
   * @param controller the event stream of the page that started it, which keeps it going; null for
   *     none
   */
  Run(int id, Experiment experiment, @Nullable String controller) {
    this.id = id;
    this.experiment = experiment;
    this.controller = controller == null ? "" : controller;
    gains = experiment.rig().gainValues();
    checked = experiment.rig().config().get();
    double seconds = experiment.profile().durationSeconds();
    trace =
        new Trace(experiment.rig().traced(), signal -> experiment.rig().capacity(signal, seconds));
  }

  /** Counts the runs since the robot program started: 1, 2, 3. */
  public int id() {
    return id;
  }

  public Experiment experiment() {
    return experiment;
  }

  /**
   * The event stream of the page that started it, which keeps it going with its requests; empty
   * when it was started without one.
   */
  public String controller() {
    return controller;
  }

  /** The rig's gains as the run started, by field, each in its knob's unit. */
  public Map<String, Double> gains() {
    return gains;
  }

  /**
   * What it recorded: the setpoint it commanded ({@link Rig#SETPOINT}), then the rig's measurements
   * and signals. Safe to read from any thread while it records.
   */
  public Trace trace() {
    return trace;
  }

  /** When its first loop ran, on the robot's clock; 0 before. Its trace's times count from it. */
  public long startNanos() {
    return startNanos;
  }

  /** Where it stands. Safe to call from any thread. */
  public State state() {
    return state;
  }

  /**
   * One loop of the run: what the signals read since the last loop, then this loop's setpoint. Once
   * the profile's time is up, it scores the run and ends it.
   *
   * @param nowNanos this loop's time on the robot's clock
   * @return whether the run is over
   */
  boolean step(long nowNanos) {
    if (state.phase() != Phase.RUNNING) {
      return true;
    }
    Rig rig = experiment.rig();
    checkGains(rig);
    if (steps == 0) {
      startNanos = nowNanos; // the first loop's samples came before the run began
    } else {
      String failed = rig.sample(nowNanos, trace);
      if (failed != null && unread == null) {
        unread = failed;
      }
    }
    Profile profile = experiment.profile();
    double seconds = (nowNanos - startNanos) / 1e9;
    if (seconds >= profile.durationSeconds()) {
      List<String> but = new ArrayList<>();
      if (!changedGains.isEmpty()) {
        but.add("its gains changed while it ran (" + changes() + "), so it's no fair comparison");
      }
      if (unread != null) {
        but.add(unread);
      }
      end(Phase.ENDED, but.isEmpty() ? "Ended" : "Ended, but " + String.join("; ", but), scores());
      return true;
    }
    double setpoint = rig.clamped(profile.setpoint(seconds));
    try {
      rig.command(setpoint);
    } catch (RuntimeException e) {
      String why = e.getMessage();
      cancel("Canceled: commanding the setpoint failed: " + (why != null ? why : e));
      return true;
    }
    trace.series(Rig.SETPOINT).add(nowNanos, rig.measurementUnit().fromBaseUnits(setpoint));
    steps++;
    return false;
  }

  /**
   * The run as a Commands v3 command requiring the rig's mechanisms: a step each loop until it's
   * over. Canceled, it ends the run; a Commands v3 cancel never resumes the coroutine, so the
   * cleanup is {@code whenCanceled}, not a {@code finally}.
   *
   * @param clock the robot's clock, in nanoseconds
   */
  Command command(LongSupplier clock) {
    Consumer<Coroutine> body =
        coroutine -> {
          while (!step(clock.getAsLong())) {
            coroutine.yield();
          }
        };
    NeedsNameBuilderStage built =
        experiment.rig().requirements().isEmpty()
            ? Command.noRequirements(body)
            : Command.requiring(experiment.rig().requirements()).executing(body);
    return built
        .whenCanceled(() -> cancel("Canceled: another command needed " + experiment.rig().name()))
        .named(experiment.name());
  }

  /**
   * Whether its gains changed while it ran, from the page's tuning table or a dashboard: then its
   * scores answer for more than one set of gains, and no fair comparison with other runs.
   */
  public boolean gainsChanged() {
    return !changedGains.isEmpty();
  }

  /**
   * Notes any gain that changed since the run started. A changed configuration is a new record, so
   * comparing one reference each loop is enough; the gains are read only when it has changed.
   */
  private void checkGains(Rig rig) {
    Record now = rig.config().get();
    if (now == checked) {
      return;
    }
    checked = now;
    Map<String, Double> changed = new LinkedHashMap<>(changedGains);
    rig.gainValues()
        .forEach(
            (field, value) -> {
              if (!value.equals(gains.get(field))) {
                changed.put(field, value);
              }
            });
    changedGains = Map.copyOf(changed);
  }

  /** The gains that changed, as {@code gains.steerKp 16.0 to 20.0}. */
  private String changes() {
    List<String> changes = new ArrayList<>();
    changedGains.forEach(
        (field, value) -> changes.add(field + " " + gains.get(field) + " to " + value));
    return String.join(", ", changes);
  }

  /** Ends the run early, if it's still going. */
  void cancel(String detail) {
    if (state.phase() == Phase.RUNNING) {
      end(Phase.CANCELED, detail, null);
    }
  }

  private void end(Phase phase, String detail, @Nullable Scores scores) {
    state = new State(phase, detail, scores);
  }

  /**
   * Each measurement's answer to each jump, in the measurement's unit, and every other signal's
   * peak over the run.
   */
  private Scores scores() {
    Rig rig = experiment.rig();
    Profile profile = experiment.profile();
    Map<String, List<Optional<StepResponse>>> steps = new LinkedHashMap<>();
    for (Trace.Signal measured : rig.measurements()) {
      steps.put(
          measured.name(),
          List.copyOf(
              StepResponse.of(
                  trace.series(measured.name()), profile, measured.unit(), startNanos)));
    }
    long endNanos = startNanos + Math.round(profile.durationSeconds() * 1e9);
    Map<String, OptionalDouble> peaks = new LinkedHashMap<>();
    for (Trace.Signal signal : rig.signals()) {
      peaks.put(signal.name(), trace.series(signal.name()).peakMagnitude(startNanos, endNanos));
    }
    return new Scores(steps, peaks);
  }

  @Override
  public String toString() {
    return "run " + id + " of " + experiment.name() + ": " + state.detail();
  }
}
