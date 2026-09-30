package com.michaelgrundvig.frc.pitwall.experiment;

import com.michaelgrundvig.frc.pitwall.ActionInputs;
import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.Outcome;
import java.time.Duration;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import org.jspecify.annotations.Nullable;

/**
 * The experiments the page can run, and the run in progress. Add one with a line: {@code
 * pitwall.experiments().add(new Experiment("Step to 90° and back", steering, profile))}.
 *
 * <p>Each experiment runs as a Commands v3 command through {@link Actions}, so it has what actions
 * have: it runs only while the robot is enabled in utility mode and never on the field, leaving
 * utility mode or disabling cancels it, and with AdvantageKit its starts and cancels are logged, so
 * a replay runs it again. Its trace comes from the rig's signals, which read logged inputs, so the
 * replay records the same trace. One experiment runs at a time.
 *
 * <p>An experiment moves a mechanism, so it has a deadman as well: it runs only while the page that
 * started it keeps in touch. Every request from that page counts, and the page sends an empty
 * heartbeat when it has been quiet. Silence for {@link #DEADMAN}, or that page's event stream
 * closing, cancels the run. The web server tells this class what it hears ({@link #heard}, {@link
 * #lost}).
 *
 * <p>An experiment never changes a gain. The page changes gains the way it changes any value,
 * through Pitwall's checks; a run records the gains it ran with.
 */
public final class Experiments {
  /** How long the page that started a run may go quiet before the run is canceled. */
  public static final Duration DEADMAN = Duration.ofMillis(500);

  /** The longest an experiment may run: its trace is kept whole in memory. */
  public static final double LONGEST_SECONDS = 10;

  /** How fast a signal may be sampled for its trace to hold a whole run: 1 kHz. */
  static final int SAMPLES_PER_SECOND = 1000;

  /**
   * The robot a run ran on, for its record, so runs from another robot or other code are told
   * apart.
   *
   * @param name the robot's name
   * @param simulated whether it's the simulator
   * @param code which code: the git SHA and branch, as the status shows it; empty when unknown
   */
  public record Robot(String name, boolean simulated, String code) {}

  /** How a run is driven: as a command on its rig's scheduler, or by a test, loop by loop. */
  @FunctionalInterface
  interface Launcher {
    Actions.Routine launch(Run run);
  }

  private final Robot robot;
  private final Actions runs;
  private final LongSupplier wallClock;
  private final boolean replaying;
  private final Launcher launcher;
  private final Map<String, Experiment> experiments = new LinkedHashMap<>();
  private final Map<String, Rig> rigs = new LinkedHashMap<>();
  private volatile List<Experiment> experimentView = List.of();
  private volatile List<Rig> rigView = List.of();
  private volatile @Nullable Run run;
  private int runsStarted;

  /** Which page asked to start each experiment: its event stream's id, or "" for none. */
  private final Map<String, String> askedBy = new ConcurrentHashMap<>();

  // The deadman: which page started the run, when it was last heard from, and whether it's gone.
  private volatile @Nullable String controller;
  private volatile long heardNanos;
  private volatile boolean controllerLost;

  /** Why the deadman canceled the run in progress, or null if it hasn't. */
  private @Nullable String fired;

  /**
   * @param robot the robot runs are recorded on
   * @param logInputs logs the starts and cancels one loop applies, or in a replay replaces them
   *     with the logged ones
   * @param robotClock the robot's clock, in nanoseconds ({@code RobotController::getTime}), which a
   *     replay reproduces
   * @param replaying whether this is a replay, where the log's cancels stand in for the deadman
   */
  public Experiments(
      Robot robot, Consumer<ActionInputs> logInputs, LongSupplier robotClock, boolean replaying) {
    this(
        robot,
        logInputs,
        System::nanoTime,
        replaying,
        run -> Actions.command(run.experiment().rig()::scheduler, () -> run.command(robotClock)));
  }

  /**
   * @param wallClock the time requests arrive, in nanoseconds, for the deadman
   * @param launcher how a run is driven
   */
  Experiments(
      Robot robot,
      Consumer<ActionInputs> logInputs,
      LongSupplier wallClock,
      boolean replaying,
      Launcher launcher) {
    this.robot = robot;
    runs = new Actions(logInputs);
    this.wallClock = wallClock;
    this.replaying = replaying;
    this.launcher = launcher;
  }

  /**
   * Adds an experiment the page can run.
   *
   * @return this, to add the next
   * @throws IllegalArgumentException if its name is taken, its rig can't run its profile (another
   *     kind of quantity, or outside the rig's limits), it runs longer than {@link
   *     #LONGEST_SECONDS}, or another rig has its rig's name
   */
  public Experiments add(Experiment experiment) {
    String name = experiment.name();
    if (experiments.containsKey(name)) {
      throw new IllegalArgumentException("two experiments are named " + name);
    }
    Rig rig = experiment.rig();
    String whyNot = rig.whyNot(experiment.profile());
    if (whyNot != null) {
      throw new IllegalArgumentException(name + " can't run on " + rig.name() + ": " + whyNot);
    }
    double seconds = experiment.profile().durationSeconds();
    if (seconds > LONGEST_SECONDS) {
      throw new IllegalArgumentException(
          name + " runs " + seconds + " s; an experiment runs at most " + LONGEST_SECONDS + " s");
    }
    Rig known = rigs.get(rig.name());
    if (known != null && known != rig) {
      throw new IllegalArgumentException("two rigs are named " + rig.name());
    }
    runs.add(name, routine(experiment));
    experiments.put(name, experiment);
    rigs.put(rig.name(), rig);
    experimentView = List.copyOf(experiments.values());
    rigView = List.copyOf(rigs.values());
    return this;
  }

  /** The robot runs are recorded on. */
  public Robot robot() {
    return robot;
  }

  /** Every experiment, in the order added. Safe to call from any thread. */
  public List<Experiment> experiments() {
    return experimentView;
  }

  /** Every rig the experiments run on, in the order first added. Safe to call from any thread. */
  public List<Rig> rigs() {
    return rigView;
  }

  /** Whether an experiment has this name. Safe to call from any thread. */
  public boolean has(String name) {
    return experimentView.stream().anyMatch(experiment -> experiment.name().equals(name));
  }

  /**
   * The run in progress, or the latest to end; null before the first. Safe to call from any thread.
   */
  public @Nullable Run run() {
    return run;
  }

  /**
   * Asks the robot loop to start an experiment. Safe to call from any thread.
   *
   * @param stream the event stream of the page asking, whose silence the deadman watches; null for
   *     a client without one
   */
  public CompletableFuture<Outcome> requestStart(String name, @Nullable String stream) {
    askedBy.put(name, stream == null ? "" : stream);
    return runs.requestStart(name);
  }

  /** Asks the robot loop to cancel an experiment, if it's running. Safe to call from any thread. */
  public CompletableFuture<Outcome> requestCancel(String name) {
    return runs.requestCancel(name);
  }

  /**
   * A request arrived: from the page that started the run, it keeps the run going. Safe to call
   * from any thread.
   *
   * @param stream the event stream of the page it came from; null for none
   */
  public void heard(@Nullable String stream) {
    if (Objects.equals(stream, controller)) {
      heardNanos = wallClock.getAsLong();
    }
  }

  /**
   * A page's event stream closed: if that page started the run, the run is canceled. Safe to call
   * from any thread.
   */
  public void lost(String stream) {
    if (stream.equals(controller)) {
      controllerLost = true;
    }
  }

  /**
   * Applies the page's requests, and the deadman's. Call it once per loop, before the command
   * scheduler runs.
   */
  public void periodic(Actions.Gate gate) {
    Run current = run;
    boolean wasRunning = current != null && current.state().phase() == Run.Phase.RUNNING;
    if (current != null && wasRunning && !replaying && fired == null) {
      String quiet = whyQuiet();
      if (quiet != null) {
        // A request like the page's own, so it's logged, and a replay cancels at the same moment.
        fired = quiet;
        runs.requestCancel(current.experiment().name());
      }
    }
    runs.periodic(gate);
    if (current == null || !wasRunning) {
      return;
    }
    // Stopped by something other than its own end: say what, as the actions say it.
    Actions.State action = stateOf(current.experiment().name());
    String why =
        fired != null
            ? "Canceled: " + fired
            : action.phase() == Actions.Phase.CANCELED
                ? action.detail()
                : "Canceled: its command stopped before the run ended";
    if (current.state().phase() == Run.Phase.RUNNING && action.phase() != Actions.Phase.RUNNING) {
      current.cancel(why); // it never got to run, so its command couldn't say
    } else if (action.phase() == Actions.Phase.CANCELED) {
      current.explain(why);
    }
  }

  /** Why the deadman cancels the run in progress, or null while its page keeps in touch. */
  private @Nullable String whyQuiet() {
    if (controllerLost) {
      return "the page that started it closed";
    }
    if (wallClock.getAsLong() - heardNanos > DEADMAN.toNanos()) {
      return "the page that started it went quiet for over " + DEADMAN.toMillis() + " ms";
    }
    return null;
  }

  private Actions.State stateOf(String name) {
    for (Actions.State state : runs.view()) {
      if (state.name().equals(name)) {
        return state;
      }
    }
    throw new IllegalStateException("no experiment named " + name);
  }

  /**
   * How an experiment starts as an action: refused while another runs; otherwise a new run, driven
   * by the launcher, watched by the deadman from now.
   */
  private Actions.Routine routine(Experiment experiment) {
    return new Actions.Routine() {
      private Actions.@Nullable Routine running;

      @Override
      public @Nullable String start() {
        Run current = run;
        if (current != null && current.state().phase() == Run.Phase.RUNNING) {
          return current.experiment().name() + " is running; one experiment runs at a time";
        }
        Run next = new Run(runsStarted + 1, experiment, capacity(experiment));
        Actions.Routine launched = launcher.launch(next);
        String refused = launched.start();
        if (refused != null) {
          return refused;
        }
        runsStarted++;
        running = launched;
        String asker = askedBy.remove(experiment.name());
        controller = asker == null || asker.isEmpty() ? null : asker;
        controllerLost = false;
        heardNanos = wallClock.getAsLong();
        fired = null;
        run = next;
        return null;
      }

      @Override
      public boolean isRunning() {
        Actions.Routine launched = running;
        return launched != null && launched.isRunning();
      }

      @Override
      public void cancel() {
        Actions.Routine launched = running;
        if (launched != null) {
          launched.cancel();
        }
      }
    };
  }

  /** Room in each signal's series for the whole run at 1 kHz, and half a second to spare. */
  private static int capacity(Experiment experiment) {
    return (int) Math.ceil((experiment.profile().durationSeconds() + 0.5) * SAMPLES_PER_SECOND);
  }
}
