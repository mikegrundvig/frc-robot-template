package com.michaelgrundvig.frc.pitwall.experiment;

import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.Outcome;
import java.time.Duration;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import org.jspecify.annotations.Nullable;

/**
 * The experiments the page can run, and the run in progress. Add one with a line: {@code
 * pitwall.experiments().add(new Experiment("Step to 90° and back", steering, profile))}.
 *
 * <p>Each run is a Commands v3 command requiring its rig's mechanisms, held to what actions are
 * held to: it starts only while the robot is enabled in utility mode and never on the field, and
 * leaving utility mode or disabling cancels it. One runs at a time. Like the page's other requests,
 * starts and cancels wait for the robot loop, which applies and logs them, so a replay runs the
 * same runs. Its trace comes from the rig's signals, which read logged inputs, so the replay
 * records the same trace.
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

  /**
   * A request from the page, waiting for the robot loop.
   *
   * @param run for a cancel, the run it's for; 0 for a start
   * @param why for a cancel, why, as the run's detail shows it; empty for "from the page"
   * @param stream for a start, the event stream of the page asking; null for none
   */
  private record Request(
      ExperimentInputs.Kind kind,
      String name,
      long run,
      String why,
      @Nullable String stream,
      CompletableFuture<Outcome> outcome) {}

  /**
   * Whom the run in progress answers to, as one value so it changes whole: the page's stream, when
   * it was last heard from, and whether its stream has closed.
   */
  private record Controller(@Nullable String stream, long heardNanos, boolean lost) {}

  private final Robot robot;
  private final Consumer<ExperimentInputs> logInputs;
  private final LongSupplier clock;
  private final boolean replaying;
  private final Launcher launcher;
  private final Consumer<String> report;
  private final Map<String, Experiment> experiments = new LinkedHashMap<>();
  private final Map<String, Rig> rigs = new LinkedHashMap<>();
  private final ConcurrentLinkedQueue<Request> queue = new ConcurrentLinkedQueue<>();
  private final ExperimentInputs inputs = new ExperimentInputs();
  private final AtomicReference<Controller> controller =
      new AtomicReference<>(new Controller(null, 0, false));
  private volatile List<Experiment> experimentView = List.of();
  private volatile List<Rig> rigView = List.of();
  private volatile @Nullable Run run;

  /** Why an experiment can't start now, as of the last loop; null while one can. */
  private volatile @Nullable String lock =
      new Actions.Gate(false, false, false).whyNot("experiments");

  /** What drives the run in progress; touched only by the robot loop. */
  private Actions.@Nullable Routine driving;

  private int runsStarted;

  /** The last run whose end was reported. */
  private int reportedThrough;

  /** Whether the deadman has asked to cancel the run in progress, so it asks once. */
  private boolean deadmanAsked;

  /**
   * @param robot the robot runs are recorded on
   * @param logInputs logs the starts and cancels one loop applies, or in a replay replaces them
   *     with the logged ones
   * @param robotClock the robot's clock, in nanoseconds ({@code RobotController::getTime}): it
   *     times the runs, which a replay reproduces, and the deadman, which in the simulator's tests
   *     then steps with the loops
   * @param replaying whether this is a replay, where the log's cancels stand in for the deadman
   * @param report told each run's end, in a sentence, such as why the deadman canceled it
   */
  public Experiments(
      Robot robot,
      Consumer<ExperimentInputs> logInputs,
      LongSupplier robotClock,
      boolean replaying,
      Consumer<String> report) {
    this(
        robot,
        logInputs,
        robotClock,
        replaying,
        run -> Actions.command(run.experiment().rig()::scheduler, () -> run.command(robotClock)),
        report);
  }

  /**
   * @param clock the time requests arrive, in nanoseconds, for the deadman
   * @param launcher how a run is driven
   */
  Experiments(
      Robot robot,
      Consumer<ExperimentInputs> logInputs,
      LongSupplier clock,
      boolean replaying,
      Launcher launcher,
      Consumer<String> report) {
    this.robot = robot;
    this.logInputs = logInputs;
    this.clock = clock;
    this.replaying = replaying;
    this.launcher = launcher;
    this.report = report;
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
   * Why an experiment can't start now, as of the last loop, such as "experiments run only with the
   * robot enabled in utility mode"; null while one can. Safe to call from any thread.
   */
  public @Nullable String whyLocked() {
    return lock;
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
   *     none, when requests naming no stream keep the run going
   */
  public CompletableFuture<Outcome> requestStart(String name, @Nullable String stream) {
    return enqueue(ExperimentInputs.Kind.START, name, 0, "", stream);
  }

  /**
   * Asks the robot loop to cancel a run, if it's still the one running. Safe to call from any
   * thread.
   *
   * @param run the run to cancel: a cancel meant for a run that has ended never stops the next
   * @param why why, as the run's detail will say, such as {@code "this tab was hidden"}; empty for
   *     "from the page"
   */
  public CompletableFuture<Outcome> requestCancel(String name, long run, String why) {
    return enqueue(ExperimentInputs.Kind.CANCEL, name, run, why, null);
  }

  /**
   * A request arrived: from the page that started the run, it keeps the run going. Safe to call
   * from any thread.
   *
   * @param stream the event stream of the page it came from; null for none
   */
  public void heard(@Nullable String stream) {
    long now = clock.getAsLong();
    controller.updateAndGet(
        c -> Objects.equals(stream, c.stream()) ? new Controller(c.stream(), now, c.lost()) : c);
  }

  /**
   * A page's event stream closed: if that page started the run, the run is canceled. Safe to call
   * from any thread.
   */
  public void lost(String stream) {
    controller.updateAndGet(
        c -> stream.equals(c.stream()) ? new Controller(c.stream(), c.heardNanos(), true) : c);
  }

  /**
   * Applies the page's requests, and the deadman's. Call it once per loop, before the command
   * scheduler runs.
   */
  public void periodic(Actions.Gate gate) {
    Run current = run;
    reportIfEnded(current); // runs end in the scheduler, after the last loop's periodic
    if (current != null && isRunning(current) && !replaying && !deadmanAsked) {
      String quiet = whyQuiet();
      if (quiet != null) {
        // A request like the page's own, so it's logged, and a replay cancels at the same moment.
        deadmanAsked = true;
        requestCancel(current.experiment().name(), current.id(), quiet);
      }
    }
    List<Request> requests = drainInto(inputs);
    logInputs.accept(inputs); // in replay, this replaces them with the logged ones

    Actions.Routine routine = driving;
    if (current != null && isRunning(current) && routine != null && !isDriven(routine)) {
      current.cancel("Canceled: its command stopped before the run ended");
    }
    String locked = gate.whyNot("experiments");
    lock = locked;
    if (current != null && isRunning(current) && locked != null) {
      stop(current, "Canceled: " + locked);
    }
    List<Outcome> outcomes = new ArrayList<>();
    for (int i = 0; i < inputs.kinds.length; i++) {
      // In a replay the requests come from the log, which keeps what was asked, not why.
      Request request = i < requests.size() ? requests.get(i) : null;
      outcomes.add(
          apply(
              ExperimentInputs.Kind.valueOf(inputs.kinds[i]),
              inputs.names[i],
              i < inputs.runs.length ? inputs.runs[i] : 0,
              request != null ? request.why() : "",
              request != null ? request.stream() : null,
              locked));
    }
    reportIfEnded(current);
    reportIfEnded(run);
    // Only now, so whoever is waiting sees the state their request left. In replay, no one is.
    for (int i = 0; i < requests.size() && i < outcomes.size(); i++) {
      requests.get(i).outcome().complete(outcomes.get(i));
    }
  }

  private Outcome apply(
      ExperimentInputs.Kind kind,
      String name,
      long runId,
      String why,
      @Nullable String stream,
      @Nullable String locked) {
    Experiment experiment = experiments.get(name);
    if (experiment == null) {
      return new Outcome(Outcome.Result.REFUSED, "There is no experiment named " + name);
    }
    Run current = run;
    boolean busy = current != null && isRunning(current);
    if (kind == ExperimentInputs.Kind.CANCEL) {
      if (current == null || !busy || !current.experiment().name().equals(name)) {
        return new Outcome(Outcome.Result.LOCKED, name + " isn't running");
      }
      if (runId != 0 && current.id() != runId) {
        return new Outcome(Outcome.Result.LOCKED, "Run " + runId + " has ended already");
      }
      stop(current, why.isEmpty() ? "Canceled from the page" : "Canceled: " + why);
      return Outcome.applied();
    }
    if (locked != null) {
      return new Outcome(Outcome.Result.LOCKED, "Experiments are locked: " + locked);
    }
    if (current != null && busy) {
      return new Outcome(
          Outcome.Result.REFUSED,
          current.experiment().name() + " is running; one experiment runs at a time");
    }
    Run next = new Run(runsStarted + 1, experiment, stream);
    Actions.Routine launched = launcher.launch(next);
    String refused;
    try {
      refused = launched.start();
    } catch (RuntimeException e) {
      refused = String.valueOf(e.getMessage());
    }
    if (refused != null) {
      return new Outcome(Outcome.Result.REFUSED, refused);
    }
    runsStarted++;
    driving = launched;
    controller.set(new Controller(stream, clock.getAsLong(), false));
    deadmanAsked = false;
    run = next;
    return Outcome.applied();
  }

  /**
   * Stops a run, saying why first: its command's own cleanup, which runs as the command is
   * canceled, then finds it ended and leaves the reason alone.
   */
  private void stop(Run current, String why) {
    current.cancel(why);
    Actions.Routine routine = driving;
    if (routine != null) {
      try {
        routine.cancel();
      } catch (RuntimeException e) {
        report.accept(current + " (canceling its command failed: " + e + ")");
      }
    }
  }

  /** Says once how a run ended: printed for the team, and with AdvantageKit, logged. */
  private void reportIfEnded(@Nullable Run ended) {
    if (ended != null && !isRunning(ended) && ended.id() > reportedThrough) {
      reportedThrough = ended.id();
      report.accept(ended.toString());
    }
  }

  private static boolean isRunning(Run run) {
    return run.state().phase() == Run.Phase.RUNNING;
  }

  private static boolean isDriven(Actions.Routine routine) {
    try {
      return routine.isRunning();
    } catch (RuntimeException e) {
      return false;
    }
  }

  /** Why the deadman cancels the run in progress, or null while its page keeps in touch. */
  private @Nullable String whyQuiet() {
    Controller now = Objects.requireNonNull(controller.get());
    if (now.lost()) {
      return "the page that started it closed";
    }
    if (clock.getAsLong() - now.heardNanos() > DEADMAN.toNanos()) {
      return "the page that started it went quiet for over " + DEADMAN.toMillis() + " ms";
    }
    return null;
  }

  private CompletableFuture<Outcome> enqueue(
      ExperimentInputs.Kind kind, String name, long run, String why, @Nullable String stream) {
    CompletableFuture<Outcome> outcome = new CompletableFuture<>();
    queue.add(new Request(kind, name, run, why, stream, outcome));
    return outcome;
  }

  /** Moves the queued requests into the inputs, and returns them in request order. */
  private List<Request> drainInto(ExperimentInputs inputs) {
    if (queue.isEmpty()) {
      inputs.clearRequests();
      return List.of();
    }
    List<Request> requests = new ArrayList<>();
    for (Request request = queue.poll(); request != null; request = queue.poll()) {
      requests.add(request);
    }
    inputs.kinds = requests.stream().map(r -> r.kind().name()).toArray(String[]::new);
    inputs.names = requests.stream().map(Request::name).toArray(String[]::new);
    inputs.runs = requests.stream().mapToLong(Request::run).toArray();
    return requests;
  }
}
