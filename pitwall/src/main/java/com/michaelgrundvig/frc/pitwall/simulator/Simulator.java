package com.michaelgrundvig.frc.pitwall.simulator;

import com.michaelgrundvig.frc.pitwall.Outcome;
import java.time.Duration;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReferenceArray;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import org.jspecify.annotations.Nullable;

/**
 * The simulator's controls, for the page: what the WPILib simulator GUI does for the robot program
 * (the Driver Station, joysticks, pausing and stepping the clock, stopping), and what it can't do,
 * failing the simulated robot's components live ({@link Components}). It exists only in the
 * simulator: on a robot, Pitwall has none, so nothing can ever write the Driver Station's state or
 * a joystick there.
 *
 * <p>Like the rest of Pitwall, only the robot loop changes anything, with three exceptions the
 * clock forces. Requests from the web server's threads wait in a queue, and {@link #periodic},
 * first thing in the loop, applies them. A Driver Station change is announced as a real Driver
 * Station's packet is: the robot program sees it from the next loop on, read as that loop begins,
 * so WPILib's mode changes, the robot's code, and AdvantageKit's log all see it in the same loop,
 * and a replay sees it there too. A joystick's input is handed over the same way: the server keeps
 * the latest for each joystick, and the loop takes it. But a paused clock runs no loops, so
 * resuming and stepping happen on the server's thread ({@link #resume}, {@link #step}), and so does
 * asking to stop ({@link #stop}), which resumes a paused clock so the loop can stop the program. A
 * resume undoes any pause asked before it, and a Stop any pause at all.
 *
 * <p>The page drives a joystick only once it sends input for it, and then with a deadman, as an
 * experiment has: when the page goes quiet for {@link #DEADMAN} (it repeats its input every 200 ms
 * while anything is pressed), the joystick is centered, and when its event stream closes, the
 * joystick is let go. A joystick the page never drove is never touched, so the GUI's keyboard or a
 * real Driver Station still can.
 *
 * <p>The Driver Station's state and the joysticks need no logging of their own: AdvantageKit logs
 * the Driver Station's data as it does on a robot, and replays it. A fault changes the simulated
 * hardware, whose readings the robot's IO layer logs; Pitwall logs which faults are on, to say why.
 */
public final class Simulator {
  /** How many joysticks the page can drive: the driver's and the operator's, 0 and 1. */
  public static final int JOYSTICKS = 2;

  /** How long the page may go quiet before a joystick it drives is centered. */
  public static final Duration DEADMAN = Duration.ofMillis(500);

  /**
   * The input a page last sent for a joystick, and when it was heard.
   *
   * @param input null when the page let it go
   * @param stream the page's event stream, whose closing lets it go
   * @param version which request this is, so the loop takes each once
   */
  private record Heard(
      @Nullable JoystickInput input, @Nullable String stream, long heardNanos, long version) {}

  private record Request(Supplier<Outcome> apply, CompletableFuture<Outcome> outcome) {}

  private final SimulatorHal hal;
  private final LongSupplier clock;
  private final Components components;
  private final Runnable stop;
  private final Consumer<List<String>> logFaults;
  private final ConcurrentLinkedQueue<Request> queue = new ConcurrentLinkedQueue<>();
  private final AtomicReferenceArray<@Nullable Heard> heard = new AtomicReferenceArray<>(JOYSTICKS);
  private final AtomicLong versions = new AtomicLong();

  /** Guards the clock: pausing, resuming, stepping, and asking to stop. */
  private final Object timing = new Object();

  /** Asked to stop: the next loop ends the program. Set and read under {@link #timing}. */
  private boolean stopAsked;

  /** How many times the clock was resumed: a pause asked before the latest resume is dropped. */
  private long resumes;

  /** The view a request asked for, shared by every request until the loop builds it. */
  private @Nullable CompletableFuture<Map<String, Map<String, Object>>> viewAsked;

  private volatile Map<String, Map<String, Object>> lastView = Map.of();

  // The loop's own state.
  private final long[] takenVersion = new long[JOYSTICKS];
  private final @Nullable JoystickInput[] written = new JoystickInput[JOYSTICKS];
  private final @Nullable String[] writtenFor = new String[JOYSTICKS];
  private final boolean[] zeroed = new boolean[JOYSTICKS];
  private boolean countingMatchTime;
  private long lastLoopNanos;

  /** A pause applied this loop: the resumes counted when it was asked; -1 for none. */
  private long pauseAsked = -1;

  private List<String> loggedFaults = List.of();

  /**
   * @param hal WPILib's simulation, or a test's stand-in
   * @param clock the robot's clock in nanoseconds ({@code RobotController::getTime}): the deadman
   *     and the match time run on it, so they stop while the simulator is paused
   * @param components the simulated robot's components that can fail
   * @param stop ends the program: the page's Stop
   * @param logFaults told the lasting faults on, whenever they change, for the log
   */
  public Simulator(
      SimulatorHal hal,
      LongSupplier clock,
      Components components,
      Runnable stop,
      Consumer<List<String>> logFaults) {
    this.hal = hal;
    this.clock = clock;
    this.components = components;
    this.stop = stop;
    this.logFaults = logFaults;
    lastLoopNanos = clock.getAsLong();
  }

  /**
   * Whether this program runs in WPILib's simulation, asked of the HAL itself. Anything else, a
   * robot or a runtime not yet known, counts as a robot: the controls are never made there.
   */
  public static boolean inWpilibSimulation() {
    return WpilibHal.simulating();
  }

  /**
   * The simulator's controls on WPILib's own simulation.
   *
   * @throws IllegalStateException anywhere but in WPILib's simulation
   */
  public static Simulator wpilib(
      LongSupplier clock, Components components, Runnable stop, Consumer<List<String>> logFaults) {
    return new Simulator(new WpilibHal(), clock, components, stop, logFaults);
  }

  /**
   * The simulated robot's components that can fail, and its scenarios: the robot's simulation
   * declares them here.
   */
  public Components components() {
    return components;
  }

  /**
   * Asks the robot loop to change the Driver Station's state, such as {@code state ->
   * state.withEnabled(true)}. Safe from any thread.
   */
  public CompletableFuture<Outcome> requestDriverStation(UnaryOperator<DriverStationState> change) {
    return enqueue(
        () -> {
          DriverStationState now = hal.driverStation();
          DriverStationState changed = change.apply(now);
          if (!changed.equals(now)) {
            hal.driverStation(changed);
            hal.announce();
            countingMatchTime = changed.matchTime() >= 0;
          }
          return Outcome.applied();
        });
  }

  /**
   * Asks the robot loop to pause the clock once it has applied everything asked before. The loop
   * pauses it rather than this thread, so the loop's last patch tells every page. A resume or a
   * Stop before the loop gets to it drops it. Safe from any thread.
   *
   * @return done at once when the clock is paused already: no loop would come to answer it
   */
  public CompletableFuture<Outcome> requestPause() {
    synchronized (timing) {
      if (hal.isPaused()) {
        return CompletableFuture.completedFuture(Outcome.applied());
      }
      long asked = resumes;
      return enqueue(
          () -> {
            synchronized (timing) {
              if (asked != resumes) {
                return new Outcome(Outcome.Result.LOCKED, "Resumed since it was asked: not paused");
              }
            }
            pauseAsked = asked;
            return Outcome.applied();
          });
    }
  }

  /**
   * Runs the clock again, now, on the calling thread: a paused clock runs no loop to do it. Any
   * pause asked before this is dropped, so the clock stays running. Safe from any thread.
   *
   * @return done once the loop has run again
   */
  public CompletableFuture<Outcome> resume() {
    synchronized (timing) {
      resumes++;
      hal.resume();
      return enqueue(Outcome::applied);
    }
  }

  /**
   * Runs one loop of a paused simulator, now, on the calling thread; the loop applies whatever was
   * asked while it was paused. Safe from any thread.
   *
   * @return done once that loop has run; locked when the clock isn't paused
   */
  public CompletableFuture<Outcome> step() {
    synchronized (timing) {
      if (!hal.isPaused()) {
        return CompletableFuture.completedFuture(
            new Outcome(
                Outcome.Result.LOCKED, "Step runs one loop of a paused simulator: pause it"));
      }
      CompletableFuture<Outcome> ran = enqueue(Outcome::applied);
      hal.step();
      return ran;
    }
  }

  /** Whether the simulator's clock is paused. Safe from any thread. */
  public boolean isPaused() {
    return hal.isPaused();
  }

  /**
   * Asks the robot loop to end the program, resuming a paused clock so a loop runs to do it; no
   * pause after this takes. Safe from any thread.
   */
  public void stop() {
    synchronized (timing) {
      stopAsked = true;
      if (hal.isPaused()) {
        hal.resume();
      }
    }
  }

  /**
   * Asks the robot loop to apply a fault. Safe from any thread.
   *
   * @return refused, saying why, when there's no such fault, a value is out of range, or the
   *     fault's own code refused
   */
  public CompletableFuture<Outcome> requestFault(Setting setting) {
    return enqueue(() -> outcome(components.apply(setting)));
  }

  /** Asks the robot loop to undo a lasting fault. Safe from any thread. */
  public CompletableFuture<Outcome> requestUndo(String component, String fault) {
    return enqueue(() -> outcome(components.undo(component, fault)));
  }

  /** Asks the robot loop to undo every lasting fault: the robot as built. Safe from any thread. */
  public CompletableFuture<Outcome> requestUndoAll() {
    return enqueue(() -> outcome(components.undoAll()));
  }

  /** Asks the robot loop to apply a scenario's faults. Safe from any thread. */
  public CompletableFuture<Outcome> requestScenario(String name) {
    return enqueue(() -> outcome(components.applyScenario(name)));
  }

  /**
   * The page's input for a joystick: the robot loop takes the latest each loop. Safe from any
   * thread.
   *
   * <p>A joystick is driven by the page that last sent it input. Only that page can let it go: from
   * any other, null is ignored, so a page that stops driving never lets go of a joystick another
   * page drives now.
   *
   * @param port 0 to {@link #JOYSTICKS} - 1
   * @param input null to let the joystick go: centered, and no longer the page's
   * @param stream the page's event stream; its closing lets the joystick go
   */
  public void joystick(int port, @Nullable JoystickInput input, @Nullable String stream) {
    Objects.checkIndex(port, JOYSTICKS);
    while (true) {
      Heard last = heard.get(port);
      if (input == null
          && last != null
          && last.input() != null
          && !Objects.equals(last.stream(), stream)) {
        return; // another page's: not this one's to let go
      }
      if (heard.compareAndSet(
          port, last, new Heard(input, stream, clock.getAsLong(), versions.incrementAndGet()))) {
        return;
      }
    }
  }

  /**
   * A page's event stream closed: the joysticks it drove are let go. Safe from any thread.
   *
   * @param stream the stream's id
   */
  public void lost(String stream) {
    for (int port = 0; port < JOYSTICKS; port++) {
      Heard last = heard.get(port);
      if (last != null && last.input() != null && stream.equals(last.stream())) {
        heard.compareAndSet(
            port, last, new Heard(null, stream, last.heardNanos(), versions.incrementAndGet()));
      }
    }
  }

  /**
   * Applies what the page asked, in order, then the joysticks' input and their deadman, and counts
   * the match time down. Call it first thing in the robot loop. The robot program sees the Driver
   * Station's changes from the next loop on, as it sees a real Driver Station's.
   */
  public void periodic() {
    boolean stopping;
    synchronized (timing) {
      stopping = stopAsked;
      stopAsked = false;
    }
    if (stopping) {
      stop.run();
      return;
    }
    long now = clock.getAsLong();
    List<Request> requests = new ArrayList<>();
    for (Request request = queue.poll(); request != null; request = queue.poll()) {
      requests.add(request);
    }
    CompletableFuture<Map<String, Map<String, Object>>> asked = takeViewAsked();
    List<Outcome> outcomes = new ArrayList<>();
    try {
      boolean announce = joysticks(now);
      for (Request request : requests) {
        try {
          outcomes.add(request.apply().get());
        } catch (RuntimeException e) {
          outcomes.add(new Outcome(Outcome.Result.REFUSED, String.valueOf(e.getMessage())));
        }
      }
      announce |= countMatchTime(now);
      if (announce) {
        hal.announce();
      }
      lastLoopNanos = now;
      List<String> on = components.on();
      if (!on.equals(loggedFaults)) {
        loggedFaults = on;
        logFaults.accept(on);
      }
      pauseIfAsked();
      // A request's answer, and a request for the view, see the simulator as this loop left it.
      if (asked != null || !requests.isEmpty()) {
        Map<String, Map<String, Object>> view = view();
        if (asked != null) {
          asked.complete(view);
        }
      }
    } finally {
      // Every request is answered, even when the loop failed partway: none waits for nothing.
      for (int i = 0; i < requests.size(); i++) {
        requests
            .get(i)
            .outcome()
            .complete(
                i < outcomes.size()
                    ? outcomes.get(i)
                    : new Outcome(
                        Outcome.Result.REFUSED, "The simulator's loop failed before it got to it"));
      }
      if (asked != null && !asked.isDone()) {
        asked.completeExceptionally(new IllegalStateException("The simulator's loop failed"));
      }
    }
  }

  /** Pauses the clock, if asked this loop, unless a resume or a Stop came since. */
  private void pauseIfAsked() {
    long pause = pauseAsked;
    pauseAsked = -1;
    if (pause < 0) {
      return;
    }
    synchronized (timing) {
      if (pause == resumes && !stopAsked) {
        hal.pause();
      }
    }
  }

  /**
   * The simulator as the page shows it, section by section: {@code simulator} (the Driver Station,
   * the clock, and which joysticks the page drives), {@code components}, and {@code scenarios}.
   * Call it from the robot loop.
   */
  public Map<String, Map<String, Object>> view() {
    DriverStationState station = hal.driverStation();
    Map<String, Object> state = new LinkedHashMap<>();
    state.put("enabled", station.enabled());
    state.put("mode", station.mode().name());
    state.put("allianceStation", station.allianceStation().name());
    state.put("fmsAttached", station.fmsAttached());
    state.put("dsAttached", station.dsAttached());
    // Whole seconds, as a Driver Station shows it, so a page hears of it once a second.
    state.put("matchTime", station.matchTime() < 0 ? -1.0 : Math.ceil(station.matchTime()));
    state.put("paused", hal.isPaused());
    Map<String, Object> joysticks = new LinkedHashMap<>();
    for (int port = 0; port < JOYSTICKS; port++) {
      Map<String, Object> joystick = new LinkedHashMap<>();
      joystick.put("fromPage", written[port] != null);
      joystick.put("centeredByDeadman", zeroed[port] && written[port] != null);
      // Which page drives it, by its event stream, so a page can tell it isn't the only one.
      joystick.put("stream", Objects.requireNonNullElse(writtenFor[port], ""));
      joysticks.put(Integer.toString(port), joystick);
    }
    state.put("joysticks", joysticks);
    Map<String, Map<String, Object>> view = new LinkedHashMap<>();
    view.put("simulator", state);
    view.put("components", components.view());
    view.put("scenarios", components.scenarioView());
    lastView = view;
    return view;
  }

  /**
   * Asks the robot loop for the simulator as the page shows it ({@link #view}), for a request that
   * isn't following the event stream. A paused clock runs no loop to answer: then see {@link
   * #lastView}. Safe from any thread.
   */
  public synchronized CompletableFuture<Map<String, Map<String, Object>>> requestView() {
    CompletableFuture<Map<String, Map<String, Object>>> asked = viewAsked;
    if (asked == null) {
      asked = new CompletableFuture<>();
      viewAsked = asked;
    }
    return asked;
  }

  private synchronized @Nullable CompletableFuture<Map<String, Map<String, Object>>>
      takeViewAsked() {
    CompletableFuture<Map<String, Map<String, Object>>> asked = viewAsked;
    viewAsked = null;
    return asked;
  }

  /**
   * The simulator as the loop last showed it, with whether the clock is paused now: a paused clock
   * runs no loop to say so. Safe from any thread.
   */
  public Map<String, Map<String, Object>> lastView() {
    Map<String, Map<String, Object>> view = new LinkedHashMap<>(lastView);
    Map<String, Object> state = new LinkedHashMap<>(view.getOrDefault("simulator", Map.of()));
    state.put("paused", hal.isPaused());
    view.put("simulator", state);
    return view;
  }

  /**
   * Takes each joystick's latest input from the page, and centers one whose page went quiet.
   *
   * @return whether any joystick changed
   */
  private boolean joysticks(long now) {
    boolean changed = false;
    for (int port = 0; port < JOYSTICKS; port++) {
      Heard last = heard.get(port);
      if (last == null) {
        continue; // never driven from the page: left to whatever else drives it
      }
      JoystickInput input = last.input();
      if (last.version() != takenVersion[port]) {
        takenVersion[port] = last.version();
        JoystickInput next = input != null ? input : JoystickInput.CENTERED;
        if (!next.equals(written[port]) || zeroed[port]) {
          hal.joystick(port, next);
          changed = true;
        }
        written[port] = input;
        writtenFor[port] = input != null ? last.stream() : null;
        zeroed[port] = false;
      }
      JoystickInput driving = written[port];
      if (driving != null
          && !zeroed[port]
          && !driving.isCentered()
          && now - last.heardNanos() > DEADMAN.toNanos()) {
        hal.joystick(port, JoystickInput.CENTERED);
        zeroed[port] = true;
        changed = true;
      }
    }
    return changed;
  }

  /**
   * Counts the match time down while the robot is enabled, as the GUI does, once the page has set
   * it.
   *
   * @return whether it changed
   */
  private boolean countMatchTime(long now) {
    if (!countingMatchTime) {
      return false;
    }
    DriverStationState station = hal.driverStation();
    if (!station.enabled() || station.matchTime() <= 0) {
      return false;
    }
    hal.matchTime(Math.max(0, station.matchTime() - (now - lastLoopNanos) / 1e9));
    return true;
  }

  private CompletableFuture<Outcome> enqueue(Supplier<Outcome> apply) {
    CompletableFuture<Outcome> outcome = new CompletableFuture<>();
    queue.add(new Request(apply, outcome));
    return outcome;
  }

  private static Outcome outcome(@Nullable String refused) {
    return refused == null ? Outcome.applied() : new Outcome(Outcome.Result.REFUSED, refused);
  }
}
