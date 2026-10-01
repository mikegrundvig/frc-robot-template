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
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import org.jspecify.annotations.Nullable;

/**
 * The simulator's controls, for the page: what the WPILib simulator GUI does for the robot program
 * (the Driver Station, joysticks, pausing and stepping the clock, stopping). It exists only in the
 * simulator: on a robot, Pitwall has none, so nothing can ever write the Driver Station's state or
 * a joystick there.
 *
 * <p>Like the rest of Pitwall, only the robot loop changes anything, with three exceptions the
 * clock forces. Requests from the web server's threads wait in a queue, and {@link #periodic},
 * first thing in the loop, applies them, so the rest of the loop and AdvantageKit's log see the
 * same Driver Station, and a replay sees it as the robot did. A joystick's input is handed over the
 * same way: the server keeps the latest for each joystick, and the loop takes it. But a paused
 * clock runs no loops, so resuming and stepping happen on the server's thread ({@link #resume},
 * {@link #step}), and so does asking to stop ({@link #stop}), which resumes a paused clock so the
 * loop can stop the program.
 *
 * <p>The page drives a joystick only once it sends input for it, and then with a deadman, as an
 * experiment has: when the page goes quiet for {@link #DEADMAN} (it repeats its input every 200 ms
 * while anything is pressed), the joystick is centered, and when its event stream closes, the
 * joystick is let go. A joystick the page never drove is never touched, so the GUI's keyboard or a
 * real Driver Station still can.
 *
 * <p>The Driver Station's state and the joysticks need no logging of their own: AdvantageKit logs
 * the Driver Station's data as it does on a robot, and replays it.
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
  private final Runnable stop;
  private final ConcurrentLinkedQueue<Request> queue = new ConcurrentLinkedQueue<>();
  private final AtomicReferenceArray<@Nullable Heard> heard = new AtomicReferenceArray<>(JOYSTICKS);
  private final AtomicLong versions = new AtomicLong();
  private volatile boolean stopAsked;

  /** The view a request asked for, shared by every request until the loop builds it. */
  private @Nullable CompletableFuture<Map<String, Map<String, Object>>> viewAsked;

  private volatile Map<String, Map<String, Object>> lastView = Map.of();

  // The loop's own state.
  private final long[] takenVersion = new long[JOYSTICKS];
  private final @Nullable JoystickInput[] written = new JoystickInput[JOYSTICKS];
  private final boolean[] zeroed = new boolean[JOYSTICKS];
  private boolean countingMatchTime;
  private long lastLoopNanos;
  private boolean pauseAsked;
  private final Object timing = new Object();

  /**
   * @param hal WPILib's simulation, or a test's stand-in
   * @param clock the robot's clock in nanoseconds ({@code RobotController::getTime}): the deadman
   *     and the match time run on it, so they stop while the simulator is paused
   * @param stop ends the program: the page's Stop
   */
  public Simulator(SimulatorHal hal, LongSupplier clock, Runnable stop) {
    this.hal = hal;
    this.clock = clock;
    this.stop = stop;
    lastLoopNanos = clock.getAsLong();
  }

  /** The simulator's controls on WPILib's own simulation. */
  public static Simulator wpilib(LongSupplier clock, Runnable stop) {
    return new Simulator(new WpilibHal(), clock, stop);
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
   * pauses it rather than this thread, so the loop's last patch tells every page. Safe from any
   * thread.
   */
  public CompletableFuture<Outcome> requestPause() {
    return enqueue(
        () -> {
          pauseAsked = true;
          return Outcome.applied();
        });
  }

  /**
   * Runs the clock again, now, on the calling thread: a paused clock runs no loop to do it. Safe
   * from any thread.
   *
   * @return done once the loop has run again
   */
  public CompletableFuture<Outcome> resume() {
    synchronized (timing) {
      hal.resume();
    }
    return enqueue(Outcome::applied);
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
   * Asks the robot loop to end the program, resuming a paused clock so a loop runs to do it. Safe
   * from any thread.
   */
  public void stop() {
    stopAsked = true;
    synchronized (timing) {
      if (hal.isPaused()) {
        hal.resume();
      }
    }
  }

  /**
   * The page's input for a joystick: the robot loop takes the latest each loop. Safe from any
   * thread.
   *
   * @param port 0 to {@link #JOYSTICKS} - 1
   * @param input null to let the joystick go: centered, and no longer the page's
   * @param stream the page's event stream; its closing lets the joystick go
   */
  public void joystick(int port, @Nullable JoystickInput input, @Nullable String stream) {
    Objects.checkIndex(port, JOYSTICKS);
    heard.set(port, new Heard(input, stream, clock.getAsLong(), versions.incrementAndGet()));
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
   * the match time down. Call it first thing in the robot loop, before anything reads the Driver
   * Station, so the whole loop sees the same.
   */
  public void periodic() {
    if (stopAsked) {
      stopAsked = false;
      stop.run();
      return;
    }
    long now = clock.getAsLong();
    boolean announce = joysticks(now);
    List<Request> requests = new ArrayList<>();
    for (Request request = queue.poll(); request != null; request = queue.poll()) {
      requests.add(request);
    }
    List<Outcome> outcomes = new ArrayList<>();
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
    if (pauseAsked) {
      pauseAsked = false;
      synchronized (timing) {
        hal.pause();
      }
    }
    // A request's answer, and a request for the view, see the simulator as this loop left it.
    CompletableFuture<Map<String, Map<String, Object>>> asked = takeViewAsked();
    if (asked != null || !requests.isEmpty()) {
      Map<String, Map<String, Object>> view = view();
      if (asked != null) {
        asked.complete(view);
      }
    }
    // Only now, so whoever waits sees the state their request left.
    for (int i = 0; i < requests.size(); i++) {
      requests.get(i).outcome().complete(outcomes.get(i));
    }
  }

  /**
   * The simulator as the page shows it, section by section: {@code simulator} (the Driver Station,
   * the clock, and which joysticks the page drives). Call it from the robot loop.
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
      joysticks.put(Integer.toString(port), joystick);
    }
    state.put("joysticks", joysticks);
    Map<String, Map<String, Object>> view = new LinkedHashMap<>();
    view.put("simulator", state);
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
    double left = Math.max(0, station.matchTime() - (now - lastLoopNanos) / 1e9);
    hal.driverStation(station.withMatchTime(left));
    return true;
  }

  private CompletableFuture<Outcome> enqueue(Supplier<Outcome> apply) {
    CompletableFuture<Outcome> outcome = new CompletableFuture<>();
    queue.add(new Request(apply, outcome));
    return outcome;
  }
}
