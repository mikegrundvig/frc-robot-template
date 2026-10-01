package com.michaelgrundvig.frc.pitwall;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;

/**
 * Named routines started from the page, such as a pit check or homing a mechanism, each with its
 * state shown live. An action runs only while the robot is enabled in utility mode and the field
 * system isn't connected, so the Driver Station's enable stays the safety switch; leaving utility
 * mode or disabling cancels whatever is running.
 *
 * <p>Add one with a line: {@code actions.add("Home arm", arm::home)} for a Commands v3 command
 * (made fresh each run), or any {@link Routine}.
 *
 * <p>Like tuning, only the robot loop starts or cancels anything: requests from the page wait in a
 * queue, and {@link #periodic} applies and logs them, so a replay reproduces them.
 */
public final class Actions {
  /** What an action runs. */
  public interface Routine {
    /**
     * Starts it.
     *
     * @return why it couldn't start, or null when it did
     */
    @Nullable String start();

    /** Whether it is still going. */
    boolean isRunning();

    /** Stops it early. */
    void cancel();
  }

  /** Where an action stands. */
  public enum Phase {
    /** Hasn't run yet. */
    READY,
    RUNNING,
    /** Finished, or was stopped by something other than the page. */
    ENDED,
    CANCELED,
    /** Its last start was refused. */
    REFUSED
  }

  /**
   * One action, as the page shows it.
   *
   * @param name what it's called on the page
   * @param phase where it stands
   * @param detail a sentence about it: why it was refused or canceled
   */
  public record State(String name, Phase phase, String detail) {}

  /**
   * Whether actions may run this loop.
   *
   * @param enabled whether the robot is enabled
   * @param utility whether the Driver Station is in utility mode
   * @param fieldConnected whether the field system is connected
   */
  public record Gate(boolean enabled, boolean utility, boolean fieldConnected) {
    /** Why actions can't run now, or null when they can. */
    @Nullable String whyNot() {
      if (fieldConnected) {
        return "the field system is connected";
      }
      if (!enabled || !utility) {
        return "actions run only with the robot enabled in utility mode";
      }
      return null;
    }
  }

  private static final class Entry {
    final Routine routine;
    Phase phase = Phase.READY;
    String detail = "";

    Entry(Routine routine) {
      this.routine = routine;
    }

    void set(Phase phase, String detail) {
      this.phase = phase;
      this.detail = detail;
    }
  }

  private record Request(ActionInputs.Kind kind, String name, CompletableFuture<Outcome> outcome) {}

  private final Map<String, Entry> actions = new LinkedHashMap<>();
  private final ConcurrentLinkedQueue<Request> queue = new ConcurrentLinkedQueue<>();
  private final ActionInputs inputs = new ActionInputs();
  private final Consumer<ActionInputs> logInputs;
  private volatile Set<String> names = Set.of();
  private volatile List<State> view = List.of();

  /** Why actions can't start now, as of the last loop; null while they can. */
  private volatile @Nullable String locked = new Gate(false, false, false).whyNot();

  /**
   * @param logInputs logs the requests one loop applies, or in a replay replaces them with the
   *     logged ones
   */
  public Actions(Consumer<ActionInputs> logInputs) {
    this.logInputs = logInputs;
  }

  /**
   * Adds an action that runs a Commands v3 command on the default scheduler.
   *
   * @param command makes the command, fresh each time the action starts
   * @return this, to add the next action
   */
  public Actions add(String name, Supplier<Command> command) {
    return add(name, command(Scheduler::getDefault, command));
  }

  /**
   * Adds an action.
   *
   * @return this, to add the next action
   * @throws IllegalArgumentException if the name is blank or already used
   */
  public Actions add(String name, Routine routine) {
    if (name.isBlank()) {
      throw new IllegalArgumentException("an action needs a name");
    }
    if (actions.putIfAbsent(name, new Entry(routine)) != null) {
      throw new IllegalArgumentException("two actions are named " + name);
    }
    names = Set.copyOf(actions.keySet());
    view = snapshot();
    return this;
  }

  /** A routine that schedules a Commands v3 command, made fresh each time it starts. */
  public static Routine command(Supplier<Scheduler> scheduler, Supplier<Command> make) {
    return new Routine() {
      private @Nullable Command running;

      @Override
      public @Nullable String start() {
        Command command = make.get();
        Scheduler.ScheduleResult result = scheduler.get().schedule(command);
        if (!result.successful()) {
          return "a higher-priority command is using what it needs";
        }
        running = command;
        return null;
      }

      @Override
      public boolean isRunning() {
        Command command = running;
        return command != null && scheduler.get().isScheduledOrRunning(command);
      }

      @Override
      public void cancel() {
        Command command = running;
        if (command != null) {
          scheduler.get().cancel(command);
        }
      }
    };
  }

  /** Every action, as of the last loop, in the order added. Safe to call from any thread. */
  public List<State> view() {
    return view;
  }

  /**
   * Why an action can't start now, as of the last loop, such as "actions run only with the robot
   * enabled in utility mode"; null while one can. Safe to call from any thread.
   */
  public @Nullable String whyLocked() {
    return locked;
  }

  /** Whether an action has this name. Safe to call from any thread. */
  public boolean has(String name) {
    return names.contains(name);
  }

  /** Asks the robot loop to start an action. Safe to call from any thread. */
  public CompletableFuture<Outcome> requestStart(String name) {
    return enqueue(ActionInputs.Kind.START, name);
  }

  /** Asks the robot loop to cancel an action, if it's running. Safe to call from any thread. */
  public CompletableFuture<Outcome> requestCancel(String name) {
    return enqueue(ActionInputs.Kind.CANCEL, name);
  }

  /**
   * Applies the requests queued since the last loop, and notices actions that ended. Call it once
   * per loop, before the command scheduler runs.
   */
  public void periodic(Gate gate) {
    List<CompletableFuture<Outcome>> outcomes = drainInto(inputs);
    logInputs.accept(inputs); // in replay, this replaces them with the logged ones

    for (Entry entry : actions.values()) {
      if (entry.phase == Phase.RUNNING) {
        String broken = null;
        boolean running;
        try {
          running = entry.routine.isRunning();
        } catch (RuntimeException e) {
          running = false;
          broken = "Ended: checking whether it runs failed: " + describe(e);
        }
        if (!running) {
          entry.set(Phase.ENDED, broken != null ? broken : "Ended");
        }
      }
    }
    String whyNot = gate.whyNot();
    locked = whyNot;
    if (whyNot != null) {
      for (Entry entry : actions.values()) {
        if (entry.phase == Phase.RUNNING) {
          entry.set(Phase.CANCELED, "Canceled: " + whyNot + cancel(entry));
        }
      }
    }
    List<Outcome> results = new ArrayList<>();
    for (int i = 0; i < inputs.kinds.length; i++) {
      results.add(apply(ActionInputs.Kind.valueOf(inputs.kinds[i]), inputs.names[i], whyNot));
    }
    view = snapshot();
    // Only now, so whoever is waiting sees the state their request left. In replay, no one is.
    for (int i = 0; i < outcomes.size() && i < results.size(); i++) {
      outcomes.get(i).complete(results.get(i));
    }
  }

  private Outcome apply(ActionInputs.Kind kind, String name, @Nullable String whyNot) {
    Entry entry = actions.get(name);
    if (entry == null) {
      return new Outcome(Outcome.Result.REFUSED, "There is no action named " + name);
    }
    if (kind == ActionInputs.Kind.CANCEL) {
      if (entry.phase == Phase.RUNNING) {
        entry.set(Phase.CANCELED, "Canceled from the page" + cancel(entry));
      }
      return Outcome.applied();
    }
    if (whyNot != null) {
      return new Outcome(Outcome.Result.LOCKED, "Actions are locked: " + whyNot);
    }
    if (entry.phase == Phase.RUNNING) {
      return new Outcome(Outcome.Result.REFUSED, name + " is already running");
    }
    String failed;
    try {
      failed = entry.routine.start();
    } catch (RuntimeException e) {
      failed = describe(e);
    }
    if (failed != null) {
      entry.set(Phase.REFUSED, "Refused: " + failed);
      return new Outcome(Outcome.Result.REFUSED, failed);
    }
    entry.set(Phase.RUNNING, "Running");
    return Outcome.applied();
  }

  /**
   * Cancels a routine: a team's routine that throws doesn't stop the robot loop.
   *
   * @return nothing, or why canceling failed, to add to the action's state
   */
  private static String cancel(Entry entry) {
    try {
      entry.routine.cancel();
      return "";
    } catch (RuntimeException e) {
      return " (canceling it failed: " + describe(e) + ")";
    }
  }

  private static String describe(RuntimeException e) {
    String why = e.getMessage();
    return why != null ? why : e.toString();
  }

  private CompletableFuture<Outcome> enqueue(ActionInputs.Kind kind, String name) {
    CompletableFuture<Outcome> outcome = new CompletableFuture<>();
    queue.add(new Request(kind, name, outcome));
    return outcome;
  }

  /** Moves the queued requests into the inputs, and returns their futures in request order. */
  private List<CompletableFuture<Outcome>> drainInto(ActionInputs inputs) {
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
    return requests.stream().map(Request::outcome).toList();
  }

  private List<State> snapshot() {
    List<State> states = new ArrayList<>(actions.size());
    for (Map.Entry<String, Entry> action : actions.entrySet()) {
      Entry entry = action.getValue();
      states.add(new State(action.getKey(), entry.phase, entry.detail));
    }
    return List.copyOf(states);
  }
}
