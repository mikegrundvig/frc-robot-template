package frc.robot.subsystems.mechanisms;

import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Time;

/**
 * A named condition that says why it's false: a v3 {@link Trigger}, so it binds ({@code
 * operator.rightBumper().and(readyToShoot).onTrue(...)}) and awaits ({@code
 * co.waitUntil(readyToShoot)}), with a name and named terms. Every joint's checks are gates already
 * ({@code shooter.isAtGoal()} is "shooter at goal"), and code makes its own:
 *
 * <pre>{@code
 * Gate aimed = Gate.within(scheduler, "aimed", () -> drive.headingError().getDegrees(), 2.0, 4.0);
 * Gate readyToShoot = Gate.all("ready to shoot", shooter.isAtGoal(), aimed)
 *     .debounce(Milliseconds.of(80), Debouncer.DebounceType.FALLING); // rides out a flywheel dip
 * }</pre>
 *
 * <p>It's worked out once a loop, before the scheduler's bindings and commands read it, and logged
 * then: {@code Gates/ready to shoot = false}, and {@code Gates/ready to shoot/Blocking = [aimed]},
 * the terms that are false. Terms go one level deep: a gate of gates, each a lambda, never an
 * expression language. A gate made inside a command lives as long as the command does, as v3's
 * triggers do.
 */
public final class Gate extends Trigger {
  /** What a gate works out each loop. */
  interface Logic {
    /** Whether it holds now; when not, the names of the terms that don't, added to {@code why}. */
    boolean holds(List<String> why);
  }

  /** The gate's value this loop, which v3's trigger reads. */
  private static final class Value implements BooleanSupplier {
    boolean holds;

    @Override
    public boolean getAsBoolean() {
      return holds;
    }
  }

  private final Scheduler scheduler;
  private final String name;
  private final Logic logic;
  private final Value value;
  private final List<String> blocking = new ArrayList<>();
  private final String key;
  private final String blockingKey;

  /** Whether it logs: a gate debounced into another logs as that one, under the same name. */
  private boolean logged = true;

  private String[] shown = new String[0];

  Gate(Scheduler scheduler, String name, Logic logic) {
    this(scheduler, name, logic, new Value());
  }

  @SuppressWarnings("this-escape")
  private Gate(Scheduler scheduler, String name, Logic logic, Value value) {
    super(scheduler, value);
    this.scheduler = scheduler;
    this.name = name;
    this.logic = logic;
    this.value = value;
    key = "Gates/" + name;
    blockingKey = key + "/Blocking";
    update();
    scheduler.addPeriodic(this::update);
  }

  /** A gate on any condition: {@code Gate.of(scheduler, "in range", () -> distance() <= 5)}. */
  public static Gate of(Scheduler scheduler, String name, BooleanSupplier condition) {
    return new Gate(scheduler, name, why -> condition.getAsBoolean());
  }

  /**
   * A gate on how far something is off, either way: it becomes true within {@code enter} of zero,
   * and stays true until it's more than {@code stay} off, so an error that wavers at the edge
   * doesn't flicker. {@code Gate.within(scheduler, "aimed", () -> headingError, 2.0, 4.0)}.
   *
   * @throws IllegalArgumentException for an {@code enter} that isn't positive, or a {@code stay}
   *     below it
   */
  public static Gate within(
      Scheduler scheduler, String name, DoubleSupplier error, double enter, double stay) {
    if (!(enter > 0 && stay >= enter)) {
      throw new IllegalArgumentException(
          name
              + ": enter within a positive band, and stay within at least it, not "
              + enter
              + " and "
              + stay);
    }
    boolean[] inside = {false};
    return new Gate(
        scheduler,
        name,
        why -> {
          double off = Math.abs(error.getAsDouble());
          inside[0] = inside[0] ? off <= stay : off <= enter;
          return inside[0];
        });
  }

  /**
   * A gate true when all its terms are, blocked by the ones that aren't: {@code Gate.all("ready to
   * shoot", shooter.isAtGoal(), aimed)}.
   *
   * @throws IllegalArgumentException with no terms, or terms of different schedulers
   */
  public static Gate all(String name, Gate... terms) {
    if (terms.length == 0) {
      throw new IllegalArgumentException(name + " has no terms");
    }
    Scheduler scheduler = terms[0].scheduler;
    for (Gate term : terms) {
      if (term.scheduler != scheduler) {
        throw new IllegalArgumentException(name + "'s terms run on different schedulers");
      }
    }
    List<Gate> all = List.of(terms);
    return new Gate(
        scheduler,
        name,
        why -> {
          boolean holds = true;
          for (Gate term : all) {
            if (!term.value.holds) {
              why.add(term.name);
              holds = false;
            }
          }
          return holds;
        });
  }

  /**
   * The same gate, debounced: on the falling edge, it stays true until it has been false for the
   * time, riding out a flywheel's dip as a shot leaves; on the rising edge, it becomes true only
   * once it has been for the time. It's logged under the same name, in place of this one. It holds
   * nothing until this gate first has.
   */
  @Override
  public Gate debounce(Time time, Debouncer.DebounceType type) {
    Debouncer debouncer = new Debouncer(time.in(Seconds), type);
    Gate inner = this;
    inner.logged = false;
    boolean[] seen = {false};
    return new Gate(
        scheduler,
        name,
        why -> {
          boolean now = inner.value.holds;
          seen[0] |= now;
          boolean debounced = debouncer.calculate(now); // every loop, so its timer keeps time
          boolean holds = seen[0] && debounced;
          if (!holds) {
            if (now) {
              why.add(name + " settling");
            } else {
              why.addAll(inner.blocking);
            }
          }
          return holds;
        });
  }

  /** Its name, as logged: "ready to shoot". */
  public String name() {
    return name;
  }

  /** Whether it holds this loop. */
  @Override
  public boolean getAsBoolean() {
    return value.holds;
  }

  /** The terms that keep it false this loop: empty while it holds, or for a gate without terms. */
  public List<String> blocking() {
    return List.copyOf(blocking);
  }

  /** Why it's false, as a reason reads: "ready to shoot is false: aimed"; or "ready to shoot". */
  String why() {
    return blocking.isEmpty()
        ? name + " is false"
        : name + " is false: " + String.join(", ", blocking);
  }

  @Override
  public String toString() {
    return name;
  }

  /** Works it out for this loop, and logs it. */
  private void update() {
    blocking.clear();
    value.holds = logic.holds(blocking);
    if (logged) {
      Logger.recordOutput(key, value.holds);
      if (shown.length != blocking.size() || !List.of(shown).equals(blocking)) {
        shown = blocking.toArray(String[]::new);
      }
      Logger.recordOutput(blockingKey, shown);
    }
  }
}
