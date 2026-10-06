package frc.robot.subsystems.mechanisms;

import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Output;
import frc.robot.behavior.Gate;
import frc.robot.behavior.Outcome;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.Measure;
import org.wpilib.units.measure.Time;

/**
 * A pre-built command on the robot's joints: a v3 {@link Command} that records how it ended, as one
 * of four outcomes, each with a reason it logs under each of its joints ({@code
 * Joints/pivot/Outcome}).
 *
 * <ul>
 *   <li>{@link Outcome#ARRIVED}: inside the joint's band for 40 ms, every joint for a move to a
 *       configuration; for {@code runUntil}, its condition held; for {@code home}, it's homed.
 *   <li>{@link Outcome#TIMED_OUT}: its deadline passed. The reason says where it was, and what
 *       interlock held it.
 *   <li>{@link Outcome#CANCELED}: another command took the joint, its binding released, or its
 *       {@link #onlyWhile} gate fell.
 *   <li>{@link Outcome#REFUSED}: it wouldn't start or can't go on: a target outside the joint's
 *       limits, a joint not answering or faulted, or its {@link #onlyIf} gate false.
 * </ul>
 *
 * <p>A move that ends on its own has a deadline, from the model: twice its profile's time from
 * where it starts, plus half a second, or 3 s for {@code runUntil}; {@link #within} overrides it.
 * One that never ends on its own, {@code run}, {@code follow} or {@code hold}, is for "while held"
 * and for background work. A move on a joint that's homing waits for it, and its deadline starts
 * once it has. Recovery is an {@code if}:
 *
 * <pre>{@code
 * Command.requiring(pivot).executing(co -> {
 *   var deploy = pivot.goTo("out");
 *   co.await(deploy);
 *   if (!deploy.arrived()) {      // its outcome and reason are already logged
 *     co.await(pivot.goTo("in"));
 *   }
 * }).named("Deploy or back off");
 * }</pre>
 *
 * <p>A move can be bound to a trigger and run again and again: each run starts afresh, and its
 * outcome is its latest run's. Its requirements are exactly the joints it moves.
 */
public final class Move implements Command {
  /** What a move does. */
  private enum Kind {
    /** To targets, ending when every joint is there. */
    GO_TO,
    /** At outputs or positions until canceled, or until a condition holds. */
    RUN,
    /** After a target that changes every loop. */
    FOLLOW,
    /** At the joint's goal. */
    HOLD,
    /** Homes the joint by its recipe. */
    HOME
  }

  /** A move's default deadline: this many times its profile's time, and this much more. */
  static final double DEADLINE_FACTOR = 2;

  static final double DEADLINE_SLACK_SECONDS = 0.5;

  /** {@code runUntil}'s default deadline, in seconds. */
  static final double RUN_UNTIL_SECONDS = 3;

  /** One joint's part in a move: a position to go to, or an output to run at. */
  private static final class Leg {
    final JointMechanism joint;
    final @Nullable Output output;
    final double position;
    final @Nullable String name;

    /** When the joint last came inside its band, in nanoseconds: -1 while it's outside. */
    long insideSince = -1;

    Leg(JointMechanism joint, @Nullable Output output, double position, @Nullable String name) {
      this.joint = joint;
      this.output = output;
      this.position = position;
      this.name = name;
    }

    Leg copy() {
      return new Leg(joint, output, position, name);
    }
  }

  /** What a move is, apart from how its latest run went: each option makes a new one. */
  private record Plan(
      Kind kind,
      String name,
      double fraction,
      double within,
      @Nullable BooleanSupplier until,
      @Nullable Supplier<? extends Measure<?>> target,
      @Nullable Gate onlyIf,
      @Nullable Gate onlyWhile,
      @Nullable String refused) {}

  private final Plan plan;
  private final Kind kind;
  private final String name;
  private final Leg[] legs;
  private final Set<Mechanism> requirements;

  // How its latest run went.
  private long startNanos;
  private long deadlineNanos;
  private boolean running;

  /** Whether its latest run got past its start, and has driven its joints. */
  private boolean started;

  private @Nullable Outcome outcome;
  private String reason = "";

  /** When each joint got there in its latest run, in seconds from its start, as it ended. */
  private Map<Joint, Double> arrivals = Map.of();

  /** Where a hold holds: the joint's goal as it starts. */
  private double held;

  private Move(Plan plan, Leg[] legs) {
    this.plan = plan;
    this.kind = plan.kind();
    this.name = plan.name();
    this.legs = legs;
    Set<Mechanism> required = new LinkedHashSet<>();
    for (Leg leg : legs) {
      required.add(leg.joint);
    }
    requirements = Set.copyOf(required);
  }

  private static Move of(Kind kind, String name, Leg... legs) {
    return new Move(new Plan(kind, name, 1, Double.NaN, null, null, null, null, null), legs);
  }

  // ---- Made by the joints ----

  /**
   * @param refused why it can't run, or null: a position of another joint's
   */
  static Move goTo(
      JointMechanism joint, double position, @Nullable String named, @Nullable String refused) {
    String where = named != null ? named : JointSetup.shown(joint.joint(), position);
    return new Move(
        new Plan(
            Kind.GO_TO,
            joint.getName() + " → " + where,
            1,
            Double.NaN,
            null,
            null,
            null,
            null,
            refused),
        new Leg[] {new Leg(joint, null, position, named)});
  }

  /**
   * @param refused why it can't run, or null: an output of another joint's
   */
  static Move run(
      JointMechanism joint,
      Output output,
      @Nullable BooleanSupplier until,
      @Nullable String refused) {
    Plan plan =
        new Plan(
            Kind.RUN,
            joint.getName()
                + ": "
                + output.name()
                + (until == null ? "" : until instanceof Gate gate ? " until " + gate : " until"),
            1,
            Double.NaN,
            until,
            null,
            null,
            null,
            refused);
    return new Move(plan, new Leg[] {new Leg(joint, output, Double.NaN, output.name())});
  }

  static Move follow(JointMechanism joint, Supplier<? extends Measure<?>> target) {
    Plan plan =
        new Plan(
            Kind.FOLLOW,
            joint.getName() + " follows",
            1,
            Double.NaN,
            null,
            target,
            null,
            null,
            null);
    return new Move(plan, new Leg[] {new Leg(joint, null, Double.NaN, null)});
  }

  static Move hold(JointMechanism joint) {
    return of(Kind.HOLD, joint.getName() + " holds", new Leg(joint, null, Double.NaN, null));
  }

  static Move home(JointMechanism joint) {
    return of(Kind.HOME, joint.getName() + " homes", new Leg(joint, null, Double.NaN, null));
  }

  /**
   * A configuration's joints, each to its named position or at its named output: to it, ending when
   * every joint is there; or held at it, until canceled. One that names a joint the template
   * doesn't drive is refused whenever it would start, saying why, rather than when it's made.
   *
   * @param joints each target's joint's mechanism, in the configuration's order; null for a joint
   *     the template doesn't drive
   * @param refused why it can't run, or null
   */
  static Move configuration(
      Configuration configuration,
      List<@Nullable JointMechanism> joints,
      boolean toIt,
      @Nullable String refused) {
    List<Leg> legs = new ArrayList<>();
    for (int i = 0; i < joints.size(); i++) {
      var at = configuration.targets().get(i);
      JointMechanism joint = joints.get(i);
      if (joint != null) {
        legs.add(
            at.isOutput()
                ? new Leg(joint, at.joint().output(at.name()), Double.NaN, at.name())
                : new Leg(joint, null, at.joint().position(at.name()).value(), at.name()));
      }
    }
    Kind kind = toIt ? Kind.GO_TO : Kind.RUN;
    String name = (toIt ? "→ " : "at ") + configuration.name();
    return new Move(
        new Plan(kind, name, 1, Double.NaN, null, null, null, null, refused),
        legs.toArray(Leg[]::new));
  }

  // ---- Its options ----

  /**
   * The same move at a share of each joint's cruise: {@code goTo("out").at(0.9)}. Its default
   * deadline grows to match.
   *
   * @param fraction above 0, at most 1
   * @throws IllegalArgumentException for a fraction outside that, or a move with no profile
   */
  public Move at(double fraction) {
    if (!(fraction > 0 && fraction <= 1)) {
      throw new IllegalArgumentException(
          name + ": a share of the cruise is above 0 and at most 1, not " + fraction);
    }
    if (kind != Kind.GO_TO) {
      throw new IllegalArgumentException(name + " isn't a move to a place: it has no profile");
    }
    return new Move(
        new Plan(
            kind,
            name + String.format(Locale.ROOT, " at %.0f%%", fraction * 100),
            fraction,
            plan.within(),
            plan.until(),
            plan.target(),
            plan.onlyIf(),
            plan.onlyWhile(),
            plan.refused()),
        copies());
  }

  /**
   * The same move with a deadline of its own, tighter or looser than the model's: {@code
   * goTo("out").within(Seconds.of(1))}.
   *
   * @throws IllegalArgumentException for a move that never ends on its own, which v3's {@code
   *     withTimeout} cancels instead
   */
  public Move within(Time deadline) {
    if (kind != Kind.GO_TO && plan.until() == null) {
      throw new IllegalArgumentException(
          name + " never ends on its own, so it has no deadline: use withTimeout");
    }
    return new Move(
        new Plan(
            kind,
            name,
            plan.fraction(),
            deadline.in(Seconds),
            plan.until(),
            plan.target(),
            plan.onlyIf(),
            plan.onlyWhile(),
            plan.refused()),
        copies());
  }

  /**
   * The same move, started only if a gate holds as it would start; otherwise it's {@link
   * Outcome#REFUSED}, with the gate's reason: {@code pivot.goTo("out").onlyIf(intakeClear)}.
   */
  public Move onlyIf(Gate gate) {
    return new Move(
        new Plan(
            kind,
            name,
            plan.fraction(),
            plan.within(),
            plan.until(),
            plan.target(),
            gate,
            plan.onlyWhile(),
            plan.refused()),
        copies());
  }

  /**
   * The same move, for only as long as a gate holds: checked every loop, it ends {@link
   * Outcome#CANCELED} when the gate falls, with the gate's reason, and is refused if it doesn't
   * hold to start with. {@code joints.run(FEEDING).onlyWhile(readyToShoot)}.
   */
  public Move onlyWhile(Gate gate) {
    return new Move(
        new Plan(
            kind,
            name,
            plan.fraction(),
            plan.within(),
            plan.until(),
            plan.target(),
            plan.onlyIf(),
            gate,
            plan.refused()),
        copies());
  }

  private Leg[] copies() {
    Leg[] copies = new Leg[legs.length];
    for (int i = 0; i < legs.length; i++) {
      copies[i] = legs[i].copy();
    }
    return copies;
  }

  // ---- How it ended ----

  /** Whether its latest run arrived: got there, or its condition held. */
  public boolean arrived() {
    return outcome == Outcome.ARRIVED;
  }

  /** How its latest run ended; empty before one has, and while one runs. */
  public Optional<Outcome> outcome() {
    return running ? Optional.empty() : Optional.ofNullable(outcome);
  }

  /** Why its latest run ended as it did, as logged: "pivot → out: ARRIVED in 0.41 s". */
  public String reason() {
    return reason;
  }

  /**
   * When each joint got there in its latest run, in seconds from its start: the joints inside their
   * bands as it ended, each since it came inside for good. For a configuration that arrived, every
   * joint, so the slowest one, the one to speed up, is plain; for one that timed out, those that
   * made it.
   */
  public Map<Joint, Double> arrivals() {
    return arrivals;
  }

  // ---- The command ----

  @Override
  public String name() {
    return name;
  }

  @Override
  public Set<Mechanism> requirements() {
    return requirements;
  }

  @Override
  public void run(Coroutine coroutine) {
    running = true;
    started = false;
    outcome = null;
    reason = "";
    if (kind != Kind.HOME) {
      while (waitsForHoming()) {
        coroutine.yield();
      }
    }
    if (!begin()) {
      return;
    }
    while (!step()) {
      coroutine.yield();
    }
  }

  @Override
  public void onCancel() {
    if (running) {
      if (kind == Kind.HOME) {
        legs[0].joint.stopHoming();
      }
      end(Outcome.CANCELED, null);
    }
  }

  @Override
  public String toString() {
    return name;
  }

  /** Whether one of its joints is homing, or will on the first enable: it waits for them. */
  private boolean waitsForHoming() {
    for (Leg leg : legs) {
      if (leg.joint.waitsForHoming()) {
        return true;
      }
    }
    return false;
  }

  /** Starts a run: false if it's refused, or done at once, which ends it. */
  boolean begin() {
    startNanos = now();
    String cannot = plan.refused();
    if (cannot != null) {
      end(Outcome.REFUSED, cannot);
      return false;
    }
    for (Leg leg : legs) {
      String refused = kind == Kind.HOME ? null : leg.joint.fault();
      if (refused == null && leg.output == null && kind == Kind.GO_TO) {
        refused = leg.joint.whyUnreachable(leg.position);
      }
      if (refused != null) {
        end(Outcome.REFUSED, refused);
        return false;
      }
    }
    Gate onlyIf = plan.onlyIf();
    if (onlyIf != null && !onlyIf.getAsBoolean()) {
      end(Outcome.REFUSED, onlyIf.why());
      return false;
    }
    Gate onlyWhile = plan.onlyWhile();
    if (onlyWhile != null && !onlyWhile.getAsBoolean()) {
      end(Outcome.REFUSED, onlyWhile.why());
      return false;
    }
    if (kind == Kind.HOME) {
      JointMechanism joint = legs[0].joint;
      if (!joint.homesToAStop() && !joint.homesByEncoder()) {
        end(Outcome.ARRIVED, null);
        return false;
      }
      joint.startHoming();
      deadlineNanos = Long.MAX_VALUE; // its recipe's timeout faults it instead
      started = true;
      return true;
    }
    double seconds = Double.NaN;
    for (Leg leg : legs) {
      leg.insideSince = -1;
      if (leg.output == null) {
        leg.joint.beginMove();
      }
      if (kind == Kind.GO_TO) {
        double leg_ =
            leg.output == null
                ? leg.joint.secondsTo(leg.position, plan.fraction())
                : leg.output.kind() == Output.Kind.VELOCITY ? leg.joint.secondsToSpeed() : 0;
        seconds = Double.isNaN(seconds) ? leg_ : Math.max(seconds, leg_);
      }
    }
    held = kind == Kind.HOLD ? legs[0].joint.goal() : Double.NaN;
    double deadline =
        !Double.isNaN(plan.within())
            ? plan.within()
            : kind == Kind.GO_TO
                ? DEADLINE_FACTOR * seconds + DEADLINE_SLACK_SECONDS
                : plan.until() != null ? RUN_UNTIL_SECONDS : Double.POSITIVE_INFINITY;
    deadlineNanos =
        Double.isInfinite(deadline) ? Long.MAX_VALUE : startNanos + Math.round(deadline * 1e9);
    started = true;
    return true;
  }

  /**
   * One loop: whether it has ended, judged on what the joints read this loop; if not, asks them for
   * this loop. A move that ends asks nothing more, so its last loop is already its joints' idle.
   */
  boolean step() {
    long now = now();
    if (kind == Kind.HOME) {
      JointMechanism joint = legs[0].joint;
      if (joint.state() == JointMechanism.State.READY) {
        end(Outcome.ARRIVED, null);
        return true;
      }
      String fault = joint.fault();
      if (fault != null) {
        end(Outcome.REFUSED, fault);
        return true;
      }
      return false;
    }
    for (Leg leg : legs) {
      String fault = leg.joint.fault();
      if (fault != null) {
        end(Outcome.REFUSED, "can't go on: " + fault);
        return true;
      }
    }
    Gate onlyWhile = plan.onlyWhile();
    if (onlyWhile != null && !onlyWhile.getAsBoolean()) {
      end(Outcome.CANCELED, onlyWhile.why());
      return true;
    }
    if (kind == Kind.GO_TO && everyLegThere(now)) {
      end(Outcome.ARRIVED, null);
      return true;
    }
    BooleanSupplier condition = plan.until();
    if (condition != null && condition.getAsBoolean()) {
      end(Outcome.ARRIVED, null);
      return true;
    }
    if (now >= deadlineNanos) {
      // A gate that never opened says why: "coral in claw is false: claw beam lost".
      end(Outcome.TIMED_OUT, condition instanceof Gate gate ? gate.why() : null);
      return true;
    }
    for (Leg leg : legs) {
      if (!ask(leg)) {
        return true;
      }
    }
    return false;
  }

  /** Asks a joint for its part this loop: false if the move can't, which has ended it. */
  private boolean ask(Leg leg) {
    JointMechanism joint = leg.joint;
    switch (kind) {
      case GO_TO, RUN -> {
        Output output = leg.output;
        if (output != null) {
          joint.askOutput(output);
        } else {
          joint.askPosition(leg.position, plan.fraction(), leg.name);
        }
      }
      case HOLD -> joint.askPosition(held, 1, null);
      case FOLLOW -> {
        Supplier<? extends Measure<?>> chased = plan.target();
        Measure<?> wanted = chased == null ? null : chased.get();
        if (wanted != null && joint.isPosition(wanted)) {
          joint.askPosition(joint.clamp(wanted.baseUnitMagnitude()), 1, null);
        } else if (wanted != null && joint.isSpeed(wanted)) {
          joint.askVelocity(wanted.baseUnitMagnitude());
        } else {
          end(
              Outcome.REFUSED,
              joint.getName()
                  + " can't follow "
                  + (wanted == null ? "nothing" : wanted.toLongString())
                  + ": it takes "
                  + (joint.joint() instanceof Joint.Revolute
                      ? "an angle or an angular velocity"
                      : "a distance or a linear velocity"));
          return false;
        }
      }
      case HOME -> {}
    }
    return true;
  }

  /** Whether every joint has been there for the dwell, as of now. */
  private boolean everyLegThere(long now) {
    boolean all = true;
    for (Leg leg : legs) {
      Output output = leg.output;
      boolean inside =
          output == null
              ? leg.joint.settledAt(leg.position)
              : output.kind() != Output.Kind.VELOCITY || leg.joint.atSpeed(output.value());
      if (!inside) {
        leg.insideSince = -1;
        all = false;
      } else if (leg.insideSince < 0) {
        leg.insideSince = now;
        all &= output != null && output.kind() != Output.Kind.VELOCITY;
      } else {
        all &= now - leg.insideSince >= Math.round(JointMechanism.DWELL_SECONDS * 1e9);
      }
    }
    return all;
  }

  /** Ends its run: records how, logs it under each joint, and leaves each to its idle. */
  private void end(Outcome how, @Nullable String why) {
    running = false;
    outcome = how;
    double seconds = (now() - startNanos) / 1e9;
    Map<Joint, Double> came = new LinkedHashMap<>();
    if (started && kind == Kind.GO_TO) {
      for (Leg leg : legs) {
        if (leg.insideSince >= 0) {
          came.put(leg.joint.joint(), (leg.insideSince - startNanos) / 1e9);
        }
      }
    }
    arrivals = Collections.unmodifiableMap(came);
    StringBuilder text = new StringBuilder(name).append(": ").append(how);
    switch (how) {
      case ARRIVED -> {
        text.append(String.format(Locale.ROOT, " in %.2f s", seconds));
        if (legs.length > 1) {
          List<String> each = new ArrayList<>();
          came.forEach(
              (joint, at) -> each.add(String.format(Locale.ROOT, "%s %.2f s", joint.label(), at)));
          text.append(" (").append(String.join(", ", each)).append(")");
        }
      }
      case TIMED_OUT, CANCELED -> {
        text.append(String.format(Locale.ROOT, " after %.2f s", seconds));
        for (Leg leg : legs) {
          if (leg.output == null && kind != Kind.HOME) {
            text.append(", ")
                .append(leg.joint.getName())
                .append(" at ")
                .append(JointSetup.shown(leg.joint.joint(), leg.joint.position()));
          }
        }
        if (how == Outcome.TIMED_OUT) {
          for (Leg leg : legs) {
            if (!leg.joint.heldBy().isEmpty()) {
              text.append(", held: ").append(leg.joint.heldBy());
            }
          }
        }
      }
      case REFUSED -> {}
    }
    if (why != null) {
      text.append(": ").append(why);
    }
    reason = text.toString();
    for (Leg leg : legs) {
      if (started && leg.output == null && kind != Kind.HOME) {
        leg.joint.moveEnded(how == Outcome.ARRIVED && kind == Kind.GO_TO, leg.position);
      }
      leg.joint.recordOutcome(reason);
    }
  }

  /** The robot's time this loop, as its joints read it: none for a move of no joints, refused. */
  private long now() {
    return legs.length == 0 ? 0 : legs[0].joint.now();
  }
}
