package com.michaelgrundvig.frc.pitwall.experiment;

import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.pitwall.Numbers;
import java.util.List;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Time;

/**
 * The standard test motions, each written by name:
 *
 * {@snippet :
 * Profiles.stepAndBack().from(Degrees.of(0)).to(Degrees.of(90)).hold(Seconds.of(0.8))
 * }
 *
 * <p>Every profile starts with a lead-in at its first setpoint ({@link #LEAD_IN} unless changed
 * with {@link Step#leadIn}), so the mechanism is settled there before the first jump. Only the
 * jumps are scored.
 */
public final class Profiles {
  /** How long a profile holds its first setpoint before the first jump, unless told otherwise. */
  public static final Time LEAD_IN = Seconds.of(0.5);

  private Profiles() {}

  /** A step: from one setpoint to another, held there. */
  public static From step() {
    return from -> to -> hold -> new Step(from, to, hold, LEAD_IN, false);
  }

  /** A step and back: from one setpoint to another, held, then back to the first, held again. */
  public static From stepAndBack() {
    return from -> to -> hold -> new Step(from, to, hold, LEAD_IN, true);
  }

  /** The first stage of a step: where it starts. */
  @FunctionalInterface
  public interface From {
    To from(Measure<?> from);
  }

  /** The second stage of a step: where it goes. */
  @FunctionalInterface
  public interface To {
    Hold to(Measure<?> to);
  }

  /** The last stage of a step: how long each setpoint is held. */
  @FunctionalInterface
  public interface Hold {
    Step hold(Time hold);
  }

  /** A step, or a step and back. */
  public static final class Step implements Profile {
    private final Unit baseUnit;
    private final double from;
    private final double to;
    private final double holdSeconds;
    private final double leadInSeconds;
    private final boolean andBack;
    private final Measure<?> fromMeasure;
    private final Measure<?> toMeasure;
    private final Time hold;

    private Step(Measure<?> from, Measure<?> to, Time hold, Time leadIn, boolean andBack) {
      baseUnit = from.unit().getBaseUnit();
      if (!baseUnit.equals(to.unit().getBaseUnit())) {
        throw new IllegalArgumentException(
            "a step goes from and to the same kind of quantity: " + from + " to " + to);
      }
      this.from = from.baseUnitMagnitude();
      this.to = to.baseUnitMagnitude();
      holdSeconds = hold.in(Seconds);
      leadInSeconds = leadIn.in(Seconds);
      if (!Double.isFinite(this.from) || !Double.isFinite(this.to) || this.from == this.to) {
        throw new IllegalArgumentException("a step goes somewhere: " + from + " to " + to);
      }
      if (!(holdSeconds > 0) || !Double.isFinite(holdSeconds)) {
        throw new IllegalArgumentException("a step holds for some time: " + hold);
      }
      if (!(leadInSeconds >= 0) || !Double.isFinite(leadInSeconds)) {
        throw new IllegalArgumentException("a lead-in can't be negative: " + leadIn);
      }
      this.andBack = andBack;
      fromMeasure = from;
      toMeasure = to;
      this.hold = hold;
    }

    /** The same step, holding its first setpoint for {@code leadIn} before the first jump. */
    public Step leadIn(Time leadIn) {
      return new Step(fromMeasure, toMeasure, hold, leadIn, andBack);
    }

    @Override
    public Unit baseUnit() {
      return baseUnit;
    }

    @Override
    public double setpoint(double seconds) {
      if (seconds < leadInSeconds) {
        return from;
      }
      if (seconds < leadInSeconds + holdSeconds || !andBack) {
        return to;
      }
      return from;
    }

    @Override
    public double durationSeconds() {
      return leadInSeconds + holdSeconds * (andBack ? 2 : 1);
    }

    @Override
    public List<Edge> edges() {
      Edge there = new Edge(leadInSeconds, from, to);
      return andBack
          ? List.of(there, new Edge(leadInSeconds + holdSeconds, to, from))
          : List.of(there);
    }

    /** What it does, as the page shows it: {@code step and back, 0 ° to 90 °, held 0.8 s}. */
    @Override
    public String toString() {
      return (andBack ? "step and back, " : "step, ")
          + shown(fromMeasure)
          + " to "
          + shown(toMeasure)
          + ", held "
          + shown(hold);
    }
  }

  /** A measure as people write it: {@code 90 °}, {@code 0.8 s}. */
  private static String shown(Measure<?> measure) {
    return Numbers.shown(measure.magnitude()) + " " + measure.unit().symbol();
  }
}
