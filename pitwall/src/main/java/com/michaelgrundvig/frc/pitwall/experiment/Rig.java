package com.michaelgrundvig.frc.pitwall.experiment;

import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.pitwall.Numbers;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

/**
 * Everything tuning needs to know about one mechanism, built by name:
 *
 * {@snippet :
 * Rig.builder()
 *     .name("Arm")
 *     .gains(config, "gains.kP", "gains.kD") // values of its LiveConfig
 *     .setpoint(Radians, radians -> setpoint = radians) // how an experiment commands it
 *     .limits(Degrees.of(-100), Degrees.of(100)) // where an experiment may send it
 *     .measurement("Angle", Degrees, this::readings) // what's scored
 *     .signal("Voltage", Volts, () -> config.get().gains().kP() * (setpoint - angle)) // beside it
 *     .build();
 * }
 *
 * <p>A rig on a mechanism the scheduler runs says so ({@code .requires(drive)}), so a run holds it
 * as a command would; the template's drivetrain is one ({@code Drive.steeringRig}).
 *
 * <p>Its gains are values of a {@link LiveConfig}, so they stay where they always were: the page
 * tunes them there, through the same checks as any other change (their tuning limits, then the
 * configuration's own checks). A rig only reads them, to record which gains a run ran with.
 *
 * <p>Its signals are read each loop an experiment runs. A signal hands over every sample taken
 * since the last loop, each with its own time, so a device's 250 Hz readings keep their rate: see
 * {@link Sampler}. A signal reads the mechanism's logged inputs (its IO layer's, as AdvantageKit
 * logs them), so a replay of the log records the same trace.
 */
public final class Rig {
  /** What the trace calls the setpoint an experiment commands; no signal may take this name. */
  public static final String SETPOINT = "Setpoint";

  /**
   * How many times a second a trace makes room for a signal read once a loop (and the setpoint):
   * twice the usual 50 Hz loop. A signal handing over a device's own readings gets room for 1 kHz.
   */
  static final int LOOP_SAMPLES_PER_SECOND = 100;

  /** Room for a device's own readings: 1 kHz. */
  static final int DEVICE_SAMPLES_PER_SECOND = 1000;

  /**
   * Reads one signal each loop an experiment runs: every sample taken since the last loop, oldest
   * first, into its series. A signal read once a loop adds one; the drivetrain's 250 Hz odometry
   * adds five, each at the time it was taken. It runs on the robot loop's thread, and reads only
   * logged inputs, so a replay reads the same.
   */
  @FunctionalInterface
  public interface Sampler {
    /**
     * @param nowNanos this loop's time on the robot's clock, for a signal read once a loop
     * @param series where the samples go: {@link Series#add(long, double)}, in the signal's unit
     */
    void sample(long nowNanos, Series series);
  }

  /**
   * @param onceALoop whether it's read once a loop, and needs room only for that
   */
  private record Read(Trace.Signal signal, Sampler sampler, boolean onceALoop) {}

  private final String name;
  private final LiveConfig<?> config;
  private final List<String> gains;
  private final Unit setpointUnit;
  private final DoubleConsumer command;
  private final Measure<?> min;
  private final Measure<?> max;
  private final List<Read> measurements;
  private final List<Read> signals;
  private final List<Mechanism> requirements;

  private Rig(Builder builder) {
    name = builder.name;
    config = required(builder.config, "its gains");
    gains = List.copyOf(builder.gains);
    setpointUnit = required(builder.setpointUnit, "how to command its setpoint");
    command = required(builder.command, "how to command its setpoint");
    min = required(builder.min, "its limits");
    max = required(builder.max, "its limits");
    measurements = List.copyOf(builder.measurements);
    signals = List.copyOf(builder.signals);
    requirements = List.copyOf(builder.requirements);
  }

  /** Starts building a rig. */
  public static Builder builder() {
    return new Builder();
  }

  /** What it's called on the page. */
  public String name() {
    return name;
  }

  /** The live configuration its gains are values of. */
  public LiveConfig<?> config() {
    return config;
  }

  /** Its gains: fields of its configuration, such as {@code gains.steerKp}. */
  public List<String> gains() {
    return gains;
  }

  /** Its gains as they are now, by field, each in its knob's unit. */
  public Map<String, Double> gainValues() {
    Map<String, Double> values = new LinkedHashMap<>();
    for (String gain : gains) {
      values.put(gain, config.value(gain));
    }
    return values;
  }

  /** What it measures, each scored: the trace's signals of that name. */
  public List<Trace.Signal> measurements() {
    return measurements.stream().map(Read::signal).toList();
  }

  /** What it records beside its measurements, such as voltage and current. */
  public List<Trace.Signal> signals() {
    return signals.stream().map(Read::signal).toList();
  }

  /**
   * The unit its measurements are in, which the trace records the setpoint in too, so they share a
   * plot.
   */
  public Unit measurementUnit() {
    return measurements.get(0).signal().unit();
  }

  /** The mechanisms an experiment on it needs to itself. */
  public List<Mechanism> requirements() {
    return requirements;
  }

  /** The scheduler its experiments run on: its mechanisms' own. */
  public Scheduler scheduler() {
    return requirements.isEmpty()
        ? Scheduler.getDefault()
        : requirements.get(0).getRegisteredScheduler();
  }

  /** Its limits, as written: {@code -180 ° to 180 °}. */
  public String limits() {
    return shown(min.magnitude(), min.unit()) + " to " + shown(max.magnitude(), max.unit());
  }

  /**
   * Why a profile can't run on this rig: its setpoints are another kind of quantity, or fall
   * outside the rig's limits.
   *
   * @return null when it can
   */
  public @Nullable String whyNot(Profile profile) {
    Unit base = setpointUnit.getBaseUnit();
    if (!profile.baseUnit().equals(base)) {
      return "its setpoints are in "
          + profile.baseUnit().name()
          + ", but "
          + name
          + " takes its setpoint in "
          + base.name();
    }
    List<Double> setpoints = new ArrayList<>();
    setpoints.add(profile.setpoint(0));
    for (Profile.Edge edge : profile.edges()) {
      setpoints.add(edge.from());
      setpoints.add(edge.to());
    }
    setpoints.add(profile.setpoint(profile.durationSeconds()));
    for (double setpoint : setpoints) {
      if (!(setpoint >= min.baseUnitMagnitude() && setpoint <= max.baseUnitMagnitude())) {
        return shown(min.unit().fromBaseUnits(setpoint), min.unit())
            + " is outside "
            + name
            + "'s limits, "
            + limits();
      }
    }
    return null;
  }

  /** A setpoint, in the base unit, brought within the rig's limits. */
  double clamped(double setpoint) {
    return Math.clamp(setpoint, min.baseUnitMagnitude(), max.baseUnitMagnitude());
  }

  /** Commands a setpoint, given in the base unit. */
  void command(double setpoint) {
    command.accept(setpointUnit.fromBaseUnits(setpoint));
  }

  /**
   * Reads every signal into the trace. A signal that throws loses this loop's samples and says so;
   * the others, and the run, go on.
   *
   * @return why a signal couldn't be read, or null when every one was
   */
  @Nullable String sample(long nowNanos, Trace trace) {
    String failed = null;
    for (List<Read> reads : List.of(measurements, signals)) {
      for (Read read : reads) {
        try {
          read.sampler().sample(nowNanos, trace.series(read.signal().name()));
        } catch (RuntimeException e) {
          if (failed == null) {
            String why = e.getMessage();
            failed = "reading " + read.signal().name() + " failed: " + (why != null ? why : e);
          }
        }
      }
    }
    return failed;
  }

  /**
   * How many samples a run of so many seconds records of a signal, and half a second to spare: at
   * loop rate for the setpoint and a signal read once a loop, at a device's rate for the rest.
   */
  int capacity(Trace.Signal signal, double seconds) {
    boolean onceALoop =
        signal.name().equals(SETPOINT)
            || Stream.concat(measurements.stream(), signals.stream())
                .anyMatch(read -> read.signal().equals(signal) && read.onceALoop());
    int perSecond = onceALoop ? LOOP_SAMPLES_PER_SECOND : DEVICE_SAMPLES_PER_SECOND;
    return (int) Math.ceil((seconds + 0.5) * perSecond);
  }

  /** Every signal a run records: the setpoint, then the measurements, then the rest. */
  List<Trace.Signal> traced() {
    List<Trace.Signal> traced = new ArrayList<>();
    traced.add(new Trace.Signal(SETPOINT, measurementUnit()));
    traced.addAll(measurements());
    traced.addAll(signals());
    return traced;
  }

  private static String shown(double value, Unit unit) {
    return Numbers.shown(value) + " " + unit.symbol();
  }

  private <V> V required(@Nullable V value, String what) {
    if (value == null) {
      throw new IllegalStateException("the rig " + name + " needs " + what);
    }
    return value;
  }

  /** A rig being built. Every part but {@link #signal} and {@link #requires} is needed. */
  public static final class Builder {
    private String name = "";
    private @Nullable LiveConfig<?> config;
    private final List<String> gains = new ArrayList<>();
    private @Nullable Unit setpointUnit;
    private @Nullable DoubleConsumer command;
    private @Nullable Measure<?> min;
    private @Nullable Measure<?> max;
    private final List<Read> measurements = new ArrayList<>();
    private final List<Read> signals = new ArrayList<>();
    private final List<Mechanism> requirements = new ArrayList<>();

    private Builder() {}

    /** What it's called on the page: {@code "Steering"}. */
    public Builder name(String name) {
      this.name = name;
      return this;
    }

    /**
     * Its gains: values of a live configuration, which the page tunes there.
     *
     * @param fields each a tunable value of that configuration, such as {@code gains.steerKp}
     * @throws IllegalArgumentException if one isn't
     */
    public Builder gains(LiveConfig<?> config, String... fields) {
      for (String field : fields) {
        config.value(field); // throws, saying so, if it isn't one of the configuration's
      }
      this.config = config;
      gains.clear();
      gains.addAll(List.of(fields));
      return this;
    }

    /**
     * How an experiment commands it: the unit it takes a setpoint in, and what takes it. An
     * experiment calls it every loop it runs, from the robot loop. It only commands the mechanism,
     * as its own commands would: it changes no gain or other configuration (those go through the
     * page's checks), and it moves nothing but this mechanism.
     */
    public Builder setpoint(Unit unit, DoubleConsumer command) {
      setpointUnit = unit;
      this.command = command;
      return this;
    }

    /**
     * Where an experiment may send it, both included. An experiment that would go further is
     * refused when it's added, and every setpoint is kept within them as well.
     */
    public Builder limits(Measure<?> min, Measure<?> max) {
      this.min = min;
      this.max = max;
      return this;
    }

    /**
     * Something it measures in answer to its setpoint, which a run scores: an angle, a position, a
     * speed. Several may be (each swerve module's angle); they share one unit.
     */
    public Builder measurement(String name, Unit unit, Sampler sampler) {
      measurements.add(new Read(new Trace.Signal(name, unit), sampler, false));
      return this;
    }

    /** A measurement read once a loop. */
    public Builder measurement(String name, Unit unit, DoubleSupplier onceALoop) {
      measurements.add(
          new Read(
              new Trace.Signal(name, unit),
              (now, series) -> series.add(now, onceALoop.getAsDouble()),
              true));
      return this;
    }

    /** Something recorded beside the measurements, such as a motor's voltage or current. */
    public Builder signal(String name, Unit unit, Sampler sampler) {
      signals.add(new Read(new Trace.Signal(name, unit), sampler, false));
      return this;
    }

    /** Something recorded once a loop beside the measurements. */
    public Builder signal(String name, Unit unit, DoubleSupplier onceALoop) {
      signals.add(
          new Read(
              new Trace.Signal(name, unit),
              (now, series) -> series.add(now, onceALoop.getAsDouble()),
              true));
      return this;
    }

    /** The mechanisms an experiment needs to itself, as any command requires them. */
    public Builder requires(Mechanism... mechanisms) {
      requirements.addAll(List.of(mechanisms));
      return this;
    }

    /**
     * The rig.
     *
     * @throws IllegalStateException if a part is missing
     * @throws IllegalArgumentException if the parts don't fit: its limits, measurements, and
     *     setpoint must be the same kind of quantity, its measurements must share a unit, and its
     *     signals need names of their own
     */
    public Rig build() {
      if (name.isBlank()) {
        throw new IllegalStateException("a rig needs a name");
      }
      if (gains.isEmpty()) {
        throw new IllegalStateException("the rig " + name + " needs its gains");
      }
      if (measurements.isEmpty()) {
        throw new IllegalStateException("the rig " + name + " needs something to measure");
      }
      Rig rig = new Rig(this);
      Unit base = rig.setpointUnit.getBaseUnit();
      if (!rig.min.unit().getBaseUnit().equals(base)
          || !rig.max.unit().getBaseUnit().equals(base)) {
        throw new IllegalArgumentException(
            name + "'s limits aren't the same kind of quantity as its setpoint: " + rig.limits());
      }
      if (!(rig.min.baseUnitMagnitude() < rig.max.baseUnitMagnitude())) {
        throw new IllegalArgumentException(name + "'s limits go nowhere: " + rig.limits());
      }
      Unit measured = rig.measurementUnit();
      for (Trace.Signal measurement : rig.measurements()) {
        if (!measurement.unit().equals(measured)) {
          throw new IllegalArgumentException(
              name
                  + "'s measurements share one unit: "
                  + measurement.name()
                  + " isn't in "
                  + measured.name());
        }
      }
      if (!measured.getBaseUnit().equals(base)) {
        throw new IllegalArgumentException(
            name
                + " measures in "
                + measured.name()
                + ", a different kind of quantity from its setpoint");
      }
      List<String> names = new ArrayList<>();
      for (Trace.Signal signal : rig.traced()) {
        if (names.contains(signal.name())) {
          throw new IllegalArgumentException(
              name
                  + " has two signals named "
                  + signal.name()
                  + " (the setpoint is "
                  + SETPOINT
                  + ")");
        }
        names.add(signal.name());
      }
      return rig;
    }
  }
}
