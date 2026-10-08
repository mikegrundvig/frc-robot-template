package frc.robot.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.lib.timing.RobotLoop;
import java.util.List;
import org.jspecify.annotations.Nullable;

/**
 * A joint's absolute encoder against its motor's count, each loop it has both and knows where it
 * is: apart by more than its margin, and more than a loop's travel at its speed, for a quarter
 * second, one of them slipped (a magnet loose on its shaft, a belt or a chain that skipped), and
 * where the joint is isn't known ({@link JointHoming}). An analog encoder can't say it's unplugged,
 * and a magnet turned on its shaft still reads: this is how either shows. Agreeing again, inside
 * the margin, the encoder is trusted again.
 */
final class EncoderGap {
  /** How long they must be apart before one is taken to have slipped. */
  static final long SLIP_NANOS = 250_000_000L;

  /**
   * The least margin, without one in the joint's control: 5° on a joint that turns, 2 cm on one
   * that slides. The backlash and give between a motor and its joint's encoder (a gearbox's lash, a
   * chain's or a belt's stretch, an elevator's rigging) is a degree or two, or a centimeter, and no
   * resolution shows it; a magnet that slipped, or a belt that skipped a tooth, is usually well
   * past it.
   */
  static final double ANGLE_MARGIN = Math.toRadians(5);

  static final double LENGTH_MARGIN = 0.02;

  /**
   * ...and never under this many of the encoder's counts at the joint: a reading flickers by a
   * count, and an encoder geared down from its joint has coarser ones there.
   */
  static final int COUNTS = 8;

  /**
   * The counts in an encoder's signal: a CANcoder's in its turn, and an analog reading's over its
   * range (12 bits); a through-bore's absolute output's (10 bits), the coarsest of the rest the
   * template reads, on a SPARK's data port or a Systemcore port.
   */
  static final int CANCODER_COUNTS = 4096;

  static final int ANALOG_COUNTS = 4096;
  static final int OTHER_COUNTS = 1024;

  private final Joint joint;
  private final double margin;
  private final double turn;

  /** Since when they've been apart, in nanoseconds: -1 while they agree. */
  private long since = -1;

  /**
   * @param margin how far apart they may be, still, in the joint's units ({@link #margin})
   * @param turn the encoder's turn at the joint, where the joint travels past it ({@link #turn}):
   *     they're apart by the nearest way round it; infinite where it doesn't
   */
  EncoderGap(Joint joint, double margin, double turn) {
    this.joint = joint;
    this.margin = margin;
    this.turn = turn;
  }

  /**
   * The encoder's turn at a joint that travels past it, in the joint's units: one that turns
   * freely, or whose range is wider than the window its encoder tells apart ({@link
   * RobotMath#absoluteWindow}). Its encoder reads within its turn while its motor's count goes on,
   * so the two are a whole turn apart where they agree. Infinite for a joint whose range the
   * encoder tells apart, where a reading a turn off is one that slipped.
   */
  static double turn(RobotMath math, Joint joint) {
    Sensor.Encoder encoder = math.absoluteEncoder(joint);
    if (encoder == null) {
      return Double.POSITIVE_INFINITY;
    }
    double window = math.absoluteWindow(List.of(encoder)).width();
    boolean past = joint.range().map(range -> range.width() > window).orElse(true);
    return past ? window : Double.POSITIVE_INFINITY;
  }

  /**
   * How far apart a joint's encoder and its motor's count may be, still: its control's ({@code
   * withEncoderGap}), else the least margin or eight of the encoder's counts at the joint,
   * whichever is more: its counts its definition's, else its device's.
   */
  static double margin(RobotMath math, Joint joint, MechanismControl control) {
    if (control.encoderGap().isPresent()) {
      return control.encoderGap().get().margin();
    }
    boolean turns = joint instanceof Joint.Revolute;
    double least = turns ? ANGLE_MARGIN : LENGTH_MARGIN;
    Sensor.Encoder encoder = math.absoluteEncoder(joint);
    if (encoder == null) {
      return least;
    }
    // Its signal's turns for each of the joint's units: radians, or meters.
    double perUnit = Math.abs(encoder.ratio(joint)) / (turns ? 2 * Math.PI : 1);
    int counts = encoder.countsPerTurn().orElseGet(() -> counts(joint));
    return perUnit > 0 ? Math.max(least, COUNTS / (counts * perUnit)) : least;
  }

  /** The counts in a joint's encoder's signal, by what it's wired to. */
  private static int counts(Joint joint) {
    Driven.AbsoluteEncoder wired = joint.driven().absoluteEncoder().orElse(null);
    if (wired != null && wired.cancoder().isPresent()) {
      return CANCODER_COUNTS;
    }
    boolean analog =
        wired != null
            && wired
                .port()
                .filter(port -> port.signal() == MechanismWiring.PortEncoder.Signal.ANALOG)
                .isPresent();
    return analog ? ANALOG_COUNTS : OTHER_COUNTS;
  }

  /** How far apart they may be, still, in the joint's units. */
  double margin() {
    return margin;
  }

  /**
   * Whether they've been apart long enough to say one slipped, as of this loop: how far, in the
   * joint's words, or null. Either reading missing this loop (NaN) says nothing, and starts over.
   *
   * @param encoder where the encoder says the joint is
   * @param motor where its motor's count says it is
   * @param velocity how fast it moves: a loop's travel at it is allowed beside the margin
   */
  @Nullable String apart(long now, double encoder, double motor, double velocity) {
    double gap = gap(encoder, motor);
    if (!(gap > margin + Math.abs(velocity) * RobotLoop.PERIOD_SECONDS)) {
      since = -1;
      return null;
    }
    if (since < 0) {
      since = now;
    }
    return now - since >= SLIP_NANOS
        ? joint.shown(gap) + " apart, more than " + joint.shown(margin)
        : null;
  }

  /** Whether they agree now, inside the margin: an encoder distrusted for its gap is trusted. */
  boolean agree(double encoder, double motor) {
    return gap(encoder, motor) <= margin;
  }

  /** How far apart they are: the nearest way round the encoder's turn, where it wraps. */
  private double gap(double encoder, double motor) {
    double apart = encoder - motor;
    return Math.abs(Double.isInfinite(turn) ? apart : apart - turn * Math.rint(apart / turn));
  }
}
