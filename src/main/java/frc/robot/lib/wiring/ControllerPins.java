package frc.robot.lib.wiring;

import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.Optional;

/**
 * What a joint's leading motor controller reads as its forward and reverse limits, from the robot's
 * sensor wiring: a switch on its own limit pin, or, for a TalonFX, a limit switch on a CANdi input
 * it reads over CAN. Worked out for each joint as the robot starts, never written by hand.
 *
 * <p>The controller stops its motor on a limit switch itself, with no code in the loop: the safest
 * stop there is. It never stops on anything else on a pin (a beam break read through it), and a pin
 * with nothing wired to it stops nothing.
 *
 * @param forward its forward limit, if it has one
 * @param reverse its reverse limit, if it has one
 */
public record ControllerPins(Optional<Limit> forward, Optional<Limit> reverse) {
  /** A controller with nothing on its limits. */
  public static final ControllerPins NONE = new ControllerPins(Optional.empty(), Optional.empty());

  /** One of its limits. */
  public sealed interface Limit permits OnPin, OnCandi {
    /** Whether the controller stops its motor on it. */
    boolean stops();
  }

  /**
   * A switch on the controller's own limit pin: read through it, the controller told its polarity.
   *
   * @param polarity which raw signal means tripped
   * @param stops whether it's a limit switch, which the controller stops on
   */
  public record OnPin(SensorWiring.Polarity polarity, boolean stops) implements Limit {}

  /**
   * A limit switch on a CANdi input on the controller's own bus, which a TalonFX reads over CAN as
   * its limit: always stopped on. The CANdi is told which state is tripped, so its "closed" is.
   *
   * @param id the CANdi's CAN ID
   * @param input 1 or 2: its S1 or S2
   */
  public record OnCandi(int id, int input) implements Limit {
    @Override
    public boolean stops() {
      return true;
    }
  }

  /** The same, with one limit set. */
  public ControllerPins with(SensorWiring.Direction direction, Limit limit) {
    return direction == SensorWiring.Direction.FORWARD
        ? new ControllerPins(Optional.of(limit), reverse)
        : new ControllerPins(forward, Optional.of(limit));
  }
}
