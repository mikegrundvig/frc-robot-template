package frc.robot.sim;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import frc.robot.subsystems.mechanisms.SensorIO;
import java.util.LinkedHashMap;
import java.util.Map;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.measure.Distance;

/**
 * The robot's switches and rangefinders in the simulator. A limit switch trips from where its joint
 * really is; what a beam, a contact switch or a rangefinder sees is a test's or a scenario's to say
 * ({@link #trip}, {@link #see}), game pieces being sensor states, never physics. Until it's said, a
 * switch is clear and a rangefinder sees nothing within its range.
 */
public final class SensorSimulation {
  /** One simulated sensor: its device's reading, as a test leaves it. */
  private static final class Simulated implements SensorIO {
    final Sensor.Placed sensor;
    final @Nullable JointSimulation joints;
    boolean tripped;
    double meters;
    boolean silent;

    /** A cut wire's reading, whatever trips it: null while its wire is whole. */
    @Nullable Boolean cut;

    Simulated(Sensor.Placed sensor, @Nullable JointSimulation joints) {
      this.sensor = sensor;
      this.joints = joints;
      meters =
          sensor instanceof Sensor.Rangefinder rangefinder ? rangefinder.range().in(Meters) : 0;
    }

    @Override
    public void updateInputs(SensorIOInputs inputs) {
      inputs.connected = !silent;
      if (silent) {
        return;
      }
      Boolean stuck = cut;
      if (stuck != null) {
        inputs.tripped = stuck;
      } else if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.kind() instanceof Sensor.Switch.Limit limit) {
        inputs.tripped =
            joints != null && joints.simulates(limit.joint())
                ? limit.trippedAt(joints.position(limit.joint()))
                : tripped;
      } else {
        inputs.tripped = tripped;
      }
      inputs.meters = meters;
    }
  }

  private final @Nullable JointSimulation joints;
  private final Map<Sensor.Placed, Simulated> sensors = new LinkedHashMap<>();

  /**
   * @param joints the joints' physics, which a limit switch reads its joint's position from; null
   *     for a robot without driven joints
   */
  public SensorSimulation(@Nullable JointSimulation joints) {
    this.joints = joints;
  }

  /** A sensor, simulated: its device, for the robot to read. */
  public SensorIO add(Sensor.Placed sensor) {
    return sensors.computeIfAbsent(sensor, each -> new Simulated(each, joints));
  }

  /**
   * Trips a beam or a contact switch, or clears it: something in its beam, or nothing. A limit
   * switch follows its joint instead.
   *
   * @throws IllegalArgumentException for a limit switch on a simulated joint, or a sensor that
   *     isn't simulated
   */
  public void trip(Sensor.Switch sensor, boolean tripped) {
    if (sensor.kind() instanceof Sensor.Switch.Limit limit
        && joints != null
        && joints.simulates(limit.joint())) {
      throw new IllegalArgumentException(
          sensor.label() + " trips where " + limit.joint().label() + " really is: move it there");
    }
    simulated(sensor).tripped = tripped;
  }

  /**
   * What a rangefinder sees: the distance to the nearest thing along its axis.
   *
   * @throws IllegalArgumentException for a sensor that isn't simulated
   */
  public void see(Sensor.Rangefinder sensor, Distance distance) {
    simulated(sensor).meters = distance.in(Meters);
  }

  /**
   * Cuts a switch's wire: it reads as its polarity reads an open circuit, whatever trips it, a
   * normally closed one tripped and a normally open one clear. Null mends it.
   */
  public void cut(Sensor.Switch sensor, SensorWiring.@Nullable Polarity polarity) {
    simulated(sensor).cut =
        polarity == null ? null : polarity == SensorWiring.Polarity.NORMALLY_CLOSED;
  }

  /**
   * Silences a sensor, or brings it back: a CAN device that stops answering, as a loose connector
   * does.
   */
  public void silence(Sensor.Placed sensor, boolean silent) {
    simulated(sensor).silent = silent;
  }

  private Simulated simulated(Sensor.Placed sensor) {
    Simulated simulated = sensors.get(sensor);
    if (simulated == null) {
      throw new IllegalArgumentException(sensor.label() + " isn't simulated");
    }
    return simulated;
  }
}
