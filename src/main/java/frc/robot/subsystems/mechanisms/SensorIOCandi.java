package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import frc.robot.lib.phoenix.PhoenixMechanism;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.lib.wiring.SensorWiring;
import org.wpilib.math.filter.Debouncer;

/**
 * A switch on one of a CANdi's two inputs. The CANdi is told which state is tripped, so its own
 * "closed" is: a switch to ground, or a sensor that pulls its signal low as it trips (an NPN
 * output), reads low when closed, and the CANdi's input floats while it's open. Normally open trips
 * low; normally closed trips on anything but low, so a cut wire, floating, reads tripped. A sensor
 * that drives its signal high as it trips (a PNP output) isn't read yet.
 *
 * <p>Two switches on one CANdi each set only their own input's state, read from the device first,
 * so neither undoes the other.
 */
public final class SensorIOCandi implements SensorIO {
  /** How long to wait for the CANdi to answer at startup, each try. */
  private static final double STARTUP_TIMEOUT_SECONDS = 0.25;

  private static final int TRIES = 5;

  private final StatusSignal<Boolean> closed;
  private final Debouncer connected = new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  /**
   * @param wiring where it's connected and which state is tripped
   * @param candi its input
   */
  public SensorIOCandi(SensorWiring.OfSwitch wiring, SensorWiring.Candi candi) {
    CANdi device = new CANdi(candi.id(), PhoenixMechanism.bus(candi.bus()));
    boolean open = wiring.polarity() == SensorWiring.Polarity.NORMALLY_OPEN;
    DigitalInputsConfigs config = new DigitalInputsConfigs();
    for (int i = 0; i < TRIES; i++) {
      if (device.getConfigurator().refresh(config, STARTUP_TIMEOUT_SECONDS).isOK()) {
        break;
      }
    }
    if (candi.input() == 1) {
      config.S1CloseState =
          open ? S1CloseStateValue.CloseWhenLow : S1CloseStateValue.CloseWhenNotLow;
      closed = device.getS1Closed();
    } else {
      config.S2CloseState =
          open ? S2CloseStateValue.CloseWhenLow : S2CloseStateValue.CloseWhenNotLow;
      closed = device.getS2Closed();
    }
    for (int i = 0; i < TRIES; i++) {
      if (device.getConfigurator().apply(config, STARTUP_TIMEOUT_SECONDS).isOK()) {
        break;
      }
    }
    BaseStatusSignal.setUpdateFrequencyForAll(RobotLoop.FREQUENCY_HZ, closed);
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.connected = connected.calculate(closed.refresh().getStatus().isOK());
    inputs.tripped = closed.getValue();
  }
}
