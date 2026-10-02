package frc.robot.lib.wiring;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;

import com.michaelgrundvig.frc.world.config.Checks;
import org.wpilib.hardware.bus.CANPort;

/** Which CAN bus a device is on: one of Systemcore's own ports, or a CANivore, by its name. */
public sealed interface CanBus {
  /** One of Systemcore's CAN ports. */
  record Port(CANPort port) implements CanBus {}

  /** A CANivore, by the name set in Phoenix Tuner X. */
  record Canivore(String name) implements CanBus {
    public Canivore {
      Checks.on(Canivore.class).field("name", name, notBlank()).orThrow();
    }
  }

  static CanBus port(CANPort port) {
    return new Port(port);
  }

  static CanBus canivore(String name) {
    return new Canivore(name);
  }
}
