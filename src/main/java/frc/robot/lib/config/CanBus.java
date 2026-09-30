package frc.robot.lib.config;

import org.wpilib.hardware.bus.CANPort;

/** Which CAN bus a device is on: one of Systemcore's own ports, or a CANivore, by its name. */
public sealed interface CanBus {
  /** One of Systemcore's CAN ports. */
  record Port(CANPort port) implements CanBus {}

  /** A CANivore, by the name set in Phoenix Tuner X. */
  record Canivore(String name) implements CanBus {
    public Canivore {
      if (name.isBlank()) {
        throw new IllegalArgumentException("a CANivore is named in Phoenix Tuner X");
      }
    }
  }

  static CanBus port(CANPort port) {
    return new Port(port);
  }

  static CanBus canivore(String name) {
    return new Canivore(name);
  }
}
