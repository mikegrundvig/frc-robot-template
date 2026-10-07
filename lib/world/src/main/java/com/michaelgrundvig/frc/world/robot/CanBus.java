package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;

import com.michaelgrundvig.frc.world.config.Checks;

/**
 * Which CAN bus a device is on: one of Systemcore's own ports ({@link #SYSTEMCORE}, its first), or
 * a CANivore, by its name.
 */
public sealed interface CanBus {
  /** Systemcore's first CAN port, where a robot's devices are unless it says otherwise. */
  CanBus SYSTEMCORE = new Port(Systemcore.CAN_S0);

  /** Systemcore's CAN ports, as WPILib names them ({@code CANPort}). */
  enum Systemcore {
    CAN_S0,
    CAN_S1,
    CAN_S2,
    CAN_S3,
    CAN_S4,
    CAN_D0,
    CAN_D1,
    CAN_D2,
    CAN_D3,
    CAN_D4
  }

  /** One of Systemcore's CAN ports. */
  record Port(Systemcore port) implements CanBus {}

  /** A CANivore, by the name set in Phoenix Tuner X. */
  record Canivore(String name) implements CanBus {
    public Canivore {
      Checks.on(Canivore.class).field("name", name, notBlank()).orThrow();
    }
  }

  /** As a sentence names it: "Systemcore's CAN_S0 port", "the CANivore canivore". */
  default String shown() {
    return switch (this) {
      case Port port -> "Systemcore's " + port.port() + " port";
      case Canivore canivore -> "the CANivore " + canivore.name();
    };
  }

  static CanBus port(Systemcore port) {
    return new Port(port);
  }

  static CanBus canivore(String name) {
    return new Canivore(name);
  }
}
