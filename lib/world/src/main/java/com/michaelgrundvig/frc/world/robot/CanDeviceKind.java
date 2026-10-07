package com.michaelgrundvig.frc.world.robot;

import java.util.function.BiFunction;

/**
 * A kind of CAN device that a block names by its bus and ID, as a motor's controller is named
 * ({@code TALON_FX.can(CANIVORE, 20)}): a CANcoder ({@link MechanismWiring#CANCODER}), a CANdi
 * ({@link SensorWiring#CANDI}) or a CANrange ({@link SensorWiring#CANRANGE}).
 *
 * @param <T> what naming one gives: the device, or the device with what's still to say about it
 */
public final class CanDeviceKind<T> {
  private final BiFunction<CanBus, Integer, T> at;

  CanDeviceKind(BiFunction<CanBus, Integer, T> at) {
    this.at = at;
  }

  /** One on Systemcore's first CAN port, at an ID: {@code CANRANGE.can(31)}. */
  public T can(int id) {
    return can(CanBus.SYSTEMCORE, id);
  }

  /** One on a CAN bus, at an ID: {@code CANRANGE.can(CANIVORE, 31)}. */
  public T can(CanBus bus, int id) {
    return at.apply(bus, id);
  }
}
