package com.michaelgrundvig.frc.pitwall.simulator;

/**
 * What the simulator's controls change in WPILib's simulation: the simulated Driver Station and
 * joysticks, and the simulator's clock. {@link Simulator#wpilib} gives WPILib's own; tests give a
 * stand-in, since they run without WPILib's native libraries.
 */
public interface SimulatorHal {
  /** The Driver Station's state as the simulation has it now. */
  DriverStationState driverStation();

  /**
   * Sets the Driver Station's state; the robot program sees it once it's announced ({@link
   * #announce}).
   */
  void driverStation(DriverStationState state);

  /** Sets a joystick's input; the robot program sees it once it's announced ({@link #announce}). */
  void joystick(int port, JoystickInput input);

  /**
   * Tells the robot program the Driver Station has new data, as a Driver Station's packet does: it
   * sees the changes at once.
   */
  void announce();

  /** Whether the simulator's clock is paused. */
  boolean isPaused();

  /**
   * Pauses the simulator's clock: the robot loop waits for its next loop's time, which never comes.
   */
  void pause();

  /** Runs the simulator's clock again. */
  void resume();

  /** While paused, moves the clock on to the robot loop's next loop, which then runs. */
  void step();
}
