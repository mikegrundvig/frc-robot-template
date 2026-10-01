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

  /** Sets the match time alone, in seconds left; -1 for none. Announced like the rest. */
  void matchTime(double seconds);

  /** Sets a joystick's input; the robot program sees it once it's announced ({@link #announce}). */
  void joystick(int port, JoystickInput input);

  /**
   * Tells the robot program the Driver Station has new data, as a Driver Station's packet does: it
   * sees the changes as its next loop begins, not partway through this one.
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

  /**
   * While paused, moves the clock on to the next alarm any notifier waits for, which then fires.
   * Between loops that's the robot loop's own, so its next loop runs; a team's own notifier due
   * sooner would fire instead.
   */
  void step();
}
