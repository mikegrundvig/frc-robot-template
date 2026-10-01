package com.michaelgrundvig.frc.pitwall.simulator;

import java.util.HashMap;
import java.util.Map;
import org.jspecify.annotations.Nullable;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;

/**
 * WPILib's simulation, as the simulator's controls see it, for tests without WPILib's native
 * libraries: the Driver Station's state, each joystick's input, and a clock that pauses and steps.
 * A test's loop asks {@link #runsLoop} before each loop, so a paused clock runs none, as a robot
 * loop waiting on its notifier runs none, until a step lets one through. Safe from any thread, as
 * the real one is.
 */
public final class FakeHal implements SimulatorHal {
  private DriverStationState state =
      new DriverStationState(false, RobotMode.UNKNOWN, AllianceStationID.UNKNOWN, false, false, -1);
  private final Map<Integer, JoystickInput> joysticks = new HashMap<>();
  private int writes;
  private int announced;
  private boolean paused;
  private int steps;
  private int stepsToRun;
  private int matchTimes;

  @Override
  public synchronized DriverStationState driverStation() {
    return state;
  }

  @Override
  public synchronized void driverStation(DriverStationState state) {
    this.state = state;
    writes++;
  }

  @Override
  public synchronized void matchTime(double seconds) {
    state = state.withMatchTime(seconds);
    matchTimes++;
  }

  @Override
  public synchronized void joystick(int port, JoystickInput input) {
    joysticks.put(port, input);
  }

  @Override
  public synchronized void announce() {
    announced++;
  }

  @Override
  public synchronized boolean isPaused() {
    return paused;
  }

  @Override
  public synchronized void pause() {
    paused = true;
  }

  @Override
  public synchronized void resume() {
    paused = false;
  }

  @Override
  public synchronized void step() {
    steps++;
    stepsToRun++;
  }

  /**
   * Whether the robot loop runs now: always while the clock runs; while it's paused, once for each
   * step.
   */
  public synchronized boolean runsLoop() {
    if (!paused) {
      return true;
    }
    if (stepsToRun > 0) {
      stepsToRun--;
      return true;
    }
    return false;
  }

  /** How many times the match time alone was written. */
  public synchronized int matchTimes() {
    return matchTimes;
  }

  /** A joystick's input as last written; null if never. */
  public synchronized @Nullable JoystickInput joystick(int port) {
    return joysticks.get(port);
  }

  /** How many times the Driver Station's state was written. */
  public synchronized int writes() {
    return writes;
  }

  /** How many times new data was announced to the robot program. */
  public synchronized int announced() {
    return announced;
  }

  /** How many loops were stepped. */
  public synchronized int steps() {
    return steps;
  }
}
