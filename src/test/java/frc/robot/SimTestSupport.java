package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Shared setup for simulation tests: the simulated HAL, paused time, and robot enable state.
 *
 * <p>The HAL is shared by every test in a run. Always publish Driver Station changes with {@code
 * notifyNewData()}, or state from an earlier test leaks into the next one.
 */
public final class SimTestSupport {
  public static final double LOOP_PERIOD_S = 0.02;

  private SimTestSupport() {}

  public static void initHal() {
    assertTrue(HAL.initialize(), "simulated HAL failed to initialize");
  }

  /** Freeze the clock (it only moves when a test steps it) and start disabled. */
  public static void pauseTimeAndDisable() {
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    setEnabled(false);
  }

  public static void resumeTime() {
    SimHooks.resumeTiming();
  }

  public static void setEnabled(boolean enabled) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  /** Advance the simulated clock by one robot loop. */
  public static void stepLoop() {
    SimHooks.stepTiming(LOOP_PERIOD_S);
  }

  public static int loops(double seconds) {
    return (int) Math.ceil(seconds / LOOP_PERIOD_S);
  }
}
