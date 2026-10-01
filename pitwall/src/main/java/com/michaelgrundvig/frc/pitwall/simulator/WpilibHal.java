package com.michaelgrundvig.frc.pitwall.simulator;

import org.wpilib.driverstation.POVDirection;
import org.wpilib.hardware.hal.simulation.DriverStationDataJNI;
import org.wpilib.hardware.hal.simulation.NotifierDataJNI;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.system.RobotController;

/**
 * WPILib's simulation, as the simulator's controls change it: the simulated Driver Station's data
 * ({@link DriverStationSim}) and the simulator's clock ({@link SimHooks}), in this program's own
 * process, the way the WPILib simulator GUI changes them.
 */
final class WpilibHal implements SimulatorHal {
  /** A gamepad's shape, as WPILib's {@code GamepadSim} declares it: 6 axes, 26 buttons, 1 POV. */
  private static final int GAMEPAD_AXES = 6;

  private static final int GAMEPAD_BUTTONS = 26;
  private static final int GAMEPAD_POVS = 1;

  /** A loop's time, for a step when no loop is waiting for its time: the usual 20 ms. */
  private static final long LOOP_NANOS = 20_000_000L;

  @Override
  public DriverStationState driverStation() {
    return new DriverStationState(
        DriverStationSim.getEnabled(),
        DriverStationSim.getRobotMode(),
        DriverStationSim.getAllianceStationId(),
        DriverStationSim.getFmsAttached(),
        DriverStationSim.getDsAttached(),
        DriverStationSim.getMatchTime());
  }

  @Override
  public void driverStation(DriverStationState state) {
    DriverStationSim.setDsAttached(state.dsAttached());
    DriverStationSim.setRobotMode(state.mode());
    DriverStationSim.setAllianceStationId(state.allianceStation());
    DriverStationSim.setFmsAttached(state.fmsAttached());
    DriverStationSim.setMatchTime(state.matchTime());
    DriverStationSim.setEnabled(state.enabled());
  }

  /** Sets every axis, button, and POV a gamepad has, and any more the input has. */
  @Override
  public void joystick(int port, JoystickInput input) {
    int axes = Math.max(GAMEPAD_AXES, input.axes().size());
    int buttons = Math.max(GAMEPAD_BUTTONS, input.buttons().size());
    int povs = Math.max(GAMEPAD_POVS, input.povs().size());
    DriverStationSim.setJoystickIsGamepad(port, true);
    DriverStationSim.setJoystickAxesMaximumIndex(port, axes);
    DriverStationSim.setJoystickButtonsMaximumIndex(port, buttons);
    DriverStationSim.setJoystickPOVsMaximumIndex(port, povs);
    for (int i = 0; i < axes; i++) {
      DriverStationSim.setJoystickAxis(port, i, i < input.axes().size() ? input.axes().get(i) : 0);
    }
    DriverStationDataJNI.setJoystickButtonsValue(port, input.buttonBits());
    for (int i = 0; i < povs; i++) {
      int angle = i < input.povs().size() ? input.povs().get(i) : -1;
      DriverStationSim.setJoystickPOV(port, i, direction(angle));
    }
  }

  @Override
  public void announce() {
    DriverStationSim.notifyNewData();
  }

  @Override
  public boolean isPaused() {
    return SimHooks.isTimingPaused();
  }

  @Override
  public void pause() {
    SimHooks.pauseTiming();
  }

  @Override
  public void resume() {
    SimHooks.resumeTiming();
  }

  /**
   * Moves the clock to the next alarm any notifier waits for, as the WPILib GUI's Step does: in a
   * robot program between loops, the loop's own. With none waiting, a loop's usual time.
   */
  @Override
  public void step() {
    long until = NotifierDataJNI.getNextTimeout() - RobotController.getMonotonicTime();
    SimHooks.stepTimingAsync((until > 0 && until < 1_000_000_000L ? until : LOOP_NANOS) / 1e9);
  }

  /** A POV's angle as the Driver Station carries it. */
  static POVDirection direction(int degrees) {
    return switch (degrees) {
      case 0 -> POVDirection.UP;
      case 45 -> POVDirection.UP_RIGHT;
      case 90 -> POVDirection.RIGHT;
      case 135 -> POVDirection.DOWN_RIGHT;
      case 180 -> POVDirection.DOWN;
      case 225 -> POVDirection.DOWN_LEFT;
      case 270 -> POVDirection.LEFT;
      case 315 -> POVDirection.UP_LEFT;
      default -> POVDirection.CENTER;
    };
  }
}
