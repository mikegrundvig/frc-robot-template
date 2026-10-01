package frc.robot;

import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.sim.Scenario;
import frc.robot.sim.SensorErrors;
import frc.robot.sim.SimulatedWorld;
import java.util.Objects;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GamepadSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/**
 * The whole robot in the simulator, for a study: built the way {@link Robot} builds it and run loop
 * by loop the way {@link Robot} runs it, as the tests' {@code RobotSim} does, but outside a test.
 * The simulator's clock moves only when a loop runs, so a run takes as long as the computer needs
 * and repeats exactly.
 *
 * <p>WPILib's simulation is one per program (its clock, the Driver Station), so one robot runs at a
 * time: close it before building the next. Closing puts everything back as it was found.
 */
public final class SimulatedRobot implements AutoCloseable {
  private final Scheduler scheduler = Scheduler.createIndependentScheduler();
  private final GamepadSim driver;
  private final Pitwall pitwall;
  private final LoopHealth loopHealth = new LoopHealth();
  private final RobotContainer container;

  /**
   * Builds the robot, disabled on the blue alliance's station 2, as the drive team leaves it.
   *
   * @param errors how its wheels, sensors, and camera mounts differ from its configuration
   */
  public SimulatedRobot(RobotConfig robot, SensorErrors errors) {
    if (!HAL.initialize()) {
      throw new IllegalStateException("WPILib's simulation failed to start");
    }
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_2);
    // Dashboards that keep what they're sent to themselves.
    TelemetryRegistry.registerBackend("", new MockTelemetryBackend());
    TunableRegistry.registerBackend("", new MockTunableBackend());
    setEnabled(false);
    // Made after the Driver Station's data is reset, which clears a gamepad's axes.
    driver = new GamepadSim(0);
    pitwall = Pitwall.start(robot.name(), 0);
    container = Robot.assemble(Constants.Mode.SIM, scheduler, robot, pitwall, errors, loopHealth);
  }

  /** One robot loop: the clock moves a loop's time, the robot runs its loop, tunables update. */
  public void loop() {
    SimHooks.stepTiming(RobotLoop.PERIOD_SECONDS);
    Robot.loop(pitwall, container, scheduler, loopHealth);
    TunableRegistry.update();
  }

  /** Enables the robot in teleop, or disables it. */
  public void setEnabled(boolean enabled) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  /**
   * Holds the driver's sticks: forward and left as the driver sees them (blue's forward is the
   * field's +x), and turning counterclockwise, each from -1 to 1.
   */
  public void sticks(double forward, double left, double turn) {
    driver.setLeftY(-forward);
    driver.setLeftX(-left);
    driver.setRightX(-turn);
    driver.notifyNewData();
  }

  /**
   * Applies a scenario's sensor faults through the simulator's controls, as {@code ./gradlew run
   * -Pscenario=<name>} does before the robot's first loop: a camera knocked, a gyro erring.
   *
   * @throws IllegalStateException if one of them can't be applied, saying why
   */
  public void applyFaults(Scenario scenario) {
    Robot.apply(scenario.sensorFaultsIn(world()), components());
  }

  /**
   * Makes what happens in a scenario happen, now, through the simulator's controls: a collision, a
   * push.
   *
   * @throws IllegalStateException if it can't, saying why
   */
  public void start(Scenario scenario) {
    Robot.apply(scenario.eventsIn(world()), components());
  }

  private SimulatedWorld world() {
    return Objects.requireNonNull(container.world(), "only the simulator has a world");
  }

  /**
   * The simulated world's parts that can fail, as the simulator's controls on Pitwall have them.
   */
  private Components components() {
    return pitwall.simulator().orElseThrow().components();
  }

  /** Starts the gyro-scale procedure, as the Pitwall page does. */
  public Command startGyroScaleProcedure() {
    Command procedure = container.calibration().gyroScale();
    scheduler.schedule(procedure);
    return procedure;
  }

  /** Whether a command is still running. */
  public boolean isRunning(Command command) {
    return scheduler.isScheduledOrRunning(command);
  }

  /** What the last calibration procedure reported. */
  public String lastCalibration() {
    return container.calibration().lastResult();
  }

  /** Where the robot really is. */
  public Pose2d truePose() {
    return Objects.requireNonNull(container.simulation()).truePose();
  }

  /** What localization believes, as of the last loop. */
  public LocalizationEstimate estimate() {
    return container.estimate();
  }

  @Override
  public void close() {
    container.close();
    pitwall.close();
    TelemetryRegistry.reset();
    TunableRegistry.reset();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }
}
