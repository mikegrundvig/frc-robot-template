package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import frc.robot.game.Rebuilt;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.sim.DriveSimulation;
import frc.robot.sim.Scenario;
import frc.robot.sim.SensorErrors;
import frc.robot.sim.SimulatedWorld;
import java.util.List;
import java.util.Objects;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.extension.AfterEachCallback;
import org.junit.jupiter.api.extension.BeforeEachCallback;
import org.junit.jupiter.api.extension.ExtensionContext;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GamepadSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/**
 * The simulator, for tests: every test that touches WPILib's simulation (its clock, the Driver
 * Station, a simulated device) runs with this, and a test of the whole robot builds the robot here.
 *
 * <pre>{@code
 * @ExtendWith(RobotSim.class)                        // a mechanism, or any simulated hardware
 * @RegisterExtension final RobotSim sim = new RobotSim();   // the whole robot
 * }</pre>
 *
 * <p>Each test starts the same way, whatever ran before it: the clock stopped (it moves only when
 * the test steps it), the Driver Station attached and disabled on the blue alliance's station 2,
 * and fresh dashboards (WPILib's mock telemetry and tunable backends). Afterwards everything is
 * closed and reset, so nothing leaks into the next test. The HAL is shared by every test in a run:
 * change the Driver Station through this class, or call {@code notifyNewData()}, or the change
 * doesn't arrive.
 *
 * <p>The whole robot is built the way {@link Robot} builds it ({@link Robot#assemble}), Pitwall and
 * all, and {@link #loop()} runs the loop {@link Robot} runs, then what WPILib's loop does after it.
 * Unless a test builds another first, it is the simulator's robot ({@link Robots#SIM}) with sensors
 * exactly as configured, so poses come out exact.
 */
public final class RobotSim implements BeforeEachCallback, AfterEachCallback {
  /** The field the simulator's robot plays on. */
  public static final FieldModel FIELD = Rebuilt.field(Robots.SIM.body());

  /** The simulator's robot without its cameras: odometry alone. */
  public static final RobotConfig WITHOUT_CAMERAS =
      RobotConfigBuilder.builder(Robots.SIM).cameras(List.of()).build();

  private @Nullable MockTelemetryBackend telemetry;
  private @Nullable Scheduler scheduler;
  private @Nullable GamepadSim driver;
  private @Nullable Pitwall pitwall;
  private @Nullable RobotContainer container;
  private @Nullable LoopHealth loopHealth;

  @Override
  public void beforeEach(ExtensionContext context) {
    assertTrue(HAL.initialize(), "simulated HAL failed to initialize");
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_2);
    setEnabled(false);
    telemetry = new MockTelemetryBackend();
    TelemetryRegistry.registerBackend("", telemetry);
    TunableRegistry.registerBackend("", new MockTunableBackend());
    scheduler = Scheduler.createIndependentScheduler();
    // Made after the Driver Station's data is reset, which clears a gamepad's axes.
    driver = new GamepadSim(0);
  }

  @Override
  public void afterEach(ExtensionContext context) {
    closeRobot();
    TelemetryRegistry.reset();
    TunableRegistry.reset();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  /** Builds the simulator's robot with sensors that err as a real robot's do, or not at all. */
  public RobotContainer build(SensorErrors errors) {
    return build(Constants.Mode.SIM, Robots.SIM, errors);
  }

  /**
   * Builds the simulator's robot in a scenario ({@link frc.robot.sim.Scenarios}), as {@code
   * ./gradlew run -Pscenario=<name>} does: the scenario's sensor faults on sensors that err as a
   * season-worn robot's do, applied through the simulator's controls before the first loop. What
   * happens to the robot in it waits until the test starts it ({@link #start}).
   */
  public RobotContainer build(Scenario scenario) {
    RobotContainer built = build(Robots.SIM, SensorErrors.TYPICAL);
    Robot.apply(scenario.sensorFaultsIn(world()), components());
    return built;
  }

  /** Builds a robot in the simulator. */
  public RobotContainer build(RobotConfig robot, SensorErrors errors) {
    return build(Constants.Mode.SIM, robot, errors);
  }

  /**
   * Builds a robot, in place of any built before, as {@link Robot} does: in {@code REAL} mode it
   * talks to the hardware its configuration wires (CTRE's simulated devices, in a test), and in
   * {@code REPLAY} to nothing.
   *
   * @param errors in the simulator, how its wheels and sensors differ from its configuration
   */
  public RobotContainer build(Constants.Mode mode, RobotConfig robot, SensorErrors errors) {
    closeRobot();
    LoopHealth health = new LoopHealth();
    Pitwall page = Pitwall.start(robot.name(), 0);
    pitwall = page;
    loopHealth = health;
    container = Robot.assemble(mode, scheduler(), robot, page, errors, health);
    return container;
  }

  /** The robot, built as the simulator's robot on first use unless the test built another. */
  public RobotContainer robot() {
    RobotContainer built = container;
    return built != null ? built : build(Constants.Mode.SIM, Robots.SIM, SensorErrors.NONE);
  }

  /** The robot's Pitwall: its page's status, actions, and tuning. */
  public Pitwall pitwall() {
    robot();
    return Objects.requireNonNull(pitwall);
  }

  /** The simulator's physics: where the robot really is. */
  public DriveSimulation simulation() {
    return Objects.requireNonNull(robot().simulation(), "only the simulator has physics");
  }

  /** The simulated world: the robot's physics and its cameras, which scenarios act on. */
  public SimulatedWorld world() {
    return Objects.requireNonNull(robot().world(), "only the simulator has a world");
  }

  /**
   * Starts a scenario now: what happens to the robot in it (a collision, a slip) happens in the
   * simulated world. {@code ./gradlew run} starts it on the first loop the robot is enabled.
   */
  public void start(Scenario scenario) {
    Robot.apply(scenario.eventsIn(world()), components());
  }

  /**
   * The simulated world's parts that can fail, and the scenarios, as the simulator's controls on
   * the Pitwall page have them: apply a fault here as the page does.
   */
  public Components components() {
    return pitwall().simulator().orElseThrow().components();
  }

  /** The robot's command scheduler. */
  public Scheduler scheduler() {
    return Objects.requireNonNull(scheduler, "the test hasn't started");
  }

  /** The driver's gamepad, port 0. */
  public GamepadSim driver() {
    return Objects.requireNonNull(driver, "the test hasn't started");
  }

  /** What the robot published, as a dashboard sees it. */
  public MockTelemetryBackend telemetry() {
    return Objects.requireNonNull(telemetry, "the test hasn't started");
  }

  /**
   * One robot loop: the clock moves a loop's time, then the robot runs {@link Robot#loop}, then
   * WPILib's tunables update, as WPILib's own loop does after the robot's.
   */
  public void loop() {
    RobotContainer robot = robot();
    stepClock();
    Robot.loop(pitwall(), robot, scheduler(), Objects.requireNonNull(loopHealth));
    TunableRegistry.update();
  }

  /** {@code count} robot loops. */
  public void loops(int count) {
    for (int i = 0; i < count; i++) {
      loop();
    }
  }

  /** Robot loops for {@code seconds} of the robot's time. */
  public void run(double seconds) {
    loops(RobotLoop.loops(seconds));
  }

  /** Enables the robot in teleop, or disables it. */
  public static void setEnabled(boolean enabled) {
    enable(RobotMode.TELEOPERATED, enabled);
  }

  /** Puts the robot in a mode (autonomous, teleop, utility), enabled or not. */
  public static void enable(RobotMode mode, boolean enabled) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(mode);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  /** The Driver Station's alliance and station, as the drive team or the field sets it. */
  public static void station(AllianceStationID station) {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.notifyNewData();
  }

  /** Moves the simulated clock one robot loop, for a test that runs its own loop. */
  public static void stepClock() {
    SimHooks.stepTiming(RobotLoop.PERIOD_SECONDS);
  }

  private void closeRobot() {
    RobotContainer built = container;
    if (built != null) {
      built.close();
    }
    Pitwall page = pitwall;
    if (page != null) {
      page.close();
    }
    container = null;
    pitwall = null;
    loopHealth = null;
    scheduler = Scheduler.createIndependentScheduler();
  }
}
