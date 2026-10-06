// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.pitwall.simulator.Simulator;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import frc.robot.dashboard.DashboardStream;
import frc.robot.game.Rebuilt;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.sim.Scenario;
import frc.robot.sim.Scenarios;
import frc.robot.sim.SimulatedWorld;
import frc.robot.venues.VenueIO;
import frc.robot.venues.VenueIODisk;
import frc.robot.venues.VenuePit;
import frc.robot.venues.VenueStore;
import frc.robot.venues.Venues;
import java.io.File;
import java.lang.management.ManagementFactory;
import java.nio.file.Path;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.system.Filesystem;
import org.wpilib.util.Alert;

/** The robot program: logging setup, then the command scheduler every loop. */
public class Robot extends LoggedRobot {
  /** Set by {@code ./gradlew run}: how the simulated robot starts, as a {@link SimStart}'s name. */
  static final String START_PROPERTY = "frc.sim.start";

  /** How {@code ./gradlew run} starts the simulated robot. */
  enum SimStart {
    /**
     * Without WPILib's simulator GUI, the default: disabled, in teleop on the blue alliance's
     * station 2 with a Driver Station attached. Pitwall's Sim view enables it with one press, there
     * beside the keyboard driving it switches on.
     */
    READY,
    /**
     * With the GUI ({@code -Pgui}): as {@link #READY}, and enabled, since the GUI's keyboard drives
     * it at once and the GUI is on screen to disable it.
     */
    ENABLED,
    /**
     * As a real robot starts, before its Driver Station connects: disabled, with no station or mode
     * ({@code -PstartDisabled}). Tests and replays, which never set the property, too.
     */
    DISABLED;

    /** The start a property names, case aside; {@link #DISABLED} for none, or one unknown. */
    static SimStart named(@Nullable String name) {
      for (SimStart start : values()) {
        if (start.name().equalsIgnoreCase(name)) {
          return start;
        }
      }
      return DISABLED;
    }
  }

  /** Set by {@code ./gradlew run -Pscenario=<name>}: the simulator scenario to start in. */
  static final String SCENARIO_PROPERTY = "frc.sim.scenario";

  /**
   * In the simulator, what happens in the scenario asked for, until the robot is first enabled and
   * it happens.
   */
  private @Nullable List<Setting> pendingEvents;

  /**
   * Pitwall, the robot's own web page: its status, actions, and live tuning of its configuration.
   */
  private final Pitwall pitwall;

  /** Every mechanism, and the controls that drive them. */
  private final RobotContainer container;

  /** Where the robot's log goes: AdvantageKit writes to its logs folder. */
  static final String LOG_DRIVE = "/U";

  /** Whether the loop keeps up, and its log with it. */
  private final LoopHealth loopHealth = new LoopHealth();

  /**
   * Where venues are kept: the robot's disk on a robot, and in the simulator its own under {@code
   * build/venues/}. None in a replay, which takes the venue from the log; nor in tests, which use
   * the official layout unless they say otherwise.
   */
  static @Nullable VenueStore venueStore(Constants.Mode mode) {
    if (mode == Constants.Mode.REPLAY) {
      return null;
    }
    Path stored =
        mode == Constants.Mode.REAL
            ? VenueStore.ROBOT_FOLDER
            : Filesystem.getOperatingDirectory().toPath().resolve("build/venues");
    return new VenueStore(stored, Filesystem.getDeployDirectory().toPath().resolve("venues"));
  }

  /** A debug deploy ({@code deploy -PdebugMode}) waits for a debugger every time it starts. */
  private final Alert debugDeploy =
      new Alert(
          "Robot",
          "debugDeploy",
          "A debug deploy: after any restart the robot waits for a debugger. Deploy without"
              + " -PdebugMode before a match",
          Alert.Level.HIGH);

  public Robot() {
    super(RobotLoop.PERIOD_SECONDS);
    // Every log records exactly which code produced it.
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });
    // In the simulator, the scenario asked for (./gradlew run -Pscenario=<name>), which every log
    // of it names. A name no scenario has stops the program here, listing them.
    Scenario asked = scenarioAskedFor(Constants.currentMode, System.getProperty(SCENARIO_PROPERTY));
    if (asked != null) {
      Logger.recordMetadata("SimScenario", asked.name());
      System.out.println("Simulator: scenario " + asked.name() + ": " + asked.description());
    }

    switch (Constants.currentMode) {
      case REAL -> {
        // On the robot: log to the USB drive, and stream to dashboards (less often on the field,
        // where the radio's bandwidth is capped and the drive keeps everything).
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new DashboardStream(new NT4Publisher()));
        loopHealth.watchLogDrive(new File(LOG_DRIVE));
      }
      case SIM -> {
        // In the simulator: stream to AdvantageScope, and keep a log file (logs/) of every session.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      }
      case REPLAY -> {
        // Replaying a log through this code, as fast as possible, writing a "_sim" log to compare.
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    Logger.start();
    debugDeploy.set(
        Constants.currentMode == Constants.Mode.REAL
            && startedForDebugger(ManagementFactory.getRuntimeMXBean().getInputArguments()));

    RobotConfig robot = Robots.current();
    pitwall = Pitwall.start(robot.name());
    // A season-worn robot's sensors.
    container =
        assemble(
            Constants.currentMode,
            Scheduler.getDefault(),
            robot,
            pitwall,
            SensorErrors.TYPICAL,
            loopHealth,
            venueStore(Constants.currentMode));
    // The scenario's sensor faults, through the simulator's controls (where the page shows them,
    // and can undo them) before the first loop; its events once the robot is enabled.
    pendingEvents =
        startScenario(
            asked, container.world(), pitwall.simulator().map(Simulator::components).orElse(null));
  }

  /**
   * Starts the scenario asked for: its sensor faults now, through the simulator's controls.
   *
   * @param asked the scenario, or null when none was asked for
   * @param world the simulated world, or null outside the simulator
   * @param components the simulator's controls' parts that can fail, or null without them
   * @return what happens in the scenario, to happen once the robot is enabled; null for nothing
   * @throws IllegalStateException when a scenario was asked for without a simulated world and the
   *     simulator's controls to apply it through, or one of its faults can't be applied: it was
   *     asked for by name, so the program stops rather than run without it
   */
  static @Nullable List<Setting> startScenario(
      @Nullable Scenario asked, @Nullable SimulatedWorld world, @Nullable Components components) {
    if (asked == null) {
      return null;
    }
    if (world == null || components == null) {
      throw new IllegalStateException(
          "Simulator scenario "
              + asked.name()
              + ": it needs the simulated world and Pitwall's simulator controls, and this program"
              + " has "
              + (world == null ? "no simulated world" : "no simulator controls"));
    }
    apply(asked.sensorFaultsIn(world), components);
    return asked.eventsIn(world);
  }

  /**
   * Applies a scenario's faults through the simulator's controls. One that changes nothing (the
   * part is that way as built) is passed over.
   *
   * @throws IllegalStateException saying why, when one can't be applied: the scenario was asked for
   *     by name, so it stops the program rather than run without it
   */
  static void apply(List<Setting> faults, Components components) {
    String refused = components.apply(faults);
    if (refused != null) {
      throw new IllegalStateException("Simulator scenario: " + refused);
    }
  }

  /**
   * The simulator scenario asked for, in a physics simulation only: a replay runs what its log
   * recorded, and a robot has no simulator.
   *
   * @param name the scenario's name, or null when none was asked for
   * @throws IllegalArgumentException listing every scenario, when none has this name
   */
  static @Nullable Scenario scenarioAskedFor(Constants.Mode mode, @Nullable String name) {
    return mode == Constants.Mode.SIM && name != null ? Scenarios.named(name) : null;
  }

  /**
   * Puts the robot together: every mechanism, on the field it plays on, with its pit tools and
   * status on the Pitwall page. The mechanisms read their configuration through Pitwall, so tuned
   * values take effect at once. Tests build the robot here too ({@code RobotSim}), so they test the
   * robot the robot runs.
   *
   * @param simulatedErrors in the simulator, how the robot's wheels and sensors differ from its
   *     configuration
   */
  static RobotContainer assemble(
      Constants.Mode mode,
      Scheduler scheduler,
      RobotConfig robot,
      Pitwall pitwall,
      SensorErrors simulatedErrors,
      LoopHealth loopHealth,
      @Nullable VenueStore venueStore) {
    Venues.Game game = Rebuilt.venues(robot.definition().body());
    VenueIODisk disk = venueStore != null ? new VenueIODisk(venueStore) : null;
    Venues venues = new Venues(game, disk != null ? disk : new VenueIO() {});
    RobotContainer container =
        new RobotContainer(
            mode,
            scheduler,
            robot,
            Rebuilt.field(robot.definition().body()),
            venues,
            simulatedErrors,
            loopHealth);
    container.addTo(pitwall);
    if (venueStore != null && disk != null) {
      pitwall.devices().add(VenuePit.device(venueStore, disk, venues, game));
    }
    return container;
  }

  /** Whether the program was started for a debugger to attach (JDWP), as a debug deploy is. */
  static boolean startedForDebugger(List<String> jvmArguments) {
    return jvmArguments.stream().anyMatch(argument -> argument.contains("jdwp"));
  }

  /** The mechanisms, for tests that drive the robot program. */
  RobotContainer container() {
    return container;
  }

  /** Pitwall, for tests that drive the robot program through it. */
  Pitwall pitwall() {
    return pitwall;
  }

  @Override
  public void robotPeriodic() {
    loop(pitwall, container, Scheduler.getDefault(), loopHealth);
  }

  /**
   * Every loop, in every mode: the Pitwall page's requests, the mechanisms' sensors, the command
   * scheduler, then what the commands asked of the joints, each timed. Tests run the robot with
   * this too ({@code RobotSim}).
   */
  static void loop(
      Pitwall pitwall, RobotContainer container, Scheduler scheduler, LoopHealth loopHealth) {
    loopHealth.start(System.nanoTime());
    pitwall.periodic();
    loopHealth.monitor().mark("Pitwall", System.nanoTime());
    container.periodic();
    scheduler.run();
    loopHealth.monitor().mark("Commands", System.nanoTime());
    container.afterCommands();
    loopHealth.monitor().mark("Outputs", System.nanoTime());
    loopHealth.finish(System.nanoTime(), Logger.getReceiverQueueFault());
  }

  /**
   * {@code ./gradlew run} starts the simulated robot ready to drive: in teleop, on the blue
   * alliance's station 2 unless something already chose one, with a Driver Station attached, so it
   * stands on the center start. Without WPILib's simulator GUI, the default, it starts disabled:
   * Pitwall's Sim view enables it with one press, beside the keyboard driving it switches on there,
   * and a robot never enables itself before anyone is watching it. With the GUI ({@code -Pgui}) it
   * starts enabled, since the GUI's keyboard drives it at once and the GUI is on screen. {@code
   * -PstartDisabled} starts it as a real robot starts, disabled with no station or mode. Tests and
   * replay never set the property.
   */
  @Override
  public void simulationInit() {
    startAsAsked(Constants.currentMode, SimStart.named(System.getProperty(START_PROPERTY)));
  }

  /** Readies or enables the simulated robot, only in a physics simulation and only when asked. */
  static void startAsAsked(Constants.Mode mode, SimStart start) {
    if (mode != Constants.Mode.SIM || start == SimStart.DISABLED) {
      return;
    }
    if (MatchState.getAlliance().isEmpty()) {
      // The center start, and blue's forward is the field view's right: the easiest first drive.
      DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_2);
    }
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(start == SimStart.ENABLED);
    DriverStationSim.notifyNewData();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void utilityPeriodic() {}

  /**
   * The scenario's events (a collision, a slip) happen on the first loop the robot is enabled, so
   * the fault finds it where the drive team put it. That relies on two things: WPILib runs this
   * after {@link #robotPeriodic}, and the robot's container puts the robot on its start in that
   * loop at the latest.
   */
  @Override
  public void simulationPeriodic() {
    pendingEvents =
        startWhenEnabled(
            pendingEvents,
            pitwall.simulator().map(Simulator::components).orElse(null),
            RobotState.isEnabled());
  }

  /**
   * Makes a scenario's events happen in the simulated world, if the robot is enabled.
   *
   * @param components the simulator's controls, or null outside the simulator
   * @return the events still waiting to happen: null once they have
   */
  static @Nullable List<Setting> startWhenEnabled(
      @Nullable List<Setting> pending, @Nullable Components components, boolean enabled) {
    if (pending == null || components == null || !enabled) {
      return pending;
    }
    apply(pending, components);
    return null;
  }
}
