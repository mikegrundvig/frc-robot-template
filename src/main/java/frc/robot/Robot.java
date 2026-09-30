// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.michaelgrundvig.frc.pitwall.Pitwall;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.MatchState;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;

/** The robot program: logging setup, then the command scheduler every loop. */
public class Robot extends LoggedRobot {
  /** Set by {@code ./gradlew run}: start the simulator enabled in teleop. */
  static final String AUTO_ENABLE_PROPERTY = "frc.sim.autoEnable";

  /**
   * Pitwall, the robot's own web page: its status, actions, and live tuning of its configuration.
   * One robot for now; with several, choose its configuration by the robot's identity here.
   */
  private final Pitwall<RobotConfig> pitwall;

  /** Every mechanism, and the controls that drive them. */
  private final RobotContainer container;

  public Robot() {
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

    switch (Constants.currentMode) {
      case REAL -> {
        // On the robot: log to a file, and stream to dashboards.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
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

    pitwall = Pitwall.start(Robots.SIM.name(), Robots.SIM);

    // The drivetrain reads its configuration through Pitwall, so tuned values take effect at once.
    container =
        new RobotContainer(
            Constants.currentMode,
            Scheduler.getDefault(),
            Robots.SIM.body(),
            () -> pitwall.config().drivetrain(),
            Rebuilt.field(Robots.SIM.body()));
    pitwall.actions().add("Drive forward slowly (2 s)", container::driveCheck);
    pitwall
        .status()
        .add("Alliance", container::allianceStation)
        .add("Placement", () -> container.placement().describe());
  }

  /** Pitwall, for tests that drive the robot program through it. */
  Pitwall<RobotConfig> pitwall() {
    return pitwall;
  }

  /**
   * Every loop, in every mode: the Pitwall page's requests, the mechanisms' sensors, then the
   * command scheduler.
   */
  @Override
  public void robotPeriodic() {
    pitwall.periodic();
    container.periodic();
    Scheduler.getDefault().run();
  }

  /**
   * {@code ./gradlew run} starts the simulated robot enabled in teleop, on the blue alliance's
   * station 2 unless the simulator GUI already chose one, so a driver can drive right away instead
   * of hunting through the GUI first. The GUI can still disable it or change the station, and
   * {@code ./gradlew run -PstartDisabled} starts disabled with no station, like a real robot before
   * its Driver Station connects. Tests and replay never set the property.
   */
  @Override
  public void simulationInit() {
    startEnabledIfRequested(Constants.currentMode, Boolean.getBoolean(AUTO_ENABLE_PROPERTY));
  }

  /** Enable the simulated robot in teleop, only in a physics simulation and only when asked. */
  static void startEnabledIfRequested(Constants.Mode mode, boolean requested) {
    if (mode == Constants.Mode.SIM && requested) {
      if (MatchState.getAlliance().isEmpty()) {
        // The center start, and blue's forward is the field view's right: the easiest first drive.
        DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_2);
      }
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
    }
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void utilityPeriodic() {}

  @Override
  public void simulationPeriodic() {}
}
