// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import frc.robot.tuning.Tuning;
import frc.robot.tuning.TuningServer;
import java.util.List;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.util.Alert;

/** The robot program: logging setup, then the command scheduler every loop. */
public class Robot extends LoggedRobot {
  /** Set by {@code ./gradlew run}: start the simulator enabled in teleop. */
  static final String AUTO_ENABLE_PROPERTY = "frc.sim.autoEnable";

  /**
   * The robot's configuration, with any values tuned from the tuning page. One robot for now; with
   * several, choose its configuration by the robot's identity here.
   */
  private final Tuning tuning;

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

    // Tuning starts switched on in the simulator; on a robot, switch it on from the tuning page.
    // While any tuned value differs from the committed one, a warning says which.
    Alert tuned = new Alert("Tuning", "", Alert.Level.MEDIUM);
    tuning =
        new Tuning(
            Robots.SIM,
            Constants.currentMode == Constants.Mode.SIM,
            inputs -> Logger.processInputs("Tuning", inputs),
            view -> {
              tuned.setText(
                  "Tuned values differ from the committed configuration: " + view.changedPaths());
              tuned.set(view.hasChanges());
            });
    if (Constants.currentMode != Constants.Mode.REPLAY) {
      TuningServer.startOrExplain(tuning, TuningServer.PORT, System.out::println);
    }
  }

  /** Every loop, in every mode: apply tuning requests, then run the command scheduler. */
  @Override
  public void robotPeriodic() {
    tuning.periodic(RobotState.isFMSAttached());
    Scheduler.getDefault().run();
  }

  /** Every time the robot disables, print and log what tuning changed, so it can be committed. */
  @Override
  public void disabledInit() {
    List<String> report = tuning.report();
    report.forEach(System.out::println);
    Logger.recordOutput("Tuning/Report", report.toArray(String[]::new));
  }

  /**
   * {@code ./gradlew run} starts the simulated robot enabled in teleop, so a driver can drive right
   * away instead of hunting through the simulator GUI first. The GUI can still disable it, and
   * {@code ./gradlew run -PstartDisabled} starts disabled like a real robot. Tests and replay never
   * set the property.
   */
  @Override
  public void simulationInit() {
    startEnabledIfRequested(Constants.currentMode, Boolean.getBoolean(AUTO_ENABLE_PROPERTY));
  }

  /** Enable the simulated robot in teleop, only in a physics simulation and only when asked. */
  static void startEnabledIfRequested(Constants.Mode mode, boolean requested) {
    if (mode == Constants.Mode.SIM && requested) {
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
