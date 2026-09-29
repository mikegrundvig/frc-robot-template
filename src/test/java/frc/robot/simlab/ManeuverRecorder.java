package frc.robot.simlab;

import frc.robot.lib.swerve.SwerveSim;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.jspecify.annotations.Nullable;
import org.wpilib.datalog.DataLogWriter;
import org.wpilib.datalog.DoubleArrayLogEntry;
import org.wpilib.datalog.DoubleLogEntry;
import org.wpilib.datalog.StructArrayLogEntry;
import org.wpilib.datalog.StructLogEntry;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * Collects the metrics for one maneuver run and, optionally, writes a {@code .wpilog} to open in
 * AdvantageScope (true pose vs. odometry, module states, battery, per-wheel slip and load).
 */
public final class ManeuverRecorder implements AutoCloseable {
  private final SwerveSim sim;
  private final Drive drive;
  private final Maneuver maneuver;
  private final double startTimeSec;
  private double timeToThreeMpsSec = Double.NaN;

  /** Where one run is logged, when it is. */
  private static final class Log {
    final DataLogWriter writer;
    final StructLogEntry<Pose2d> truePose;
    final StructLogEntry<Pose2d> odometryPose;
    final StructArrayLogEntry<SwerveModuleVelocity> moduleStates;
    final DoubleLogEntry battery;
    final DoubleLogEntry supply;
    final DoubleArrayLogEntry slip;
    final DoubleArrayLogEntry normal;
    final DoubleArrayLogEntry stator;
    final DoubleArrayLogEntry sticks;

    Log(Path path) {
      try {
        Files.createDirectories(path.toAbsolutePath().getParent());
        writer = new DataLogWriter(path.toString());
      } catch (IOException e) {
        throw new UncheckedIOException(e);
      }
      truePose = StructLogEntry.create(writer, "/Truth/Pose", Pose2d.struct);
      odometryPose = StructLogEntry.create(writer, "/Odometry/Pose", Pose2d.struct);
      moduleStates =
          StructArrayLogEntry.create(writer, "/Drive/ModuleStates", SwerveModuleVelocity.struct);
      battery = new DoubleLogEntry(writer, "/Battery/Volts");
      supply = new DoubleLogEntry(writer, "/Battery/TotalSupplyAmps");
      slip = new DoubleArrayLogEntry(writer, "/Wheels/SlipSpeedMps");
      normal = new DoubleArrayLogEntry(writer, "/Wheels/NormalForceNewtons");
      stator = new DoubleArrayLogEntry(writer, "/Wheels/DriveStatorAmps");
      sticks = new DoubleArrayLogEntry(writer, "/Driver/Sticks");
    }
  }

  private final @Nullable Log log;

  /**
   * @param logPath where to write the log, or null for no log (faster, for large studies)
   */
  public ManeuverRecorder(
      SwerveSim sim, Drive drive, Maneuver maneuver, double stepSec, @Nullable Path logPath) {
    this.sim = sim;
    this.drive = drive;
    this.maneuver = maneuver;
    this.startTimeSec = maneuver.firstStickTimeSec(stepSec);
    sim.resetStats();
    log = logPath == null ? null : new Log(logPath);
  }

  /** Record the state after one loop. {@code timeSec} is the loop's start time. */
  public void sample(double timeSec, double loopSec, double[] driverSticks) {
    var velocity = sim.trueFieldVelocities();
    if (Double.isNaN(timeToThreeMpsSec) && Math.hypot(velocity.vx, velocity.vy) >= 3.0) {
      timeToThreeMpsSec = timeSec + loopSec - startTimeSec;
    }
    Log log = this.log;
    if (log == null) {
      return;
    }
    // Stamp at the end of the loop: this is the state after it ran. WPILib 2027's DataLog API takes
    // nanoseconds and stores microseconds in the file, which is what AdvantageScope reads. (Never
    // 0, which DataLog treats as "use the current time".)
    long nanos = (long) ((timeSec + loopSec) * 1e9);
    log.truePose.append(sim.truePose(), nanos);
    log.odometryPose.append(drive.getPose(), nanos);
    log.moduleStates.append(drive.measuredModuleVelocities(), nanos);
    log.battery.append(sim.batteryVolts(), nanos);
    log.supply.append(sim.totalSupplyAmps(), nanos);
    double[] slips = new double[4];
    double[] normals = new double[4];
    double[] stators = new double[4];
    for (int w = 0; w < 4; w++) {
      slips[w] = sim.wheelSlipSpeed(w);
      normals[w] = sim.wheelNormalForce(w);
      stators[w] = sim.driveStatorAmps(w);
    }
    log.slip.append(slips, nanos);
    log.normal.append(normals, nanos);
    log.stator.append(stators, nanos);
    log.sticks.append(driverSticks.clone(), nanos);
  }

  public ManeuverReport report() {
    Pose2d truth = sim.truePose();
    Pose2d odometry = drive.getPose();
    SwerveSim.Stats stats = sim.stats();
    return new ManeuverReport(
        maneuver.name(),
        stats.minBatteryVolts,
        stats.maxBatteryVolts,
        stats.peakTotalSupplyAmps,
        stats.peakDriveStatorAmps,
        stats.maxSlipSpeedMps,
        stats.worstSlipFraction(),
        stats.minNormalForceNewtons,
        stats.maxSpeedMps,
        timeToThreeMpsSec,
        truth.getTranslation().getDistance(odometry.getTranslation()),
        Math.abs(truth.getRotation().minus(odometry.getRotation()).getDegrees()),
        truth);
  }

  @Override
  public void close() {
    if (log != null) {
      log.writer.close();
    }
  }
}
