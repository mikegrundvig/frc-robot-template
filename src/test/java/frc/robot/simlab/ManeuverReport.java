package frc.robot.simlab;

import org.wpilib.math.geometry.Pose2d;

/** What happened during one maneuver run. */
public record ManeuverReport(
    String name,
    double minBatteryVolts,
    double maxBatteryVolts,
    double peakTotalSupplyAmps,
    double peakDriveStatorAmps,
    double maxSlipSpeedMps,
    double worstSlipFraction,
    double minNormalForceNewtons,
    double maxSpeedMps,
    double timeToThreeMpsSec,
    double odometryErrorMeters,
    double headingErrorDeg,
    Pose2d finalTruePose) {
  @Override
  public String toString() {
    return String.format(
        "%n=== Maneuver: %s ===%n"
            + "  battery      min %.2f V   max %.2f V%n"
            + "  current      peak total supply %.0f A   peak drive stator %.0f A%n"
            + "  traction     worst wheel slipping %.1f%% of the time   max slip %.2f m/s%n"
            + "  weight       min wheel normal force %.0f N%n"
            + "  speed        max %.2f m/s   0 to 3 m/s in %.2f s%n"
            + "  odometry     error %.3f m   heading error %.2f deg%n",
        name,
        minBatteryVolts,
        maxBatteryVolts,
        peakTotalSupplyAmps,
        peakDriveStatorAmps,
        worstSlipFraction * 100,
        maxSlipSpeedMps,
        minNormalForceNewtons,
        maxSpeedMps,
        timeToThreeMpsSec,
        odometryErrorMeters,
        headingErrorDeg);
  }
}
