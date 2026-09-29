package frc.robot.simlab;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import frc.robot.RobotConfig;
import frc.robot.lib.config.Body;
import frc.robot.lib.config.BodyBuilder;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.config.SwerveConfigBuilder;
import frc.robot.lib.config.SwerveConfigLimitsBuilder;
import frc.robot.lib.config.SwerveConfigModulesBuilder;
import frc.robot.lib.swerve.SwerveSim;
import java.util.ArrayList;
import java.util.List;

/**
 * One version of the world a robot might face: its body and drivetrain, plus the battery it drives
 * on.
 *
 * <p>Robust tuning runs every candidate against a spread of scenarios covering what we do not know
 * precisely (tread wear, battery health, final mass) and judges it by its worst case. Optimizing
 * against a single nominal simulation finds values tuned to the model's mistakes.
 */
public record DriveScenario(
    String name, Body body, SwerveConfig drivetrain, SwerveSim.Battery battery) {

  /** The robot as configured, on a healthy battery. */
  public static DriveScenario nominal(RobotConfig robot) {
    return new DriveScenario(
        "nominal", robot.body(), robot.drivetrain(), SwerveSim.Battery.TYPICAL);
  }

  /**
   * The robustness spread around a robot: worn to fresh tread, tired to strong battery, mass 5 kg
   * either side, and the center of gravity as designed or 0.15 m higher (mechanisms raise it as the
   * robot is built). 16 scenarios.
   */
  public static List<DriveScenario> robustnessSpread(RobotConfig robot) {
    List<DriveScenario> scenarios = new ArrayList<>();
    double[] frictions = {1.0, 1.3}; // worn or dusty tread, and fresh tread on clean carpet
    SwerveSim.Battery[] batteries = {
      new SwerveSim.Battery(12.2, 0.030, 3.0), // older battery, partly discharged
      new SwerveSim.Battery(12.9, 0.015, 3.0) // fresh, fully charged
    };
    String[] batteryNames = {"tired", "strong"};
    double[] massChanges = {-5.0, 5.0};
    double[] cgRaises = {0.0, 0.15};
    for (double mu : frictions) {
      for (int b = 0; b < batteries.length; b++) {
        for (double dm : massChanges) {
          for (double dcg : cgRaises) {
            scenarios.add(
                new DriveScenario(
                    String.format("mu%.1f-%s-%+.0fkg-cg%+.2fm", mu, batteryNames[b], dm, dcg),
                    heavierAndTaller(robot.body(), dm, dcg),
                    withFriction(robot.drivetrain(), mu),
                    batteries[b]));
          }
        }
      }
    }
    return scenarios;
  }

  /** The same scenario with a different drive stator current limit. */
  public DriveScenario withDriveStatorLimit(double amps) {
    return new DriveScenario(name, body, withDriveStatorLimit(drivetrain, amps), battery);
  }

  /** A drivetrain with a different drive stator current limit. */
  public static SwerveConfig withDriveStatorLimit(SwerveConfig drivetrain, double amps) {
    return SwerveConfigBuilder.builder(drivetrain)
        .limits(
            SwerveConfigLimitsBuilder.builder(drivetrain.limits())
                .driveStatorCurrent(Amps.of(amps))
                .build())
        .build();
  }

  private static Body heavierAndTaller(Body body, double massChangeKg, double cgRaiseMeters) {
    return BodyBuilder.builder(body)
        .mass(Kilograms.of(body.mass().in(Kilograms) + massChangeKg))
        .cgHeight(Meters.of(body.cgHeight().in(Meters) + cgRaiseMeters))
        .build();
  }

  private static SwerveConfig withFriction(SwerveConfig drivetrain, double mu) {
    return SwerveConfigBuilder.builder(drivetrain)
        .modules(
            SwerveConfigModulesBuilder.builder(drivetrain.modules())
                .wheelCoefficientOfFriction(mu)
                .build())
        .build();
  }
}
