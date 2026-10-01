package frc.robot.subsystems.localization;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.RobotConfig;
import frc.robot.RobotConfigBuilder;
import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.SimulationTest;
import frc.robot.lib.localization.ImuPlacement;
import frc.robot.lib.localization.ImuPlacementBuilder;
import frc.robot.sim.DriveSimHarness;
import frc.robot.sim.SensorErrors;
import frc.robot.sim.SensorErrorsAccelErrorsBuilder;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.Units;

/**
 * Where the IMU sits and how its accelerometer lies, against what localization believes: driving
 * that never slips must never look like slipping. The gyro's accelerometer reads as a Pigeon 2's
 * does on the simulator's season-worn robot ({@link SensorErrors.AccelErrors#PIGEON_2}), turned or
 * moved where a test says; the robot's configuration says where it sits and whether it has been
 * checked ({@link ImuPlacement}).
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class ImuPlacementSimTest {
  /**
   * Driving the wheels grip through: launches from rest at full stick, stops, a spin in place at a
   * turn a second, and driving while spinning, each from standing.
   */
  private static final double[][] CLEAN = {
    {1.5, 1, 0, 0}, {1, 0, 0, 0},
    {1.5, 0, 1, 0}, {1, 0, 0, 0},
    {2, 0, 0, 1}, {1, 0, 0, 0},
    {2, 0.7, 0, 1}, {1, 0, 0, 0}
  };

  /** What the slip check said over a drive. */
  private record Verdict(int slipping, int collisions) {}

  private static Verdict drive(ImuPlacement believed, SensorErrors.AccelErrors accelerometer) {
    RobotConfig robot = RobotConfigBuilder.builder(Robots.SIM).imus(believed).build();
    try (DriveSimHarness harness =
        new DriveSimHarness(robot, SensorErrors.TYPICAL.withAccelerometer(accelerometer))) {
      int[] counts = new int[2];
      harness.eachLoop(
          () -> {
            counts[0] += harness.localization.conditions().slipMetersPerSecond() > 0 ? 1 : 0;
            counts[1] += harness.localization.conditions().collision() ? 1 : 0;
          });
      for (double[] leg : CLEAN) {
        harness.drive(leg[0], leg[1], leg[2], leg[3]);
      }
      return new Verdict(counts[0], counts[1]);
    }
  }

  private static ImuPlacement at(Translation2d place, boolean checked) {
    return ImuPlacementBuilder.builder(ImuPlacement.UNCHECKED)
        .gyroForward(Units.Meters.of(place.getX()))
        .gyroLeft(Units.Meters.of(place.getY()))
        .accelerometerChecked(checked)
        .build();
  }

  private static SensorErrors.AccelErrors pigeon() {
    return SensorErrors.AccelErrors.PIGEON_2;
  }

  /** A Pigeon 2's accelerometer, with its noise, bias, range and sampling, sees no slip or hit. */
  @Test
  void aPigeonsAccelerometerSeesNoSlipOrHitInCleanDriving() {
    assertThat(drive(at(Translation2d.ZERO, true), pigeon())).isEqualTo(new Verdict(0, 0));
  }

  /**
   * Turned a quarter turn on its mount, or mounted logo down, an accelerometer reads speeding up as
   * a sideways slide. Until the robot passes the accelerometer check, its speeds aren't compared,
   * and a jolt counts by its size alone.
   */
  @Test
  void anUncheckedAccelerometerTurnedOrFlippedSeesNoSlip() {
    ImuPlacement unchecked = at(Translation2d.ZERO, false);
    SensorErrors.AccelErrors turned =
        SensorErrorsAccelErrorsBuilder.builder(pigeon()).turnedDegrees(90).build();
    SensorErrors.AccelErrors flipped =
        SensorErrorsAccelErrorsBuilder.builder(pigeon()).flipped(true).build();

    assertThat(drive(unchecked, turned)).isEqualTo(new Verdict(0, 0));
    assertThat(drive(unchecked, flipped)).isEqualTo(new Verdict(0, 0));
    assertThat(drive(at(Translation2d.ZERO, true), turned).slipping())
        .as("trusted as checked, the same accelerometer sees slip")
        .isPositive();
  }

  /**
   * An IMU 20 cm ahead of the center and 5 cm left feels the robot's turns swing it around. Where
   * the configuration places it, that's no slip; believed at the center, it would be.
   */
  @Test
  void anImuOffTheCenterWhereTheConfigurationPlacesItSeesNoSlip() {
    Translation2d place = new Translation2d(0.2, 0.05);
    SensorErrors.AccelErrors offCenter =
        SensorErrorsAccelErrorsBuilder.builder(pigeon())
            .forwardMeters(place.getX())
            .leftMeters(place.getY())
            .build();

    Verdict placed = drive(at(place, true), pigeon());
    Verdict centered = drive(at(Translation2d.ZERO, true), offCenter);

    assertThat(placed).isEqualTo(new Verdict(0, 0));
    assertThat(centered.slipping()).as("believed at the center").isPositive();
  }
}
