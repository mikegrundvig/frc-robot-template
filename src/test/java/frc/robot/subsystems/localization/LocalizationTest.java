package frc.robot.subsystems.localization;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.lib.localization.DrivetrainReadings;
import frc.robot.lib.localization.ImuReadings;
import frc.robot.lib.localization.OdometrySample;
import frc.robot.lib.localization.YawSource;
import frc.robot.sim.DriveSimHarness;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Localization fed by hand: what it makes of the drivetrain's readings, one loop at a time. */
@ExtendWith(RobotSim.class)
class LocalizationTest {
  private static final ImuReadings LEVEL =
      ImuReadings.builder()
          .reporting(true)
          .yawRadians(0.2)
          .pitchRadians(0.05)
          .rollRadians(0)
          .forwardAccelerationMetersPerSecSq(0)
          .leftAccelerationMetersPerSecSq(0)
          .yawTimesNanos(List.of(1_000_000L))
          .yawsRadians(List.of(0.2))
          .build();

  private static final ImuReadings SENDING_NAN =
      ImuReadings.builder()
          .reporting(true)
          .yawRadians(Double.NaN)
          .pitchRadians(Double.NaN)
          .rollRadians(Double.NaN)
          .forwardAccelerationMetersPerSecSq(Double.NaN)
          .leftAccelerationMetersPerSecSq(Double.NaN)
          .yawTimesNanos(List.of(1_000_000L))
          .yawsRadians(List.of(Double.NaN))
          .build();

  private static DrivetrainReadings readings(ImuReadings gyro, ImuReadings backup) {
    return DrivetrainReadings.builder()
        .wheels(
            List.of(
                new OdometrySample(
                    1_000_000L, new double[] {0, 0, 0, 0}, new double[] {0, 0, 0, 0})))
        .gyro(gyro)
        .backupImu(backup)
        .velocities(new ChassisVelocities())
        .tractionLimited(false)
        .build();
  }

  /** One bad reading from an IMU costs its readings, never the robot program. */
  @Test
  void aGyroSendingNaNCountsAsLostAndTheBackupStandsIn() {
    try (Localization localization =
        DriveSimHarness.localizationFor(Robots.SIM, DriveSimHarness.estimatorAt(Pose2d.ZERO))) {
      localization.periodic(readings(SENDING_NAN, ImuReadings.NONE), true);
      localization.update();

      assertThat(localization.isGyroDisconnectedAlertActive()).isTrue();
      assertThat(localization.yawSource()).isEqualTo(YawSource.WHEELS);
      assertThat(localization.pitchRadians()).as("no IMU, no tilt").isZero();
      assertThat(localization.pose().getX()).isFinite();

      localization.periodic(readings(SENDING_NAN, LEVEL), true);
      localization.update();

      assertThat(localization.isGyroDisconnectedAlertActive()).isFalse();
      assertThat(localization.isUsingBackupAlertActive()).isTrue();
      assertThat(localization.pitchRadians()).as("the backup's attitude").isEqualTo(0.05);
    }
  }
}
