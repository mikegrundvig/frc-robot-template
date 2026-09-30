package frc.robot.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Amps;

import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.config.SwerveConfigBuilder;
import frc.robot.lib.config.SwerveConfigGainsBuilder;
import frc.robot.lib.config.SwerveConfigLimitsBuilder;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.ReentrantLock;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;

/** Values tuned while the robot runs reach every module's motor controllers. */
@ExtendWith(RobotSim.class)
class DriveTuningTest {
  /** A module that remembers what its motor controllers were last given. */
  private static final class RecordingModuleIO implements ModuleIO {
    SwerveConfig.@Nullable Gains gains;
    SwerveConfig.@Nullable Limits limits;

    @Override
    public void configure(SwerveConfig.Gains gains, SwerveConfig.Limits limits) {
      this.gains = gains;
      this.limits = limits;
    }
  }

  @Test
  void tunedGainsAndLimitsReachEveryModule() {
    SwerveConfig committed = Robots.SIM.drivetrain();
    SwerveConfig tuned =
        SwerveConfigBuilder.builder(committed)
            .limits(
                SwerveConfigLimitsBuilder.builder(committed.limits())
                    .driveStatorCurrent(Amps.of(15))
                    .build())
            .gains(SwerveConfigGainsBuilder.builder(committed.gains()).driveKp(3.0).build())
            .build();
    var config = new AtomicReference<>(committed);
    RecordingModuleIO[] modules = {
      new RecordingModuleIO(),
      new RecordingModuleIO(),
      new RecordingModuleIO(),
      new RecordingModuleIO()
    };

    try (Drive drive =
        new Drive(
            "TuningTest",
            DriveHardware.builder()
                .modules(List.of(modules))
                .gyro(new GyroIO() {})
                .backupImu(new GyroIO() {})
                .odometryLock(new ReentrantLock())
                .build(),
            Robots.SIM.body(),
            config::get,
            Scheduler.createIndependentScheduler(),
            () -> Pose2d.ZERO)) {
      config.set(tuned);
      drive.periodic(false);
    }

    for (RecordingModuleIO module : modules) {
      assertThat(module.gains).isEqualTo(tuned.gains());
      assertThat(module.limits).isEqualTo(tuned.limits());
    }
  }
}
