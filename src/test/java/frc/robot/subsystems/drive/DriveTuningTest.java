package frc.robot.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Amps;

import frc.robot.Robots;
import frc.robot.SimTestSupport;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.config.SwerveConfigBuilder;
import frc.robot.lib.config.SwerveConfigGainsBuilder;
import frc.robot.lib.config.SwerveConfigLimitsBuilder;
import java.util.concurrent.atomic.AtomicReference;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;

/** Values tuned while the robot runs reach every module's motor controllers. */
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

  @BeforeAll
  static void initHal() {
    SimTestSupport.initHal();
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
            new GyroIO() {},
            modules,
            Robots.SIM.body(),
            config::get,
            Scheduler.createIndependentScheduler())) {
      config.set(tuned);
      drive.periodic(false);
    }

    for (RecordingModuleIO module : modules) {
      assertThat(module.gains).isEqualTo(tuned.gains());
      assertThat(module.limits).isEqualTo(tuned.limits());
    }
  }
}
