package frc.robot.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.SimTestSupport;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.simulation.SimHooks;

/**
 * Systemcore's onboard IMU, off a robot: WPILib's simulation doesn't model it (it reads zero), so
 * this checks only that the IO reads it as the drivetrain expects, one reading a loop.
 */
class GyroIOSystemcoreTest {
  @BeforeAll
  static void initHal() {
    SimTestSupport.initHal();
  }

  @Test
  void itReportsOneReadingALoopAndCountsAsConnected() {
    SimTestSupport.pauseTimeAndDisable();
    GyroIOSystemcore flat = new GyroIOSystemcore(OnboardIMU.MountOrientation.FLAT);
    GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    flat.updateInputs(inputs);
    SimHooks.stepTiming(0.02);
    flat.updateInputs(inputs);

    assertThat(inputs.connected).isTrue();
    assertThat(inputs.odometryTimestampsNanos).hasSize(1);
    assertThat(inputs.odometryYawsRad).containsExactly(inputs.yaw.getRadians());
    assertThat(inputs.yawVelocityRadPerSec).isFinite();
    SimTestSupport.resumeTime();
  }

  @Test
  void onEdgeItReportsNoAcceleration() {
    GyroIOSystemcore onEdge = new GyroIOSystemcore(OnboardIMU.MountOrientation.PORTRAIT);
    GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    onEdge.updateInputs(inputs);

    assertThat(inputs.accelerationXMetersPerSecSq).isZero();
    assertThat(inputs.accelerationYMetersPerSecSq).isZero();
  }
}
