package frc.robot.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.RobotSim;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.simulation.SimHooks;

/**
 * Systemcore's onboard IMU, off a robot: WPILib's simulation doesn't model it (it reads zero), so
 * this checks only that the IO reads it as the drivetrain expects, one reading a loop.
 */
@ExtendWith(RobotSim.class)
class GyroIOSystemcoreTest {
  @Test
  void itReportsOneReadingALoopAndCountsAsConnected() {
    RobotSim.setEnabled(false);
    GyroIOSystemcore flat = new GyroIOSystemcore(OnboardIMU.MountOrientation.FLAT);
    GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    flat.updateInputs(inputs);
    SimHooks.stepTiming(0.02);
    flat.updateInputs(inputs);

    assertThat(inputs.connected).isTrue();
    assertThat(inputs.odometryTimestampsNanos).hasSize(1);
    assertThat(inputs.odometryYawsRad).containsExactly(inputs.yaw.getRadians());
    assertThat(inputs.yawVelocityRadPerSec).isFinite();
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
