package frc.robot.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.RobotSim;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
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
    GyroIOSystemcore flat = new GyroIOSystemcore(OnboardIMU.MountOrientation.FLAT, Rotation3d.ZERO);
    GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    flat.updateInputs(inputs);
    SimHooks.stepTiming(0.02);
    flat.updateInputs(inputs);

    assertThat(inputs.connected).isTrue();
    assertThat(inputs.odometryTimestampsNanos).hasSize(1);
    assertThat(inputs.odometryYawsRad).containsExactly(inputs.yaw.getRadians());
    assertThat(inputs.yawVelocityRadPerSec).isFinite();
  }

  /**
   * The board reads along its own axes, which its orientation on the robot turns into the robot's.
   * Turned a quarter turn left, flat, it reads the robot's forward on its -y axis; on its long
   * edge, its x axis up and z axis backward, it reads the robot's forward on its -z axis and the
   * floor holding it up on its x axis.
   */
  @Test
  void theBoardsAccelerationIsTurnedToTheRobotsAxesByItsOrientation() {
    Rotation3d turnedLeft = new Rotation3d(0, 0, Math.PI / 2);
    Rotation3d onEdge = new Rotation3d(0, -Math.PI / 2, 0);

    Translation3d forward = GyroIOSystemcore.robotAxes(turnedLeft, 0, -1, 0);
    Translation3d upright = GyroIOSystemcore.robotAxes(onEdge, 9.81, 0, -1);

    assertThat(forward.getDistance(new Translation3d(1, 0, 0))).isLessThan(1e-12);
    assertThat(upright.getDistance(new Translation3d(1, 0, 9.81))).isLessThan(1e-12);
  }
}
