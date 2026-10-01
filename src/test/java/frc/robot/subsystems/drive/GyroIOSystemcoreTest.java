package frc.robot.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;

import frc.robot.RobotSim;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Translation2d;
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
    GyroIOSystemcore flat = new GyroIOSystemcore(OnboardIMU.MountOrientation.FLAT, Degrees.of(0));
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
   * On its edge, its accelerometer's axes aren't the robot's: it says it can't tell, rather than
   * that the robot isn't accelerating, which the slip check would take the wheels' every change of
   * speed against.
   */
  @Test
  void onEdgeItSaysItCantTellTheAcceleration() {
    GyroIOSystemcore onEdge =
        new GyroIOSystemcore(OnboardIMU.MountOrientation.PORTRAIT, Degrees.of(0));
    GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    onEdge.updateInputs(inputs);

    assertThat(inputs.accelerationXMetersPerSecSq).isNaN();
    assertThat(inputs.accelerationYMetersPerSecSq).isNaN();
  }

  /**
   * Turned a quarter turn left on the robot, the board reads the robot's forward on its -y axis.
   */
  @Test
  void aTurnedBoardsAccelerationIsTurnedBackToTheRobotsAxes() {
    Translation2d forward = GyroIOSystemcore.robotAxes(Math.PI / 2, 0, -1);

    assertThat(forward.getX()).isCloseTo(1, within(1e-12));
    assertThat(forward.getY()).isCloseTo(0, within(1e-12));
  }
}
