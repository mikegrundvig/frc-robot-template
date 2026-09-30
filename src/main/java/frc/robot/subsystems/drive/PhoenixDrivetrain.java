package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import frc.robot.lib.config.CanBus;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.config.SwerveWiring;
import java.util.concurrent.locks.Lock;

/**
 * The drivetrain's hardware on CTRE devices: four modules, each a TalonFX driving the wheel, a
 * TalonFX turning it, and a CANcoder reading its angle; a Pigeon 2; and the thread that reads them
 * at 250 Hz for odometry. Built on the robot from its wiring; the simulator and replay never touch
 * it.
 *
 * <p>Needs Phoenix 6 26.70 or later on the devices (firmware 26.70) for WPILib 2027 alpha 7.
 */
public final class PhoenixDrivetrain implements AutoCloseable {
  private final PhoenixOdometryThread odometry = new PhoenixOdometryThread();
  private final ModuleIO[] modules = new ModuleIO[4];
  private final GyroIO gyro;

  /**
   * Configures every device (waiting for each, at startup) and starts reading them.
   *
   * @param wiring where the devices are, and how the encoders read
   * @param hardware the modules' gearing and wheels
   */
  public PhoenixDrivetrain(SwerveWiring wiring, SwerveConfig.Modules hardware) {
    CANBus bus = bus(wiring.canBus());
    for (int i = 0; i < 4; i++) {
      modules[i] =
          new ModuleIOTalonFX(
              wiring.modules().get(i), bus, hardware, wiring.couplingRatio(), odometry);
    }
    gyro = new GyroIOPigeon2(wiring.gyroId(), bus, wiring.gyroYawScale(), odometry);
    odometry.start();
  }

  /** The modules: front left, front right, back left, back right. */
  public ModuleIO[] modules() {
    return modules.clone();
  }

  public GyroIO gyro() {
    return gyro;
  }

  /**
   * The lock the drivetrain holds while reading its IO, so the odometry thread's readings stay in
   * step across every device.
   */
  public Lock odometryLock() {
    return odometry.lock();
  }

  static CANBus bus(CanBus bus) {
    return switch (bus) {
      case CanBus.Port port -> new CANBus(port.port());
      case CanBus.Canivore canivore -> new CANBus(canivore.name());
    };
  }

  /** Stops reading the devices. */
  @Override
  public void close() {
    odometry.interrupt();
  }
}
