package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
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
 *
 * <p>Phoenix's own signal log (a {@code .hoot} file) is off: on Systemcore it would otherwise start
 * in every session, a second writer on the USB drive beside the AdvantageKit log, which is the
 * record and the one replay reads. A routine that needs a {@code .hoot} file, such as a
 * characterization in Phoenix Tuner X, calls {@code SignalLogger.start()} and {@code stop()} around
 * its run.
 */
public final class PhoenixDrivetrain implements AutoCloseable {
  /**
   * How often each module's CANcoder sends its position and velocity, in Hz: Phoenix 6's default,
   * set explicitly so nothing else slows it. Without a Phoenix Pro license, the steering motor
   * closes its loop on these, taken remotely, so however fast its loop runs, it sees a new angle
   * only this often, and that wait sets how high the steering's gains can go. Faster allows higher
   * gains, for more traffic on the CAN bus: measure the bus's load before raising it. The
   * simulator's steering hears its CANcoder at this rate too.
   */
  public static final double ENCODER_FREQUENCY_HZ = 100;

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
    SignalLogger.enableAutoLogging(false);
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
