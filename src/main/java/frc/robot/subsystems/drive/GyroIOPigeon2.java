package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.lib.timing.RobotLoop;
import java.util.Queue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;

/**
 * The drivetrain's gyro on a CTRE Pigeon 2: yaw at 250 Hz for odometry, and the attitude and
 * acceleration once a loop (acceleration twice, since a collision's jolt is brief).
 *
 * <p>The Pigeon 2's axes are WPILib's (+x forward, +y left, +z up; yaw counterclockwise, pitch nose
 * down), so its angles pass through as they are. Its yaw, pitch, and roll follow the mount pose set
 * in Phoenix Tuner X; its accelerometer doesn't (CTRE's Pigeon 2.0 User's Guide, "Enclosure
 * Orientated Signals": it reads along the case's own axes). So this reads the mount pose saved on
 * the Pigeon at startup and turns the accelerometer's axes into the robot's with it. Set the mount
 * pose in Phoenix Tuner X whichever way the Pigeon is mounted, then check the accelerometer as
 * docs/calibration.md says before localization trusts it. Its accelerometer reports up to ±2 g on
 * each axis.
 */
final class GyroIOPigeon2 implements GyroIO {
  private static final double ACCELERATION_HZ = 100;

  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;
  private final StatusSignal<LinearAcceleration> accelerationZ;

  /** How the Pigeon sits on the robot: turns its case's axes into the robot's. */
  private final Rotation3d mount;

  private final double radiansPerYawUnit;
  private final double yawScale;
  private final Queue<Long> odometryTimes;
  private final Queue<Double> odometryYaws;

  /** Disconnected once silent for half a second, not at one glitch. */
  private final Debouncer connected = new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  /**
   * @param yawScale the gyro's measured scale: its readings times this are the true rotation
   */
  GyroIOPigeon2(int id, CANBus bus, double yawScale, PhoenixOdometryThread odometry) {
    this.yawScale = yawScale;
    Pigeon2 pigeon = new Pigeon2(id, bus);
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    accelerationX = pigeon.getAccelerationX();
    accelerationY = pigeon.getAccelerationY();
    accelerationZ = pigeon.getAccelerationZ();
    MountPoseConfigs pose = new MountPoseConfigs();
    // Unread (the Pigeon not answering at startup), the case's axes are taken for the robot's:
    // the accelerometer check shows whether they are.
    mount = pigeon.getConfigurator().refresh(pose).isOK() ? mount(pose) : Rotation3d.ZERO;
    // The odometry thread records raw values; this turns the yaw's unit (degrees) into radians.
    radiansPerYawUnit = yaw.getValue().unit().toBaseUnits(1);

    yaw.setUpdateFrequency(PhoenixOdometryThread.FREQUENCY_HZ);
    BaseStatusSignal.setUpdateFrequencyForAll(RobotLoop.FREQUENCY_HZ, yawVelocity, pitch, roll);
    BaseStatusSignal.setUpdateFrequencyForAll(
        ACCELERATION_HZ, accelerationX, accelerationY, accelerationZ);
    pigeon.optimizeBusUtilization();

    // The thread reads its own copy of the yaw, so it never shares one with this loop.
    odometryTimes = odometry.timestamps();
    odometryYaws = odometry.register(yaw.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    boolean ok =
        BaseStatusSignal.refreshAll(
                yaw, yawVelocity, pitch, roll, accelerationX, accelerationY, accelerationZ)
            .isOK();
    inputs.connected = connected.calculate(ok);
    inputs.yaw = new Rotation2d(yawScale * yaw.getValue().in(Radians));
    inputs.yawVelocityRadPerSec = yawScale * yawVelocity.getValue().in(RadiansPerSecond);
    inputs.pitchRad = pitch.getValue().in(Radians);
    inputs.rollRad = roll.getValue().in(Radians);
    Translation3d acceleration =
        robotAxes(
            mount,
            accelerationX.getValue().in(MetersPerSecondPerSecond),
            accelerationY.getValue().in(MetersPerSecondPerSecond),
            accelerationZ.getValue().in(MetersPerSecondPerSecond));
    inputs.accelerationXMetersPerSecSq = acceleration.getX();
    inputs.accelerationYMetersPerSecSq = acceleration.getY();

    // The drivetrain holds the odometry thread's lock while reading, so these are in step with the
    // modules' readings.
    int readings = Math.min(odometryTimes.size(), odometryYaws.size());
    long[] times = new long[readings];
    double[] yaws = new double[readings];
    for (int k = 0; k < readings; k++) {
      Long time = odometryTimes.poll();
      Double reading = odometryYaws.poll();
      if (time == null || reading == null) {
        break; // cannot happen while the lock is held
      }
      times[k] = time;
      yaws[k] = yawScale * reading * radiansPerYawUnit;
    }
    inputs.odometryTimestampsNanos = times;
    inputs.odometryYawsRad = yaws;
  }

  /** How the Pigeon sits on the robot, as read from its mount pose at startup. */
  Rotation3d mount() {
    return mount;
  }

  /**
   * The Pigeon's mount pose as a rotation: how its case is turned on the robot, its yaw, pitch, and
   * roll applied in that order about its own axes, so it turns the case's axes into the robot's.
   */
  static Rotation3d mount(MountPoseConfigs pose) {
    return new Rotation3d(
        Math.toRadians(pose.MountPoseRoll),
        Math.toRadians(pose.MountPosePitch),
        Math.toRadians(pose.MountPoseYaw));
  }

  /** A vector the case reads, along the robot's axes. */
  static Translation3d robotAxes(Rotation3d mount, double x, double y, double z) {
    return new Translation3d(x, y, z).rotateBy(mount);
  }
}
