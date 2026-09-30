package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.lib.timing.RobotLoop;
import java.util.Queue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;

/**
 * The drivetrain's gyro on a CTRE Pigeon 2: yaw at 250 Hz for odometry, and the attitude and
 * acceleration once a loop (acceleration twice, since a collision's jolt is brief).
 *
 * <p>The Pigeon 2's axes are WPILib's (+x forward, +y left, +z up; yaw counterclockwise, pitch nose
 * down), so its angles pass through as they are. Its yaw, pitch, and roll follow the mount
 * calibration done in Phoenix Tuner X; its accelerometer doesn't, so mount it flat (logo up or
 * down), where the size of a sideways jolt reads the same whichever way it faces.
 */
final class GyroIOPigeon2 implements GyroIO {
  private static final double ACCELERATION_HZ = 100;

  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;
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
    // The odometry thread records raw values; this turns the yaw's unit (degrees) into radians.
    radiansPerYawUnit = yaw.getValue().unit().toBaseUnits(1);

    yaw.setUpdateFrequency(PhoenixOdometryThread.FREQUENCY_HZ);
    BaseStatusSignal.setUpdateFrequencyForAll(RobotLoop.FREQUENCY_HZ, yawVelocity, pitch, roll);
    BaseStatusSignal.setUpdateFrequencyForAll(ACCELERATION_HZ, accelerationX, accelerationY);
    pigeon.optimizeBusUtilization();

    // The thread reads its own copy of the yaw, so it never shares one with this loop.
    odometryTimes = odometry.timestamps();
    odometryYaws = odometry.register(yaw.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    boolean ok =
        BaseStatusSignal.refreshAll(yaw, yawVelocity, pitch, roll, accelerationX, accelerationY)
            .isOK();
    inputs.connected = connected.calculate(ok);
    inputs.yaw = new Rotation2d(yawScale * yaw.getValue().in(Radians));
    inputs.yawVelocityRadPerSec = yawScale * yawVelocity.getValue().in(RadiansPerSecond);
    inputs.pitchRad = pitch.getValue().in(Radians);
    inputs.rollRad = roll.getValue().in(Radians);
    inputs.accelerationXMetersPerSecSq = accelerationX.getValue().in(MetersPerSecondPerSecond);
    inputs.accelerationYMetersPerSecSq = accelerationY.getValue().in(MetersPerSecondPerSecond);

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
}
