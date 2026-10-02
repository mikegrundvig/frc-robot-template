package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;

/**
 * Where an IMU sits on the robot, how it's turned, and whether its accelerometer has been checked
 * to read along the robot's axes. Localization compares the wheels with what the IMU feels where it
 * sits: an IMU off the robot's center is swung around it as the robot turns, and feels that too.
 * The drivetrain's IO turns the IMU's own axes into the robot's by its orientation, and the
 * simulator mounts its IMUs here.
 *
 * <p>Positions are from the robot's origin, the frame's center on the floor, as a camera's are.
 *
 * @param forward how far ahead of the center it sits (negative: behind it)
 * @param left how far left of the center (negative: to the right)
 * @param up how far above the floor
 * @param orientation how it's turned on the robot: the rotation that takes its own axes (the x, y
 *     and z printed on its case or board) into the robot's (forward, left, up). {@code
 *     Rotation3d.ZERO} for one mounted level with its x axis forward and z up; a quarter turn about
 *     z ({@code new Rotation3d(0, 0, Math.PI / 2)}) for one turned to face left; a half turn about
 *     x ({@code new Rotation3d(Math.PI, 0, 0)}) for one mounted upside down
 * @param accelerometerChecked whether this robot has passed the accelerometer check for this IMU
 *     (docs/calibration.md): driving forward reads forward, strafing left reads left, and level and
 *     still reads near zero. Until it has, the slip check compares the wheels' speed with its
 *     accelerometer by size, not direction (one turned on its mount would read every change of
 *     speed as a slide sideways, but just as large), and a jolt is judged by its size alone.
 *     Comparing the wheels' turn with the gyro's needs no check.
 */
@ConfigRecord
public record ImuMount(
    Distance forward,
    Distance left,
    Distance up,
    Rotation3d orientation,
    boolean accelerometerChecked) {
  /**
   * Where a new robot's IMUs start: at the center, on the floor, level with x forward, and the
   * accelerometer not yet checked. Measure where each really sits, and check it.
   */
  public static final ImuMount UNCHECKED =
      ImuMountBuilder.builder()
          .forward(Meters.of(0))
          .left(Meters.of(0))
          .up(Meters.of(0))
          .orientation(Rotation3d.ZERO)
          .accelerometerChecked(false)
          .build();

  public ImuMount {
    Checks.on(ImuMount.class)
        .field("forward", forward, finite())
        .field("left", left, finite())
        .field("up", up, finite())
        .check(
            "orientation",
            () ->
                Double.isFinite(orientation.getQuaternion().getW())
                        && Double.isFinite(orientation.getQuaternion().getX())
                        && Double.isFinite(orientation.getQuaternion().getY())
                        && Double.isFinite(orientation.getQuaternion().getZ())
                    ? null
                    : orientation + " must be a rotation, numbers all")
        .orThrow();
  }

  /** Starts describing an IMU's mount, one named value at a time. */
  public static ImuMountBuilder.ForwardStage builder() {
    return ImuMountBuilder.builder();
  }

  /** Where it sits, from the robot's origin. */
  public Translation3d position() {
    return new Translation3d(forward, left, up);
  }

  /**
   * Where it sits seen from above, from the robot's center: what the robot's turn swings around it.
   */
  public Translation2d place() {
    return new Translation2d(forward, left);
  }
}
