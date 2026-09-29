package frc.robot.lib.config;

import static org.wpilib.units.Units.Meters;

import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;

/**
 * The robot as a whole: its frame, bumpers, and mass. These live here rather than under the
 * drivetrain because simulation, path planning, and rule checks all read them.
 *
 * <p>Lengths run along the robot's x axis (front to back), widths along y (side to side). The
 * robot's origin is the center of the frame, on the floor.
 *
 * @param frameLength front to back, across the outside of the frame
 * @param frameWidth side to side, across the outside of the frame
 * @param height the tallest point, in the starting configuration
 * @param bumperThickness how far the bumpers stick out beyond the frame
 * @param mass everything that moves: robot, bumpers, and battery
 * @param yawMomentOfInertia how hard the robot is to spin, about the vertical axis through its
 *     center
 * @param cgHeight the center of gravity's height above the floor; it is taken to be centered front
 *     to back and side to side
 */
@ConfigRecord
public record Body(
    Distance frameLength,
    Distance frameWidth,
    Distance height,
    Distance bumperThickness,
    Mass mass,
    MomentOfInertia yawMomentOfInertia,
    Distance cgHeight) {
  public Body {
    Require.positive("frameLength", frameLength);
    Require.positive("frameWidth", frameWidth);
    Require.positive("height", height);
    Require.positive("bumperThickness", bumperThickness);
    Require.positive("mass", mass);
    Require.positive("yawMomentOfInertia", yawMomentOfInertia);
    Require.positive("cgHeight", cgHeight);
    if (cgHeight.gte(height)) {
      throw new IllegalArgumentException(
          "cgHeight "
              + Require.shown(cgHeight)
              + " must be below the robot's height of "
              + Require.shown(height));
    }
  }

  /** Starts building a body, one named value at a time. */
  public static BodyBuilder.FrameLengthStage builder() {
    return BodyBuilder.builder();
  }

  /** Front to back, across the outside of the bumpers. */
  public Distance bumperLength() {
    return frameLength.plus(bumperThickness.times(2));
  }

  /** Side to side, across the outside of the bumpers. */
  public Distance bumperWidth() {
    return frameWidth.plus(bumperThickness.times(2));
  }

  /** Around the outside of the frame. The game rules limit it. */
  public Distance framePerimeter() {
    return frameLength.plus(frameWidth).times(2);
  }

  /**
   * Whether a point, measured from the robot's origin, lies within the robot: inside the bumpers,
   * and between the floor and the robot's height.
   */
  public boolean contains(Translation3d point) {
    return Math.abs(point.getX()) <= bumperLength().in(Meters) / 2
        && Math.abs(point.getY()) <= bumperWidth().in(Meters) / 2
        && point.getZ() >= 0
        && point.getZ() <= height.in(Meters);
  }
}
