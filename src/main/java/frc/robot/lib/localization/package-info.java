/**
 * Localization's measurements: what the wheels, the IMU, and the cameras report, in types no
 * vendor's library leaks into. Each piece of hardware (or its simulation) reports through an IO
 * implementation that produces these; the estimator consumes them. New hardware means a new IO
 * implementation, and nothing downstream changes.
 *
 * <p>Conventions:
 *
 * <ul>
 *   <li><b>Time:</b> every measurement carries the moment it was captured, not when it arrived, in
 *       nanoseconds on the robot's clock ({@code RobotController.getTime()}, WPILib 2027's integer
 *       clock). Vendors report seconds: convert with {@link frc.robot.lib.localization.Timestamps}.
 *       Nothing here reads a clock.
 *   <li><b>Frames:</b> as written in {@code frc.robot.lib.field}: poses in the field frame, the
 *       robot's own motion in the robot frame.
 *   <li><b>Uncertainty:</b> a {@link frc.robot.lib.localization.PoseCovariance}, in square meters
 *       and square radians. An infinite heading variance means the measurement says nothing about
 *       heading, which is how a solve that took its heading from the gyro reports it.
 * </ul>
 */
package frc.robot.lib.localization;
