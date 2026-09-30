/**
 * The math of the calibration procedures: measurements that replace the configuration's guesses
 * with the robot's own numbers. Each procedure is code with tests, a written procedure ({@code
 * docs/calibration.md}), and a result that goes into the robot's configuration with the date it was
 * measured. The robot code runs the procedures and reads the sensors; these classes take the
 * readings as arguments.
 *
 * <ul>
 *   <li>{@link frc.robot.lib.calibration.WheelRadiusCalibration}: spin in place against the gyro.
 *   <li>{@link frc.robot.lib.calibration.GyroScaleCalibration}: spin whole turns against the
 *       cameras.
 *   <li>{@link frc.robot.lib.calibration.LevelCheck}: the IMU's attitude on a level floor.
 *   <li>{@link frc.robot.lib.calibration.CameraMountCalibration}: where a camera really is, from
 *       the tags it sees while the other cameras say where the robot is.
 * </ul>
 */
package frc.robot.lib.calibration;
