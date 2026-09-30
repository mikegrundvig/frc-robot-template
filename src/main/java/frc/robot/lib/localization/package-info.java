/**
 * Localization: where the robot is, from its wheels, its IMUs, its cameras, and the field's
 * terrain. Hardware (or its simulation) reports through an IO implementation outside this package,
 * in the types here, which no vendor's library leaks into; new hardware means a new IO
 * implementation, and nothing here changes.
 *
 * <p>Where to start reading:
 *
 * <ul>
 *   <li><b>The estimator:</b> {@link frc.robot.lib.localization.PoseEstimator} takes every
 *       measurement and keeps the estimate, tuned by {@link
 *       frc.robot.lib.localization.EstimatorConfig}. What it reports is a {@link
 *       frc.robot.lib.localization.LocalizationEstimate}; {@link
 *       frc.robot.lib.localization.LocalizationGate} says whether that's good enough to score on.
 *   <li><b>Wheels:</b> an {@link frc.robot.lib.localization.OdometrySample} of every wheel, turned
 *       into the robot's {@link frc.robot.lib.localization.BodyMotion} by the drivetrain's {@link
 *       frc.robot.lib.localization.DriveKinematicsModel} (swerve or differential). {@link
 *       frc.robot.lib.localization.DriveConditions} says how far to trust them right now.
 *   <li><b>IMUs:</b> {@link frc.robot.lib.localization.YawTrack} places yaw readings in time,
 *       {@link frc.robot.lib.localization.ImuCrossCheck} compares two IMUs, and {@link
 *       frc.robot.lib.localization.TerrainCrossing} follows the robot over raised terrain.
 *   <li><b>Cameras:</b> the raw {@link frc.robot.lib.localization.TagDetections} of one image go
 *       through the {@link frc.robot.lib.localization.VisionPipeline}, which solves them (with
 *       {@link frc.robot.lib.localization.TagSolver}) into a {@link
 *       frc.robot.lib.localization.VisionMeasurement}, or says why not; the estimator's {@link
 *       frc.robot.lib.localization.VisionDecision} says what became of it.
 *   <li><b>Landmarks:</b> {@link frc.robot.lib.localization.CrestLandmarks} turns a bump's crest,
 *       crossed, into a {@link frc.robot.lib.localization.LandmarkEvent}: a one-axis fix.
 * </ul>
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
