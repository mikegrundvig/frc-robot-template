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
 *       Inside it, {@code OdometryHistory} keeps the steps a late measurement is replayed through,
 *       {@code PoseFilter} does the filter's math (and keeps the errors that repeat, below), and
 *       {@code Recovery} judges when the cameras agree that odometry is wrong.
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
 *       frc.robot.lib.localization.VisionDecision} says what became of it. {@link
 *       frc.robot.lib.localization.MountCheck} judges, from where a camera's tags land, whether it
 *       still sits where its configuration says, and how much less to trust it if not; a {@link
 *       frc.robot.lib.localization.TagAllowlist} keeps a task to the tags it trusts.
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
 *       heading, which is how a solve that took its heading from the gyro reports it; such a
 *       measurement also says how far its tags were ({@link
 *       frc.robot.lib.localization.VisionMeasurement#robotToTags}), since the gyro's heading error
 *       moves its position by that distance times the error.
 * </ul>
 *
 * <p><b>An honest uncertainty.</b> Gates, χ² tests, and any bound on the robot's error trust the
 * covariance the estimator reports, so it must not claim more than it knows: the error should go
 * past 3.72 times the claimed spread no more than once in a thousand loops. The consistency study
 * checks that in the simulator, where the truth is known (docs/testing.md), and a test keeps it.
 * What makes an estimator overconfident is an error that repeats, averaged as if each instance were
 * new. So the estimator never does that with the ones it knows:
 *
 * <ul>
 *   <li>a position a camera solved with the gyro's heading leans on that heading, in every image:
 *       it carries the heading's doubt, and never corrects the heading (the gyro owns it; only a
 *       measurement with a heading of its own corrects its drift);
 *   <li>a gyro reading its turns a little long or short is off the same way every turn;
 *   <li>each camera's turn on its mount, and its shift (its mount's offset, and how far the tags it
 *       sees sit from the field's layout), are in every image it takes, until it moves on and sees
 *       other tags.
 * </ul>
 *
 * <p>The filter keeps these as "consider" parameters: errors it accounts for but never estimates.
 * Their sizes are in {@link frc.robot.lib.localization.EstimatorConfig}, and docs/values.md says
 * how to measure each.
 */
package frc.robot.lib.localization;
