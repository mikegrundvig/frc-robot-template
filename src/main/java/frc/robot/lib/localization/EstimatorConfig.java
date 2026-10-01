package frc.robot.lib.localization;

import static frc.robot.lib.config.Rules.atLeast;
import static frc.robot.lib.config.Rules.below;
import static frc.robot.lib.config.Rules.between;
import static frc.robot.lib.config.Rules.nonNegative;
import static frc.robot.lib.config.Rules.positive;
import static frc.robot.lib.config.Rules.shown;

import frc.robot.lib.config.Checks;
import frc.robot.lib.config.ConfigRecord;

/**
 * How the pose estimator weighs odometry, the gyro, and the cameras. The defaults are reasonable
 * starting points, not measurements: fit them from logged residuals (the calibration procedures),
 * never from one run.
 *
 * @param translationStdDevPerMeter how far odometry's position may be off after one meter of
 *     travel, in meters. Its variance grows with distance.
 * @param translationStdDevPerRadian how far odometry's position may be off after turning one
 *     radian, in meters: wheels scrub as they turn
 * @param residualScale how much of the wheels' disagreement (the kinematic residual) counts as
 *     suspect travel: 1 counts all of it
 * @param tractionLimitedSlipFraction while the wheels are at the edge of their grip, this share of
 *     their travel counts as suspect
 * @param tiltStartDegrees below this tilt the wheels are trusted fully
 * @param tiltZeroDegrees at and above this tilt the wheels' travel counts for nothing: a robot
 *     climbing a bump or a ramp has wheels in the air (6328 scaled to zero at 25 degrees). Between
 *     the two, their travel counts for less, and the share left out counts as suspect.
 * @param slipCorrelationSeconds how long slip keeps its direction. Suspect travel within this time
 *     adds up before it's squared: a slip that builds one way for a quarter second grows the
 *     uncertainty as one error, not as many small ones that would average out.
 * @param collisionStdDevMeters how far a collision may have moved the robot beyond what its wheels
 *     saw: added at once, since the robot has already moved by the time the jolt is read
 * @param collisionWindowSeconds how long after a jolt the robot counts as just hit: more jolts
 *     within it are the same collision, and a scoring gate waits it out
 * @param handlingStdDevPerSqrtSecond while the robot is disabled, how far people may carry it where
 *     its wheels can't see, in meters after one second; the variance grows with time
 * @param headingDriftDegreesPerMinute how far the gyro's heading may drift in a minute
 * @param headingStdDevDegreesPerTurn how far the gyro's heading may be off after a full turn: its
 *     scale error, the same in every turn, so five turns are five times as far off (the estimator
 *     considers it, but never corrects it: the gyro-scale procedure measures it)
 * @param wheelHeadingStdDevDegreesPerTurn the same for heading from the wheels, used only while the
 *     gyro is lost; its errors are taken as new with every turn
 * @param cameraTurnStdDevDegrees how far each camera may be turned from its configured mount, the
 *     same in every image: what calibrating its mount leaves, and its lens. A camera turned by an
 *     angle puts its tags, and so the robot, that angle times their distance off, as a heading
 *     error does, and its headings by the angle. However many images it takes, the estimator never
 *     trusts a camera's positions or headings more than this allows.
 * @param cameraShiftStdDevMeters how far each camera's positions may be off the same way image
 *     after image, beyond its turn: its mount's shift, and how far the tags it sees sit from where
 *     the field's layout says (the field's tolerance, and how it was built)
 * @param cameraShiftStdDevDegrees and how far its headings may, beyond its turn: tags off the
 *     layout turn a multi-tag solve's heading by how far off they are over how far apart they sit
 * @param cameraShiftCorrelationMeters how far the robot moves before a camera's shift is mostly a
 *     new one: it sees other tags, or the same ones from elsewhere
 * @param cameraShiftCorrelationDegrees and how far it turns
 * @param historySeconds how far back a late camera measurement can still be applied at its capture
 *     time
 * @param futureToleranceSeconds how far a measurement may be stamped after the newest odometry
 *     sample (clocks differ a little) before it's rejected
 * @param visionTimeoutSeconds how long without an accepted camera measurement before the estimate
 *     counts as dead reckoning
 * @param disagreementWindowSeconds how far back the health's vision-odometry disagreement looks
 * @param gateChiSquaredPosition the gate for a position measurement: its innovation's squared
 *     Mahalanobis distance may not exceed this (13.8 passes 99.9% of good measurements, 2 degrees
 *     of freedom)
 * @param gateChiSquaredPose the same for a measurement with heading (16.3: 99.9%, 3 degrees)
 * @param gateChiSquaredAxis the same for a landmark, which measures one axis (10.8: 99.9%, 1
 *     degree)
 * @param recoveryMeasurements how many rejected measurements must agree with each other, within the
 *     recovery window, before odometry is judged wrong and the estimate jumps to them
 * @param recoveryWindowSeconds how recent those rejected measurements must be
 * @param recoveryAgreementMeters how closely their innovations must agree
 * @param recoveryStdDevMeters the position uncertainty after a recovery
 * @param recoveryAgreementDegrees how closely rejected measurements' headings must agree before the
 *     gyro's heading is judged wrong (a bad heading reset, a robot placed backwards, a gyro that
 *     jumped) and the estimate turns to them
 * @param recoveryHeadingStdDevDegrees the heading uncertainty after a heading recovery
 * @param lostAfterRejections this many camera measurements rejected in a row, with none accepted
 *     between, and the estimate is lost: the cameras keep disagreeing with it
 * @param lostPositionStdDevMeters the position uncertainty after a heading recovery: the position
 *     was worked out with the wrong heading, so it could be far off
 */
@ConfigRecord
public record EstimatorConfig(
    double translationStdDevPerMeter,
    double translationStdDevPerRadian,
    double residualScale,
    double tractionLimitedSlipFraction,
    double tiltStartDegrees,
    double tiltZeroDegrees,
    double slipCorrelationSeconds,
    double collisionStdDevMeters,
    double collisionWindowSeconds,
    double handlingStdDevPerSqrtSecond,
    double headingDriftDegreesPerMinute,
    double headingStdDevDegreesPerTurn,
    double wheelHeadingStdDevDegreesPerTurn,
    double cameraTurnStdDevDegrees,
    double cameraShiftStdDevMeters,
    double cameraShiftStdDevDegrees,
    double cameraShiftCorrelationMeters,
    double cameraShiftCorrelationDegrees,
    double historySeconds,
    double futureToleranceSeconds,
    double visionTimeoutSeconds,
    double disagreementWindowSeconds,
    double gateChiSquaredPosition,
    double gateChiSquaredPose,
    double gateChiSquaredAxis,
    int recoveryMeasurements,
    double recoveryWindowSeconds,
    double recoveryAgreementMeters,
    double recoveryStdDevMeters,
    double recoveryAgreementDegrees,
    double recoveryHeadingStdDevDegrees,
    int lostAfterRejections,
    double lostPositionStdDevMeters) {
  /** Starting points for a swerve robot with a good gyro. */
  public static final EstimatorConfig DEFAULTS =
      builder()
          .translationStdDevPerMeter(0.03)
          .translationStdDevPerRadian(0.02)
          .residualScale(1.0)
          .tractionLimitedSlipFraction(0.25)
          .tiltStartDegrees(5)
          .tiltZeroDegrees(25)
          .slipCorrelationSeconds(0.5)
          .collisionStdDevMeters(0.25)
          .collisionWindowSeconds(0.25)
          .handlingStdDevPerSqrtSecond(0.02)
          .headingDriftDegreesPerMinute(0.5)
          .headingStdDevDegreesPerTurn(1.0)
          .wheelHeadingStdDevDegreesPerTurn(10)
          .cameraTurnStdDevDegrees(0.25)
          .cameraShiftStdDevMeters(0.01)
          .cameraShiftStdDevDegrees(2.0)
          .cameraShiftCorrelationMeters(1.0)
          .cameraShiftCorrelationDegrees(45)
          .historySeconds(1.5)
          .futureToleranceSeconds(0.05)
          .visionTimeoutSeconds(1.0)
          .disagreementWindowSeconds(0.5)
          .gateChiSquaredPosition(13.816)
          .gateChiSquaredPose(16.266)
          .gateChiSquaredAxis(10.828)
          .recoveryMeasurements(3)
          .recoveryWindowSeconds(1.0)
          .recoveryAgreementMeters(0.25)
          .recoveryStdDevMeters(0.25)
          .recoveryAgreementDegrees(5)
          .recoveryHeadingStdDevDegrees(2)
          .lostAfterRejections(2)
          .lostPositionStdDevMeters(1.0)
          .build();

  public EstimatorConfig {
    Checks.on(EstimatorConfig.class)
        .field("translationStdDevPerMeter", translationStdDevPerMeter, nonNegative())
        .field("translationStdDevPerRadian", translationStdDevPerRadian, nonNegative())
        .field("residualScale", residualScale, nonNegative())
        .field("tractionLimitedSlipFraction", tractionLimitedSlipFraction, between(0, 1))
        .field("tiltStartDegrees", tiltStartDegrees, nonNegative())
        .field("tiltZeroDegrees", tiltZeroDegrees, below(90))
        .field("slipCorrelationSeconds", slipCorrelationSeconds, positive())
        .field("collisionStdDevMeters", collisionStdDevMeters, nonNegative())
        .field("collisionWindowSeconds", collisionWindowSeconds, positive())
        .field("handlingStdDevPerSqrtSecond", handlingStdDevPerSqrtSecond, nonNegative())
        .field("headingDriftDegreesPerMinute", headingDriftDegreesPerMinute, nonNegative())
        .field("headingStdDevDegreesPerTurn", headingStdDevDegreesPerTurn, nonNegative())
        .field("wheelHeadingStdDevDegreesPerTurn", wheelHeadingStdDevDegreesPerTurn, nonNegative())
        .field("cameraTurnStdDevDegrees", cameraTurnStdDevDegrees, nonNegative())
        .field("cameraShiftStdDevMeters", cameraShiftStdDevMeters, nonNegative())
        .field("cameraShiftStdDevDegrees", cameraShiftStdDevDegrees, nonNegative())
        .field("cameraShiftCorrelationMeters", cameraShiftCorrelationMeters, positive())
        .field("cameraShiftCorrelationDegrees", cameraShiftCorrelationDegrees, positive())
        .field("historySeconds", historySeconds, positive())
        .field("futureToleranceSeconds", futureToleranceSeconds, nonNegative())
        .field("visionTimeoutSeconds", visionTimeoutSeconds, positive())
        .field("disagreementWindowSeconds", disagreementWindowSeconds, positive())
        .field("gateChiSquaredPosition", gateChiSquaredPosition, positive())
        .field("gateChiSquaredPose", gateChiSquaredPose, positive())
        .field("gateChiSquaredAxis", gateChiSquaredAxis, positive())
        .field("recoveryMeasurements", recoveryMeasurements, atLeast(2))
        .field("recoveryWindowSeconds", recoveryWindowSeconds, positive())
        .field("recoveryAgreementMeters", recoveryAgreementMeters, nonNegative())
        .field("recoveryStdDevMeters", recoveryStdDevMeters, positive())
        .field("recoveryAgreementDegrees", recoveryAgreementDegrees, positive())
        .field("recoveryHeadingStdDevDegrees", recoveryHeadingStdDevDegrees, positive())
        .field("lostAfterRejections", lostAfterRejections, atLeast(1))
        .field("lostPositionStdDevMeters", lostPositionStdDevMeters, positive())
        .check(
            "tiltZeroDegrees",
            () ->
                tiltZeroDegrees > tiltStartDegrees
                    ? null
                    : shown(tiltZeroDegrees)
                        + " must be above tiltStartDegrees ("
                        + shown(tiltStartDegrees)
                        + "): wheels stop counting above where they start to count less")
        .orThrow();
  }

  /** Starts building estimator settings, one named value at a time. */
  public static EstimatorConfigBuilder.TranslationStdDevPerMeterStage builder() {
    return EstimatorConfigBuilder.builder();
  }
}
