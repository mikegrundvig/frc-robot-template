package com.michaelgrundvig.frc.localization;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * Whether the cameras, disagreeing with the estimate, agree with each other: then odometry is the
 * one that's wrong (a collision, a spin, a bad heading reset), and the estimate should go to them.
 * It keeps the recent rejected measurements' disagreements and says what they agree on; the
 * estimator does the moving.
 */
final class Recovery {
  private final int needed;
  private final long windowNanos;
  private final double agreementMeters;
  private final double agreementRadians;
  private final ArrayDeque<Rejected> recent = new ArrayDeque<>();

  /**
   * A rejected measurement's disagreement with the estimate.
   *
   * @param innovationHeading NaN when the measurement had no heading
   * @param positionAgreed whether its position alone passed the gate: only its heading disagreed
   */
  record Rejected(
      long time,
      double innovationX,
      double innovationY,
      double innovationHeading,
      boolean positionAgreed) {}

  Recovery(EstimatorConfig config) {
    needed = config.recoveryMeasurements();
    windowNanos = Timestamps.fromSeconds(config.recoveryWindowSeconds());
    agreementMeters = config.recoveryAgreementMeters();
    agreementRadians = Math.toRadians(config.recoveryAgreementDegrees());
  }

  void rejected(Rejected rejected) {
    recent.addLast(rejected);
  }

  /** Forgets every rejection: a measurement was accepted, or the estimate was reset. */
  void clear() {
    recent.clear();
  }

  /**
   * Forgets rejections older than the window, as of now.
   *
   * @return whether enough are left to agree on anything
   */
  boolean enoughAsOf(long nowNanos) {
    while (!recent.isEmpty() && recent.getFirst().time() < nowNanos - windowNanos) {
      recent.removeFirst();
    }
    return recent.size() >= needed;
  }

  /**
   * The turn the rejected measurements with heading agree the estimate's heading needs, if enough
   * agree on one that isn't the estimate's. Their positions were worked out with that heading, so
   * the position is left to the next measurement.
   *
   * @return NaN if they don't
   */
  double headingCorrection() {
    double first = Double.NaN;
    double sum = 0;
    int count = 0;
    for (Rejected rejected : recent) {
      if (!Double.isNaN(rejected.innovationHeading())) {
        if (count == 0) {
          first = rejected.innovationHeading();
        }
        // Measured from the first, so headings either side of half a turn average rightly.
        sum += MathUtil.angleModulus(rejected.innovationHeading() - first);
        count++;
      }
    }
    if (count < needed) {
      return Double.NaN;
    }
    double mean = first + sum / count;
    for (Rejected rejected : recent) {
      if (!Double.isNaN(rejected.innovationHeading())
          && Math.abs(MathUtil.angleModulus(rejected.innovationHeading() - mean))
              > agreementRadians) {
        return Double.NaN;
      }
    }
    return Math.abs(MathUtil.angleModulus(mean)) <= agreementRadians ? Double.NaN : mean;
  }

  /**
   * The move the rejected measurements agree the estimate's position needs, if enough whose
   * positions disagreed agree with each other on one.
   */
  @Nullable Translation2d positionCorrection() {
    // Only the measurements whose positions disagreed say the position is wrong.
    List<Rejected> offPosition = new ArrayList<>();
    for (Rejected rejected : recent) {
      if (!rejected.positionAgreed()) {
        offPosition.add(rejected);
      }
    }
    if (offPosition.size() < needed) {
      return null;
    }
    double meanX = 0;
    double meanY = 0;
    for (Rejected rejected : offPosition) {
      meanX += rejected.innovationX();
      meanY += rejected.innovationY();
    }
    meanX /= offPosition.size();
    meanY /= offPosition.size();
    for (Rejected rejected : offPosition) {
      double spread = Math.hypot(rejected.innovationX() - meanX, rejected.innovationY() - meanY);
      if (spread > agreementMeters) {
        return null;
      }
    }
    return new Translation2d(meanX, meanY);
  }
}
