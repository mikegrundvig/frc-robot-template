package com.michaelgrundvig.frc.localization;

import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;

/**
 * What became of one camera image's measurement in an update: as a whole, by the coprocessor's
 * multi-tag solution, and by tag. Logs, the pit's page, and the integrity views read these, so none
 * of them reaches into the estimator.
 *
 * @param camera the camera's name, as in the robot's definition
 * @param captureNanos when the image was taken, on the robot's clock
 * @param multiTag the verdict on the coprocessor's multi-tag solution, when the measurement was
 *     that solution; empty when it was solved another way
 * @param tags each tag the measurement rested on, with its verdict
 * @param measured the measurement as one: where it put the robot, how it was solved, and its
 *     verdict
 */
public record FrameVerdict(
    String camera,
    long captureNanos,
    Optional<Verdict> multiTag,
    List<TagVerdict> tags,
    Measured measured) {
  public FrameVerdict {
    tags = List.copyOf(tags);
  }

  /**
   * One image's measurement, as the estimator weighed it.
   *
   * @param pose the robot's pose, as the image measured it
   * @param solver how the image was solved
   * @param verdict whether it moved the estimate, or why not
   * @param innovationMeters how far it was from the estimate at its capture time; NaN if it was
   *     refused before being compared
   * @param chiSquared its squared Mahalanobis distance from the estimate; NaN if not compared
   * @param threshold the gate its χ² was held to; NaN if not compared
   */
  public record Measured(
      Pose2d pose,
      VisionMeasurement.Solver solver,
      Verdict verdict,
      double innovationMeters,
      double chiSquared,
      double threshold) {
    /** Whether it moved the estimate. */
    public boolean accepted() {
      return verdict == Verdict.ACCEPTED;
    }

    /** Why, in words with its numbers, as the log has them: "gate: 45.2 > 13.8". */
    public String reason() {
      return verdict.describe(chiSquared, threshold);
    }
  }

  /** What the estimator decided about one camera measurement, by tag. */
  static FrameVerdict of(VisionDecision decision) {
    VisionMeasurement measurement = decision.measurement();
    Verdict verdict = decision.verdict();
    List<Integer> ids = measurement.tagIds();
    TagVerdict[] tags = new TagVerdict[ids.size()];
    for (int i = 0; i < tags.length; i++) {
      tags[i] =
          new TagVerdict(
              ids.get(i),
              verdict,
              decision.innovationMeters(),
              decision.chiSquared(),
              decision.threshold());
    }
    return new FrameVerdict(
        measurement.camera(),
        measurement.timestampNanos(),
        measurement.solver() == VisionMeasurement.Solver.MULTI_TAG
            ? Optional.of(verdict)
            : Optional.empty(),
        List.of(tags),
        new Measured(
            measurement.pose(),
            measurement.solver(),
            verdict,
            decision.innovationMeters(),
            decision.chiSquared(),
            decision.threshold()));
  }
}
