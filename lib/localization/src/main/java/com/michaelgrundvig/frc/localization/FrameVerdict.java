package com.michaelgrundvig.frc.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;

/**
 * What became of one camera image in an update: as a whole, by the coprocessor's multi-tag
 * solution, and by tag. Every image localization was given is one, whether the estimator weighed
 * its measurement or it was refused before it became one. Logs, the pit's page, and the integrity
 * checks read these, so none of them reaches into the estimator.
 *
 * @param camera the camera's name, as in the robot's definition
 * @param captureNanos when the image was taken, on the robot's clock
 * @param verdict what became of it: accepted, or why not
 * @param threshold the limit the verdict's value was held to; NaN where the verdict has none
 * @param value the verdict's number: a χ² for the estimator's verdicts, the tags' distance, the
 *     robot's turning, a tag's ambiguity or height for the pipeline's; NaN where it has none
 * @param multiTag the verdict on the coprocessor's multi-tag solution, when the measurement was
 *     that solution; empty when it was solved another way, or not at all
 * @param tags the tags the verdict applies to, each with its own: those the measurement rested on,
 *     or every tag the image saw, if it was refused before becoming one
 * @param measured the image's measurement as the estimator weighed it; empty for an image refused
 *     before it got there
 */
public record FrameVerdict(
    String camera,
    long captureNanos,
    Verdict verdict,
    double threshold,
    double value,
    Optional<Verdict> multiTag,
    List<TagVerdict> tags,
    Optional<Measured> measured) {
  public FrameVerdict {
    tags = List.copyOf(tags);
  }

  /** Whether it moved the estimate. */
  public boolean accepted() {
    return verdict == Verdict.ACCEPTED;
  }

  /** Why, in words with its numbers, as the log has them: "gate: 45.2 > 13.8", "too far: 6.3 m". */
  public String reason() {
    return verdict.describe(value, threshold);
  }

  /**
   * An image's measurement, as the estimator weighed it.
   *
   * @param pose the robot's pose, as the image measured it
   * @param solver how the image was solved
   * @param innovationMeters how far it was from the estimate at its capture time; NaN if it was
   *     refused before being compared
   * @param chiSquared its squared Mahalanobis distance from the estimate; NaN if not compared
   */
  public record Measured(
      Pose2d pose, VisionMeasurement.Solver solver, double innovationMeters, double chiSquared) {}

  /** What the estimator decided about one camera measurement, by tag. */
  static FrameVerdict of(VisionDecision decision) {
    VisionMeasurement measurement = decision.measurement();
    Verdict verdict = decision.verdict();
    List<TagVerdict> tags = new ArrayList<>(measurement.tagIds().size());
    for (int id : measurement.tagIds()) {
      tags.add(
          new TagVerdict(
              id,
              verdict,
              decision.innovationMeters(),
              decision.chiSquared(),
              decision.threshold(),
              decision.chiSquared()));
    }
    return new FrameVerdict(
        measurement.camera(),
        measurement.timestampNanos(),
        verdict,
        decision.threshold(),
        decision.chiSquared(),
        measurement.solver() == VisionMeasurement.Solver.MULTI_TAG
            ? Optional.of(verdict)
            : Optional.empty(),
        tags,
        Optional.of(
            new Measured(
                measurement.pose(),
                measurement.solver(),
                decision.innovationMeters(),
                decision.chiSquared())));
  }

  /** An image the pipeline refused, before it became a measurement: every tag it saw, refused. */
  static FrameVerdict refused(VisionResult result) {
    Verdict verdict = result.refused();
    if (verdict == null) {
      throw new IllegalArgumentException("the image became a measurement: it wasn't refused");
    }
    TagDetections image = result.detections();
    List<TagVerdict> tags = new ArrayList<>(image.tags().size());
    for (TagDetections.Tag tag : image.tags()) {
      tags.add(
          new TagVerdict(
              tag.id(), verdict, Double.NaN, Double.NaN, result.threshold(), result.value()));
    }
    return new FrameVerdict(
        image.camera(),
        image.timestampNanos(),
        verdict,
        result.threshold(),
        result.value(),
        Optional.empty(),
        tags,
        Optional.empty());
  }
}
