package com.michaelgrundvig.frc.localization;

import java.util.List;

/**
 * What one {@link LocalizationService#update} made of the loop: the estimate, how far each source
 * is trusted, and what became of each camera measurement and landmark it weighed. Immutable, so the
 * loop can hand it to a view on another thread whole.
 *
 * @param estimate the estimate, with its uncertainty, mode, and health
 * @param integrity how far each source is trusted
 * @param frames what became of each camera image's measurement, in the order the estimator decided
 *     them: those it couldn't apply first, then the rest in the order of their capture times
 * @param landmarks what became of each landmark, in the same order
 */
public record LocalizationOutput(
    LocalizationEstimate estimate,
    Integrity integrity,
    List<FrameVerdict> frames,
    List<LandmarkDecision> landmarks) {
  public LocalizationOutput {
    frames = List.copyOf(frames);
    landmarks = List.copyOf(landmarks);
  }
}
