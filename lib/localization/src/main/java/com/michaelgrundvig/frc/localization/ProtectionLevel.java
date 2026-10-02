package com.michaelgrundvig.frc.localization;

/**
 * A bound on how far the estimated position may be from the truth, holding with the probability the
 * integrity budget sets: GNSS's protection level, which a task compares with its alert limit ("PL 7
 * cm > 5 cm"). Nothing computes one yet, so {@link Integrity#protection} is empty.
 *
 * @param meters the bound, in meters
 */
public record ProtectionLevel(double meters) {
  public ProtectionLevel {
    if (!(meters >= 0)) {
      throw new IllegalArgumentException("a protection level is a distance: " + meters);
    }
  }
}
