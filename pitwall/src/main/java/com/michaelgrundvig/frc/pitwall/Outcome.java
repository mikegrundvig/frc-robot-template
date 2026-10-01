package com.michaelgrundvig.frc.pitwall;

/**
 * What became of a request from the page, once the robot loop handled it.
 *
 * @param result whether it was applied, refused, or locked out
 * @param detail why it wasn't applied; empty when it was
 */
public record Outcome(Result result, String detail) {
  /** Whether a request was applied, and if not, why not. */
  public enum Result {
    /** Applied, all of it. */
    APPLIED,
    /** Refused by the configuration's checks or by the action itself; nothing changed. */
    REFUSED,
    /**
     * Locked out: tuning is off, the robot isn't enabled in utility mode (for actions and
     * experiments), or the field system is connected; or there is nothing to act on, such as a
     * cancel for an experiment that isn't running. Nothing changed.
     */
    LOCKED
  }

  /** A request applied, all of it. */
  public static Outcome applied() {
    return new Outcome(Result.APPLIED, "");
  }
}
