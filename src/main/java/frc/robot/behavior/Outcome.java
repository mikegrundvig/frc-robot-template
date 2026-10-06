package frc.robot.behavior;

/** How a move ended, a joint's or the drive's: each move records one, with a reason it logs. */
public enum Outcome {
  /** It got there: inside its band for 40 ms, or its condition held. */
  ARRIVED,
  /** Its deadline passed first. */
  TIMED_OUT,
  /** Another command took what it requires, its binding released, or its gate fell. */
  CANCELED,
  /** It wouldn't start, or couldn't go on. */
  REFUSED
}
