package frc.robot.simlab;

/** A scripted driver: stick positions over time, for a fixed duration. */
public record Maneuver(String name, double durationSec, Script script) {
  /** Stick positions {x, y, omega}, each -1..1, with the same conventions as the driver. */
  public interface Script {
    double[] sticks(double timeSec);
  }

  /** The same maneuver under another name, so its log file does not overwrite the original. */
  public Maneuver named(String newName) {
    return new Maneuver(newName, durationSec, script);
  }

  /** When the driver first moves a stick, so acceleration times start there. */
  double firstStickTimeSec(double stepSec) {
    for (double t = 0; t < durationSec; t += stepSec) {
      double[] s = script.sticks(t);
      if (s[0] != 0 || s[1] != 0 || s[2] != 0) {
        return t;
      }
    }
    return 0.0;
  }
}
