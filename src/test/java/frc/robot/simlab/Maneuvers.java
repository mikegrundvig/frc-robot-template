package frc.robot.simlab;

/** The catalog of standard maneuvers. Tests and studies use these, so results are comparable. */
public final class Maneuvers {
  private Maneuvers() {}

  /**
   * Full-stick sprints, reversing at top speed: the hardest case for both traction (full torque,
   * with weight shifting off one pair of wheels) and the battery (all four drive motors at their
   * current limits at once).
   */
  public static final Maneuver UP_DOWN_SPRINTS =
      new Maneuver(
          "up-down-sprints",
          13.5,
          t -> {
            double stick;
            if (t < 0.5) {
              stick = 0.0;
            } else if (t < 3.5) {
              stick = 1.0;
            } else if (t < 6.5) {
              stick = -1.0;
            } else if (t < 9.5) {
              stick = 1.0;
            } else if (t < 12.5) {
              stick = -1.0;
            } else {
              stick = 0.0;
            }
            return new double[] {stick, 0.0, 0.0};
          });

  /** Field-relative translation while spinning: the path should stay straight. */
  public static final Maneuver TRANSLATE_WHILE_SPINNING =
      new Maneuver("translate-while-spinning", 3.0, t -> new double[] {0.7, 0.0, 0.5});
}
