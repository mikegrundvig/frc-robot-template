package com.michaelgrundvig.frc.pitwall.experiment;

/**
 * A test motion to run on a rig, the same way every time, so runs compare:
 *
 * <pre>{@code
 * new Experiment(
 *     "Step to 90° and back",
 *     steering,
 *     Profiles.stepAndBack().from(Degrees.of(0)).to(Degrees.of(90)).hold(Seconds.of(0.8)))
 * }</pre>
 *
 * <p>Teams write their own, in code, and add them with {@link Experiments#add(Experiment)}, which
 * checks that the rig can run it.
 *
 * @param name what it's called on the page; unique on the robot
 * @param rig what it runs on
 * @param profile the motion: the setpoint at each moment of a run
 */
public record Experiment(String name, Rig rig, Profile profile) {
  public Experiment {
    if (name.isBlank()) {
      throw new IllegalArgumentException("an experiment needs a name");
    }
  }

  /** What it does, as the page shows it: {@code step and back, 0 ° to 90 °, held 0.8 s}. */
  public String settings() {
    return profile.toString();
  }
}
