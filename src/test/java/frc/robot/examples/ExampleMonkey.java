package frc.robot.examples;

import frc.robot.Monkey;

/** The guide's robot with the guide's controls, for the monkey to drive ({@code MonkeySimTest}). */
public final class ExampleMonkey {
  private ExampleMonkey() {}

  public static final Monkey.Robot ROBOT = Monkey.robot(ExampleRobot.COMPETITION, Controls::new);
}
