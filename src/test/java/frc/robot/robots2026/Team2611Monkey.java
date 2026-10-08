package frc.robot.robots2026;

import frc.robot.Monkey;

/** 2611's 2026 robot with their controls, for the monkey to drive ({@code MonkeySimTest}). */
public final class Team2611Monkey {
  private Team2611Monkey() {}

  public static final Monkey.Robot ROBOT = Monkey.robot(Team2611.CONFIG, Team2611Controls::new);
}
