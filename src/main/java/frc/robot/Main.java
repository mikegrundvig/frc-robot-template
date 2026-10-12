// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.michaelgrundvig.frc.runtime.BuildInfo;
import com.michaelgrundvig.frc.runtime.BuildInfo.Changes;
import com.michaelgrundvig.frc.runtime.RobotProgram;
import com.michaelgrundvig.frc.world.game.Rebuilt;

/**
 * The team's robot program starts here, handing the template's robot program what is the team's:
 * which robot this is, this season's field, what the robot does, and which build this is. Do NOT
 * add any static variables to this class, or any initialization at all.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your robot's class, field or controls, change the parameter.
   */
  public static void main(String... args) {
    RobotProgram.start(
        Robots::current, // which robot this is
        Rebuilt::game, // this season's field and venues
        Robots::controls, // what it does: the controls written for that robot
        build());
  }

  /** The build's generated constants, as the log records them. */
  private static BuildInfo build() {
    return new BuildInfo(
        BuildConstants.MAVEN_NAME,
        BuildConstants.BUILD_DATE,
        BuildConstants.GIT_SHA,
        BuildConstants.GIT_DATE,
        BuildConstants.GIT_BRANCH,
        Changes.fromFlag(BuildConstants.DIRTY));
  }
}
