// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.michaelgrundvig.frc.runtime.RobotProgram;
import com.michaelgrundvig.frc.world.game.Rebuilt;

/** Where the robot program starts. */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotProgram.start(
        () -> ChassisRobot.CONFIG, // which robot this is
        Rebuilt::game, // the field it drives on
        Controls::new); // what the sticks and buttons do
  }
}
