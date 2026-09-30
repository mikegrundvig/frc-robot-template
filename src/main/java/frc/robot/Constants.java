// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import java.util.Locale;
import org.wpilib.framework.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a robot. In simulation it is "sim" (physics sim), or "replay" (log replay from a file) when
 * the {@code frc.simMode} system property says so, which {@code ./gradlew run -Preplay} sets.
 */
public final class Constants {
  public static final Mode simMode =
      Mode.valueOf(System.getProperty("frc.simMode", "SIM").toUpperCase(Locale.ROOT));
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
