package frc.robot;

import com.michaelgrundvig.frc.pitwall.Status;
import java.util.Locale;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.RobotController;

/**
 * What Pitwall's status shows for this robot: which robot and which code, the mode, and the
 * battery. Add your own items here, one line each: {@code .add("Arm homed", arm::isHomed)}.
 */
final class RobotStatus {
  private RobotStatus() {}

  static Status of(RobotConfig robot) {
    return new Status()
        .add("Robot", robot::name)
        .add("Code", RobotStatus::code)
        .add("Mode", RobotStatus::mode)
        .add(
            "Battery",
            () -> String.format(Locale.ROOT, "%.1f V", RobotController.getBatteryVoltage()));
  }

  /** The commit the running code was built from, and whether it had uncommitted changes. */
  static String code() {
    String sha = BuildConstants.GIT_SHA;
    String shown = sha.length() > 8 ? sha.substring(0, 8) : sha;
    return shown
        + " on "
        + BuildConstants.GIT_BRANCH
        + (BuildConstants.DIRTY == 1 ? ", with uncommitted changes" : "");
  }

  static String mode() {
    String mode =
        switch (RobotState.getRobotMode()) {
          case AUTONOMOUS -> "Autonomous";
          case TELEOPERATED -> "Teleop";
          case UTILITY -> "Utility";
          case UNKNOWN -> "No mode";
        };
    return mode
        + (RobotState.isEnabled() ? ", enabled" : ", disabled")
        + (RobotState.isFMSAttached() ? ", on the field" : "");
  }
}
