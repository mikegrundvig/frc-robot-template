package com.michaelgrundvig.frc.pitwall;

import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.system.RobotController;
import org.wpilib.util.Alert;

/**
 * Pitwall in a robot program, in two lines:
 *
 * <pre>{@code
 * pitwall = Pitwall.start(Robots.PRACTICE.name(), Robots.PRACTICE); // after Logger.start()
 * pitwall.periodic(); // first thing in robotPeriodic()
 * }</pre>
 *
 * <p>That serves the page (http://robot.local:5820, or localhost in the simulator), shows the
 * standard status (which robot and code, the mode, the battery), tunes the configuration's {@link
 * Tunable} values, and runs actions. Read the configuration through {@link #config()}, which
 * includes tuned values, and add status items and actions through {@link #status()} and {@link
 * #actions()}.
 *
 * <p>With AdvantageKit, every request from the page is logged as an input, so a replay makes the
 * same changes at the same moments; while replaying, the page doesn't start. Without AdvantageKit
 * there is no replay, so nothing needs logging.
 *
 * @param <T> the robot's configuration record
 */
public final class Pitwall<T extends Record> implements AutoCloseable {
  private final Tuning<T> tuning;
  private final Status status;
  private final Actions actions;
  private final Alert tuned = new Alert("Pitwall", "", Alert.Level.MEDIUM);
  private final boolean advantageKit;
  private final @Nullable PitwallServer server;
  private boolean wasEnabled;

  private Pitwall(String name, T config, int port, boolean advantageKit) {
    this.advantageKit = advantageKit;
    // Tuning starts switched on in the simulator; on a robot, switch it on from the page. While a
    // tuned value differs from the committed one, an alert says which.
    tuning =
        new Tuning<>(
            name,
            config,
            RobotBase.isSimulation(),
            advantageKit ? AdvantageKitLog.tuning() : inputs -> {},
            view -> {
              tuned.setText(
                  "Tuned values differ from the committed configuration: " + view.changedPaths());
              tuned.set(view.hasChanges());
            });
    status = new Status().add("Robot", () -> name);
    Supplier<String> code = code();
    if (code != null) {
      status.add("Code", code);
    }
    status
        .add("Mode", Pitwall::mode)
        .add(
            "Battery",
            () -> String.format(Locale.ROOT, "%.1f V", RobotController.getBatteryVoltage()));
    actions = new Actions(advantageKit ? AdvantageKitLog.actions() : inputs -> {});
    boolean replaying = advantageKit && AdvantageKitLog.replaying();
    server =
        replaying
            ? null
            : PitwallServer.startOrExplain(status, tuning, actions, port, System.out::println)
                .orElse(null);
  }

  /** Starts Pitwall and its page on {@link PitwallServer#PORT}. */
  public static <T extends Record> Pitwall<T> start(String name, T config) {
    return start(name, config, PitwallServer.PORT);
  }

  /**
   * Starts Pitwall and its page.
   *
   * @param name the robot's name, for the page and the logs
   * @param config the robot's committed configuration
   * @param port where to serve the page; 0 picks a free one
   */
  public static <T extends Record> Pitwall<T> start(String name, T config, int port) {
    return new Pitwall<>(name, config, port, advantageKitPresent());
  }

  /** The robot's configuration with every tuned value applied. Read it in the robot loop. */
  public T config() {
    return tuning.config();
  }

  /** The status the page shows. Add items here: {@code status().add("Arm homed", arm::isHomed)}. */
  public Status status() {
    return status;
  }

  /** The actions the page can run. Add them here: {@code actions().add("Home arm", arm::home)}. */
  public Actions actions() {
    return actions;
  }

  /**
   * The port the page is served on, or -1 when it isn't (while replaying, or it couldn't start).
   */
  public int port() {
    PitwallServer running = server;
    return running != null ? running.port() : -1;
  }

  /**
   * Applies the page's requests, then reads the status. Call it first thing in {@code
   * robotPeriodic()}, before anything reads {@link #config()} and before the command scheduler
   * runs. When the robot disables, it prints (and logs) what tuning changed, so it can be
   * committed.
   */
  public void periodic() {
    boolean enabled = RobotState.isEnabled();
    boolean fieldConnected = RobotState.isFMSAttached();
    tuning.periodic(fieldConnected);
    actions.periodic(
        new Actions.Gate(enabled, RobotState.getRobotMode() == RobotMode.UTILITY, fieldConnected));
    status.periodic();
    if (wasEnabled && !enabled) {
      List<String> report = tuning.report();
      report.forEach(System.out::println);
      if (advantageKit) {
        AdvantageKitLog.recordOutput("Pitwall/TuningReport", report.toArray(String[]::new));
      }
    }
    wasEnabled = enabled;
  }

  /** Stops the page and releases the alert, so a test can start another. */
  @Override
  public void close() {
    PitwallServer running = server;
    if (running != null) {
      try {
        running.close();
      } catch (Exception e) {
        // Stopping is best effort; the page is a convenience.
      }
    }
    tuned.close();
  }

  /** The mode as the status shows it: "Teleop, enabled", "Autonomous, disabled, on the field". */
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

  /**
   * Which code is running, from the {@code frc.robot.BuildConstants} that AdvantageKit's templates
   * (and this one) generate from git; null when the robot program has none.
   */
  static @Nullable Supplier<String> code() {
    try {
      Class<?> constants = Class.forName("frc.robot.BuildConstants");
      String sha = (String) constants.getField("GIT_SHA").get(null);
      String branch = (String) constants.getField("GIT_BRANCH").get(null);
      int dirty = constants.getField("DIRTY").getInt(null);
      String text =
          (sha.length() > 8 ? sha.substring(0, 8) : sha)
              + " on "
              + branch
              + (dirty == 1 ? ", with uncommitted changes" : "");
      return () -> text;
    } catch (ReflectiveOperationException | ClassCastException | NullPointerException e) {
      return null;
    }
  }

  /** Whether the robot program has AdvantageKit, checked without loading any of it. */
  static boolean advantageKitPresent() {
    try {
      Class.forName("org.littletonrobotics.junction.Logger", false, Pitwall.class.getClassLoader());
      return true;
    } catch (ClassNotFoundException e) {
      return false;
    }
  }
}
