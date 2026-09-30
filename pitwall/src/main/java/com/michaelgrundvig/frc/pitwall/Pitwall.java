package com.michaelgrundvig.frc.pitwall;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.backend.NetworkTablesTunableBackend;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.RobotController;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableRegistry;
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
 * <p>Every WPILib tunable in the program goes through Pitwall's {@link TunableTee}, around WPILib's
 * own NetworkTables backend: dashboards work as before, changes apply only while tuning is on and
 * never on the field, and the configuration's values are published as tunables under {@code
 * Config/}, where a change from a dashboard goes through the same checks as one from the page.
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
  private final TunableTee tee;
  private final List<String> configTunables = new ArrayList<>();
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
    // Every WPILib tunable in the program goes through Pitwall's tee, wrapped around WPILib's own
    // NetworkTables backend: dashboards see what they always did, while changes are locked with
    // tuning and logged for replay, and the page lists every tunable.
    tee =
        new TunableTee(
            new NetworkTablesTunableBackend(NetworkTableInstance.getDefault(), "/Tunables"),
            this::whyLocked,
            advantageKit ? AdvantageKitLog.tunables() : inputs -> {},
            replaying);
    TunableRegistry.registerBackend("", tee);
    publishConfigTunables();
    server = replaying ? null : startPage(status, tuning, actions, tee, port);
  }

  /**
   * Serves the page, or says why it can't. The web server is the one built into Java; a runtime
   * trimmed without it (the {@code jdk.httpserver} module) only loses the page.
   */
  private static @Nullable PitwallServer startPage(
      Status status, Tuning<?> tuning, Actions actions, TunableTee tee, int port) {
    try {
      return PitwallServer.startOrExplain(status, tuning, actions, tee, port, System.out::println)
          .orElse(null);
    } catch (LinkageError e) {
      System.out.println(
          "Pitwall's page is unavailable: this Java has no built-in web server (jdk.httpserver): "
              + e);
      return null;
    }
  }

  /**
   * Starts Pitwall and its page on {@link PitwallServer#PORT}, or on the port the {@code
   * pitwall.port} system property names (0 picks a free one).
   */
  public static <T extends Record> Pitwall<T> start(String name, T config) {
    return start(name, config, Integer.getInteger("pitwall.port", PitwallServer.PORT));
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

  /** Every WPILib tunable in the robot program, as Pitwall sees it. */
  public TunableTee tunables() {
    return tee;
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

  /**
   * Stops the page, takes down Pitwall's tunables, and puts WPILib's own NetworkTables backend
   * back, so a test can start another.
   */
  @Override
  public void close() {
    configTunables.forEach(TunableRegistry::remove);
    TunableRegistry.registerBackend(
        "", new NetworkTablesTunableBackend(NetworkTableInstance.getDefault(), "/Tunables"));
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

  /**
   * Publishes each tunable configuration value as a WPILib tunable at {@code Config/<path>}, so
   * dashboards show and change it too. A change from a dashboard goes through {@link Tuning},
   * checked and logged like one from the page; a refused one snaps back.
   */
  private void publishConfigTunables() {
    for (Tuning.Value value : tuning.view().values()) {
      String path = value.path();
      TunableDouble tunable =
          TunableDouble.create(
              () -> tuning.value(path), v -> tuning.requestChanges(Map.of(path, v)));
      tee.passThrough(tunable);
      String published = "Config/" + path.replace('.', '/');
      if (TunableRegistry.publish(published, tunable)) {
        configTunables.add(published);
      }
    }
  }

  /** Why changes to tunables are refused right now, or null while they're allowed. */
  private @Nullable String whyLocked() {
    Tuning.View view = tuning.view();
    if (view.fieldConnected()) {
      return "the field system is connected";
    }
    return view.enabled() ? null : "tuning is off";
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
