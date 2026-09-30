package com.michaelgrundvig.frc.pitwall;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.backend.NetworkTablesTunableBackend;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.RobotController;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.util.Alert;

/**
 * Pitwall in a robot program, in two lines:
 *
 * <pre>{@code
 * pitwall = Pitwall.start(robot.name()); // after Logger.start(), before the mechanisms
 * pitwall.periodic(); // first thing in robotPeriodic()
 * }</pre>
 *
 * <p>That serves the page (http://robot.local:5820, or localhost in the simulator), shows the
 * standard status (which robot and code, the mode, the battery), tunes, and runs actions. Add
 * status items and actions through {@link #status()} and {@link #actions()}.
 *
 * <p>Tuning is WPILib's tunables. Every tunable in the program goes through Pitwall's {@link
 * TunableTee}, around WPILib's own NetworkTables backend: dashboards work as before, while changes
 * apply only while tuning is on, never on the field, and within each tunable's {@link Limits}. A
 * mechanism's configuration is tuned as a {@link LiveConfig} it publishes, checked whole before a
 * change applies, returned to committed when the field connects, and reported as the builder calls
 * that commit it.
 *
 * <p>With AdvantageKit, every request from the page is logged as an input, so a replay makes the
 * same changes at the same moments; while replaying, the page doesn't start. Without AdvantageKit
 * there is no replay, so nothing needs logging.
 *
 * @param <T> the robot's configuration record
 */
public final class Pitwall implements AutoCloseable {
  private final Tuning tuning;
  private final Status status;
  private final Actions actions;
  private final TunableTee tee;
  private final Alert tuned = new Alert("Pitwall", "", Alert.Level.MEDIUM);
  private final Alert changedSwitches = new Alert("Pitwall", "", Alert.Level.HIGH);
  private final boolean advantageKit;
  private final @Nullable PitwallServer server;
  private boolean wasEnabled;
  private boolean wasFieldConnected;

  private Pitwall(String name, int port, boolean advantageKit) {
    this.advantageKit = advantageKit;
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
    // Tuning starts switched on in the simulator; on a robot, switch it on from the page. While a
    // tuned value differs from the committed one, an alert says which.
    tuning =
        new Tuning(
            name,
            tee::configs,
            tee::pathOf,
            RobotBase.isSimulation(),
            advantageKit ? AdvantageKitLog.tuning() : inputs -> {},
            view -> {
              tuned.setText(
                  "Tuned values differ from the committed configuration: " + view.changedPaths());
              tuned.set(view.hasChanges());
            });
    TunableRegistry.registerBackend("", tee);
    server = replaying ? null : startPage(status, tuning, actions, tee, port);
  }

  /**
   * Serves the page, or says why it can't. The web server is the one built into Java; a runtime
   * trimmed without it (the {@code jdk.httpserver} module) only loses the page.
   */
  private static @Nullable PitwallServer startPage(
      Status status, Tuning tuning, Actions actions, TunableTee tee, int port) {
    try {
      return PitwallServer.startOrExplain(
              status, tuning, actions, tee, port, thisComputerOnly(), System.out::println)
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
  public static Pitwall start(String name) {
    return start(name, Integer.getInteger("pitwall.port", PitwallServer.PORT));
  }

  /**
   * Starts Pitwall and its page. Start it before the mechanisms, so the tunables they publish go
   * through it.
   *
   * @param name the robot's name, for the page and the logs
   * @param port where to serve the page; 0 picks a free one
   */
  public static Pitwall start(String name, int port) {
    return new Pitwall(name, port, advantageKitPresent());
  }

  /** Live tuning's switch, and the live configurations' state. */
  public Tuning tuning() {
    return tuning;
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
   * robotPeriodic()}, before anything reads a configuration and before the command scheduler runs.
   * When the robot disables, it prints (and logs) what tuning changed, so it can be committed.
   */
  public void periodic() {
    boolean enabled = RobotState.isEnabled();
    boolean fieldConnected = RobotState.isFMSAttached();
    if (fieldConnected && !wasFieldConnected) {
      // Tuning returns every tuned value to its committed one now: say what they were first.
      report(
          "The field system connected: tuned values return to the committed configuration",
          tuning.report());
    }
    tuning.periodic(fieldConnected);
    actions.periodic(
        new Actions.Gate(enabled, RobotState.getRobotMode() == RobotMode.UTILITY, fieldConnected));
    status.periodic();
    if (wasEnabled && !enabled) {
      report("", tuning.report());
    }
    // Switches (a camera left off, say) keep their setting on the field: say which differ.
    List<String> switches = fieldConnected ? tee.changedSinceStartup() : List.of();
    if (!switches.isEmpty()) {
      String text =
          "On the field with settings changed since startup: " + String.join(", ", switches);
      if (!text.equals(changedSwitches.getText())) {
        changedSwitches.setText(text);
      }
    }
    changedSwitches.set(!switches.isEmpty());
    wasEnabled = enabled;
    wasFieldConnected = fieldConnected;
  }

  /** Prints what tuning changed, and logs it, when there's anything to say. */
  private void report(String heading, List<String> lines) {
    if (lines.isEmpty()) {
      return;
    }
    List<String> all = new ArrayList<>();
    if (!heading.isEmpty()) {
      all.add(heading + ":");
    }
    all.addAll(lines);
    all.forEach(System.out::println);
    if (advantageKit) {
      AdvantageKitLog.recordOutput("Pitwall/TuningReport", all.toArray(String[]::new));
    }
  }

  /**
   * Stops the page, takes down Pitwall's tunables, and puts WPILib's own NetworkTables backend
   * back, so a test can start another.
   */
  @Override
  public void close() {
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
    changedSwitches.close();
  }

  /** Why changes to tunables are refused right now, or null while they're allowed. */
  private @Nullable String whyLocked() {
    return tuning.whyLocked();
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

  /**
   * Whether the page serves only this computer: in the simulator, whose laptop may be on a school's
   * or an event's network, unless {@code -Dpitwall.network=true} asks for the network too (to open
   * it on a phone). On the robot it always serves the robot's network.
   */
  static boolean thisComputerOnly() {
    return RobotBase.isSimulation() && !Boolean.getBoolean("pitwall.network");
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
