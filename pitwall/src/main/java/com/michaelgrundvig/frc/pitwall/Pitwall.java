package com.michaelgrundvig.frc.pitwall;

import com.michaelgrundvig.frc.pitwall.experiment.Experiments;
import com.michaelgrundvig.frc.pitwall.push.Broadcaster;
import com.michaelgrundvig.frc.pitwall.scene.Scenes;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Simulator;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
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
 * {@snippet :
 * pitwall = Pitwall.start(robot.name()); // after Logger.start(), before the mechanisms
 * ...
 * pitwall.periodic(); // first thing in the robot's loop
 * }
 *
 * <p>That serves the page (http://robot.local:5820, or localhost in the simulator), shows the
 * standard status (the code running and the battery), tunes, runs actions, and runs experiments.
 * Add status items, actions, experiments, devices, and scenes through {@link #status()}, {@link
 * #actions()}, {@link #experiments()}, {@link #devices()}, and {@link #scenes()}.
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
 * <p>In WPILib's simulation, and not while replaying, Pitwall also has the simulator's controls
 * ({@link #simulator()}): the Driver Station, joysticks, pausing and stepping, stopping, and
 * failing the simulated robot's parts live: an API the page's Sim view drives, in the place of
 * WPILib's simulator GUI. On a robot there is no such thing.
 */
public final class Pitwall implements AutoCloseable {
  private final Tuning tuning;
  private final Status status;
  private final Actions actions;
  private final Experiments experiments;
  private final Devices devices;
  private final Scenes scenes;
  private final TunableTee tee;
  private final Page page;
  private final Broadcaster broadcaster;
  private final @Nullable Simulator simulator;

  /** What the simulator's controls said this loop, for the log; the robot loop's alone. */
  private final List<String> said = new ArrayList<>();

  private boolean saidLastLoop;
  private final Alert tuned = new Alert("Pitwall", "", Alert.Level.MEDIUM);
  private final Alert changedSwitches = new Alert("Pitwall", "", Alert.Level.HIGH);
  private final boolean advantageKit;
  private final @Nullable PitwallServer server;
  private boolean wasEnabled;
  private boolean wasFieldConnected;

  /** Whether the field system has connected since the robot program started. */
  private boolean fieldSinceStart;

  private Pitwall(String name, int port, boolean advantageKit) {
    this.advantageKit = advantageKit;
    // Status items may change at most so often, timed on the robot's clock. Named Robot/... so the
    // page groups them under one heading, with whatever else a robot program adds the same way. The
    // robot's name and mode aren't among them: the page's bar always shows both.
    status = new Status(RobotController::getTime);
    Supplier<String> code = code();
    if (code != null) {
      status.add("Robot/Code", code);
    }
    status.add("Robot/Battery", RobotController::getBatteryVoltage, "V", 0.1);
    actions = new Actions(advantageKit ? AdvantageKitLog.actions() : inputs -> {});
    boolean replaying = advantageKit && AdvantageKitLog.replaying();
    // Runs are timed on the robot's clock, which a replay reproduces.
    experiments =
        new Experiments(
            new Experiments.Robot(name, RobotBase.isSimulation(), code != null ? code.get() : ""),
            advantageKit ? AdvantageKitLog.experiments() : inputs -> {},
            RobotController::getTime,
            replaying,
            ended -> {
              // How a run ended, and why: in the Driver Station's console, and the log.
              System.out.println("Pitwall: " + ended);
              if (advantageKit) {
                AdvantageKitLog.runEnded(ended);
              }
            });
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
    // In WPILib's simulation the page can drive it; on a robot, or anything not known to be the
    // simulator, nothing can. A replay's Driver Station and hardware come from its log, so none
    // there either.
    Simulator simulated =
        Simulator.inWpilibSimulation() && !replaying
            ? simulatorControls(advantageKit, said::add)
            : null;
    simulator = simulated;
    // Devices' cards are read on the robot's clock, at most once a second while a page watches.
    devices = new Devices(RobotController::getTime);
    // Scenes' patches are timed on the robot's clock too: at most 20 a second, while a page shows.
    scenes = new Scenes(RobotController::getTime, System.out::println);
    page =
        new Page(
            status, actions, tuning, tee, experiments, Pitwall::mode, simulated, devices, scenes);
    // A page whose stream closes can't keep an experiment it started going, or drive a joystick.
    // The page's state is compared at most 20 times a second, on the robot's clock.
    broadcaster =
        new Broadcaster(
            Broadcaster.MAX_STREAMS,
            Broadcaster.MAILBOX,
            stream -> {
              experiments.lost(stream);
              if (simulated != null) {
                simulated.lost(stream);
              }
            },
            RobotController::getTime);
    server = replaying ? null : startPage(page, broadcaster, port);
  }

  /**
   * Serves the page, or says why it can't. The web server is the one built into Java; a runtime
   * trimmed without it (the {@code jdk.httpserver} module) only loses the page.
   */
  private static @Nullable PitwallServer startPage(Page page, Broadcaster broadcaster, int port) {
    try {
      return PitwallServer.startOrExplain(
              page, broadcaster, port, thisComputerOnly(), System.out::println)
          .orElse(null);
    } catch (LinkageError e) {
      System.out.println(
          "Pitwall's page is unavailable: this Java has no built-in web server (jdk.httpserver): "
              + e);
      return null;
    }
  }

  /**
   * The simulator's controls, on WPILib's simulation: what they do goes to the console, and to the
   * log with AdvantageKit (an output: a replay has no simulator, so it never writes them); the
   * page's Stop ends the log, then the program.
   *
   * @param said told each sentence the controls say, for the loop to log
   */
  private static Simulator simulatorControls(boolean advantageKit, Consumer<String> said) {
    Components components =
        new Components(
            sentence -> {
              System.out.println("Simulator: " + sentence);
              said.accept(sentence);
            });
    return Simulator.wpilib(
        RobotController::getTime,
        components,
        () -> {
          if (!Simulator.inWpilibSimulation()) {
            System.out.println("Pitwall: Stop refused: this isn't WPILib's simulation");
            return;
          }
          System.out.println("Simulator: stopped from Pitwall's page");
          if (advantageKit) {
            AdvantageKitLog.end();
          }
          System.exit(0);
        },
        on -> {
          if (advantageKit) {
            AdvantageKitLog.recordOutput("Pitwall/Simulator/Faults", on.toArray(String[]::new));
          }
        });
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
   * The experiments the page can run on the robot's rigs. Add them here: {@code
   * experiments().add(new Experiment("Step to 90° and back", steering, profile))}.
   */
  public Experiments experiments() {
    return experiments;
  }

  /**
   * The devices the page's Pit view shows a card for, each ending in what can be done about it. Add
   * them here: {@code devices().add(Devices.Device.named("venue", "Field", pit::card))}.
   */
  public Devices devices() {
    return devices;
  }

  /**
   * The scenes the page draws in 3D, each a view of its own: the field, the robot on it, what its
   * cameras see. Add them here: {@code scenes().add(Scene.named("world", "World").layer(...))}.
   * With none, the page has no such view.
   */
  public Scenes scenes() {
    return scenes;
  }

  /**
   * The simulator's controls, in WPILib's simulation: the robot's simulation declares the parts
   * that can fail here ({@code simulator.components()}). Empty on a robot, where nothing may write
   * the Driver Station's state, and while replaying a log.
   */
  public Optional<Simulator> simulator() {
    return Optional.ofNullable(simulator);
  }

  /**
   * The port the page is served on, or -1 when it isn't (while replaying, or it couldn't start).
   */
  public int port() {
    PitwallServer running = server;
    return running != null ? running.port() : -1;
  }

  /**
   * Applies the page's requests, then reads the status and tells any open page what changed. Call
   * it first thing in {@code robotPeriodic()}, before anything reads a configuration or the Driver
   * Station and before the command scheduler runs. When the robot disables, it prints (and logs)
   * what tuning changed, so it can be committed.
   */
  public void periodic() {
    // First, so everything after sees the Driver Station the page set, as AdvantageKit logs it.
    Simulator simulated = simulator;
    if (simulated != null) {
      simulated.periodic();
      logSaid();
    }
    boolean enabled = RobotState.isEnabled();
    boolean fieldConnected = RobotState.isFMSAttached();
    if (fieldConnected && !wasFieldConnected) {
      // Tuning returns every tuned value to its committed one now: say what they were first.
      report(
          "The field system connected: tuned values return to the committed configuration",
          tuning.report());
    }
    tuning.periodic(fieldConnected);
    fieldSinceStart |= fieldConnected;
    Actions.Gate gate =
        new Actions.Gate(
            enabled,
            RobotState.getRobotMode() == RobotMode.UTILITY,
            fieldConnected,
            fieldSinceStart);
    actions.periodic(gate);
    experiments.periodic(gate);
    status.periodic(broadcaster.watched());
    devices.periodic(broadcaster.watched());
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
    // While the simulator's clock is paused, every loop is compared: the one that paused it (its
    // last patch tells every page) and each step. A paused clock stands still, so none would come
    // due on it.
    if (simulated != null && simulated.isPaused()) {
      broadcaster.compareNow();
    }
    // Last, so an open page hears of everything this loop changed. With none open, nothing.
    broadcaster.periodic(page::sections, experiments.run(), scenes.feeds());
  }

  /**
   * Logs what the simulator's controls said since the last loop, every sentence, and an empty list
   * the loop after, so the same sentence said again later shows in the log.
   */
  private void logSaid() {
    if (said.isEmpty() && !saidLastLoop) {
      return;
    }
    if (advantageKit) {
      AdvantageKitLog.recordOutput("Pitwall/Simulator/Said", said.toArray(String[]::new));
    }
    saidLastLoop = !said.isEmpty();
    said.clear();
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
    broadcaster.close();
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
