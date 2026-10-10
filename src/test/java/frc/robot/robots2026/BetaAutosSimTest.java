package frc.robot.robots2026;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Placement;
import frc.robot.AutoCheck;
import frc.robot.BetaControls;
import frc.robot.BetaRobot;
import frc.robot.RobotConfig;
import frc.robot.RobotSim;
import frc.robot.autos.Autos;
import frc.robot.game.Rebuilt;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.sim.JointSimulation;
import frc.robot.subsystems.drive.DriveMove;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.command3.SchedulerEvent;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Beta's two ported autos, Left Single and Left Double (the robot's original code,
 * deploy/pathplanner/autos/), run on the template's Beta robot in the simulator, each on both
 * alliances, as the pit chooses them and the Driver Station starts them. Checked against the
 * simulator's truth: where each ends against its path's last waypoint, that the intake is out
 * before the first path, and that it feeds only once the robot is aimed and the shooter at speed. A
 * test fixture, not 2026 play.
 */
class BetaAutosSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  private static final RobotConfig BETA = Parts.withoutCameras(BetaRobot.CONFIG);

  /** The last waypoint of Left Single's path: back on the start, as Left Half 2.path ends. */
  private static final Place SINGLE_LAST = Rebuilt.starts(BetaRobot.BODY).left();

  /** The last waypoint of Left Double's third path (EXP Left 3.path): (3.0, 6.069), facing 136°. */
  private static final Place DOUBLE_LAST =
      Place.at("EXP Left 3 end", new Pose2d(3.0, 6.069, Rotation2d.fromDegrees(136)));

  /** How near its last waypoint an auto ends, in meters: its aiming turns it, not moves it. */
  private static final double ENDS_WITHIN = 0.08;

  /** How long the robot may feed while it isn't aimed and up to speed, in seconds. */
  private static final double UNREADY_FEEDING_SECONDS = 0.2;

  /**
   * What happened in one run of an auto, from the simulator's truth.
   *
   * @param ended where the robot really ended
   * @param pivotAtFirstPath the intake pivot's angle when the first drive move started, degrees
   * @param feedStarts when each burst of feeding began, in seconds from the start of autonomous
   * @param firstPathEnd when the first drive move arrived, in seconds
   * @param lastPathEnd when the last drive move arrived, in seconds
   * @param aimAtFirstFeed the aiming error as the first burst began, degrees
   * @param shooterAtFirstFeed the shooter's speed as it began, rpm
   * @param unreadyFeeding the longest the robot fed unaimed or short of speed, seconds
   * @param result how the auto ended
   * @param moves each drive move's outcome line
   */
  record Run(
      Pose2d ended,
      double pivotAtFirstPath,
      List<Double> feedStarts,
      double firstPathEnd,
      double lastPathEnd,
      double aimAtFirstFeed,
      double shooterAtFirstFeed,
      double unreadyFeeding,
      Autos.Run result,
      List<String> moves) {}

  private double rpm(JointSimulation truth, com.michaelgrundvig.frc.world.robot.Joint joint) {
    return RPM.convertFrom(truth.velocity(joint), RadiansPerSecond);
  }

  /** How far the robot's back is from the hub, from where it really is, in degrees. */
  private static double aimError(Pose2d truth, Alliance alliance) {
    Translation2d hub = Rebuilt.HUB.on(RobotSim.FIELD, alliance).getTranslation();
    Translation2d to = hub.minus(truth.getTranslation());
    double backAtHub = Math.toDegrees(Math.atan2(to.getY(), to.getX())) + 180;
    return Math.IEEEremainder(truth.getRotation().getDegrees() - backAtHub, 360);
  }

  private void build() {
    sim.build(BETA, SensorErrors.NONE);
    sim.controls(BetaControls::new);
  }

  private void station(Alliance alliance) {
    RobotSim.station(alliance == Alliance.RED ? AllianceStationID.RED_2 : AllianceStationID.BLUE_2);
  }

  /** Chooses an auto in the pit and runs a whole autonomous period on a fresh robot. */
  private Run run(String auto, Alliance alliance) {
    RobotSim.setEnabled(false);
    build();
    station(alliance);
    sim.autos().choose(auto);
    sim.loops(5);
    return autonomous(alliance);
  }

  /** Runs autonomous for its 15 s from where the robot stands, watching the truth each loop. */
  private Run autonomous(Alliance alliance) {
    JointSimulation truth = Objects.requireNonNull(sim.world().joints());
    List<String> moves = new ArrayList<>();
    double[] pivotAtFirstPath = {Double.NaN};
    double[] pathEnds = {Double.NaN, Double.NaN};
    double[] now = {0};
    sim.scheduler()
        .addEventListener(
            event -> {
              if (event instanceof SchedulerEvent.Mounted mounted
                  && mounted.command() instanceof DriveMove
                  && Double.isNaN(pivotAtFirstPath[0])) {
                pivotAtFirstPath[0] = Math.toDegrees(truth.position(BetaRobot.PIVOT));
              }
              if (event instanceof SchedulerEvent.Completed done
                  && done.command() instanceof DriveMove move) {
                moves.add(move.reason());
                if (Double.isNaN(pathEnds[0])) {
                  pathEnds[0] = now[0];
                }
                pathEnds[1] = now[0];
              }
            });
    List<Double> feedStarts = new ArrayList<>();
    boolean feeding = false;
    double unready = 0;
    double longestUnready = 0;
    double aimAtFirst = Double.NaN;
    double shooterAtFirst = Double.NaN;
    RobotSim.enable(RobotMode.AUTONOMOUS, true);
    for (int i = 0; i < RobotLoop.loops(AutoCheck.AUTONOMOUS_SECONDS); i++) {
      sim.loop();
      now[0] = i * RobotLoop.PERIOD_SECONDS;
      boolean feedingNow = rpm(truth, BetaRobot.FULL_SEND) > 200;
      Pose2d pose = sim.simulation().truePose();
      double aim = aimError(pose, alliance);
      double shooter = rpm(truth, BetaRobot.SHOOTER);
      if (feedingNow && !feeding) {
        feedStarts.add(now[0]);
        if (Double.isNaN(aimAtFirst)) {
          aimAtFirst = aim;
          shooterAtFirst = shooter;
        }
      }
      feeding = feedingNow;
      boolean ready = Math.abs(aim) <= 5 && shooter >= 0.9 * BetaControls.SHOT_RPM;
      unready = feedingNow && !ready ? unready + RobotLoop.PERIOD_SECONDS : 0;
      longestUnready = Math.max(longestUnready, unready);
    }
    Pose2d ended = sim.simulation().truePose();
    RobotSim.enable(RobotMode.AUTONOMOUS, false);
    sim.loops(3);
    return new Run(
        ended,
        pivotAtFirstPath[0],
        List.copyOf(feedStarts),
        pathEnds[0],
        pathEnds[1],
        aimAtFirst,
        shooterAtFirst,
        longestUnready,
        sim.autos().lastRun().orElseThrow(),
        List.copyOf(moves));
  }

  private void assertRan(String auto, Alliance alliance, Place last, int bursts) {
    Run run = run(auto, alliance);
    String what = auto + " on " + alliance;
    Pose2d expected = last.on(RobotSim.FIELD, alliance);

    assertThat(run.ended().getTranslation().getDistance(expected.getTranslation()))
        .as(what + " ends at its last waypoint " + expected)
        .isLessThan(ENDS_WITHIN);
    assertThat(run.result().ending()).as(what).isEqualTo(Autos.Ending.CUT_OFF);
    assertThat(run.pivotAtFirstPath())
        .as(what + ": intake out before the first path")
        .isGreaterThan(125);
    assertThat(run.moves()).as(what).allMatch(line -> line.contains(": ARRIVED in "));
    assertThat(run.feedStarts()).as(what + ": bursts of feeding").hasSize(bursts);
    assertThat(run.feedStarts().get(0))
        .as(what + ": first fed after the first path")
        .isGreaterThan(run.firstPathEnd());
    assertThat(Math.abs(run.aimAtFirstFeed())).as(what + ": aimed at the first").isLessThan(4.5);
    assertThat(run.shooterAtFirstFeed())
        .as(what + ": at speed at the first")
        .isCloseTo(BetaControls.SHOT_RPM, within(0.05 * BetaControls.SHOT_RPM));
    assertThat(run.unreadyFeeding())
        .as(what + ": fed unaimed or slow for")
        .isLessThan(UNREADY_FEEDING_SECONDS);
  }

  @Test
  void leftSingleDrivesItsPathsAndShootsOnTheStartOnEitherAlliance() {
    for (Alliance alliance : Alliance.values()) {
      assertRan("Left Single", alliance, SINGLE_LAST, 1);
    }
  }

  @Test
  void leftDoubleDrivesItsPathsAndShootsTwiceOnEitherAlliance() {
    for (Alliance alliance : Alliance.values()) {
      assertRan("Left Double", alliance, DOUBLE_LAST, 2);
    }
  }

  /** The pit has Do nothing first, then their two autos, and Do nothing is what it starts with. */
  @Test
  void thePitChoosesAmongDoNothingAndTheirTwo() {
    build();

    assertThat(sim.autos().names()).containsExactly("Do nothing", "Left Single", "Left Double");
    assertThat(sim.autos().chosen().name()).isEqualTo("Do nothing");
  }

  /** Nothing chosen, autonomous runs Do nothing: no move, no deploy, and it ends cleanly. */
  @Test
  void choosingNothingRunsDoNothing() {
    build();
    sim.loops(5);
    JointSimulation truth = Objects.requireNonNull(sim.world().joints());
    Pose2d before = sim.simulation().truePose();
    List<String> moves = new ArrayList<>();
    sim.scheduler()
        .addEventListener(
            event -> {
              if (event instanceof SchedulerEvent.Mounted mounted
                  && mounted.command() instanceof DriveMove move) {
                moves.add(move.name());
              }
            });

    RobotSim.enable(RobotMode.AUTONOMOUS, true);
    sim.run(AutoCheck.AUTONOMOUS_SECONDS);
    RobotSim.enable(RobotMode.AUTONOMOUS, false);
    sim.loops(3);

    assertThat(sim.autos().lastRun().orElseThrow().describe()).startsWith("Do nothing: COMPLETED");
    assertThat(moves).isEmpty();
    assertThat(sim.simulation().truePose()).isEqualTo(before);
    assertThat(Math.toDegrees(truth.position(BetaRobot.PIVOT)))
        .as("the intake stays in")
        .isCloseTo(1.5, within(2.0));
  }

  /**
   * Autonomous run twice in a row, on the same choice and with no restart, is the same as once:
   * each run makes its own command, nothing is composed twice, and every move ends as it should.
   * The second starts where the first left the robot, so it ends where its paths end from there.
   */
  @Test
  void runningAutonomousTwiceInARowWorks() {
    build();
    station(Alliance.BLUE);
    sim.autos().choose("Left Double");
    sim.loops(5);

    Run first = autonomous(Alliance.BLUE);
    sim.loops(5);
    Run second = autonomous(Alliance.BLUE);

    for (Run run : List.of(first, second)) {
      assertThat(run.result().ending()).isEqualTo(Autos.Ending.CUT_OFF);
      assertThat(run.moves()).hasSize(3).allMatch(line -> line.contains(": ARRIVED in "));
      assertThat(run.feedStarts()).hasSize(2);
    }
    assertThat(second.ended().getTranslation().getDistance(first.ended().getTranslation()))
        .as("the same end both times")
        .isLessThan(0.1);
  }

  /**
   * The placement check reads ready on the auto's start, on either alliance, and not once the drive
   * team has set the robot down on another start; set right, it reads ready again.
   */
  @Test
  void thePlacementCheckReadsTheAutosStart() {
    for (Alliance alliance : Alliance.values()) {
      RobotSim.setEnabled(false);
      build();
      station(alliance);
      sim.autos().choose("Left Double");
      sim.loops(5);
      Placement onStart = sim.placement();
      Pose2d center = Rebuilt.starts(BetaRobot.BODY).center().on(RobotSim.FIELD, alliance);
      Pose2d left = Rebuilt.starts(BetaRobot.BODY).left().on(RobotSim.FIELD, alliance);

      sim.placeAt(center);
      sim.loops(3);
      Placement onAnother = sim.placement();
      sim.placeAt(left);
      sim.loops(3);
      Placement setRight = sim.placement();

      assertThat(onStart.status())
          .as(alliance + ": placed for the auto")
          .isEqualTo(Placement.Status.READY);
      assertThat(onStart.describe()).startsWith("Ready on Left: ");
      assertThat(onAnother.status()).isEqualTo(Placement.Status.WRONG_START);
      assertThat(onAnother.describe()).isEqualTo("On Center, not the expected start");
      assertThat(setRight.status()).isEqualTo(Placement.Status.READY);
    }
  }

  /**
   * Their autos pass the template's own check, on both alliances: the one to run before a match.
   */
  @Test
  void theirAutosPassTheTemplatesCheck() {
    List<AutoCheck.Result> results = AutoCheck.check(sim, BETA, BetaControls::new);

    assertThat(results).hasSize(6);
    assertThat(results)
        .filteredOn(result -> result.auto().equals("Do nothing"))
        .allMatch(result -> result.moves().isEmpty());
  }
}
