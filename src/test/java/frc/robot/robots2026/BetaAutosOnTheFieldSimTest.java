package frc.robot.robots2026;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.AutoCheck;
import com.michaelgrundvig.frc.runtime.Clearance;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import frc.robot.BetaControls;
import frc.robot.BetaRobot;
import java.util.stream.Stream;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Beta's autos run on the season's own field, with the hub, the trenches, the towers and the walls
 * standing, where the other tests of them drive an open floor. Each must run clean: end at its last
 * waypoint, and never come near enough to anything to touch it.
 */
class BetaAutosOnTheFieldSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  private static final RobotConfig BETA = Parts.withoutCameras(BetaRobot.CONFIG);
  private static final FieldModel FIELD = Rebuilt.field();

  /** How near its last waypoint an auto ends, in meters: its aiming turns it, not moves it. */
  private static final double ENDS_WITHIN = 0.08;

  /**
   * The least room the bumpers must keep from everything, in meters. The paths are laid about 10 cm
   * clear. The tightest spot is not on a path: Left Single ends on its start in the trench's mouth
   * and turns there to aim, and a corner of its bumpers swings to 7 cm from the wall.
   */
  private static final double STAYS_CLEAR_BY = 0.05;

  static Stream<Arguments> autos() {
    Place start = Place.at("Left", Rebuilt.STARTS.left().poseFor(BetaRobot.BODY.bumpers()));
    Place shot = Place.at("Shot", new Pose2d(3.0, 6.069, Rotation2d.fromDegrees(136)));
    return Stream.of(
        Arguments.argumentSet("Left Single on blue", "Left Single", Alliance.BLUE, start),
        Arguments.argumentSet("Left Single on red", "Left Single", Alliance.RED, start),
        Arguments.argumentSet("Left Double on blue", "Left Double", Alliance.BLUE, shot),
        Arguments.argumentSet("Left Double on red", "Left Double", Alliance.RED, shot));
  }

  @ParameterizedTest
  @MethodSource("autos")
  void theAutoRunsCleanAmongWhatStandsOnTheField(String auto, Alliance alliance, Place last) {
    RobotSim.setEnabled(false);
    sim.build(BETA, Rebuilt.GAME, BetaControls::new, SensorErrors.NONE);
    RobotSim.station(alliance == Alliance.RED ? AllianceStationID.RED_2 : AllianceStationID.BLUE_2);
    sim.autos().choose(auto);
    sim.loops(5);
    Clearance clearance = new Clearance(FIELD, BetaRobot.BODY);

    double leastRoom = Double.POSITIVE_INFINITY;
    Pose2d tightest = sim.simulation().truePose();
    RobotSim.enable(RobotMode.AUTONOMOUS, true);
    for (int loop = 0; loop < RobotLoop.loops(AutoCheck.AUTONOMOUS_SECONDS); loop++) {
      sim.loop();
      Pose2d truth = sim.simulation().truePose();
      double room = clearance.at(truth);
      if (room < leastRoom) {
        leastRoom = room;
        tightest = truth;
      }
    }
    Pose2d ended = sim.simulation().truePose();
    RobotSim.enable(RobotMode.AUTONOMOUS, false);
    sim.loops(3);

    assertThat(leastRoom)
        .as("meters of room at the tightest spot, %s", tightest)
        .isGreaterThan(STAYS_CLEAR_BY);
    Pose2d expected = last.on(FIELD, alliance);
    assertThat(ended.getTranslation().getDistance(expected.getTranslation()))
        .as("meters from its last waypoint %s", expected)
        .isLessThan(ENDS_WITHIN);
  }
}
