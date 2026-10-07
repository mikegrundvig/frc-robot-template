package frc.robot.robots2026;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_FLEX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static frc.robot.subsystems.mechanisms.JointFixtures.unmotored;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;

import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import frc.robot.GoldenFile;
import frc.robot.RobotSim;
import frc.robot.SimulationTest;
import frc.robot.sim.JointsSimHarness;
import frc.robot.subsystems.mechanisms.JointRecordings;
import frc.robot.subsystems.mechanisms.Joints;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * 2611's 2026 joints, recorded ({@code JointRecordings}): what their motor controllers are told, as
 * their robot has them, with the pivot's loop closed on its duty-cycle encoder, and with the pivot
 * and the shooter on SPARKs; and a session in the simulator, the pivot seeded from its encoder and,
 * again, its loop closed on it. Both must come out as recorded: a change meant to change neither is
 * shown to change nothing. When a change is meant to move them, record them again with {@code
 * GOLDEN_UPDATE=1} and say why they moved (docs/testing.md).
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class Team2611RecordedSimTest {
  /** The pivot's loop closed on its duty-cycle encoder, on the robot, each loop. */
  private static RobotDefinition closed() {
    return Team2611.ROBOT.with(
        Team2611.PIVOT.encoderOn(
            MechanismWiring.PortEncoder.dutyCycle(1)
                .offset(Degrees.of(-296))
                .inverted()
                .closesLoop()));
  }

  /** The pivot on a NEO on a SPARK MAX, and the shooter on four Vortexes on SPARK Flexes. */
  private static RobotDefinition sparks() {
    Joint.Revolute pivot =
        unmotored(Team2611.PIVOT).motor(Motor.NEO, Gearing.of(25), SPARK_MAX.can(32).clockwise());
    Joint.Revolute shooter =
        unmotored(Team2611.SHOOTER)
            .motors(
                Motor.NEO_VORTEX,
                Gearing.of(1),
                SPARK_FLEX.can(41),
                SPARK_FLEX.can(42),
                SPARK_FLEX.can(43).clockwise(),
                SPARK_FLEX.can(44).clockwise());
    return Team2611.ROBOT.with(pivot, shooter);
  }

  @Test
  void theirSettingsAreAsRecorded() throws IOException {
    List<String> rows = new ArrayList<>();
    for (RobotDefinition robot : List.of(Team2611.ROBOT, closed(), sparks())) {
      rows.addAll(JointRecordings.settings(robot));
    }
    GoldenFile.check(getClass(), "team2611-settings.csv", JointRecordings.SETTINGS, rows);
  }

  @Test
  void theirSessionIsAsRecorded() throws IOException {
    List<String> rows = new ArrayList<>(session(Team2611.ROBOT));
    rows.addAll(session(closed()));
    GoldenFile.check(getClass(), "team2611-session.csv", rows.get(0), rows);
  }

  /**
   * The pivot out, its roller held until it's past 110°, the shooter spun up and fed, then the
   * pivot in, the roller stopped, and everything disabled: 5 s.
   */
  private static List<String> session(RobotDefinition robot) {
    List<Joint> joints = Joints.driven(robot);
    JointsSimHarness sim = new JointsSimHarness(robot);
    JointRecordings.Session session = new JointRecordings.Session(sim, joints);

    session.run(0.2);
    sim.scheduler.schedule(sim.joints.of(Team2611.PIVOT).goTo(Team2611.PIVOT_OUT));
    sim.scheduler.schedule(sim.joints.of(Team2611.INTAKE).run(Team2611.INTAKING));
    sim.scheduler.schedule(sim.joints.of(Team2611.SHOOTER).follow(() -> RPM.of(3500)));
    session.run(1.5);
    sim.scheduler.schedule(sim.joints.run(Team2611.FEEDING));
    session.run(1);
    sim.scheduler.schedule(sim.joints.of(Team2611.PIVOT).goTo(Team2611.PIVOT_IN));
    sim.scheduler.schedule(sim.joints.of(Team2611.INTAKE).run(Team2611.OUTTAKING));
    sim.scheduler.schedule(sim.joints.run(Team2611.UNJAMMING));
    session.run(1.3);
    sim.setEnabled(false);
    session.run(1);
    return session.rows();
  }
}
