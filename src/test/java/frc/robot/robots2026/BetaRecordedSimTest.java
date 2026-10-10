package frc.robot.robots2026;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_FLEX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;

import com.michaelgrundvig.frc.runtime.GoldenFile;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointRecordings;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import frc.robot.BetaRobot;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * A team's 2026 robot's joints, recorded ({@code JointRecordings}): what their motor controllers
 * are told, as their robot has them, with the pivot's loop closed on its duty-cycle encoder, and
 * with the pivot and the shooter on SPARKs; and a session in the simulator, the pivot seeded from
 * its encoder and, again, its loop closed on it. Both must come out as recorded: a change meant to
 * change neither is shown to change nothing. When a change is meant to move them, record them again
 * with {@code GOLDEN_UPDATE=1} and say why they moved (docs/testing.md).
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class BetaRecordedSimTest {
  /** The pivot's loop closed on its duty-cycle encoder, on the robot, each loop. */
  private static RobotDefinition closed() {
    return BetaRobot.ROBOT.with(
        BetaRobot.PIVOT.encoderOn(
            MechanismWiring.PortEncoder.dutyCycle(1)
                .offset(Degrees.of(-296))
                .inverted()
                .closesLoop()));
  }

  /** The pivot on a NEO on a SPARK MAX, and the shooter on four Vortexes on SPARK Flexes. */
  private static RobotDefinition sparks() {
    Joint.Revolute pivot =
        unmotored(BetaRobot.PIVOT).motor(Motor.NEO, Gearing.of(25), SPARK_MAX.can(32).clockwise());
    Joint.Revolute shooter =
        unmotored(BetaRobot.SHOOTER)
            .motors(
                Motor.NEO_VORTEX,
                Gearing.of(1),
                SPARK_FLEX.can(41),
                SPARK_FLEX.can(42),
                SPARK_FLEX.can(43).clockwise(),
                SPARK_FLEX.can(44).clockwise());
    return BetaRobot.ROBOT.with(pivot, shooter);
  }

  @Test
  void theirSettingsAreAsRecorded() throws IOException {
    List<String> rows = new ArrayList<>();
    for (RobotDefinition robot : List.of(BetaRobot.ROBOT, closed(), sparks())) {
      rows.addAll(JointRecordings.settings(robot));
    }
    GoldenFile.check(getClass(), "beta-settings.csv", JointRecordings.SETTINGS, rows);
  }

  @Test
  void theirSessionIsAsRecorded() throws IOException {
    List<String> rows = new ArrayList<>(session(BetaRobot.ROBOT));
    rows.addAll(session(closed()));
    GoldenFile.check(getClass(), "beta-session.csv", rows.get(0), rows);
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
    sim.scheduler.schedule(sim.joints.of(BetaRobot.PIVOT).goTo(BetaRobot.PIVOT_OUT));
    sim.scheduler.schedule(sim.joints.of(BetaRobot.INTAKE).run(BetaRobot.INTAKING));
    sim.scheduler.schedule(sim.joints.of(BetaRobot.SHOOTER).follow(() -> RPM.of(3500)));
    session.run(1.5);
    sim.scheduler.schedule(sim.joints.run(BetaRobot.FEEDING));
    session.run(1);
    sim.scheduler.schedule(sim.joints.of(BetaRobot.PIVOT).goTo(BetaRobot.PIVOT_IN));
    sim.scheduler.schedule(sim.joints.of(BetaRobot.INTAKE).run(BetaRobot.OUTTAKING));
    sim.scheduler.schedule(sim.joints.run(BetaRobot.UNJAMMING));
    session.run(1.3);
    sim.setEnabled(false);
    session.run(1);
    return session.rows();
  }
}
