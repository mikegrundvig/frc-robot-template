package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.math.geometry.Pose2d;

/**
 * The swerve test chassis in the simulator, as {@code ./gradlew run} runs it: with the team's
 * controls, it drives where the driver's stick points, no faster than its bench limit. It starts on
 * the blue center start, its bumpers against the hub, so it backs away from it.
 */
@SimulationTest
class ChassisRobotSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  @Test
  void itDrivesWhereTheStickPointsAtItsBenchSpeed() {
    sim.build(ChassisRobot.CONFIG, Rebuilt::game, Controls::new, SensorErrors.NONE);
    sim.run(0.2);
    Pose2d start = sim.simulation().truePose();

    RobotSim.setEnabled(true);
    sim.driver().setLeftY(1.0); // pulled back: a gamepad's Y is positive pulled toward the driver
    sim.driver().notifyNewData();
    sim.run(1.5);
    Pose2d end = sim.simulation().truePose();

    double back = start.getX() - end.getX(); // toward the blue drivers, the field's -x
    double sideways = Math.abs(end.getY() - start.getY());
    assertThat(back).as("no more than 1.5 m/s").isBetween(1.0, 1.5 * 1.5);
    assertThat(sideways).isLessThan(0.1);
  }
}
