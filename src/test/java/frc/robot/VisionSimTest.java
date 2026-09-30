package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.sim.DriveSimulation;
import frc.robot.sim.SensorErrors;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * The whole robot in simulation, with PhotonVision's simulated cameras rendering the field from
 * where the robot really is: localization end to end, and the pre-event check that finds a camera
 * knocked out of calibration.
 */
@SimulationTest
class VisionSimTest {
  private static final double[][] ROUTE = {
    {1.5, -0.5, 0, 0},
    {1.0, 0, 0.7, 0},
    {1.5, 0, 0, 0.6},
    {2.0, 0.4, -0.5, 0.2},
    {1.5, -0.3, -0.3, -0.4},
    {1.0, 0, 0, 0}
  };

  @RegisterExtension final RobotSim sim = new RobotSim();

  private double error() {
    DriveSimulation simulation = sim.simulation();
    Pose2d truth = simulation.truePose();
    return sim.robot().drive().getPose().getTranslation().getDistance(truth.getTranslation());
  }

  private double drive() {
    sim.run(1.0); // placed, and the cameras settle
    RobotSim.setEnabled(true);
    double worst = 0;
    for (double[] leg : ROUTE) {
      sim.driver().setLeftY(-leg[1]);
      sim.driver().setLeftX(-leg[2]);
      sim.driver().setRightX(-leg[3]);
      sim.driver().notifyNewData();
      for (int i = 0; i < RobotLoop.loops(leg[0]); i++) {
        sim.loops(1);
        worst = Math.max(worst, error());
      }
    }
    return worst;
  }

  /** How far a camera's tags land from their real places, against the worst of the others. */
  private double standsOut(String camera) {
    double others = 0;
    for (var config : Robots.SIM.cameras()) {
      if (!config.name().equals(camera)) {
        others = Math.max(others, sim.robot().vision().meanTagResidualMeters(config.name()));
      }
    }
    return sim.robot().vision().meanTagResidualMeters(camera) / others;
  }

  /**
   * A season-worn robot driving a minute's worth of moves, four cameras on it: the design's targets
   * are 8 cm while moving and 3 cm standing still.
   */
  @Test
  void theCamerasHoldTheEstimateToTheTruthAsTheRobotDrives() {
    sim.build(SensorErrors.TYPICAL);

    double worst = drive();

    assertThat(worst).as("worst while driving").isLessThan(0.08);
    assertThat(error()).as("stopped").isLessThan(0.03);
    LocalizationEstimate estimate = sim.robot().estimate();
    assertThat(estimate.mode()).isEqualTo(LocalizationEstimate.Mode.TRACKING);
    LocalizationEstimate.Health health = estimate.health();
    assertThat(health.accepted()).isGreaterThan(19 * health.rejected()).isGreaterThan(500);
  }

  /** 2026's calibration drifted all season; a camera 4 in low stands out before a match. */
  @Test
  void aCameraKnockedLowStandsOutInItsTagResiduals() {
    sim.build(
        SensorErrors.TYPICAL.withCameraMountError(
            "front-left", new Transform3d(0, 0, -0.1016, Rotation3d.ZERO)));

    double worst = drive();

    assertThat(standsOut("front-left")).isGreaterThan(2.5);
    assertThat(worst).as("the other cameras and the gate keep the estimate").isLessThan(0.08);
  }

  /** On a ramp, nose up and leaning: the images are solved with the robot tilted as it is. */
  @Test
  void aTiltedRobotStillSeesWhereItIs() {
    sim.build(SensorErrors.TYPICAL);
    sim.run(1.0);
    int before = sim.robot().estimate().health().accepted();

    sim.simulation().tilt(Math.toRadians(-8), Math.toRadians(3));
    sim.run(1.0);

    assertThat(sim.robot().estimate().health().accepted() - before).isGreaterThan(50);
    assertThat(error()).isLessThan(0.03);
  }

  @Test
  void aCameraTurnedTwoDegreesStandsOutToo() {
    sim.build(
        SensorErrors.TYPICAL.withCameraMountError(
            "front-left", new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.toRadians(2)))));

    drive();

    assertThat(standsOut("front-left")).isGreaterThan(1.4);
  }
}
