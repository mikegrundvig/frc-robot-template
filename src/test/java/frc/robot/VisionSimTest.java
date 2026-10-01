package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.sim.DriveSimulation;
import frc.robot.sim.SensorErrors;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.util.AlertDataJNI;

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

  /** The most each camera's measurements were doubted for its mount, at any loop of the drive. */
  private final Map<String, Double> mostDoubted = new HashMap<>();

  private double drive() {
    return drive(1);
  }

  /** The mean error over the last time through the route of the last drive. */
  private double lastLapMean;

  /**
   * Drives the route, there and back again as many times as asked: about 8.5 s each.
   *
   * @return the worst error on the last time through
   */
  private double drive(int laps) {
    sim.run(1.0); // placed, and the cameras settle
    RobotSim.setEnabled(true);
    double worst = 0;
    for (int lap = 0; lap < laps; lap++) {
      double back = lap % 2 == 0 ? 1 : -1; // odd laps drive back where the even ones went
      worst = 0;
      double sum = 0;
      int loops = 0;
      for (double[] leg : ROUTE) {
        sim.driver().setLeftY(-leg[1] * back);
        sim.driver().setLeftX(-leg[2] * back);
        sim.driver().setRightX(-leg[3]);
        sim.driver().notifyNewData();
        for (int i = 0; i < RobotLoop.loops(leg[0]); i++) {
          sim.loops(1);
          worst = Math.max(worst, error());
          sum += error();
          loops++;
          for (CameraConfig camera : Robots.SIM.cameras()) {
            mostDoubted.merge(
                camera.name(), sim.robot().vision().mountDoubtMeters(camera.name()), Math::max);
          }
        }
      }
      lastLapMean = sum / loops;
    }
    return worst;
  }

  /** The alerts a dashboard shows now. */
  private static List<String> activeAlerts() {
    List<String> active = new ArrayList<>();
    for (AlertDataJNI.AlertInfo alert : AlertDataJNI.getAlerts()) {
      if (alert.activeStartTime != 0) {
        active.add(alert.text);
      }
    }
    return active;
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

  /**
   * Four cameras as calibrated, driving half a minute on a season-worn robot: each is judged by
   * where its tags land, and none is doubted, ever, or said to be out of calibration.
   */
  @Test
  void calibratedCamerasKeepTheirFullTrust() {
    sim.build(SensorErrors.TYPICAL);

    double worst = drive(4);

    for (CameraConfig camera : Robots.SIM.cameras()) {
      assertThat(sim.robot().vision().mountOffsetMeters(camera.name()))
          .as(camera.name() + ": judged, and within the tolerance")
          .isLessThan(Robots.SIM.vision().mountToleranceMeters());
      assertThat(mostDoubted.get(camera.name())).as(camera.name()).isZero();
    }
    assertThat(activeAlerts()).noneMatch(text -> text.contains("out of calibration"));
    assertThat(worst).isLessThan(0.08);
  }

  /**
   * A camera knocked 4 in low is out of calibration, and the alert says so. Knocked straight down,
   * it still places the robot right on the floor, so it counts as much as before.
   */
  @Test
  void aCameraKnockedLowIsOutOfCalibrationButStillTrusted() {
    sim.build(
        SensorErrors.TYPICAL.withCameraMountError(
            "front-left", new Transform3d(0, 0, -0.1016, Rotation3d.ZERO)));

    double worst = drive(3);

    assertThat(sim.robot().vision().mountOffsetMeters("front-left"))
        .isCloseTo(0.1016, within(0.02));
    assertThat(activeAlerts())
        .anyMatch(text -> text.startsWith("Camera front-left is out of calibration: its tags land"))
        .noneMatch(text -> text.contains("counts for less"));
    assertThat(mostDoubted.values()).allMatch(doubt -> doubt == 0);
    assertThat(worst).isLessThan(0.08);
  }

  /**
   * A camera turned 4° on its mount puts the robot off to the side by 7 cm for every meter to its
   * tags, and turns its own headings by 4°. Once judged (about 20 s in: it sees tags only part of
   * the time, and only while another camera holds the pose), it's found out of calibration and
   * counts for less, and the back cameras never are.
   *
   * <p>But from one camera, a turned mount and a heading error look the same, and only a
   * measurement with a heading of its own turns the estimate: its headings pull the estimate's
   * heading a degree or more its way, its front neighbor's tags then land off too, and the neighbor
   * is doubted about as much. So here the check doesn't make the estimate better (the last lap's
   * mean error is about 5 cm with it, 4 cm trusted as before); comparing the cameras with each
   * other, not with the estimate, is what would.
   */
  @Test
  void aCameraTurnedOnItsMountIsFoundOutOfCalibration() {
    Transform3d turned = new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.toRadians(4)));
    sim.build(SensorErrors.TYPICAL.withCameraMountError("front-left", turned));

    drive(4);

    assertThat(activeAlerts())
        .anyMatch(
            text ->
                text.startsWith("Camera front-left is out of calibration")
                    && text.contains("so it counts for less"));
    assertThat(sim.robot().vision().mountDoubtMeters("front-left")).isGreaterThan(0.04);
    assertThat(mostDoubted.get("back-left")).isZero();
    assertThat(mostDoubted.get("back-right")).isZero();
    assertThat(lastLapMean).as("the last lap's mean error").isLessThan(0.07);
  }

  /**
   * A camera turned 2°: its headings pull the estimate's heading partway its way, so its tags land
   * little farther off than its neighbor's, and it doesn't stand out on its own. The estimate stays
   * within a few centimeters.
   */
  @Test
  void aCameraTurnedTwoDegreesKeepsTheEstimateWithinCentimeters() {
    sim.build(
        SensorErrors.TYPICAL.withCameraMountError(
            "front-left", new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.toRadians(2)))));

    double worst = drive();

    assertThat(lastLapMean).as("the lap's mean error").isLessThan(0.035);
    assertThat(worst).as("worst").isLessThan(0.08);
  }
}
