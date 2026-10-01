package frc.robot.sim;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.SimulationTest;
import frc.robot.lib.localization.TagSolver;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.subsystems.vision.CameraIO;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.system.RobotController;

/**
 * The simulated cameras stamp each image when it was taken, as a real camera's time sync does.
 * PhotonVision's own simulation renders an image as of when it fell due but stamps it as of the
 * loop that renders it, up to a loop later; while the robot turns, every image would then put it
 * where it was a little earlier, the same way, as if the heading were off ({@code VisionSim}).
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class CameraTimingSimTest {
  /** How fast the robot is carried along the field, m/s: a loop late would be 4 cm. */
  private static final double SPEED = 2.0;

  private static final Pose2d START = new Pose2d(1.5, 4.0, Rotation2d.ZERO);

  /** Where the robot is carried to, this long after it set off. */
  private static Pose2d carried(long nanos) {
    return new Pose2d(
        START.getX() + SPEED * Math.max(0, nanos) / 1e9, START.getY(), START.getRotation());
  }

  @Test
  void eachImageShowsTheRobotWhereItWasAtTheImagesStamp() {
    try (SimulatedWorld world =
        new SimulatedWorld(
            RobotSim.FIELD,
            Robots.SIM.drivetrain(),
            Robots.SIM.body(),
            Robots.SIM.cameras(),
            SensorErrors.NONE)) {
      world.place(START);
      CameraIO[] cameras = world.cameras();
      for (int i = 0; i < RobotLoop.loops(1.0); i++) {
        step(world, START);
        for (CameraIO camera : cameras) {
          read(camera);
        }
      }
      long setOff = RobotController.getTime();
      List<Double> along = new ArrayList<>();
      for (int i = 0; i < RobotLoop.loops(0.5); i++) {
        step(world, carried(RobotController.getTime() + PERIOD_NANOS - setOff));
        for (int c = 0; c < cameras.length; c++) {
          Transform3d mount = Robots.SIM.cameras().get(c).robotToCamera();
          for (CameraIO.Frame frame : read(cameras[c]).frames) {
            if (!frame.hasMultiTag() || frame.timestampNanos() < setOff) {
              continue;
            }
            Transform3d seen = frame.multiTagFieldToCamera();
            Pose3d robot =
                TagSolver.robotFromCamera(
                    new Pose3d(seen.getTranslation(), seen.getRotation()), mount);
            along.add(robot.getX() - carried(frame.timestampNanos() - setOff).getX());
          }
        }
      }

      assertThat(along).as("images with several tags while carried").hasSizeGreaterThan(20);
      double mean = along.stream().mapToDouble(Double::doubleValue).average().orElseThrow();
      // Stamped up to a loop late, the robot would be about 1.5 to 2 cm behind where its stamp
      // says.
      assertThat(Math.abs(mean)).as("meters behind or ahead, on average").isLessThan(0.004);
    }
  }

  private static final long PERIOD_NANOS = Math.round(RobotLoop.PERIOD_SECONDS * 1e9);

  /** One loop, the robot carried to here by its end. */
  private static void step(SimulatedWorld world, Pose2d where) {
    RobotSim.stepClock();
    world.place(where);
    world.step(RobotController.getTime());
  }

  private static CameraIO.CameraIOInputs read(CameraIO camera) {
    CameraIO.CameraIOInputs inputs = new CameraIO.CameraIOInputs();
    camera.updateInputs(inputs);
    return inputs;
  }
}
