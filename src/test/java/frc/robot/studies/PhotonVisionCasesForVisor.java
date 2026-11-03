package frc.robot.studies;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.subsystems.vision.VisionSim;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfEnvironmentVariable;
import org.junit.jupiter.api.extension.ExtendWith;
import org.opencv.core.Point;
import org.photonvision.targeting.PnpResult;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * Writes PhotonVision's own view of tags, for Visor's tests to check against: across 2026's field,
 * the corners PhotonVision's simulation sees (pixel noise and all) and PhotonVision's own solve of
 * each tag's two poses from them. Visor solves the same corners, and its robot client must arrive
 * at the same transforms. Runs only when VISOR_CASES names the file to write.
 */
@ExtendWith(RobotSim.class)
@EnabledIfEnvironmentVariable(named = "VISOR_CASES", matches = ".+")
class PhotonVisionCasesForVisor {
  private static final FieldModel FIELD = TagLayouts.named("FRC_2026_REBUILT_WELDED");
  private static final CameraConfig.Sensor SENSOR =
      ExampleRobot.SIM.definition().cameras().get(0).sensor();
  private static final int CASES = 2000;

  /**
   * PhotonVision lists a tag's corners from its model (TargetModel.kAprilTag36h11): bottom left,
   * bottom right, top right, top left, as the printed tag faces you (its y is to your right). Visor
   * sends OpenCV's order: top left, top right, bottom right, bottom left.
   */
  private static final int[] OPENCV_ORDER = {3, 2, 1, 0};

  @Test
  void write() throws IOException {
    CameraImageSim study =
        new CameraImageSim(FIELD, SENSOR, Conditions.nominal(CameraStudyConfig.DEFAULTS));
    var intrinsics = VisionSim.properties(SENSOR, 0).getIntrinsics();
    StringBuilder json = new StringBuilder();
    json.append(
        String.format(
            Locale.ROOT,
            "{\"width\": %d, \"height\": %d, \"fx\": %.17g, \"fy\": %.17g, \"cx\": %.17g,"
                + " \"cy\": %.17g, \"tag_size_m\": 0.1651, \"cases\": [\n",
            SENSOR.widthPixels(),
            SENSOR.heightPixels(),
            intrinsics.get(0, 0),
            intrinsics.get(1, 1),
            intrinsics.get(0, 2),
            intrinsics.get(1, 2)));
    SplittableRandom random = new SplittableRandom(2027);
    int written = 0;
    for (long image = 0; written < CASES; image++) {
      // A camera anywhere on the field, at a robot's height, looking any way, tilted a little.
      Pose3d camera =
          new Pose3d(
              random.nextDouble(0.5, 16.0),
              random.nextDouble(0.5, 7.5),
              random.nextDouble(0.15, 0.9),
              new Rotation3d(
                  random.nextDouble(-0.2, 0.2),
                  -random.nextDouble(-0.1, 0.5),
                  random.nextDouble(-Math.PI, Math.PI)));
      for (CameraImageSim.Seen seen : study.seen(camera, image, List.of())) {
        PnpResult solved = seen.solved();
        if (solved == null || written >= CASES) {
          continue;
        }
        if (written > 0) {
          json.append(",\n");
        }
        json.append("{\"tag\": ").append(seen.tag().id()).append(", \"corners\": [");
        for (int i = 0; i < 4; i++) {
          Point corner = seen.corners()[OPENCV_ORDER[i]];
          json.append(i > 0 ? ", " : "")
              .append(number(corner.x))
              .append(", ")
              .append(number(corner.y));
        }
        json.append("], \"best\": ")
            .append(transform(solved.best))
            .append(", \"alt\": ")
            .append(transform(solved.alt))
            .append(", \"best_error\": ")
            .append(number(solved.bestReprojErr))
            .append(", \"alt_error\": ")
            .append(number(solved.altReprojErr))
            .append(", \"ambiguity\": ")
            .append(number(solved.ambiguity))
            .append('}');
        written++;
      }
    }
    json.append("\n]}\n");
    Files.writeString(Path.of(System.getenv("VISOR_CASES")), json);
  }

  private static String transform(Transform3d transform) {
    Quaternion q = transform.getRotation().getQuaternion();
    return "["
        + String.join(
            ", ",
            number(transform.getX()),
            number(transform.getY()),
            number(transform.getZ()),
            number(q.getW()),
            number(q.getX()),
            number(q.getY()),
            number(q.getZ()))
        + "]";
  }

  private static String number(double value) {
    return String.format(Locale.ROOT, "%.17g", value);
  }
}
