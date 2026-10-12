package frc.robot.studies;

import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.SplittableRandom;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Transform3d;

/**
 * What the estimator study puts between the simulated robot and WPILib's estimators: a tap on each
 * camera's IO that keeps every image for the estimators that solve images their own way, and can
 * make a camera misread a tag ({@link Cameras}). The yaw WPILib's estimator is turned by is the
 * localizer's own to work out ({@code WpilibLocalizer}).
 */
final class EstimatorRig {
  private EstimatorRig() {}

  /** The IMU a robot's code reads its turning and tilt from this loop: the gyro, or the backup. */
  static ImuReadings imu(DrivetrainReadings loop) {
    if (loop.gyro().usable()) {
      return loop.gyro();
    }
    return loop.backupImu().usable() ? loop.backupImu() : ImuReadings.NONE;
  }

  /**
   * A tap on every camera's IO: each loop's images as the cameras sent them, and the misreads a
   * scenario asks of them. A misread changes what the camera sends, so the robot's own pipeline and
   * every other estimator get the same wrong image.
   */
  static final class Cameras {
    /** The ambiguity a camera reports for a tag whose two poses it took the wrong way round. */
    static final double FLIPPED_AMBIGUITY = 0.15;

    /** How a camera misreads a tag. */
    enum Misread {
      /** The nearest tag alone, read as another tag of the field: the pose lands meters off. */
      WRONG_ID,
      /** The nearest tag alone, its two poses the wrong way round, reported as fairly sure. */
      FLIPPED
    }

    private final FieldModel field;
    private final List<CameraConfig> configs;
    private final SplittableRandom random;
    private final List<WpilibContender.Image> seen = new ArrayList<>();

    private @Nullable Misread misread;
    private int misreadsLeft;

    /** The one camera that misreads, once the first has: a misreading camera keeps at it. */
    private @Nullable String misreader;

    private double flipShare;
    private int misreads;

    Cameras(FieldModel field, List<CameraConfig> configs, long seed) {
      this.field = field;
      this.configs = List.copyOf(configs);
      random = new SplittableRandom(seed ^ 0x6f75746c69657273L);
    }

    /** One camera's IO, tapped. */
    CameraIO watch(String name, CameraIO camera) {
      CameraConfig config =
          configs.stream().filter(each -> each.name().equals(name)).findFirst().orElseThrow();
      return new CameraIO() {
        @Override
        public void updateInputs(CameraIOInputs inputs) {
          camera.updateInputs(inputs);
          take(config, inputs);
        }

        @Override
        public void close() {
          camera.close();
        }
      };
    }

    /** From now, the next images of one camera that see a tag misread it: this many of them. */
    void misread(Misread how, int images) {
      misread = how;
      misreadsLeft = images;
      misreader = null;
    }

    /** From now, this share of every camera's images of tags come with a tag's poses flipped. */
    void flipShare(double share) {
      flipShare = share;
    }

    /** How many images were misread so far. */
    int misreads() {
      return misreads;
    }

    /** The images the cameras sent since the last call, camera by camera, as they were read. */
    List<WpilibContender.Image> drain() {
      List<WpilibContender.Image> images = List.copyOf(seen);
      seen.clear();
      return images;
    }

    private void take(CameraConfig config, CameraIO.CameraIOInputs inputs) {
      List<CameraIO.Frame> frames = new ArrayList<>();
      List<CameraIO.SeenTag> all = new ArrayList<>();
      int next = 0;
      for (CameraIO.Frame frame : inputs.frames) {
        int end = Math.min(next + frame.tagCount(), inputs.tags.length);
        List<CameraIO.SeenTag> tags = new ArrayList<>(List.of(inputs.tags).subList(next, end));
        next = end;
        Misread how = howToMisread(config.name(), tags);
        if (how != null) {
          CameraIO.SeenTag wrong = misread(nearest(tags), how);
          tags = List.of(wrong);
          frame = new CameraIO.Frame(frame.timestampNanos(), 1, false, Transform3d.ZERO, 0);
          misreads++;
        }
        frames.add(frame);
        all.addAll(tags);
        seen.add(new WpilibContender.Image(config.name(), config.robotToCamera(), frame, tags));
      }
      inputs.frames = frames.toArray(CameraIO.Frame[]::new);
      inputs.tags = all.toArray(CameraIO.SeenTag[]::new);
    }

    /** How this image is misread, or null when it's read right. */
    private @Nullable Misread howToMisread(String camera, List<CameraIO.SeenTag> tags) {
      if (tags.isEmpty()) {
        return null;
      }
      Misread asked = misread;
      boolean itsTheMisreader = misreader == null || misreader.equals(camera);
      if (asked != null && misreadsLeft > 0 && itsTheMisreader) {
        misreader = camera;
        misreadsLeft--;
        return asked;
      }
      if (flipShare > 0 && random.nextDouble() < flipShare) {
        return Misread.FLIPPED;
      }
      return null;
    }

    private static CameraIO.SeenTag nearest(List<CameraIO.SeenTag> tags) {
      CameraIO.SeenTag nearest = tags.get(0);
      for (CameraIO.SeenTag tag : tags) {
        if (distance(tag) < distance(nearest)) {
          nearest = tag;
        }
      }
      return nearest;
    }

    private static double distance(CameraIO.SeenTag tag) {
      return tag.bestCameraToTag().getTranslation().getNorm();
    }

    private CameraIO.SeenTag misread(CameraIO.SeenTag tag, Misread how) {
      int id = tag.id();
      Transform3d best = tag.bestCameraToTag();
      Transform3d alternate = tag.altCameraToTag();
      double ambiguity = tag.ambiguity();
      boolean hasAlternate = tag.altCameraToTag().getTranslation().getNorm() > 1e-6;
      if (how == Misread.WRONG_ID) {
        id = anotherTag(id);
      } else if (hasAlternate) {
        best = tag.altCameraToTag();
        alternate = tag.bestCameraToTag();
        ambiguity = FLIPPED_AMBIGUITY;
      }
      return new CameraIO.SeenTag(
          id,
          tag.corner0X(),
          tag.corner0Y(),
          tag.corner1X(),
          tag.corner1Y(),
          tag.corner2X(),
          tag.corner2Y(),
          tag.corner3X(),
          tag.corner3Y(),
          best,
          alternate,
          ambiguity,
          false);
    }

    /** The field's next tag by number after this one, around to the first. */
    private int anotherTag(int id) {
      List<Integer> ids = field.tags().keySet().stream().sorted().toList();
      int at = ids.indexOf(id);
      return ids.get((at + 1) % ids.size());
    }
  }
}
