package frc.robot.subsystems.vision;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.localization.VisionDecision;
import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.util.Alert;

/**
 * One of the robot's cameras, as {@link Vision} runs it: its IO and what it read this loop, its
 * alerts, the switch that leaves it out, and its tallies. What became of this loop's images and
 * which tags it saw are logged under {@code Vision/<name>}.
 */
final class Camera implements AutoCloseable {
  final CameraConfig config;
  private final CameraIO io;
  final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();
  private final String key;
  private final Alert disconnected;
  private final Alert unreadable;

  /** Off leaves the camera out: fogged over, knocked loose, or blinded by stage lights. */
  final TunableBoolean enabled = TunableBoolean.create(true);

  private long lastFrame = Long.MIN_VALUE;
  private int accepted;
  private int rejected;
  private double residualSum;
  private int residualCount;

  // This loop: what became of each image, and the tags seen, placed on the field.
  private final List<String> outcomes = new ArrayList<>();
  private final List<Pose3d> seenTags = new ArrayList<>();
  private double loopResidualSum;
  private int loopResidualCount;

  Camera(CameraConfig config, CameraIO io) {
    this.config = config;
    this.io = io;
    String name = config.name();
    key = "Vision/" + name;
    disconnected =
        new Alert(
            "Vision", name + "Disconnected", "Camera " + name + " disconnected", Alert.Level.HIGH);
    unreadable =
        new Alert(
            "Vision", name + "Unreadable", "Camera " + name + " can't be read", Alert.Level.HIGH);
    TunableRegistry.publish(key + "/enabled", enabled);
  }

  String name() {
    return config.name();
  }

  /**
   * Reads the camera and starts this loop's record. Nothing its IO throws gets past here: PhotonLib
   * throws on purpose when the coprocessor runs a different PhotonVision version than the robot's
   * library, and again every few seconds, which would end the robot program each time. Instead the
   * camera reads as unreadable, with the reason, and the robot drives on its other sensors.
   */
  void read() {
    try {
      io.updateInputs(inputs);
      inputs.error = "";
    } catch (RuntimeException e) {
      inputs.connected = false;
      inputs.frames = new CameraIO.Frame[] {};
      inputs.tags = new CameraIO.SeenTag[] {};
      inputs.skippedFrames = 0;
      String why = e.getMessage();
      inputs.error = why != null ? why.strip() : e.toString();
    }
    Logger.processInputs(key, inputs);
    boolean readable = inputs.error.isEmpty();
    disconnected.set(!inputs.connected && readable);
    if (!readable) {
      unreadable.setText("Camera " + name() + " can't be read: " + inputs.error);
    }
    unreadable.set(!readable);

    outcomes.clear();
    seenTags.clear();
    loopResidualSum = 0;
    loopResidualCount = 0;
    if (inputs.skippedFrames > 0) {
      outcomes.add(inputs.skippedFrames + " image(s) without a capture time, left out");
    }
  }

  /** Whether an image is new: a camera can hand over the same image twice. */
  boolean isNew(long timestampNanos) {
    if (timestampNanos <= lastFrame) {
      return false;
    }
    lastFrame = timestampNanos;
    return true;
  }

  /** What became of one of this loop's images. */
  void outcome(String outcome) {
    outcomes.add(outcome);
  }

  /**
   * A tag this loop's image saw, placed on the field through the estimated pose and the camera's
   * configured mount, and where the tag really is, if the field has it.
   */
  void saw(Pose3d seen, @Nullable Pose3d real) {
    seenTags.add(seen);
    if (real != null) {
      loopResidualSum += seen.getTranslation().getDistance(real.getTranslation());
      loopResidualCount++;
    }
  }

  /** Logs this loop's images and tags, and the tag residual, this loop's and since startup. */
  void logLoop() {
    Logger.recordOutput(key + "/Outcomes", outcomes.toArray(String[]::new));
    Logger.recordOutput(key + "/SeenTags", seenTags.toArray(Pose3d[]::new));
    if (loopResidualCount > 0) {
      Logger.recordOutput(key + "/TagResidualMeters", loopResidualSum / loopResidualCount);
      residualSum += loopResidualSum;
      residualCount += loopResidualCount;
      Logger.recordOutput(key + "/MeanTagResidualMeters", meanTagResidualMeters());
    }
  }

  /** Counts and logs what the estimator decided about this camera's measurements. */
  void record(List<VisionDecision> decisions) {
    List<String> reasons = new ArrayList<>();
    for (VisionDecision decision : decisions) {
      if (decision.measurement().camera().equals(name())) {
        reasons.add(decision.reason());
        if (decision.accepted()) {
          accepted++;
        } else {
          rejected++;
        }
      }
    }
    Logger.recordOutput(key + "/Decisions", reasons.toArray(String[]::new));
    Logger.recordOutput(key + "/Accepted", accepted);
    Logger.recordOutput(key + "/Rejected", rejected);
  }

  /**
   * How far, on average since the robot started, this camera's tags land from where they really
   * are.
   *
   * @return NaN until the camera has seen a tag
   */
  double meanTagResidualMeters() {
    return residualCount == 0 ? Double.NaN : residualSum / residualCount;
  }

  boolean isConnected() {
    return inputs.connected;
  }

  int accepted() {
    return accepted;
  }

  int rejected() {
    return rejected;
  }

  boolean isDisconnectedAlertActive() {
    return disconnected.get();
  }

  boolean isUnreadableAlertActive() {
    return unreadable.get();
  }

  @Override
  public void close() {
    io.close();
    disconnected.close();
    unreadable.close();
    TunableRegistry.remove(key + "/enabled");
  }
}
