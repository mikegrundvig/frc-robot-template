package frc.robot.views.world;

import com.michaelgrundvig.frc.pitwall.scene.Inspection;
import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.SceneBuilder;
import com.michaelgrundvig.frc.pitwall.scene.SceneState;
import com.michaelgrundvig.frc.pitwall.scene.Shapes;
import com.michaelgrundvig.frc.pitwall.scene.Style;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.localization.VisionDecision;
import frc.robot.lib.localization.VisionMeasurement;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Function;
import java.util.stream.Collectors;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;

/**
 * Each camera's sight lines to the tags its measurement used: solid where the estimator accepted
 * the measurement, dashed where it refused it. A click on one says why, from the decision itself,
 * and offers the camera's calibration. A line stays a moment after its measurement, so it doesn't
 * flicker; the robot keeps what a click says for two seconds after it's gone, and the page's Hold
 * keeps it drawn.
 */
final class SightLines implements Layer {
  /** How long a line stays after its camera's last measurement of its tag. */
  private static final long HOLD_NANOS = 300_000_000L;

  /**
   * How long a click on a line can still say what it stood for after its measurement: longer than
   * the line stays and Pitwall keeps a gone line clickable (two seconds) together.
   */
  private static final long KEEP_NANOS = 2_500_000_000L;

  /** One camera's latest measurement of one tag, and when it was decided. */
  private record Seen(
      String id, String camera, int tag, VisionDecision decision, long decidedNanos) {
    Seen(String camera, int tag, VisionDecision decision, long decidedNanos) {
      this(
          "ray." + WorldView.name(camera) + "." + tag + (decision.accepted() ? ".ok" : ".bad"),
          camera,
          tag,
          decision,
          decidedNanos);
    }
  }

  private final FieldModel field;
  private final Set<String> cameras;
  private final WorldView.Readings readings;
  private final Function<String, Optional<String>> cameraAction;

  /**
   * By camera and tag: the lines drawn, each the latest of its camera's measurements of its tag.
   */
  private final Map<String, Seen> drawn = new LinkedHashMap<>();

  /**
   * By line: what each stood for when last measured, for a click to read from a web server thread.
   * A line is drawn again only when which lines there are changes, so its click reads this rather
   * than what it was drawn from.
   */
  private final Map<String, Seen> latest = new ConcurrentHashMap<>();

  private List<VisionDecision> lastDecisions = List.of();

  SightLines(
      FieldModel field,
      List<CameraConfig> cameras,
      WorldView.Readings readings,
      Function<String, Optional<String>> cameraAction) {
    this.field = field;
    this.cameras = cameras.stream().map(CameraConfig::name).collect(Collectors.toSet());
    this.readings = readings;
    this.cameraAction = cameraAction;
  }

  @Override
  public String id() {
    return "sight-lines";
  }

  @Override
  public String title() {
    return "Sight lines (dashed: rejected)";
  }

  /** Its lines run between the robot's cameras and the field's tags: it adds no frames. */
  @Override
  public void describe(SceneBuilder scene) {}

  @Override
  public void update(SceneState state) {
    long now = state.nowNanos();
    boolean changed = false;
    List<VisionDecision> decisions = readings.decisions();
    if (decisions != lastDecisions) {
      lastDecisions = decisions;
      for (VisionDecision decision : decisions) {
        VisionMeasurement measurement = decision.measurement();
        if (!cameras.contains(measurement.camera())) {
          continue;
        }
        for (int tag : measurement.tagIds()) {
          if (field.tag(tag).isEmpty()) {
            continue;
          }
          Seen line = new Seen(measurement.camera(), tag, decision, now);
          Seen before = drawn.put(measurement.camera() + "/" + tag, line);
          changed |= before == null || !before.id().equals(line.id());
          latest.put(line.id(), line);
        }
      }
    }
    changed |= drawn.values().removeIf(line -> now - line.decidedNanos() > HOLD_NANOS);
    latest.values().removeIf(line -> now - line.decidedNanos() > KEEP_NANOS);
    if (!changed) {
      return;
    }
    Shapes shapes = state.shapes();
    Set<String> ids = new HashSet<>();
    for (Seen line : drawn.values()) {
      if (!ids.add(line.id())) {
        continue;
      }
      String id = line.id();
      shapes
          .line(id, SceneBuilder.ROOT)
          .between(WorldView.cameraFrame(line.camera()), WorldView.tagFrame(line.tag()))
          .style(line.decision().accepted() ? Style.OK : Style.BAD.dashed())
          .inspect(() -> inspection(latest.getOrDefault(id, line)));
    }
  }

  /**
   * What a click on a line says, from the decision alone (a record the loop no longer touches): the
   * verdict and its numbers, called on a web server thread.
   */
  private Inspection inspection(Seen line) {
    VisionDecision decision = line.decision();
    VisionMeasurement measurement = decision.measurement();
    Pose2d measured = measurement.pose();
    Optional<Pose3d> tag = field.tag(line.tag());
    Inspection said =
        Inspection.of(line.camera() + " → tag " + line.tag())
            .detail("Verdict", decision.accepted() ? "accepted" : "rejected: " + decision.reason())
            .detail(
                "Innovation",
                Double.isFinite(decision.innovationMeters())
                    ? WorldView.meters(decision.innovationMeters()) + " from the estimate"
                    : "not compared")
            .detail(
                "χ²",
                Double.isFinite(decision.chiSquared())
                    ? String.format(Locale.ROOT, "%.1f", decision.chiSquared())
                    : "not compared")
            .detail(
                "Distance",
                tag.isPresent()
                    ? WorldView.meters(
                            measured
                                .getTranslation()
                                .getDistance(tag.get().toPose2d().getTranslation()))
                        + " from the measured pose to the tag"
                    : "the field has no such tag")
            .detail(
                "Measured pose",
                String.format(
                    Locale.ROOT,
                    "x %.2f m, y %.2f m, heading %.1f°",
                    measured.getX(),
                    measured.getY(),
                    measured.getRotation().getDegrees()))
            .detail(
                "Solved",
                measurement.solver().name().toLowerCase(Locale.ROOT).replace('_', ' ')
                    + " from tags "
                    + measurement.tagIds().stream()
                        .map(String::valueOf)
                        .collect(Collectors.joining(", ")));
    Optional<String> calibrate = cameraAction.apply(line.camera());
    return calibrate.isPresent() ? said.action(calibrate.get()) : said;
  }
}
