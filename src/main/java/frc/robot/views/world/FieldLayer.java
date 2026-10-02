package frc.robot.views.world;

import com.michaelgrundvig.frc.pitwall.scene.Inspection;
import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.SceneBuilder;
import com.michaelgrundvig.frc.world.field.Bump;
import com.michaelgrundvig.frc.world.field.FieldModel;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The field as the field model has it: its walls, its bumps, and its AprilTags where the layout
 * puts them, each with its number. Nothing here moves.
 */
final class FieldLayer implements Layer {
  /** An AprilTag's printed square, as FRC's 36h11 tags are: 6.5 in. */
  private static final double TAG_SIZE = 0.1651;

  private final FieldModel field;

  FieldLayer(FieldModel field) {
    this.field = field;
  }

  @Override
  public String id() {
    return "field";
  }

  @Override
  public String title() {
    return "Field";
  }

  @Override
  public void describe(SceneBuilder scene) {
    double length = field.length();
    double width = field.width();
    scene
        .polygon("carpet", SceneBuilder.ROOT)
        .point(0, 0, 0)
        .point(length, 0, 0)
        .point(length, width, 0)
        .point(0, width, 0);
    scene
        .polyline("walls", SceneBuilder.ROOT)
        .point(0, 0, 0)
        .point(length, 0, 0)
        .point(length, width, 0)
        .point(0, width, 0)
        .closed();
    List<Bump> bumps = field.terrain().bumps();
    for (int i = 0; i < bumps.size(); i++) {
      bump(scene, "bump." + i, bumps.get(i));
    }
    for (Map.Entry<Integer, Pose3d> tag : field.tags().entrySet()) {
      int id = tag.getKey();
      Pose3d pose = tag.getValue();
      String frame = WorldView.tagFrame(id);
      scene.frame(frame, SceneBuilder.ROOT, pose);
      // The tag's x points out of its face: its square stands in the y-z plane.
      scene
          .box(frame + ".square", frame)
          .size(0.01, TAG_SIZE, TAG_SIZE)
          .inspect(() -> tag(id, pose));
      // In front of its face: tags back to back, or side by side on one panel, keep apart.
      scene.label(frame + ".id", frame).text(Integer.toString(id)).at(0.25, 0, TAG_SIZE);
    }
  }

  /** A bump: a ridge along its crest, with a ramp down to the floor on each side. */
  private static void bump(SceneBuilder scene, String id, Bump bump) {
    Translation2d across = new Translation2d(bump.depthMeters() / 2, bump.across());
    Translation2d start = bump.crestStart();
    Translation2d end = bump.crestEnd();
    double height = bump.heightMeters();
    scene
        .mesh(id, SceneBuilder.ROOT)
        .vertex(start.getX() - across.getX(), start.getY() - across.getY(), 0) // 0
        .vertex(start.getX() + across.getX(), start.getY() + across.getY(), 0) // 1
        .vertex(start.getX(), start.getY(), height) // 2
        .vertex(end.getX() - across.getX(), end.getY() - across.getY(), 0) // 3
        .vertex(end.getX() + across.getX(), end.getY() + across.getY(), 0) // 4
        .vertex(end.getX(), end.getY(), height) // 5
        .triangle(0, 3, 5)
        .triangle(0, 5, 2)
        .triangle(1, 2, 5)
        .triangle(1, 5, 4)
        .triangle(0, 2, 1)
        .triangle(3, 4, 5)
        .inspect(
            () ->
                Inspection.of(bump.name())
                    .detail("Height", WorldView.meters(bump.heightMeters()))
                    .detail(
                        "Ramps",
                        String.format(Locale.ROOT, "%.1f°", Math.toDegrees(bump.slopeRadians())))
                    .detail("Crest", WorldView.meters(bump.length()) + " long"));
  }

  private static Inspection tag(int id, Pose3d pose) {
    return Inspection.of("AprilTag " + id)
        .detail(
            "Where",
            String.format(
                Locale.ROOT,
                "x %.3f m, y %.3f m, %.3f m up",
                pose.getX(),
                pose.getY(),
                pose.getZ()))
        .detail(
            "Facing",
            String.format(Locale.ROOT, "%.1f°", Math.toDegrees(pose.getRotation().getZ())))
        .detail("From", "the field's layout, as the robot program has it");
  }
}
