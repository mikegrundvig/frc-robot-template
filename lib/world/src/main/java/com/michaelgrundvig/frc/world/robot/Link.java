package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Shape;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.List;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Mass;

/**
 * What a joint moves: an arm, a carriage, a wrist with what it holds. Everything here is in its
 * joint's frame, which moves with the joint.
 *
 * @param label what logs and writers call it: unique among the robot's links
 * @param mass its mass, center of mass and inertia, in its joint's frame: from CAD's
 *     mass-properties report, or weighed on a scale. None known is {@code MassProperties.NONE},
 *     zero, never a guess: its joint then has no gravity to hold (no kG seed), it adds nothing to
 *     the robot's center of mass, and its joint's kA seeds from the motors' rotors alone. The
 *     robot's missing-data report lists it ({@code RobotDefinition.missing}).
 * @param shapes its solid shape, simplified by hand from CAD: what interlocks and the extension
 *     limit are checked against, and what the views draw. None: it isn't drawn or checked.
 */
@ConfigRecord
public record Link(
    String label,
    MassProperties mass,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Shape> shapes) {
  public Link {
    Checks.on(Link.class).field("label", label, notBlank()).orThrow();
    shapes = List.copyOf(shapes);
  }

  /** Starts describing a link, one named value at a time. */
  public static LinkBuilder.LabelStage builder() {
    return LinkBuilder.builder();
  }

  /**
   * A link nobody has weighed, drawn and checked as a box of a size centered at a point in its
   * joint's frame: its mass none, never a guess.
   */
  public static Link box(String label, Translation3d center, Translation3d size) {
    return builder()
        .label(label)
        .mass(MassProperties.NONE)
        .shapes(List.of(box(center, size)))
        .build();
  }

  /** A link nobody has weighed or drawn: where its shape would be a guess on a guess. */
  public static Link unseen(String label) {
    return builder().label(label).mass(MassProperties.NONE).build();
  }

  /**
   * A link of a known mass, its center at a point in its joint's frame, as hard to turn as a solid
   * box of a size there, and drawn so: CAD's or a scale's mass, without CAD's inertia.
   */
  public static Link weighed(String label, Mass mass, Translation3d cg, Translation3d size) {
    return builder()
        .label(label)
        .mass(MassProperties.builder().mass(mass).cg(cg).inertia(Inertia.box(mass, size)).build())
        .shapes(List.of(box(cg, size)))
        .build();
  }

  private static Shape box(Translation3d center, Translation3d size) {
    return Shape.Box.builder()
        .pose(new Pose3d(center, Rotation3d.ZERO))
        .size(size)
        .material(Material.ALUMINUM)
        .build();
  }
}
