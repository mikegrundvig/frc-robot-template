package frc.robot.game;

import static frc.robot.game.Manuals.IN;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.TagLayout;
import com.michaelgrundvig.frc.world.field.Zone;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.robot.Body;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * 2025's field, REEFSCAPE, kept to check that the field model can describe a real field: not to
 * play the game. Written as {@link Rebuilt} is: blue's alliance only, in meters in WPILib's field
 * frame, positions from WPILib's 2025 tag layout wherever a tag marks them, and sizes from the 2025
 * game manual's section 5, cited by section. The few figures neither gives come from maple-sim's
 * arena ({@code Arena2025Reefscape} and its reef, at 1b1e8ea), and say so where they're used.
 *
 * <p>The field is flat: the manual describes nothing robots drive over.
 */
final class Reefscape {
  private Reefscape() {}

  /** The welded field. The AndyMark build is 1 cm narrower, with its own layout. */
  static final Fields LAYOUT = Fields.FRC_2025_REEFSCAPE_WELDED;

  /** Red's half is blue's turned half a turn: the processors are on opposite guardrails. */
  static final Symmetry SYMMETRY = Symmetry.ROTATED;

  private static final double LENGTH = LAYOUT.length;
  private static final double WIDTH = LAYOUT.width;

  // ---- Materials, where the manual names them ----

  /** The reef's pipes and the cages: Schedule 40 steel pipe (5.3, 5.4.1). */
  static final Material STEEL = Material.builder().name("steel").build();

  /**
   * What the manual doesn't name a material for: the reef's base, the barge, the stations, the
   * processor.
   */
  static final Material STRUCTURE = Material.builder().name("field structure").build();

  // ---- The reef, 12 ft from the alliance wall and centered between the guardrails (5.3) ----

  /** The reef's near face, 12 ft out: where tag 18 hangs. */
  static final double REEF_NEAR_FACE = 144 * IN;

  /** A hexagon 5 ft 5½ in across: across its flats, as the tags on opposite faces are. */
  static final double REEF_ACROSS = 65.5 * IN;

  static final double REEF_X = REEF_NEAR_FACE + REEF_ACROSS / 2;
  static final double REEF_Y = WIDTH / 2;

  // ---- The barge, across the center of the field (5.4) ----

  /** The cages' centers from mid field: 3 ft 5½ in, 7 ft ⅜ in, and 10 ft 7⅜ in. */
  static final List<Double> CAGES = List.of(41.5 * IN, 84.375 * IN, 127.375 * IN);

  /**
   * Which way across the field blue's half of the barge is, its cages and net: to the blue drivers'
   * left, where figure 5-4 draws blue's barge zone, and maple-sim's arena scores blue's net ({@code
   * ReefscapeBargeSimulation} at 1b1e8ea). The manual's text doesn't say.
   */
  static final double BLUE_BARGE_SIDE = 1;

  // ---- The coral stations, one in each corner (5.6.2) ----

  // ---- The processor, set into the guardrail by the alliance's reef (5.5) ----

  // ---- Places and zones ----

  /** The robot starting line, 7 ft 4 in from the reef: taken to the tape's near edge (5.2). */
  static final double STARTING_LINE_X = REEF_X + REEF_ACROSS / 2 + 88 * IN;

  /** The field's tape is 2 in wide (5.2). */
  static final double TAPE_WIDTH = 2 * IN;

  /** The reef's center, among its six tags. */
  static final Place REEF = Place.between("Reef", 17, 18, 19, 20, 21, 22);

  /**
   * The barge zone, 3 ft 10 in deep and 12 ft 2½ in long around the alliance's half of the barge
   * (5.2), on the center line: centered on the middle cage, which the text doesn't say.
   */
  static final Zone BARGE_ZONE =
      Zone.ours(
          "Barge zone",
          Polygon2d.rectangle(
              new Pose2d(LENGTH / 2, WIDTH / 2 + BLUE_BARGE_SIDE * CAGES.get(1), Rotation2d.ZERO),
              46 * IN,
              146.5 * IN));

  /** The reef zone: 7 ft 9½ in across, face to face (5.2). */
  static final double REEF_ZONE_ACROSS = 93.5 * IN;

  /**
   * The reef zone, the six-sided tape around the alliance's reef (5.2): centered on the reef and
   * turned as it is, its faces toward the walls, as figure 5-4 shows it.
   */
  static final Zone REEF_ZONE = Zone.ours("Reef zone", hexagon(REEF_ZONE_ACROSS));

  /**
   * A hexagon around the reef's center, its flats facing the way the reef's tags do: 0°, 60°, and
   * on round.
   *
   * @param across between opposite flats
   */
  private static Polygon2d hexagon(double across) {
    List<Translation2d> corners = new ArrayList<>();
    double circumradius = across / 2 / Math.cos(Math.PI / 6);
    for (int corner = 0; corner < 6; corner++) {
      corners.add(
          new Translation2d(REEF_X, REEF_Y)
              .plus(new Translation2d(circumradius, Rotation2d.fromDegrees(30 + 60 * corner))));
    }
    return new Polygon2d(corners);
  }

  /** A point on the reef: out from its center the way a face looks, across it, and up. */
  private static Translation3d at(Rotation2d out, double outward, double across, double up) {
    Translation2d point =
        new Translation2d(REEF_X, REEF_Y).plus(new Translation2d(outward, across).rotateBy(out));
    return new Translation3d(point.getX(), point.getY(), up);
  }

  /**
   * The field model for a robot of this size, which sets where it stands on the starting line.
   *
   * <p>Starts, left to right as the drivers see them, front bumpers on the starting line's far edge
   * so the bumpers overlap it: in line with each middle cage, and between them.
   */
  static FieldModel field(Body body) {
    return field(body, LAYOUT);
  }

  /** The field model on another of the season's layouts: the AndyMark build, say. */
  static FieldModel field(Body body, Fields layout) {
    double x = STARTING_LINE_X + TAPE_WIDTH - body.bumperLength().in(Meters) / 2;
    double cage = CAGES.get(1);
    return FieldModel.builder(TagLayout.of(layout.loadField()), SYMMETRY)
        .starts(
            List.of(
                Place.at("Left", new Pose2d(x, WIDTH / 2 + cage, Rotation2d.ZERO)),
                Place.at("Center", new Pose2d(x, WIDTH / 2, Rotation2d.ZERO)),
                Place.at("Right", new Pose2d(x, WIDTH / 2 - cage, Rotation2d.ZERO))))
        .places(List.of(REEF))
        .zones(List.of(REEF_ZONE, BARGE_ZONE))
        .build();
  }
}
