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
 * 2024's field, CRESCENDO, kept to check that the field model can describe a real field, a mirrored
 * one: not to play the game. Written as {@link Rebuilt} is: blue's alliance only, in meters in
 * WPILib's field frame, positions from WPILib's 2024 tag layout wherever a tag marks them, and
 * sizes from the 2024 game manual's section 5, cited by section. The few figures neither gives come
 * from maple-sim's arena ({@code Arena2024Crescendo} at 1b1e8ea), and say so where they're used.
 *
 * <p>The field is flat: the manual describes nothing robots drive over.
 */
final class Crescendo {
  private Crescendo() {}

  static final Fields LAYOUT = Fields.FRC_2024_CRESCENDO;

  /** Red's half is blue's reflected across the center line: both amps are on one guardrail. */
  static final Symmetry SYMMETRY = Symmetry.MIRRORED;

  private static final double LENGTH = LAYOUT.length;
  private static final double WIDTH = LAYOUT.width;

  // ---- Materials, where the manual names them ----

  /** The subwoofer's vertical faces, and its side inclined ones (5.6.1). */
  static final Material HDPE = Material.builder().name("HDPE").build();

  /**
   * What the manual doesn't name a material for: the stage's truss legs, the source, the amp, and
   * the speaker's part of the alliance wall.
   */
  static final Material STRUCTURE = Material.builder().name("field structure").build();

  // ---- The stage, three legs around a six-sided core (5.5) ----

  /** The stage's center: the middle of the tags on its core's three wide faces (14, 15, 16). */
  static final double STAGE_X = 4.868;

  /** Across the field, the tags' middle is the field's, within a millimeter. */
  static final double STAGE_Y = WIDTH / 2;

  /**
   * The truss legs: posts 0.35 m square standing 1.48 m from the stage's center, each facing it,
   * from maple-sim's arena ({@code Arena2024Crescendo} at 1b1e8ea). No tag marks them, and the
   * manual gives only that the stage stands 10 ft 1 in from the alliance wall.
   */
  static final double LEG_FROM_CENTER = 1.48;

  static final double LEG_SIZE = 0.35;

  // ---- The speaker's subwoofer, against the alliance wall (5.6.1) ----

  /** The subwoofer's center across the field: tag 7's, centered above it. */
  static final double SPEAKER_Y = 5.548;

  /** 3 ft ⅛ in out from the alliance wall, 3 ft 1 in tall, its vertical panels 8⅜ in tall. */
  static final double SUBWOOFER_DEPTH = 36.125 * IN;

  // ---- The amp, set into the guardrail by the alliance wall (5.3) ----

  // ---- The sources, each in the corner by the opposing alliance's wall (5.4) ----

  /**
   * The source on blue's half, the red alliance's, in the corner by the blue alliance wall: midway
   * between its tags, 9 and 10, on its wall, and the way they face.
   */
  static final Translation2d SOURCE_TAGS = new Translation2d(0.909, 0.565);

  static final Rotation2d SOURCE_FACING = Rotation2d.fromDegrees(60);

  // ---- Places and zones (5.2) ----

  /** The amp zone: 10 ft 10 in along the guardrail from the alliance wall, 1 ft 5¾ in deep. */
  static final double AMP_ZONE_LENGTH = 130 * IN;

  static final double AMP_ZONE_WIDTH = 17.75 * IN;

  /** The robot starting zone: 6 ft 4⅛ in deep, 23 ft 8⅛ in long beside the amp zone. */
  static final double STARTING_ZONE_DEPTH = 76.125 * IN;

  static final double STARTING_ZONE_LENGTH = 284.125 * IN;

  /** The source zone: 1 ft 6¾ in deep from the source wall, its tape included. */
  static final double SOURCE_ZONE_DEPTH = 18.75 * IN;

  /** The stage's center, among the tags on its core. */
  static final Place STAGE = Place.between("Stage", 14, 15, 16);

  static final Zone AMP_ZONE =
      Zone.ours(
          "Amp zone",
          Polygon2d.rectangle(
              new Translation2d(0, WIDTH - AMP_ZONE_WIDTH),
              new Translation2d(AMP_ZONE_LENGTH, WIDTH)));

  /**
   * The robot starting zone: from the alliance wall to the black tape, 6 ft 4⅛ in out, beside the
   * amp zone and down to the opponent's source zone, whose tape cuts its far corner off along the
   * source wall. The layout's source tags put its far edge 2 cm longer than the manual's 23 ft 8⅛
   * in.
   */
  static final Zone STARTING_ZONE =
      Zone.ours(
          "Robot starting zone",
          new Polygon2d(
              List.of(
                  sourceTape(0),
                  sourceTape(STARTING_ZONE_DEPTH),
                  new Translation2d(STARTING_ZONE_DEPTH, WIDTH - AMP_ZONE_WIDTH),
                  new Translation2d(0, WIDTH - AMP_ZONE_WIDTH))));

  /**
   * The source zone, in front of the alliance's source, by the opposing alliance's wall: a
   * parallelogram along the source wall, 1 ft 6¾ in deep, from the opposing alliance wall (5.2).
   * Its far end lines up with the robot starting zone's far edge, as figure 5-4 shows it; the text
   * doesn't place it. The layout's source tags have the source wall meet the guardrail 5 cm short
   * of there, so its corner by the guardrail reaches 3 cm past the field's edge.
   */
  static final Zone SOURCE_ZONE =
      Zone.ours(
          "Source zone",
          SYMMETRY.apply(
              Polygon2d.hull(
                  List.of(
                      sourceWall(0),
                      sourceWall(STARTING_ZONE_DEPTH),
                      sourceTape(STARTING_ZONE_DEPTH),
                      sourceTape(0))),
              LENGTH,
              WIDTH));

  /**
   * The stage zone, the alliance-colored tape around the stage, six-sided (5.2). The manual gives
   * no size for it, so it's the outline around the stage's legs as they're drawn here (from
   * maple-sim), which is how figure 5-5 shows the tape running.
   */
  static final Zone STAGE_ZONE = Zone.ours("Stage zone", Polygon2d.hull(legCorners()));

  /**
   * Where the source wall on blue's half, red's, crosses a line across the field: its tags' face.
   */
  private static Translation2d sourceWall(double x) {
    return across(SOURCE_TAGS, x);
  }

  /**
   * Where the tape in front of the source on blue's half, red's, crosses a line across the field.
   */
  private static Translation2d sourceTape(double x) {
    return across(SOURCE_TAGS.plus(new Translation2d(SOURCE_ZONE_DEPTH, SOURCE_FACING)), x);
  }

  /** Where a line along the source wall, through a point, crosses a line across the field at x. */
  private static Translation2d across(Translation2d point, double x) {
    Rotation2d along = SOURCE_FACING.plus(Rotation2d.CCW_PI_2);
    double run = (x - point.getX()) / along.getCos();
    return new Translation2d(x, point.getY() + run * along.getSin());
  }

  /** The corners of the stage's three legs, seen from above, as {@link #stage} draws them. */
  private static List<Translation2d> legCorners() {
    Translation2d center = new Translation2d(STAGE_X, STAGE_Y);
    List<Translation2d> corners = new ArrayList<>();
    for (int face = 0; face < 3; face++) {
      Rotation2d leg = Rotation2d.fromDegrees(120 * face + 60);
      Translation2d middle = center.plus(new Translation2d(LEG_FROM_CENTER, leg));
      for (double[] sign : new double[][] {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}) {
        corners.add(
            middle.plus(
                new Translation2d(sign[0] * LEG_SIZE / 2, sign[1] * LEG_SIZE / 2).rotateBy(leg)));
      }
    }
    return corners;
  }

  private static Translation3d above(Translation2d spot, double height) {
    return new Translation3d(spot.getX(), spot.getY(), height);
  }

  /**
   * The field model for a robot of this size, which sets where it stands in the starting zone.
   *
   * <p>Starts, facing the field: back to the alliance wall beside the amp zone, back to the
   * subwoofer's front face, and centered across the field with the front bumper on the starting
   * zone's line.
   */
  static FieldModel field(Body body) {
    double halfLength = body.bumperLength().in(Meters) / 2;
    double halfWidth = body.bumperWidth().in(Meters) / 2;
    return FieldModel.builder(TagLayout.of(LAYOUT.loadField()), SYMMETRY)
        .starts(
            List.of(
                Place.at(
                    "Amp side",
                    new Pose2d(halfLength, WIDTH - AMP_ZONE_WIDTH - halfWidth, Rotation2d.ZERO)),
                Place.at(
                    "Subwoofer",
                    new Pose2d(SUBWOOFER_DEPTH + halfLength, SPEAKER_Y, Rotation2d.ZERO)),
                Place.at(
                    "Center",
                    new Pose2d(STARTING_ZONE_DEPTH - halfLength, WIDTH / 2, Rotation2d.ZERO))))
        .places(List.of(STAGE))
        .zones(List.of(STARTING_ZONE, AMP_ZONE, SOURCE_ZONE, STAGE_ZONE))
        .build();
  }
}
