package frc.robot.game;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;

import com.michaelgrundvig.frc.world.field.Bump;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Platform;
import com.michaelgrundvig.frc.world.field.Surface;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.TagLayout;
import com.michaelgrundvig.frc.world.field.Zone;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.robot.Body;
import frc.robot.venues.Venues;
import java.util.Collections;
import java.util.List;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The field the robot plays on: 2026's, REBUILT, until the next game's is written at kickoff.
 *
 * <p>Everything here is written as if the robot were on the blue alliance, in meters in WPILib's
 * field frame (see {@code com.michaelgrundvig.frc.world.field}); the field model works out the red
 * alliance's half, turned half a turn. Places near AprilTags are derived from them, so a layout
 * measured at an event moves them for both alliances. Sizes come from the game manual's section 5,
 * and where things stand from WPILib's 2026 tag layout. The two exceptions say so where they're
 * used: the tower's uprights and the depot, which no tag marks, are placed from maple-sim's arena.
 *
 * <p>Every figure here is the manual's nominal one. The official drawings hold the tolerances, and
 * a venue's survey holds where an event's tags really are.
 *
 * <p>At kickoff, this class moves to the test tree as a fixture and the new game gets its own, the
 * same way: see docs/kickoff.md.
 */
public final class Rebuilt {
  private Rebuilt() {}

  /**
   * What 2026's manual allows a robot: a 110 in frame perimeter and 30 in tall at the start (R104),
   * bumpers within 2.5 to 5.75 in of the floor (R405), and 135 lb with bumpers and without its
   * battery (R408), a battery with its leads weighing about 14.5 lb (R601). Placeholders until the
   * next kickoff. The extension limit is a guess: 2026's wasn't read (2024 allowed 12 in).
   */
  public static final SeasonLimits LIMITS =
      SeasonLimits.builder()
          .framePerimeter(Inches.of(110))
          .startingHeight(Inches.of(30))
          .extension(Inches.of(12))
          .bumperZoneBottom(Inches.of(2.5))
          .bumperZoneTop(Inches.of(5.75))
          .weight(Pounds.of(135))
          .battery(Pounds.of(14.5))
          .build();

  /** The welded field. The AndyMark build is 2 cm smaller, with its own layout. */
  static final Fields LAYOUT = Fields.FRC_2026_REBUILT_WELDED;

  /** Red's half is blue's turned half a turn: every tag lands on a partner within a millimeter. */
  static final Symmetry SYMMETRY = Symmetry.ROTATED;

  private static final double WIDTH = LAYOUT.width;

  /** One inch, in meters: the manual measures in inches. */
  private static final double IN = 0.0254;

  // ---- Materials, where the manual names them. Contact numbers come later, per pair. ----

  /** The bumps' ramps: half-inch HDPE with an orange-peel texture (5.5). */
  static final Material HDPE = Material.builder().name("HDPE, orange peel").build();

  /** The tower, and the depot's barriers: powder-coated steel (5.7, 5.8). */
  static final Material STEEL = Material.builder().name("powder-coated steel").build();

  /**
   * What the manual doesn't name a material for: the hub's and the trenches' faces, and the
   * alliance wall's, the tower's and the outpost's.
   */
  static final Material STRUCTURE = Material.builder().name("field structure").build();

  // ---- The hub, the bumps beside it, and the trenches beyond them, across the field ----

  /** The robot starting line, the alliance zone's edge, runs along the hub's near face (tag 26). */
  static final double STARTING_LINE_X = 4.022;

  /** The hub's center line, midway between the tags on its near and far faces (26 and 20). */
  static final double HUB_X = 4.626;

  /** The hub's footprint: 47 in square (5.4). */
  static final double HUB_SIZE = 47.0 * IN;

  // Each bump, beside the hub: 73.0 in wide, 44.4 in deep, and 6.513 in tall, a ridge with 15°
  // ramps down toward both zones, its crest on the hub's center line (5.5).
  static final double BUMP_WIDTH = 1.854;
  static final double BUMP_DEPTH = 1.128;
  static final double BUMP_HEIGHT = 0.1654;

  /** Where a bump ends at the trench, across the field: past the hub's half and a bump's width. */
  private static final double BUMP_EDGE = WIDTH / 2 - HUB_SIZE / 2 - BUMP_WIDTH;

  /** Each trench, beside the hub: 47 in deep, with a 50.34 in opening from the guardrail (5.6). */
  static final double TRENCH_DEPTH = 1.194;

  static final double TRENCH_OPENING = 1.279;

  /** The bump on the drivers' left of their hub, between it and the trench. */
  public static final Bump LEFT_BUMP =
      new Bump(
          "Left bump",
          new Translation2d(HUB_X, WIDTH / 2 + HUB_SIZE / 2),
          new Translation2d(HUB_X, WIDTH / 2 + HUB_SIZE / 2 + BUMP_WIDTH),
          BUMP_HEIGHT,
          BUMP_DEPTH,
          HDPE);

  /** The bump on the drivers' right of their hub. */
  public static final Bump RIGHT_BUMP =
      new Bump(
          "Right bump",
          new Translation2d(HUB_X, BUMP_EDGE),
          new Translation2d(HUB_X, WIDTH / 2 - HUB_SIZE / 2),
          BUMP_HEIGHT,
          BUMP_DEPTH,
          HDPE);

  // ---- The tower, built into the alliance wall (5.8) ----

  /** The tower's center line across the field: tag 31's, the tower wall's center tag. */
  static final double TOWER_Y = 3.746;

  /** The base: a steel plate 39.0 in wide and 45.18 in out from the wall, edges about 0.25 in. */
  static final double BASE_WIDTH = 39.0 * IN;

  static final double BASE_DEPTH = 45.18 * IN;
  static final double BASE_HEIGHT = 0.25 * IN;

  // ---- The outpost, in the alliance wall's corner by the guardrail (5.9.2) ----

  // ---- The depot, along the alliance wall (5.7) ----

  /** The depot: 42.0 in wide and 27.0 in deep. */
  static final double DEPOT_WIDTH = 42.0 * IN;

  static final double DEPOT_DEPTH = 27.0 * IN;

  /** Its barriers: steel 3.0 in wide and 1.0 in tall, 1.125 in with the hook fastener under it. */
  static final double BARRIER_WIDTH = 3.0 * IN;

  static final double BARRIER_HEIGHT = 1.125 * IN;

  /**
   * The depot's center across the field: centered on maple-sim's depot fuel ({@code
   * Arena2026Rebuilt} at 1b1e8ea), not checked against the field drawings. Its barriers run along
   * its three open sides, which is an assumption too: the manual's text doesn't say.
   */
  static final double DEPOT_Y = 5.908;

  /** The center of the hub, where shots go: midway between the tags on its near and far faces. */
  public static final Place HUB = Place.between("Hub", 26, 20);

  /** Squared up to the tower's center tag, just clear of its front face: where a climb lines up. */
  public static final Place TOWER = Place.facing("Tower", 31, 1.6);

  /** From the alliance wall to the starting line, wall to wall. */
  public static final Zone ALLIANCE_ZONE =
      Zone.ours(
          "Alliance zone",
          Polygon2d.rectangle(Translation2d.ZERO, new Translation2d(STARTING_LINE_X, WIDTH)));

  /** Under a trench's 22.25 in bar, beside either hub. */
  public static final Zone TRENCH =
      Zone.both(
          "Trench",
          Polygon2d.rectangle(
              new Translation2d(HUB_X - TRENCH_DEPTH / 2, WIDTH - TRENCH_OPENING),
              new Translation2d(HUB_X + TRENCH_DEPTH / 2, WIDTH)),
          Polygon2d.rectangle(
              new Translation2d(HUB_X - TRENCH_DEPTH / 2, 0),
              new Translation2d(HUB_X + TRENCH_DEPTH / 2, TRENCH_OPENING)));

  /**
   * What robots drive over on blue's half: the bumps, the tower's base plate, and the depot's
   * barriers. Each vertical edge is a step a wheel climbs, not a ramp.
   */
  static List<Surface> surfaces() {
    double depotFront = DEPOT_DEPTH - BARRIER_WIDTH;
    double depotLeft = DEPOT_Y + DEPOT_WIDTH / 2;
    double depotRight = DEPOT_Y - DEPOT_WIDTH / 2;
    return List.of(
        LEFT_BUMP,
        RIGHT_BUMP,
        plate(
            "Tower base",
            0,
            BASE_DEPTH,
            TOWER_Y - BASE_WIDTH / 2,
            TOWER_Y + BASE_WIDTH / 2,
            BASE_HEIGHT,
            STEEL),
        plate(
            "Depot barrier, front",
            depotFront,
            DEPOT_DEPTH,
            depotRight,
            depotLeft,
            BARRIER_HEIGHT,
            STEEL),
        plate(
            "Depot barrier, left",
            0,
            depotFront,
            depotLeft - BARRIER_WIDTH,
            depotLeft,
            BARRIER_HEIGHT,
            STEEL),
        plate(
            "Depot barrier, right",
            0,
            depotFront,
            depotRight,
            depotRight + BARRIER_WIDTH,
            BARRIER_HEIGHT,
            STEEL));
  }

  /** A flat plate on the floor between two corners, a vertical edge all round. */
  private static Platform plate(
      String name,
      double x0,
      double x1,
      double y0,
      double y1,
      double heightMeters,
      Material material) {
    Polygon2d top =
        Polygon2d.rectangle(
            new Pose2d((x0 + x1) / 2, (y0 + y1) / 2, Rotation2d.ZERO),
            Math.abs(x1 - x0),
            Math.abs(y1 - y0));
    return new Platform(
        name, top, heightMeters, Collections.nCopies(4, Platform.Side.EDGE), material);
  }

  /**
   * Where a robot of this size starts, left to right as the drivers see them: bumpers on the
   * starting line and facing the field, before each trench and before the hub. An auto starts on
   * one ({@code Rebuilt.starts(robot.body()).left()}), and the placement check reads whether the
   * robot stands on it.
   */
  public record Starts(Place left, Place center, Place right) {
    /** All three, left to right: the order of Driver Station locations 1, 2, and 3. */
    public List<Place> all() {
      return List.of(left, center, right);
    }
  }

  /** The starts for a robot of this size, written for blue. */
  public static Starts starts(Body body) {
    double x = STARTING_LINE_X - body.bumperLength().in(Meters) / 2;
    return new Starts(
        Place.at("Left", new Pose2d(x, WIDTH - TRENCH_OPENING / 2, Rotation2d.ZERO)),
        Place.at("Center", new Pose2d(x, WIDTH / 2, Rotation2d.ZERO)),
        Place.at("Right", new Pose2d(x, TRENCH_OPENING / 2, Rotation2d.ZERO)));
  }

  /**
   * The field model for a robot of this size, which sets where it stands on the starting line.
   *
   * <p>Starts, left to right as the drivers see them ({@link #starts}). Driver Station locations 1,
   * 2, and 3 choose them when no auto is chosen; a chosen auto names its own.
   */
  public static FieldModel field(Body body) {
    return field(body, LAYOUT);
  }

  /**
   * The season's field for venues: the welded field by default, either build a venue may be
   * measured against, and the field model on any layout.
   */
  public static Venues.Game venues(Body body) {
    return new Venues.Game(
        LAYOUT.loadField(),
        List.of(
            Fields.FRC_2026_REBUILT_WELDED.loadField(),
            Fields.FRC_2026_REBUILT_ANDY_MARK.loadField()),
        SYMMETRY,
        layout -> field(body, layout));
  }

  /** The field model on another of the season's layouts: the AndyMark build, say. */
  static FieldModel field(Body body, Fields layout) {
    return field(body, layout.loadField());
  }

  /** The field model on a WPILib layout, its tags where it puts them. */
  public static FieldModel field(Body body, Field layout) {
    return field(body, TagLayout.of(layout));
  }

  /**
   * The field model on the field's tags: one of the season's official layouts, or a venue's, its
   * tags as surveyed.
   */
  public static FieldModel field(Body body, TagLayout layout) {
    return FieldModel.builder(layout, SYMMETRY)
        .starts(starts(body).all())
        .places(List.of(HUB, TOWER))
        .zones(List.of(ALLIANCE_ZONE, TRENCH))
        .surfaces(surfaces())
        .build();
  }
}
