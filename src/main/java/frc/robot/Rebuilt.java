package frc.robot;

import static org.wpilib.units.Units.Meters;

import frc.robot.lib.config.Body;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.field.Place;
import frc.robot.lib.field.Symmetry;
import frc.robot.lib.field.Zone;
import java.util.List;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.shape.Rectangle2d;

/**
 * The field the robot plays on: 2026's, REBUILT, until the next game's is written at kickoff.
 *
 * <p>Everything here is written as if the robot were on the blue alliance, in meters in WPILib's
 * field frame (see {@code frc.robot.lib.field}); the field model works out the red alliance's.
 * Places near AprilTags are derived from them, so a layout measured at an event moves them for both
 * alliances. Numbers come from WPILib's 2026 tag layout and the game manual.
 *
 * <p>At kickoff: choose the new game's layout from {@link Fields}, declare its symmetry (the season
 * test checks it against the tags), and write its starts, places, and zones the same way.
 */
public final class Rebuilt {
  private Rebuilt() {}

  /** The welded field. The AndyMark build is 2 cm smaller, with its own layout. */
  static final Fields LAYOUT = Fields.FRC_2026_REBUILT_WELDED;

  /** Red's half is blue's turned half a turn: every tag lands on a partner within a millimeter. */
  static final Symmetry SYMMETRY = Symmetry.ROTATED;

  private static final double WIDTH = LAYOUT.width;

  /** The robot starting line, the alliance zone's edge, runs along the hub's near face (tag 26). */
  static final double STARTING_LINE_X = 4.022;

  /** The hub's center line, midway between the tags on its near and far faces (26 and 20). */
  static final double HUB_X = 4.626;

  /** Each trench, beside the hub: 47 in deep, with a 50.34 in opening from the wall. */
  static final double TRENCH_DEPTH = 1.194;

  static final double TRENCH_OPENING = 1.279;

  /** The center of the hub, where shots go: midway between the tags on its near and far faces. */
  public static final Place HUB = Place.between("Hub", 26, 20);

  /** Squared up to the tower's center tag, just clear of its front face: where a climb lines up. */
  public static final Place TOWER = Place.facing("Tower", 31, 1.6);

  /** From the alliance wall to the starting line, wall to wall. */
  public static final Zone ALLIANCE_ZONE =
      Zone.ours(
          "Alliance zone",
          new Rectangle2d(Translation2d.ZERO, new Translation2d(STARTING_LINE_X, WIDTH)));

  /** Under a trench's 22.25 in bar, beside either hub. */
  public static final Zone TRENCH =
      Zone.both(
          "Trench",
          new Rectangle2d(
              new Translation2d(HUB_X - TRENCH_DEPTH / 2, WIDTH - TRENCH_OPENING),
              new Translation2d(HUB_X + TRENCH_DEPTH / 2, WIDTH)),
          new Rectangle2d(
              new Translation2d(HUB_X - TRENCH_DEPTH / 2, 0),
              new Translation2d(HUB_X + TRENCH_DEPTH / 2, TRENCH_OPENING)));

  /**
   * The field model for a robot of this size, which sets where it stands on the starting line.
   *
   * <p>Starts, left to right as the drivers see them, bumpers on the starting line and facing the
   * field: before each trench, and before the hub. Driver Station locations 1, 2, and 3 choose them
   * until autos do.
   */
  public static FieldModel field(Body body) {
    return field(body, LAYOUT);
  }

  /** The field model on another of the season's layouts: the AndyMark build, say. */
  static FieldModel field(Body body, Fields layout) {
    double x = STARTING_LINE_X - body.bumperLength().in(Meters) / 2;
    return new FieldModel(
        layout.loadField(),
        SYMMETRY,
        List.of(
            Place.at("Left", new Pose2d(x, WIDTH - TRENCH_OPENING / 2, Rotation2d.ZERO)),
            Place.at("Center", new Pose2d(x, WIDTH / 2, Rotation2d.ZERO)),
            Place.at("Right", new Pose2d(x, TRENCH_OPENING / 2, Rotation2d.ZERO))),
        List.of(HUB, TOWER),
        List.of(ALLIANCE_ZONE, TRENCH));
  }
}
