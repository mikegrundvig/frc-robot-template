package frc.robot.lib.field;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.wpilib.driverstation.Alliance;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.shape.Rectangle2d;

/**
 * One season's field: its size, AprilTags, and symmetry, the start positions, places, and zones
 * robot code refers to, and the terrain robots drive over. Built once at startup from a tag layout
 * (WPILib's for the season, or one measured at an event) and never changed after. Every place and
 * zone is checked against the layout as the model is built, so a wrong tag number fails at startup,
 * and first in the tests.
 *
 * <p>Everything is written for the blue alliance, and {@link #forAlliance} is the one conversion to
 * red (see the package documentation).
 */
public final class FieldModel {
  /**
   * How close a tag's image under the symmetry must be to a tag facing the same way for the two to
   * be partners: room for a layout measured at an event, and well under the spacing of different
   * tags.
   */
  static final double PARTNER_DISTANCE_METERS = 0.25;

  /** How closely a partner must face the way the symmetry turns the tag. */
  static final double PARTNER_ANGLE_RADIANS = Math.toRadians(20);

  private final String name;
  private final double length;
  private final double width;
  private final Symmetry symmetry;
  private final Map<Integer, Pose3d> tags;
  private final Map<Integer, Integer> partners;
  private final List<Place> starts;
  private final List<Place> places;
  private final List<Zone> zones;
  private final Terrain terrain;

  /**
   * @param layout the field's size and AprilTags; the model keeps its own copy
   * @param symmetry how red's half relates to blue's
   * @param starts where robots start a match
   * @param places other named places: targets, scoring and loading spots
   * @param zones named areas of the field
   * @throws IllegalArgumentException if a place or zone doesn't fit the layout (a tag that isn't in
   *     it, or has no partner on the other half), or two share a name
   */
  public FieldModel(
      Field layout, Symmetry symmetry, List<Place> starts, List<Place> places, List<Zone> zones) {
    this(layout, symmetry, starts, places, zones, List.of());
  }

  /**
   * @param bumps the bumps on the blue alliance's half, each named as blue's drivers would ("Left
   *     bump"): the model adds each one's partner on red's half, and names them "(blue)" and
   *     "(red)"
   */
  public FieldModel(
      Field layout,
      Symmetry symmetry,
      List<Place> starts,
      List<Place> places,
      List<Zone> zones,
      List<Bump> bumps) {
    name = layout.getName();
    length = layout.getFieldLength();
    width = layout.getFieldWidth();
    this.symmetry = symmetry;
    Map<Integer, Pose3d> byId = new LinkedHashMap<>();
    for (FieldTag tag : layout.getTags()) {
      byId.put(tag.getID(), tag.getPose());
    }
    tags = Collections.unmodifiableMap(byId);
    partners = pairTags();
    this.starts = List.copyOf(starts);
    this.places = List.copyOf(places);
    this.zones = List.copyOf(zones);
    List<Bump> both = new ArrayList<>();
    for (Bump bump : bumps) {
      both.add(
          new Bump(
              bump.name() + " (blue)",
              bump.crestStart(),
              bump.crestEnd(),
              bump.heightMeters(),
              bump.depthMeters()));
      both.add(bump.image(bump.name() + " (red)", symmetry, length, width));
    }
    terrain = new Terrain(both);
    check();
  }

  /** The layout's name, such as "2026 FRC Rebuilt Welded". */
  public String name() {
    return name;
  }

  /** Meters, along x: from the blue alliance wall to the red. */
  public double length() {
    return length;
  }

  /** Meters, along y. */
  public double width() {
    return width;
  }

  public Symmetry symmetry() {
    return symmetry;
  }

  /** Every tag's pose, by ID, in the field frame. */
  public Map<Integer, Pose3d> tags() {
    return tags;
  }

  /** The layout as WPILib's {@link Field}, for tools that take one, such as a vision simulation. */
  public Field toField() {
    List<FieldTag> list = new ArrayList<>();
    for (Map.Entry<Integer, Pose3d> tag : tags.entrySet()) {
      list.add(new FieldTag(tag.getKey(), tag.getValue()));
    }
    return new Field(name, "", "", null, length, width, "frc", list);
  }

  /** A tag's pose, if the layout has it. */
  public Optional<Pose3d> tag(int id) {
    return Optional.ofNullable(tags.get(id));
  }

  /** Where robots start a match, in the order they were given. */
  public List<Place> starts() {
    return starts;
  }

  /** The named places other than the starts. */
  public List<Place> places() {
    return places;
  }

  public List<Zone> zones() {
    return zones;
  }

  /** The bumps robots drive over, on both halves of the field. */
  public Terrain terrain() {
    return terrain;
  }

  /**
   * A pose written for the blue alliance, for an alliance: the same pose for blue, and its image
   * under the field's symmetry for red.
   */
  public Pose2d forAlliance(Alliance alliance, Pose2d blue) {
    return alliance == Alliance.BLUE ? blue : symmetry.apply(blue, length, width);
  }

  /** A point written for the blue alliance, for an alliance. */
  public Translation2d forAlliance(Alliance alliance, Translation2d blue) {
    return alliance == Alliance.BLUE ? blue : symmetry.apply(blue, length, width);
  }

  /** An area written for the blue alliance, for an alliance. */
  public Rectangle2d forAlliance(Alliance alliance, Rectangle2d blue) {
    return alliance == Alliance.BLUE
        ? blue
        : new Rectangle2d(
            forAlliance(alliance, blue.getCenter()), blue.getXWidth(), blue.getYWidth());
  }

  /**
   * Which way is straight ahead for an alliance's drivers: +x for blue, -x for red. Always half a
   * turn apart, whatever the field's symmetry, because the driver stations face each other; so
   * field-relative driving turns the sticks by this, never by the symmetry.
   */
  public static Rotation2d driverForward(Alliance alliance) {
    return alliance == Alliance.BLUE ? Rotation2d.ZERO : Rotation2d.PI;
  }

  /**
   * The tag that plays a blue tag's part for an alliance: the tag itself for blue, and for red the
   * tag the symmetry puts in its place, found in the layout (so as measured, when it was).
   *
   * @throws IllegalArgumentException if the layout has no such tag, or no partner for it
   */
  public int tagFor(Alliance alliance, int blueTag) {
    if (!tags.containsKey(blueTag)) {
      throw new IllegalArgumentException("the layout has no tag " + blueTag);
    }
    if (alliance == Alliance.BLUE) {
      return blueTag;
    }
    Integer partner = partners.get(blueTag);
    if (partner == null) {
      throw new IllegalArgumentException(
          "tag " + blueTag + " has no partner on the red alliance's half: is the symmetry right?");
    }
    return partner;
  }

  /** On the floor, where {@link #tagFor} puts a blue tag for an alliance. */
  Pose2d tagPose(Alliance alliance, int blueTag) {
    return pose(tagFor(alliance, blueTag)).toPose2d();
  }

  private Pose3d pose(int tag) {
    Pose3d pose = tags.get(tag);
    if (pose == null) {
      throw new IllegalArgumentException("the layout has no tag " + tag);
    }
    return pose;
  }

  /** Whether a point is on the field, within the walls. */
  public boolean contains(Translation2d point) {
    return point.getX() >= 0
        && point.getX() <= length
        && point.getY() >= 0
        && point.getY() <= width;
  }

  /**
   * How far the layout is from the declared symmetry: the largest distance between a tag's image
   * and the nearest tag facing the same way, in meters. Near zero when the field has the symmetry;
   * worth a test, since every place on the red alliance depends on it.
   */
  public double symmetryError() {
    double worst = 0;
    for (Pose3d tag : tags.values()) {
      Pose2d image = symmetry.apply(tag.toPose2d(), length, width);
      worst =
          Math.max(
              worst,
              nearestFacing(image).map(id -> distance(id, image)).orElse(Double.POSITIVE_INFINITY));
    }
    return worst;
  }

  /**
   * Each tag's partner: the tag where the symmetry puts it, facing the way the symmetry turns it.
   */
  private Map<Integer, Integer> pairTags() {
    Map<Integer, Integer> pairs = new LinkedHashMap<>();
    for (Map.Entry<Integer, Pose3d> tag : tags.entrySet()) {
      Pose2d image = symmetry.apply(tag.getValue().toPose2d(), length, width);
      nearestFacing(image)
          .filter(id -> distance(id, image) <= PARTNER_DISTANCE_METERS)
          .ifPresent(id -> pairs.put(tag.getKey(), id));
    }
    return Collections.unmodifiableMap(pairs);
  }

  /** The nearest tag facing within {@link #PARTNER_ANGLE_RADIANS} of the pose's heading. */
  private Optional<Integer> nearestFacing(Pose2d pose) {
    Integer nearest = null;
    double best = Double.POSITIVE_INFINITY;
    for (Map.Entry<Integer, Pose3d> tag : tags.entrySet()) {
      Pose2d other = tag.getValue().toPose2d();
      double turn = Math.abs(other.getRotation().minus(pose.getRotation()).getRadians());
      double distance = other.getTranslation().getDistance(pose.getTranslation());
      if (turn <= PARTNER_ANGLE_RADIANS && distance < best) {
        nearest = tag.getKey();
        best = distance;
      }
    }
    return Optional.ofNullable(nearest);
  }

  private double distance(int tag, Pose2d pose) {
    return pose(tag).toPose2d().getTranslation().getDistance(pose.getTranslation());
  }

  /** Every place resolves for both alliances, and names are unique. */
  private void check() {
    List<Place> named = new ArrayList<>(starts);
    named.addAll(places);
    Set<String> names = new HashSet<>();
    for (Place place : named) {
      if (!names.add(place.name())) {
        throw new IllegalArgumentException("two places are named " + place.name());
      }
      for (Alliance alliance : Alliance.values()) {
        try {
          place.on(this, alliance);
        } catch (IllegalArgumentException e) {
          throw new IllegalArgumentException(place.name() + ": " + e.getMessage(), e);
        }
      }
    }
    Set<String> zoneNames = new HashSet<>();
    for (Zone zone : zones) {
      if (!zoneNames.add(zone.name())) {
        throw new IllegalArgumentException("two zones are named " + zone.name());
      }
      for (Rectangle2d area : zone.blueAreas()) {
        if (!contains(area.getCenter().getTranslation())) {
          throw new IllegalArgumentException(zone.name() + ": an area is off the field");
        }
      }
    }
    Set<String> bumpNames = new HashSet<>();
    for (Bump bump : terrain.bumps()) {
      if (!bumpNames.add(bump.name())) {
        throw new IllegalArgumentException("two bumps are named " + bump.name());
      }
      if (!contains(bump.crestStart()) || !contains(bump.crestEnd())) {
        throw new IllegalArgumentException(bump.name() + ": its crest runs off the field");
      }
    }
  }
}
