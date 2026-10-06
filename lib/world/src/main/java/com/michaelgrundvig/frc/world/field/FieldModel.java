package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * One season's field: its size, AprilTags, and symmetry, the start positions, places, and zones
 * robot code refers to, and the terrain robots drive over. Built once at startup ({@link #builder})
 * from the field's tags (WPILib's official layout, or a venue's survey of it) and never changed
 * after. Everything is checked against the tags as the model is built, so a wrong tag number fails
 * at startup, and first in the tests.
 *
 * <p>Everything is written for the blue alliance, and {@link #forAlliance} is the one conversion to
 * red (see the package documentation). Tags pair with their partners on the other half by their
 * official poses, which the symmetry turns; where a survey found them never changes which tag plays
 * which part.
 *
 * <p>The model names the two copies of a surface written for blue's half "(blue)" and "(red)": for
 * the half each is on, not for whose it is. Whose something is lives on the places and zones robot
 * code asks about ({@link Zone#ours}), which say it whichever half they're on.
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
  private final Map<Integer, Tag> tags;
  private final Map<Integer, Integer> partners;
  private final List<Place> starts;
  private final List<Place> places;
  private final List<Zone> zones;
  private final Terrain terrain;

  /**
   * A field on a WPILib layout, its tags as it gives them, with no terrain.
   *
   * @param layout the field's size and AprilTags; the model keeps its own copy
   * @param symmetry how red's half relates to blue's
   * @param starts where robots start a match
   * @param places other named places on the floor: spots to line up on, to score or load from
   * @param zones named areas of the field
   * @throws IllegalArgumentException if a place or zone doesn't fit the layout (a tag that isn't in
   *     it, or has no partner on the other half), or two share a name
   */
  public FieldModel(
      Field layout, Symmetry symmetry, List<Place> starts, List<Place> places, List<Zone> zones) {
    this(layout, symmetry, starts, places, zones, List.of());
  }

  /**
   * A field on a WPILib layout, its tags as it gives them, with its terrain.
   *
   * @param surfaces the raised ground on the blue alliance's half, each named as blue's drivers
   *     would ("Left bump"): the model adds each one's partner on red's half, and names them
   *     "(blue)" and "(red)" for the half each is on
   */
  public FieldModel(
      Field layout,
      Symmetry symmetry,
      List<Place> starts,
      List<Place> places,
      List<Zone> zones,
      List<? extends Surface> surfaces) {
    this(
        builder(TagLayout.of(layout), symmetry)
            .starts(starts)
            .places(places)
            .zones(zones)
            .surfaces(surfaces));
  }

  /**
   * Starts describing the field on its tags: the official layout's, or a venue's ({@link
   * Venue#tagLayout}). Everything else is named as it's given, and anything left out is none.
   *
   * @param layout the field's size and tags
   * @param symmetry how red's half relates to blue's
   */
  public static Builder builder(TagLayout layout, Symmetry symmetry) {
    return new Builder(layout, symmetry);
  }

  /** A field being described, each list named as it's given. */
  public static final class Builder {
    private final TagLayout layout;
    private final Symmetry symmetry;
    private List<Place> starts = List.of();
    private List<Place> places = List.of();
    private List<Zone> zones = List.of();
    private List<? extends Surface> surfaces = List.of();

    private Builder(TagLayout layout, Symmetry symmetry) {
      this.layout = layout;
      this.symmetry = symmetry;
    }

    /** Where robots start a match. */
    public Builder starts(List<Place> starts) {
      this.starts = List.copyOf(starts);
      return this;
    }

    /** Other named places on the floor: spots to line up on, to score or load from. */
    public Builder places(List<Place> places) {
      this.places = List.copyOf(places);
      return this;
    }

    /** Named areas of the field. */
    public Builder zones(List<Zone> zones) {
      this.zones = List.copyOf(zones);
      return this;
    }

    /**
     * The raised ground on the blue alliance's half, each named as blue's drivers would ("Left
     * bump"): the model adds each one's partner on red's half, and names them "(blue)" and "(red)"
     * for the half each is on, with their crossings.
     */
    public Builder surfaces(List<? extends Surface> surfaces) {
      this.surfaces = List.copyOf(surfaces);
      return this;
    }

    /**
     * The field, checked.
     *
     * @throws IllegalArgumentException if anything doesn't fit the tags or the field (a tag that
     *     isn't in it, or has no partner on the other half; a surface or zone off the field), or
     *     two share a name
     */
    public FieldModel build() {
      return new FieldModel(this);
    }
  }

  private FieldModel(Builder field) {
    TagLayout layout = field.layout;
    name = layout.name();
    length = layout.lengthMeters();
    width = layout.widthMeters();
    symmetry = field.symmetry;
    Map<Integer, Tag> byId = new LinkedHashMap<>();
    for (Tag tag : layout.tags()) {
      byId.put(tag.id(), tag);
    }
    tags = Collections.unmodifiableMap(byId);
    partners = pairTags();
    starts = field.starts;
    places = field.places;
    zones = field.zones;
    List<Surface> both = new ArrayList<>();
    for (Surface surface : field.surfaces) {
      both.add(renamed(surface, " (blue)"));
      both.add(renamed(symmetry.apply(surface, length, width), " (red)"));
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

  /**
   * Every tag, by ID: where the official layout puts it, where a survey found it, and how sure we
   * are.
   */
  public Map<Integer, Tag> tags() {
    return tags;
  }

  /**
   * The tags where they are, as WPILib's {@link Field}, for tools that take one, such as a vision
   * simulation: each as surveyed, if it was.
   */
  public Field toField() {
    List<FieldTag> list = new ArrayList<>();
    for (Tag tag : tags.values()) {
      list.add(new FieldTag(tag.id(), tag.pose()));
    }
    return new Field(name, "", "", null, length, width, "frc", list);
  }

  /** Where a tag is, in the field frame, if the field has it: as surveyed, if it was. */
  public Optional<Pose3d> tag(int id) {
    Tag tag = tags.get(id);
    return tag == null ? Optional.empty() : Optional.of(tag.pose());
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

  /** The raised ground robots drive over, on both halves of the field. */
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

  /**
   * A pose in 3D written for the blue alliance, for an alliance: its image under the field's
   * symmetry for red ({@link Symmetry#apply(Pose3d, double, double)}).
   */
  public Pose3d forAlliance(Alliance alliance, Pose3d blue) {
    return alliance == Alliance.BLUE ? blue : symmetry.apply(blue, length, width);
  }

  /** A point written for the blue alliance, for an alliance. */
  public Translation2d forAlliance(Alliance alliance, Translation2d blue) {
    return alliance == Alliance.BLUE ? blue : symmetry.apply(blue, length, width);
  }

  /** An area written for the blue alliance, for an alliance. */
  public Polygon2d forAlliance(Alliance alliance, Polygon2d blue) {
    return alliance == Alliance.BLUE ? blue : symmetry.apply(blue, length, width);
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
    return tagPose3d(alliance, blueTag).toPose2d();
  }

  /** Where {@link #tagFor} puts a blue tag for an alliance: as surveyed, if it was. */
  Pose3d tagPose3d(Alliance alliance, int blueTag) {
    return pose(tagFor(alliance, blueTag));
  }

  private Pose3d pose(int tag) {
    return tag(tag).orElseThrow(() -> new IllegalArgumentException("the layout has no tag " + tag));
  }

  /** Whether a point is on the field, within the walls. */
  public boolean contains(Translation2d point) {
    return point.getX() >= 0
        && point.getX() <= length
        && point.getY() >= 0
        && point.getY() <= width;
  }

  /**
   * How far the official layout is from the declared symmetry: the largest distance between a tag's
   * image and the nearest tag facing the same way, in meters. Near zero when the field has the
   * symmetry; worth a test, since every place on the red alliance depends on it.
   */
  public double symmetryError() {
    double worst = 0;
    for (Tag tag : tags.values()) {
      Pose2d image = symmetry.apply(tag.nominal().toPose2d(), length, width);
      worst =
          Math.max(
              worst,
              nearestFacing(image).map(id -> distance(id, image)).orElse(Double.POSITIVE_INFINITY));
    }
    return worst;
  }

  /**
   * Each tag's partner: the tag where the symmetry puts it, facing the way the symmetry turns it,
   * both as the official layout has them.
   */
  private Map<Integer, Integer> pairTags() {
    Map<Integer, Integer> pairs = new LinkedHashMap<>();
    for (Tag tag : tags.values()) {
      Pose2d image = symmetry.apply(tag.nominal().toPose2d(), length, width);
      nearestFacing(image)
          .filter(id -> distance(id, image) <= PARTNER_DISTANCE_METERS)
          .ifPresent(id -> pairs.put(tag.id(), id));
    }
    return Collections.unmodifiableMap(pairs);
  }

  /**
   * The nearest tag facing within {@link #PARTNER_ANGLE_RADIANS} of the pose's heading, as the
   * official layout has it.
   */
  private Optional<Integer> nearestFacing(Pose2d pose) {
    Integer nearest = null;
    double best = Double.POSITIVE_INFINITY;
    for (Tag tag : tags.values()) {
      Pose2d other = tag.nominal().toPose2d();
      double turn = Math.abs(other.getRotation().minus(pose.getRotation()).getRadians());
      double distance = other.getTranslation().getDistance(pose.getTranslation());
      if (turn <= PARTNER_ANGLE_RADIANS && distance < best) {
        nearest = tag.id();
        best = distance;
      }
    }
    return Optional.ofNullable(nearest);
  }

  /** How far a tag's official pose is from a pose, on the floor. */
  private double distance(int tag, Pose2d pose) {
    return nominal(tag).toPose2d().getTranslation().getDistance(pose.getTranslation());
  }

  private Pose3d nominal(int tag) {
    Tag known = tags.get(tag);
    if (known == null) {
      throw new IllegalArgumentException("the layout has no tag " + tag);
    }
    return known.nominal();
  }

  /**
   * A surface under its name with a suffix, its crossings too: a mesh's are named in it, and the
   * others' are named for the surface, so they follow.
   */
  private static Surface renamed(Surface surface, String suffix) {
    String renamed = surface.name() + suffix;
    return switch (surface) {
      case Bump bump ->
          new Bump(
              renamed,
              bump.crestStart(),
              bump.crestEnd(),
              bump.heightMeters(),
              bump.depthMeters(),
              bump.material());
      case Ramp ramp ->
          new Ramp(
              renamed,
              ramp.foot(),
              ramp.top(),
              ramp.widthMeters(),
              ramp.heightMeters(),
              ramp.material());
      case Platform platform ->
          new Platform(
              renamed,
              platform.top(),
              platform.heightMeters(),
              platform.sides(),
              platform.material());
      case MeshSurface mesh ->
          new MeshSurface(
              renamed,
              mesh.mesh(),
              mesh.crossings().stream()
                  .map(
                      crossing ->
                          new Crossing(
                              crossing.name() + suffix,
                              crossing.start(),
                              crossing.end(),
                              crossing.kind()))
                  .toList(),
              mesh.material());
    };
  }

  /**
   * Every place resolves for both alliances, the zones and terrain are on the field, and names are
   * unique.
   */
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
      for (Polygon2d area : zone.blueAreas()) {
        if (!contains(area.centroid())) {
          throw new IllegalArgumentException(zone.name() + ": an area is off the field");
        }
      }
    }
    Set<String> surfaceNames = new HashSet<>();
    for (Surface surface : terrain.surfaces()) {
      if (!surfaceNames.add(surface.name())) {
        throw new IllegalArgumentException("two surfaces are named " + surface.name());
      }
      boolean on = contains(surface.footprint().centroid());
      for (Crossing crossing : surface.crossings()) {
        on &= contains(crossing.start()) && contains(crossing.end());
      }
      if (!on) {
        throw new IllegalArgumentException(surface.name() + ": it runs off the field");
      }
    }
  }
}
