package com.michaelgrundvig.frc.world.field;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;
import java.util.TreeSet;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;

/**
 * Whether a venue fits the game's field, checked before the robot stores or uses it: every problem
 * at once, in words for the pit. The bounds catch a wrong file or a typo, not a careful survey's
 * small corrections.
 */
public final class VenueCheck {
  /**
   * How far a tag may sit from its official place with no reason given: four inches, more than any
   * field's tolerance and less than a tag mistaken for its neighbor.
   */
  public static final double MAX_SHIFT_METERS = 0.10;

  /** How far a tag may turn from its official orientation with no reason given. */
  public static final double MAX_TURN_DEGREES = 5;

  /** How closely an official venue's tags must match WPILib's layout. */
  static final double OFFICIAL_METERS = 0.001;

  private VenueCheck() {}

  /**
   * What's wrong with a venue against the game's official layout; empty when it fits.
   *
   * @param base the official layout of the field variant the venue says it was measured against
   * @param symmetry the game's, which must still pair every tag with its partner
   */
  public static List<String> problems(Venue venue, Field base, Symmetry symmetry) {
    List<String> problems = new ArrayList<>();
    if (!venue.base().equals(base.getName())) {
      problems.add(
          "measured against \""
              + venue.base()
              + "\", but the field's official layout is \""
              + base.getName()
              + "\"");
    }
    Field layout = venue.layout();
    if (Math.abs(layout.getFieldLength() - base.getFieldLength()) > 0.01
        || Math.abs(layout.getFieldWidth() - base.getFieldWidth()) > 0.01) {
      problems.add(
          String.format(
              Locale.ROOT,
              "the field is %.3f x %.3f m, the official one %.3f x %.3f m",
              layout.getFieldLength(),
              layout.getFieldWidth(),
              base.getFieldLength(),
              base.getFieldWidth()));
    }
    Map<Integer, Pose3d> official = new HashMap<>();
    for (FieldTag tag : base.getTags()) {
      official.put(tag.getID(), tag.getPose());
    }
    Map<Integer, Pose3d> measured = new HashMap<>();
    for (FieldTag tag : layout.getTags()) {
      measured.put(tag.getID(), tag.getPose());
    }
    TreeSet<Integer> missing = new TreeSet<>(official.keySet());
    missing.removeAll(measured.keySet());
    if (!missing.isEmpty()) {
      problems.add("the layout is missing tags " + missing);
    }
    TreeSet<Integer> extra = new TreeSet<>(measured.keySet());
    extra.removeAll(official.keySet());
    if (!extra.isEmpty()) {
      problems.add("the official layout has no tags " + extra);
    }
    for (Map.Entry<Integer, Pose3d> tag : new TreeMap<>(measured).entrySet()) {
      int id = tag.getKey();
      Pose3d pose = tag.getValue();
      Pose3d nominal = official.get(id);
      if (nominal == null) {
        continue;
      }
      double shift = pose.getTranslation().getDistance(nominal.getTranslation());
      double turn = Math.toDegrees(angleBetween(pose.getRotation().getQuaternion(), nominal));
      if (venue.kind() == Venue.Kind.OFFICIAL) {
        if (shift > OFFICIAL_METERS || turn > Math.toDegrees(OFFICIAL_METERS)) {
          problems.add(
              String.format(
                  Locale.ROOT,
                  "an official venue, but tag %d sits %.1f mm and %.2f° from WPILib's layout",
                  id,
                  shift * 1000,
                  turn));
        }
      } else if ((shift > MAX_SHIFT_METERS || turn > MAX_TURN_DEGREES)
          && !venue.moved().containsKey(id)) {
        problems.add(
            String.format(
                Locale.ROOT,
                "tag %d sits %.3f m and %.1f° from its official place: if that's right, say why"
                    + " under \"moved\"",
                id,
                shift,
                turn));
      }
    }
    unknownTags(problems, "moved", venue.moved().keySet(), measured);
    pairs(problems, venue, base, symmetry);
    return problems;
  }

  /**
   * Every tag the official layout pairs with a partner under the symmetry is still paired with the
   * same one: places on red's half come from red's tags as measured, through these pairs.
   */
  private static void pairs(List<String> problems, Venue venue, Field base, Symmetry symmetry) {
    FieldModel official = new FieldModel(base, symmetry, List.of(), List.of(), List.of());
    FieldModel measured;
    try {
      measured = new FieldModel(venue.layout(), symmetry, List.of(), List.of(), List.of());
    } catch (IllegalArgumentException e) {
      problems.add("the layout doesn't make a field: " + e.getMessage());
      return;
    }
    for (int id : new TreeSet<>(official.tags().keySet())) {
      int partner;
      try {
        partner = official.tagFor(Alliance.RED, id);
      } catch (IllegalArgumentException e) {
        continue; // the official layout doesn't pair it either
      }
      try {
        int now = measured.tagFor(Alliance.RED, id);
        if (now != partner) {
          problems.add(
              "tag "
                  + id
                  + " now pairs with tag "
                  + now
                  + " under the field's symmetry, not "
                  + partner);
        }
      } catch (IllegalArgumentException e) {
        if (venue.layout().getTags().stream().anyMatch(tag -> tag.getID() == id)) {
          problems.add(
              "tag " + id + " no longer pairs with tag " + partner + " under the field's symmetry");
        }
      }
    }
  }

  private static double angleBetween(Quaternion a, Pose3d nominal) {
    Quaternion difference = a.times(nominal.getRotation().getQuaternion().inverse());
    return 2 * Math.acos(Math.min(1, Math.abs(difference.getW())));
  }

  private static void unknownTags(
      List<String> problems, String where, Iterable<Integer> ids, Map<Integer, Pose3d> measured) {
    TreeSet<Integer> unknown = new TreeSet<>();
    for (int id : ids) {
      if (!measured.containsKey(id)) {
        unknown.add(id);
      }
    }
    if (!unknown.isEmpty()) {
      problems.add(where + " names tags the layout doesn't have: " + unknown);
    }
  }
}
