package com.michaelgrundvig.frc.world.field;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HexFormat;
import java.util.List;
import java.util.Map;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;

/**
 * A field as measured: the official layout corrected where a venue's tags were found, such as
 * WPIcal's output for an event's field. Venues are runtime data, never code: a file uploaded in the
 * pit, or committed under {@code venues/}, read by {@link VenueFile} and checked by {@link
 * VenueCheck} before the robot uses it. Set against the official layout it was measured against, it
 * gives its {@link Tag}s ({@link #tags}), which a {@link FieldModel} is built on.
 *
 * @param name what people call it, such as "Practice field, October"
 * @param kind a practice field, an event, or the official layout itself
 * @param base the name of the official layout it was measured against (WPILib's, such as "2026 FRC
 *     Rebuilt Welded")
 * @param layout every tag where the survey found it, in WPILib's field frame: the file's layout,
 *     unchanged, as the coprocessors get it
 * @param moved why each tag the survey found far from its official place is there
 * @param method how it was measured: "hand", "wpical", "bundle adjustment", or "unknown"
 * @param date when it was measured (ISO 8601), or empty when unknown
 * @param wrapped true when the file was a bare WPILib layout, given the stated defaults
 * @param text the file as read, which is what's logged, stored and downloaded
 */
public record Venue(
    String name,
    Kind kind,
    String base,
    Field layout,
    Map<Integer, String> moved,
    String method,
    String date,
    boolean wrapped,
    String text) {

  /** The one format this code reads; a file says which it was written in. */
  public static final int FORMAT = 1;

  /** What kind of field a venue is. */
  public enum Kind {
    /** The team's practice field: never valid with the field management system attached. */
    PRACTICE,
    /** An event's field, surveyed there. */
    EVENT,
    /** The official layout, as WPILib ships it. */
    OFFICIAL
  }

  public Venue {
    moved = Map.copyOf(moved);
  }

  /**
   * The venue's tags, set against the official layout it was measured against: each where that
   * layout puts it (its nominal pose) and, unless the venue is the official layout itself, where
   * the survey found it.
   *
   * @param base the official layout named by {@link #base}
   * @throws IllegalArgumentException if the venue has a tag the base doesn't, which {@link
   *     VenueCheck} reports first
   */
  public List<Tag> tags(Field base) {
    Map<Integer, Pose3d> official = new HashMap<>();
    for (FieldTag tag : base.getTags()) {
      official.put(tag.getID(), tag.getPose());
    }
    List<Tag> tags = new ArrayList<>();
    for (FieldTag tag : layout.getTags()) {
      Pose3d nominal = official.get(tag.getID());
      if (nominal == null) {
        throw new IllegalArgumentException(
            name + ": " + base.getName() + " has no tag " + tag.getID());
      }
      tags.add(new Tag(tag.getID(), nominal, kind == Kind.OFFICIAL ? null : tag.getPose()));
    }
    return tags;
  }

  /** The venue's tags ({@link #tags}), with the field's size and the venue's name. */
  public TagLayout tagLayout(Field base) {
    return new TagLayout(
        layout.getName(), layout.getFieldLength(), layout.getFieldWidth(), tags(base));
  }

  /**
   * The file's SHA-256, in hex: what tells an uploaded venue from one in Git with the same name.
   */
  public String hash() {
    return hash(text);
  }

  /** A venue file's SHA-256, in hex, from its text as stored. */
  public static String hash(String text) {
    try {
      return HexFormat.of()
          .formatHex(
              MessageDigest.getInstance("SHA-256").digest(text.getBytes(StandardCharsets.UTF_8)));
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException("every Java has SHA-256", e);
    }
  }
}
