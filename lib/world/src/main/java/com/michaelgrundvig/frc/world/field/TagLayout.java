package com.michaelgrundvig.frc.world.field;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;

/**
 * The field's size and its AprilTags: what a {@link FieldModel} is built on. The official layout
 * gives only each tag's nominal pose ({@link #of}); a venue adds where its survey found them, and
 * how sure it is ({@link Venue#tagLayout}).
 *
 * @param name what it's called: the official layout's name (such as "2026 FRC Rebuilt Welded"), or
 *     a venue's
 * @param lengthMeters along x, from the blue alliance wall to the red
 * @param widthMeters along y
 * @param tags every tag, each number once
 */
public record TagLayout(String name, double lengthMeters, double widthMeters, List<Tag> tags) {
  public TagLayout {
    if (!(lengthMeters > 0)
        || !(widthMeters > 0)
        || Double.isInfinite(lengthMeters + widthMeters)) {
      throw new IllegalArgumentException(
          name + ": a field's size is positive, not " + lengthMeters + " by " + widthMeters + " m");
    }
    tags = List.copyOf(tags);
    Set<Integer> ids = new HashSet<>();
    for (Tag tag : tags) {
      if (!ids.add(tag.id())) {
        throw new IllegalArgumentException(name + " has tag " + tag.id() + " twice");
      }
    }
  }

  /**
   * WPILib's layout, each tag where it says: nominal, with nothing surveyed and the stated default
   * sigma.
   */
  public static TagLayout of(Field layout) {
    List<Tag> tags = new ArrayList<>();
    for (FieldTag tag : layout.getTags()) {
      tags.add(Tag.nominal(tag.getID(), tag.getPose()));
    }
    return new TagLayout(layout.getName(), layout.getFieldLength(), layout.getFieldWidth(), tags);
  }
}
