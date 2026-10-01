package com.michaelgrundvig.frc.pitwall;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;

/**
 * What the robot program has to tell whoever is at the page, right now and above everything else:
 * "Safe to switch off", say. A notice shows in the state bar on every view, and as a banner at the
 * top of Pit, only while it has something to say.
 *
 * <pre>{@code
 * pitwall.notices().add("Coprocessors", coprocessors::notice); // null while there's nothing
 * }</pre>
 *
 * <p>Notices are read in the robot loop, every loop while a page watches, so keep each cheap. One
 * that throws says what it threw, rather than stopping the robot.
 */
public final class Notices {
  /** How a notice stands: the page marks each its own way, never by color alone. */
  public enum Level {
    /** Good news, such as "Safe to switch off": ✓. */
    OK,
    /** Under way, or worth knowing: •. */
    NOTE,
    /** Worth acting on: ▲. */
    WARNING,
    /** Wrong, and in the way: ✗. */
    PROBLEM
  }

  /**
   * One notice.
   *
   * @param text what it says, in a few words: it shows in the state bar
   * @param level how it stands
   */
  public record Notice(String text, Level level) {}

  private final Map<String, Supplier<@Nullable Notice>> sources = new LinkedHashMap<>();
  private volatile Map<String, Object> section = Map.of();

  /**
   * Adds a source of notices, shown in the order added.
   *
   * @param name whose notice it is, such as "Coprocessors"
   * @param notice the notice now, or null when there's none
   * @return this, to add the next
   * @throws IllegalArgumentException if the name is blank or already used
   */
  public synchronized Notices add(String name, Supplier<@Nullable Notice> notice) {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a notice needs a name");
    }
    if (sources.putIfAbsent(name, notice) != null) {
      throw new IllegalArgumentException("two notices are named " + name);
    }
    return this;
  }

  /** Reads every notice, if a page watches. Call it once per loop, in the robot loop. */
  public synchronized void periodic(boolean watched) {
    if (!watched) {
      return;
    }
    Map<String, Object> shown = new LinkedHashMap<>();
    sources.forEach(
        (name, source) -> {
          Notice notice = read(source);
          if (notice != null && !notice.text().isBlank()) {
            Map<String, Object> each = new LinkedHashMap<>();
            each.put("text", notice.text());
            each.put("level", notice.level().name().toLowerCase(Locale.ROOT));
            shown.put(name, each);
          }
        });
    section = shown;
  }

  /** The notices with something to say, as of the last read: {@code {name: {"text", "level"}}}. */
  Map<String, Object> view() {
    return section;
  }

  private static @Nullable Notice read(Supplier<@Nullable Notice> source) {
    try {
      return source.get();
    } catch (RuntimeException e) {
      String why = e.getMessage();
      return new Notice("Error: " + (why != null ? why : e.toString()), Level.WARNING);
    }
  }
}
