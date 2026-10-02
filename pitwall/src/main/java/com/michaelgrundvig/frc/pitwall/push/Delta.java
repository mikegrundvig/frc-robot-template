package com.michaelgrundvig.frc.pitwall.push;

import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.jspecify.annotations.Nullable;

/**
 * What the page has been sent, and what has changed since: the heart of pushing only changes.
 *
 * <p>The state is a document of sections (status, values, actions, ...), each a map from keys to
 * values. Each call to {@link #patch} compares the current state with what was last sent and
 * returns a JSON merge patch (RFC 7396) holding only what changed: a changed or new key with its
 * value, a removed key or section as {@code null}. Nothing changed, nothing to send: the patch is
 * empty.
 *
 * <p>A value that is itself a map or a list is compared whole, and sent whole when it changes; a
 * map's keys that are gone are sent as null, so a merge patch removes them.
 *
 * <p>Values compare by {@code equals}, except a {@link Reading}, which changes only once it moves
 * by its resolution from the value last sent. What's sent is what's remembered, so a slow drift
 * still gets sent once it adds up.
 *
 * <p>Every stream shares one delta. A stream that opens later starts from {@link #document()}, the
 * state as sent, and then receives the same patches as every other stream.
 *
 * <p>Not thread-safe: one owner calls it.
 */
public final class Delta {
  private final Map<String, Map<String, Object>> sent = new LinkedHashMap<>();

  /**
   * What changed since the last patch, as a JSON merge patch, and remembers it as sent.
   *
   * @param current the state now: sections by name, each a map from keys to strings, booleans,
   *     numbers, {@link Reading}s, or lists and maps (compared whole)
   * @return the changes, ready to send as JSON; empty when nothing changed
   */
  public Map<String, @Nullable Object> patch(Map<String, Map<String, Object>> current) {
    Map<String, @Nullable Object> patch = new LinkedHashMap<>();
    for (Map.Entry<String, Map<String, Object>> section : current.entrySet()) {
      Map<String, Object> was = sent.get(section.getKey());
      if (was == null) {
        was = new LinkedHashMap<>();
        sent.put(section.getKey(), was);
        patch.put(section.getKey(), sectionChanges(was, section.getValue(), true));
        continue;
      }
      Map<String, @Nullable Object> changes = sectionChanges(was, section.getValue(), false);
      if (!changes.isEmpty()) {
        patch.put(section.getKey(), changes);
      }
    }
    // Every section now is among those sent, so only fewer now means one has gone.
    if (sent.size() > current.size()) {
      for (Iterator<String> names = sent.keySet().iterator(); names.hasNext(); ) {
        String gone = names.next();
        if (!current.containsKey(gone)) {
          names.remove();
          patch.put(gone, null);
        }
      }
    }
    return patch;
  }

  /** The whole state as sent, ready to send as JSON: where a newly opened stream starts. */
  public Map<String, Object> document() {
    Map<String, Object> document = new LinkedHashMap<>();
    sent.forEach(
        (name, section) -> {
          Map<String, Object> values = new LinkedHashMap<>();
          section.forEach((key, value) -> values.put(key, json(value)));
          document.put(name, values);
        });
    return document;
  }

  /**
   * The changes within one section, recording each as sent. A new section's changes are all of its
   * keys, even when it's empty, so the page learns that it exists.
   */
  private static Map<String, @Nullable Object> sectionChanges(
      Map<String, Object> was, Map<String, Object> now, boolean all) {
    // Made only once something changed: most sections, most loops, have nothing to say.
    Map<String, @Nullable Object> changes = all ? new LinkedHashMap<>() : null;
    for (Map.Entry<String, Object> entry : now.entrySet()) {
      Object before = was.get(entry.getKey());
      Object after = entry.getValue();
      if (all || before == null || changed(before, after)) {
        was.put(entry.getKey(), after);
        if (changes == null) {
          changes = new LinkedHashMap<>();
        }
        changes.put(entry.getKey(), replacing(before, after));
      }
    }
    // Every key now is in what was sent (just put there if it wasn't), so only a section with
    // fewer keys now has lost any: no copy of it to find out.
    if (was.size() > now.size()) {
      for (Iterator<String> keys = was.keySet().iterator(); keys.hasNext(); ) {
        String gone = keys.next();
        if (!now.containsKey(gone)) {
          keys.remove();
          if (changes == null) {
            changes = new LinkedHashMap<>();
          }
          changes.put(gone, null);
        }
      }
    }
    return changes == null ? Map.of() : changes;
  }

  /**
   * A changed value as the patch carries it: whole, except that a map says which of its keys are
   * gone (as null), since a merge patch merges a map into the one before rather than replacing it.
   */
  private static @Nullable Object replacing(@Nullable Object before, Object after) {
    if (!(before instanceof Map<?, ?> was && after instanceof Map<?, ?> now)) {
      return json(after);
    }
    Map<Object, @Nullable Object> replaced = new LinkedHashMap<>();
    now.forEach(
        (key, value) -> replaced.put(key, value == null ? null : replacing(was.get(key), value)));
    was.keySet().forEach(key -> replaced.putIfAbsent(key, null));
    return Collections.unmodifiableMap(replaced);
  }

  private static boolean changed(Object before, Object after) {
    if (before instanceof Reading sent && after instanceof Reading now) {
      return now.differsFrom(sent);
    }
    return !Objects.equals(before, after);
  }

  /**
   * A value as JSON can carry it. JSON has no NaN or infinity, and in a merge patch {@code null}
   * means removed, so those numbers go as their names.
   */
  private static Object json(Object value) {
    return switch (value) {
      case Reading reading -> number(reading.value());
      case Double d -> number(d);
      case Float f -> number(f);
      case Map<?, ?> map -> {
        Map<Object, Object> copy = new LinkedHashMap<>();
        map.forEach((k, v) -> copy.put(k, json(v)));
        yield Collections.unmodifiableMap(copy);
      }
      case List<?> list -> list.stream().map(Delta::json).toList();
      default -> value;
    };
  }

  private static Object number(double value) {
    return Double.isFinite(value) ? value : Double.toString(value);
  }
}
