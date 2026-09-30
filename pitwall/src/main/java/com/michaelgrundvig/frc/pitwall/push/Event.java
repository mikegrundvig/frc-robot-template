package com.michaelgrundvig.frc.pitwall.push;

import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;

/**
 * One server-sent event: a name ({@code snapshot}, {@code patch}, {@code samples}) and its data as
 * JSON. The data is serialized once, by whichever stream writes it first, so the robot loop that
 * made the event never pays for it and every stream shares the text.
 */
public final class Event {
  /** Nulls included: in a merge patch, null is how a value is removed. */
  private static final JsonType<Object> JSON =
      Jsonb.builder().serializeNulls(true).build().type(Object.class);

  private final String name;
  private final Supplier<Object> data;
  private @Nullable String wire;

  /**
   * @param data makes the data, as maps, lists, strings, booleans, and numbers; called once, off
   *     the robot loop, so it must only read what can't change
   */
  Event(String name, Supplier<Object> data) {
    this.name = name;
    this.data = data;
  }

  /** What kind of event it is: the page listens for each by name. */
  public String name() {
    return name;
  }

  /** The event as it goes on the wire: {@code event: patch}, its data on one line, a blank line. */
  public synchronized String wire() {
    String text = wire;
    if (text == null) {
      text = "event: " + name + "\ndata: " + JSON.toJson(data.get()) + "\n\n";
      wire = text;
    }
    return text;
  }

  @Override
  public String toString() {
    return name;
  }
}
