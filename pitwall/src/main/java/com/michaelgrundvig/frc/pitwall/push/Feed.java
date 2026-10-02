package com.michaelgrundvig.frc.pitwall.push;

import java.util.List;
import java.util.Map;
import org.jspecify.annotations.Nullable;

/**
 * A part of the page's state sent only to the pages that show it, such as a scene's moving parts.
 * Everything else in the state goes to every open page; a feed goes only to a stream whose page
 * names its view ({@code /api/events?views=world}, or later through {@code PUT /api/views}), and
 * costs the robot nothing while no page does.
 *
 * <p>While any page shows its view, the {@link Broadcaster} asks it once per loop what changed
 * ({@link #patch}), and sends that, nested at {@link #at()}, inside the loop's {@code patch} event
 * to each stream showing it. A stream that starts showing it (opening with it named, or naming it
 * later) is sent everything as last sent ({@link #sent()}) in a {@code replace} event first, which
 * the page puts in place whole, so whatever it held there before doesn't linger.
 *
 * <p>Both are called in the robot loop. What they return is serialized later, off the loop, so it
 * must never change after it's returned.
 */
public interface Feed {
  /** The view a page names to be sent this feed: a scene's id, say. */
  String view();

  /** Where it goes in the page's state, such as {@code ["scenes", "world", "state"]}. */
  List<String> at();

  /**
   * What changed since it was last sent, as a JSON merge patch, remembered as sent; empty when
   * nothing did, or it isn't time to send yet. Called once per loop while any page shows the view.
   */
  Map<String, @Nullable Object> patch();

  /** Everything as last sent: where a page that starts showing the view starts from. */
  Map<String, Object> sent();
}
