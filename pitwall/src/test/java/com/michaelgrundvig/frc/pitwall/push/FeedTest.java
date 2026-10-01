package com.michaelgrundvig.frc.pitwall.push;

import static org.assertj.core.api.Assertions.assertThat;

import java.time.Duration;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;

/**
 * A part of the state only some pages are sent: it goes to the streams whose page shows its view,
 * after everything of it as sent, and costs nothing while no page shows it.
 */
class FeedTest {
  private final Broadcaster broadcaster = new Broadcaster(4, 16, id -> {});
  private final Map<String, Map<String, Object>> state = new LinkedHashMap<>();
  private final Moving moving = new Moving();

  /** A feed whose one number moves when the test says, counting how often it's asked. */
  private static final class Moving implements Feed {
    int asked;
    int now;
    int sent;

    @Override
    public String view() {
      return "world";
    }

    @Override
    public List<String> at() {
      return List.of("scenes", "world", "state");
    }

    @Override
    public Map<String, @Nullable Object> patch() {
      asked++;
      if (now == sent) {
        return Map.of();
      }
      sent = now;
      return Map.of("x", now);
    }

    @Override
    public Map<String, Object> sent() {
      return Map.of("x", sent);
    }
  }

  FeedTest() {
    state.put("values", new LinkedHashMap<>(Map.of("kP", 16.0)));
  }

  private void loop() {
    broadcaster.periodic(() -> state, null, List.of(moving));
  }

  private static List<String> drain(Stream stream) throws InterruptedException {
    List<String> events = new ArrayList<>();
    for (Event event = stream.next(Duration.ZERO);
        event != null;
        event = stream.next(Duration.ZERO)) {
      events.add(event.wire());
    }
    return events;
  }

  @Test
  void aFeedNoPageShowsIsNeverAsked() throws Exception {
    Stream stream = broadcaster.open(null).orElseThrow();
    for (int i = 0; i < 10; i++) {
      moving.now = i;
      loop();
    }

    assertThat(moving.asked).isZero();
    assertThat(drain(stream)).singleElement().asString().startsWith("event: snapshot");
  }

  @Test
  void aPageShowingTheViewGetsAllOfItThenOnlyWhatChanged() throws Exception {
    moving.now = 3;
    Stream stream = broadcaster.open(null, Set.of("world")).orElseThrow();
    loop();
    loop();

    assertThat(drain(stream))
        .hasSize(2)
        .last()
        .isEqualTo(
            "event: replace\ndata: {\"at\":[\"scenes\",\"world\",\"state\"],\"value\":{\"x\":3}}\n\n");

    moving.now = 4;
    loop();
    assertThat(drain(stream))
        .containsExactly("event: patch\ndata: {\"scenes\":{\"world\":{\"state\":{\"x\":4}}}}\n\n");
  }

  @Test
  void aFeedsChangesRideInTheLoopsOnePatchWithTheRest() throws Exception {
    Stream stream = broadcaster.open(null, Set.of("world")).orElseThrow();
    Stream other = broadcaster.open(null).orElseThrow();
    loop();
    drain(stream);
    drain(other);

    moving.now = 7;
    Objects.requireNonNull(state.get("values")).put("kP", 18.0);
    loop();

    assertThat(drain(stream))
        .containsExactly(
            "event: patch\ndata: {\"values\":{\"kP\":18.0},"
                + "\"scenes\":{\"world\":{\"state\":{\"x\":7}}}}\n\n");
    assertThat(drain(other))
        .as("a page not showing the view is sent the rest only")
        .containsExactly("event: patch\ndata: {\"values\":{\"kP\":18.0}}\n\n");
  }

  @Test
  void aPageThatStartsShowingTheViewIsSentAllOfItAndOneThatStopsIsSentNoMore() throws Exception {
    Stream stream = broadcaster.open(null).orElseThrow();
    loop();
    drain(stream);

    assertThat(broadcaster.show(stream.id(), Set.of("world"))).isTrue();
    moving.now = 5;
    loop();
    assertThat(drain(stream))
        .containsExactly(
            "event: replace\ndata: {\"at\":[\"scenes\",\"world\",\"state\"],\"value\":{\"x\":5}}\n\n");

    broadcaster.show(stream.id(), Set.of());
    moving.now = 6;
    loop();
    assertThat(drain(stream)).isEmpty();
    assertThat(broadcaster.show("no such stream", Set.of("world"))).isFalse();
  }
}
