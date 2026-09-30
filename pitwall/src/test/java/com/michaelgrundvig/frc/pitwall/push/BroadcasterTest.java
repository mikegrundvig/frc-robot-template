package com.michaelgrundvig.frc.pitwall.push;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.experiment.ByHand;
import com.michaelgrundvig.frc.pitwall.experiment.Run;
import java.time.Duration;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;

/** The broadcaster and its streams, driven loop by loop, with no web server in between. */
class BroadcasterTest {
  private static final Duration NOW = Duration.ZERO;

  private final List<String> closed = new ArrayList<>();
  private final Broadcaster broadcaster = new Broadcaster(2, 4, closed::add);
  private final AtomicInteger built = new AtomicInteger();
  private final Map<String, Map<String, Object>> state = new LinkedHashMap<>();

  BroadcasterTest() {
    state.put("values", new LinkedHashMap<>(Map.of("kP", 16.0)));
  }

  /** The state as the robot loop would hand it over, counting how often it's built. */
  private Map<String, Map<String, Object>> state() {
    built.incrementAndGet();
    return state;
  }

  private void loop(@Nullable Run run) {
    broadcaster.periodic(this::state, run);
  }

  private static List<String> drain(Stream stream) throws InterruptedException {
    List<String> events = new ArrayList<>();
    for (Event event = stream.next(NOW); event != null; event = stream.next(NOW)) {
      events.add(event.wire());
    }
    return events;
  }

  /** The times of the angle readings an event carries. */
  private static String[] angleTimes(String event) {
    return event.split("\"Angle\":\\{\"t\":\\[")[1].split("]")[0].split(",");
  }

  private Stream open() {
    return broadcaster.open().orElseThrow();
  }

  @Test
  void withNoStreamOpenTheStateIsNeverBuilt() {
    for (int i = 0; i < 10; i++) {
      loop(null);
    }

    assertThat(built).hasValue(0);
    assertThat(broadcaster.watched()).isFalse();
  }

  @Test
  void aNewStreamGetsASnapshotThenOnlyWhatChanged() throws Exception {
    Stream stream = open();
    assertThat(broadcaster.watched()).isTrue();

    loop(null);
    for (int i = 0; i < 500; i++) {
      loop(null);
    }

    assertThat(drain(stream))
        .containsExactly(
            "event: snapshot\ndata: {\"stream\":\"1\",\"document\":{\"values\":{\"kP\":16.0}}}\n\n");

    Objects.requireNonNull(state.get("values")).put("kP", 18.5);
    loop(null);
    loop(null);

    assertThat(drain(stream)).containsExactly("event: patch\ndata: {\"values\":{\"kP\":18.5}}\n\n");
  }

  @Test
  void aStreamThatOpensLaterStartsFromTheStateAsSent() throws Exception {
    Stream first = open();
    loop(null);
    Objects.requireNonNull(state.get("values")).put("kP", 18.5);
    loop(null);

    Stream second = open();
    loop(null);

    assertThat(drain(first)).hasSize(2);
    assertThat(drain(second))
        .containsExactly(
            "event: snapshot\ndata: {\"stream\":\"2\",\"document\":{\"values\":{\"kP\":18.5}}}\n\n");
  }

  @Test
  void aRemovedValueIsNullInThePatch() throws Exception {
    Stream stream = open();
    loop(null);
    drain(stream);

    Objects.requireNonNull(state.get("values")).remove("kP");
    loop(null);

    assertThat(drain(stream)).containsExactly("event: patch\ndata: {\"values\":{\"kP\":null}}\n\n");
  }

  @Test
  void aStreamThatFallsBehindIsClosedAndSaysWhy() throws Exception {
    Stream stream = open();
    for (int i = 0; i < 6; i++) {
      Objects.requireNonNull(state.get("values")).put("kP", 16.0 + i);
      loop(null);
    }

    assertThat(stream.isOpen()).isFalse();
    assertThat(stream.why()).isEqualTo("it fell 4 events behind");
    assertThat(closed).containsExactly("1");
    assertThat(broadcaster.size()).isZero();
    assertThat(stream.next(NOW)).as("nothing more for it").isNull();
  }

  @Test
  void aStreamClosedByItsPageIsForgotten() {
    Stream stream = open();
    loop(null);

    stream.close("the page went away");
    stream.close("again");

    assertThat(broadcaster.size()).isZero();
    assertThat(closed).containsExactly("1");
    assertThat(stream).hasToString("stream 1, closed: the page went away");
  }

  @Test
  void onlySoManyStreamsOpenAndNoneOnceClosed() {
    Stream one = open();
    open();

    assertThat(broadcaster.open()).as("at the limit").isEmpty();

    broadcaster.close();

    assertThat(one.isOpen()).isFalse();
    assertThat(one.why()).isEqualTo("Pitwall stopped");
    assertThat(broadcaster.open()).isEmpty();
  }

  @Test
  void aQuietStreamWaitsItsIdleTimeThenHasNothing() throws Exception {
    Stream stream = open();

    long start = System.nanoTime();
    Event event = stream.next(Duration.ofMillis(30));

    assertThat(event).isNull();
    assertThat(stream.isOpen()).isTrue();
    assertThat(System.nanoTime() - start).isGreaterThanOrEqualTo(Duration.ofMillis(25).toNanos());
  }

  @Test
  void anEventIsTurnedIntoJsonOnceWhoeverWritesIt() {
    AtomicInteger made = new AtomicInteger();
    Map<String, @Nullable Object> data = new HashMap<>();
    data.put("gone", null);
    Event event =
        new Event(
            "patch",
            () -> {
              made.incrementAndGet();
              return data;
            });

    assertThat(event.wire()).isEqualTo("event: patch\ndata: {\"gone\":null}\n\n");
    assertThat(event.wire()).isSameAs(event.wire());
    assertThat(made).hasValue(1);
    assertThat(event).hasToString("patch");
    assertThat(event.name()).isEqualTo("patch");
  }

  /** A run's samples go out every loop, each once; a stream opened mid-run gets them all. */
  @Test
  void aRunsSamplesGoOutAsTheyAreTakenAndASnapshotHasThemAll() throws Exception {
    AtomicLong wallClock = new AtomicLong();
    ByHand byHand = new ByHand(wallClock::get);
    Actions.Gate ready = new Actions.Gate(true, true, false);
    Broadcaster wide = new Broadcaster(2, 1000, id -> {});
    Stream first = wide.open().orElseThrow();
    byHand.experiments().requestStart(ByHand.STEP, null);
    int loops = 30;
    for (int i = 0; i < loops; i++) {
      byHand.experiments().heard(null);
      byHand.experiments().periodic(ready);
      wide.periodic(this::state, byHand.experiments().run());
      byHand.step();
    }
    Stream second = wide.open().orElseThrow();
    wide.periodic(this::state, byHand.experiments().run());

    List<String> samples =
        drain(first).stream().filter(event -> event.startsWith("event: samples")).toList();
    // Each loop's step goes out at the next loop: the last with the second stream's snapshot.
    assertThat(samples).hasSize(loops);
    assertThat(samples.get(0))
        .startsWith("event: samples\ndata: {\"run\":1,\"signals\":{\"Setpoint\":{\"t\":[0.0],");
    Run run = Objects.requireNonNull(byHand.experiments().run());
    int angles = run.trace().series("Angle").count();
    long sentAngles =
        samples.stream()
            .filter(event -> event.contains("\"Angle\""))
            .mapToLong(event -> angleTimes(event).length)
            .sum();
    assertThat(sentAngles).as("every angle reading, once").isEqualTo(angles);
    String snapshot = drain(second).get(0);
    assertThat(snapshot).startsWith("event: snapshot\ndata: {\"stream\":\"2\",");
    assertThat(angleTimes(snapshot)).as("the trace so far").hasSize(angles);
  }
}
