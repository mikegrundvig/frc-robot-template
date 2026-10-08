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
import java.util.UUID;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
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
    return broadcaster.open(null).orElseThrow();
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
            "event: snapshot\ndata: {\"stream\":\""
                + stream.id()
                + "\",\"document\":{\"values\":{\"kP\":16.0}}}\n\n");

    Objects.requireNonNull(state.get("values")).put("kP", 18.5);
    loop(null);
    loop(null);

    assertThat(drain(stream)).containsExactly("event: patch\ndata: {\"values\":{\"kP\":18.5}}\n\n");
  }

  @Test
  void onTheRobotsClockTheStateIsComparedAtMostTwentyTimesASecond() throws Exception {
    AtomicLong clock = new AtomicLong();
    Broadcaster timed = new Broadcaster(2, 1000, id -> {}, clock::get);
    Stream first = timed.open(null).orElseThrow();
    timed.periodic(this::state, null);
    assertThat(built).as("a new stream's snapshot is compared at once").hasValue(1);

    Objects.requireNonNull(state.get("values")).put("kP", 18.5);
    for (int i = 0; i < 50; i++) {
      clock.addAndGet(Duration.ofMillis(20).toNanos());
      timed.periodic(this::state, null);
    }
    assertThat(built.get()).as("built in a second of loops").isBetween(16, 21);
    assertThat(drain(first)).contains("event: patch\ndata: {\"values\":{\"kP\":18.5}}\n\n");

    int before = built.get();
    Stream second = timed.open(null).orElseThrow();
    clock.addAndGet(Duration.ofMillis(20).toNanos());
    timed.periodic(this::state, null);
    assertThat(built).as("a waiting stream doesn't wait for its turn").hasValue(before + 1);
    assertThat(drain(second)).hasSize(1).first().asString().contains("\"kP\":18.5");
  }

  /**
   * The loop that pauses the simulator's clock is the last before the clock stands still, so no
   * comparison would come due after it: asked to, it's compared at once, however soon after the
   * last, and so is each step after it.
   */
  @Test
  void aLoopAskedToBeComparedNowIsHoweverSoonAfterTheLast() throws Exception {
    AtomicLong clock = new AtomicLong();
    Broadcaster timed = new Broadcaster(2, 1000, id -> {}, clock::get);
    state.put("simulator", new LinkedHashMap<>(Map.of("paused", false)));
    Stream stream = timed.open(null).orElseThrow();
    timed.periodic(this::state, null);
    drain(stream);
    Map<String, Object> simulator = Objects.requireNonNull(state.get("simulator"));

    // The loop after a comparison pauses the clock: not due on it, and never due after.
    clock.addAndGet(Duration.ofMillis(20).toNanos());
    simulator.put("paused", true);
    timed.periodic(this::state, null);
    assertThat(drain(stream)).as("not due, and not asked").isEmpty();

    timed.compareNow();
    timed.periodic(this::state, null);
    assertThat(drain(stream))
        .containsExactly("event: patch\ndata: {\"simulator\":{\"paused\":true}}\n\n");

    // Each step moves the paused clock on one loop, and each is heard.
    for (int step = 1; step <= 3; step++) {
      clock.addAndGet(Duration.ofMillis(20).toNanos());
      simulator.put("step", step);
      timed.compareNow();
      timed.periodic(this::state, null);
      assertThat(drain(stream))
          .as("step " + step)
          .containsExactly("event: patch\ndata: {\"simulator\":{\"step\":" + step + "}}\n\n");
    }
  }

  /**
   * A new run's section reaches the page in the loop that hands the run over, however soon after
   * the last comparison, and so before any of its samples: the page keeps samples only for a run it
   * knows, and would lose the step's start.
   */
  @Test
  void aNewRunIsComparedAtOnceSoThePageKnowsItBeforeItsSamples() throws Exception {
    AtomicLong clock = new AtomicLong();
    ByHand byHand = new ByHand(new AtomicLong()::get);
    Actions.Gate ready = new Actions.Gate(true, true, false);
    Broadcaster timed = new Broadcaster(2, 1000, id -> {}, clock::get);
    Stream stream = timed.open(null).orElseThrow();
    timed.periodic(this::state, null);
    drain(stream);

    byHand.experiments().requestStart(ByHand.STEP, null);
    List<String> events = new ArrayList<>();
    for (int i = 0; i < 5; i++) {
      clock.addAndGet(Duration.ofMillis(20).toNanos());
      byHand.experiments().heard(null);
      byHand.experiments().periodic(ready);
      Run run = byHand.experiments().run();
      state.put("run", run == null ? Map.of() : Map.of("id", run.id()));
      timed.periodic(this::state, run);
      byHand.step();
      events.addAll(drain(stream));
    }

    assertThat(events).filteredOn(event -> event.startsWith("event: samples")).isNotEmpty();
    assertThat(events.getFirst())
        .as("the run's section, in the loop 20 ms after a comparison that handed it over")
        .isEqualTo("event: patch\ndata: {\"run\":{\"id\":1}}\n\n");
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
            "event: snapshot\ndata: {\"stream\":\""
                + second.id()
                + "\",\"document\":{\"values\":{\"kP\":18.5}}}\n\n");
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
    assertThat(closed).containsExactly(stream.id());
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
    assertThat(closed).containsExactly(stream.id());
    assertThat(stream).hasToString("stream " + stream.id() + ", closed: the page went away");
  }

  /**
   * At the limit, a page that's here takes the place of the one that wrote least lately: a page
   * that has gone without its stream noticing (a laptop asleep) holds its place no longer.
   */
  @Test
  void atTheLimitTheStalestStreamMakesWayForANewOne() throws Exception {
    Stream stale = open();
    Stream fresh = open();
    Thread.sleep(2); // the clocks need to tell them apart
    fresh.wrote();

    Stream third = open();

    assertThat(stale.isOpen()).isFalse();
    assertThat(stale.why()).isEqualTo("another page needed its place");
    assertThat(fresh.isOpen()).isTrue();
    assertThat(third.isOpen()).isTrue();
    assertThat(broadcaster.size()).isEqualTo(2);
    assertThat(closed).containsExactly(stale.id());
  }

  @Test
  void aPageThatReconnectsReplacesTheStreamItHad() {
    Stream before = open();
    Stream other = open();

    Stream after = broadcaster.open(before.id()).orElseThrow();

    assertThat(before.why()).isEqualTo("its page reconnected");
    assertThat(other.isOpen()).isTrue();
    assertThat(after.id()).isNotIn(before.id(), other.id());
  }

  /** A stream's id lets a request speak for its page, so no other page may guess it: random. */
  @Test
  void aStreamsIdIsRandom() {
    Stream one = open();
    Stream two = open();

    assertThat(UUID.fromString(one.id())).isNotEqualTo(UUID.fromString(two.id()));
  }

  @Test
  void noStreamOpensOnceItsClosed() {
    Stream one = open();

    broadcaster.close();

    assertThat(one.isOpen()).isFalse();
    assertThat(one.why()).isEqualTo("Pitwall stopped");
    assertThat(broadcaster.open(null)).isEmpty();
  }

  /**
   * A stream that falls behind is usually one whose thread is stuck writing to a page that has
   * stopped answering. Closing it interrupts that thread, which frees it.
   */
  @Test
  void closingAStreamFreesItsThreadEvenFromAStuckWrite() throws Exception {
    Stream stream = open();
    CountDownLatch freed = new CountDownLatch(1);
    Thread stuck =
        Thread.ofVirtual()
            .start(
                () -> {
                  stream.writtenBy(Thread.currentThread());
                  try {
                    Thread.sleep(Duration.ofMinutes(1)); // stands in for a write that never ends
                  } catch (InterruptedException interrupted) {
                    freed.countDown();
                  }
                });
    for (int i = 0; i < 6; i++) {
      Objects.requireNonNull(state.get("values")).put("kP", 16.0 + i);
      loop(null);
    }

    assertThat(freed.await(5, TimeUnit.SECONDS)).as("the stuck thread was freed").isTrue();
    assertThat(stream.why()).isEqualTo("it fell 4 events behind");
    stuck.join();
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
    Stream first = wide.open(null).orElseThrow();
    byHand.experiments().requestStart(ByHand.STEP, null);
    int loops = 30;
    for (int i = 0; i < loops; i++) {
      byHand.experiments().heard(null);
      byHand.experiments().periodic(ready);
      wide.periodic(this::state, byHand.experiments().run());
      byHand.step();
    }
    Stream second = wide.open(null).orElseThrow();
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
    assertThat(snapshot).startsWith("event: snapshot\ndata: {\"stream\":\"" + second.id() + "\",");
    assertThat(angleTimes(snapshot)).as("the trace so far").hasSize(angles);
  }
}
