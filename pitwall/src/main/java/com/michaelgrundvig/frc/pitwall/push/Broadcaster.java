package com.michaelgrundvig.frc.pitwall.push;

import com.michaelgrundvig.frc.pitwall.experiment.Run;
import com.michaelgrundvig.frc.pitwall.experiment.Series;
import com.michaelgrundvig.frc.pitwall.experiment.Trace;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;

/**
 * Pushes the page's state to every open event stream, only when something changed.
 *
 * <p>Once per loop, and only while a stream is open, the robot loop hands over the state, section
 * by section, and the run in progress. What changed since it was last compared goes to every stream
 * as one {@code patch} event (a JSON merge patch, from {@link Delta}), and the run's new samples as
 * one {@code samples} event. Nothing changed, nothing sent. The state is built and compared at most
 * every {@link #COMPARE_NANOS} on the robot's clock (every loop for a broadcaster without one), as
 * a scene's moving parts are: no page shows more. A loop that must be heard at once, such as the
 * one that pauses the simulator's clock (after which no time passes to make a comparison due), asks
 * for it with {@link #compareNow}; the loop a new run is handed over in is compared at once too, so
 * its section reaches the page before its samples. A stream that has just opened gets a {@code
 * snapshot} first, compared that loop: the whole state as sent, the run's trace so far, and the
 * stream's own id; the patches after it apply to that.
 *
 * <p>Some of the state goes only to the pages that show it: a {@link Feed}, such as a scene's
 * moving parts. A stream names the views its page shows, and gets each feed's changes in its
 * patches only while it does, after a {@code replace} event with everything of the feed as sent.
 *
 * <p>The loop never touches a socket and never waits: it drops events into each stream's mailbox,
 * and each stream's own thread writes them (see {@link Stream}). Events are turned into JSON by the
 * first stream to write them, once, off the loop.
 */
public final class Broadcaster implements AutoCloseable {
  /** How many pages may have a stream open at once. */
  public static final int MAX_STREAMS = 8;

  /** How many events may wait for a stream before it counts as fallen behind: 2.56 s of loops. */
  public static final int MAILBOX = 128;

  /** The least time between two comparisons of the state: at most 20 patches a second. */
  public static final long COMPARE_NANOS = 50_000_000L;

  private final int maxStreams;
  private final int mailbox;
  private final Consumer<String> onClosed;
  private final @Nullable LongSupplier clock;
  private final Delta delta = new Delta();

  /** When the state is next compared, on {@link #clock}. */
  private long compareNanos = Long.MIN_VALUE;

  /** Streams that have had their snapshot, and those still waiting for it. */
  private final List<Stream> streams = new CopyOnWriteArrayList<>();

  private final List<Stream> waiting = new CopyOnWriteArrayList<>();
  private boolean closed;

  /** The run whose samples are being sent, and how many of each signal's have been. */
  private @Nullable Run traced;

  private int[] sent = new int[0];

  /**
   * A broadcaster that compares the state every loop: a test's, that steps it loop by loop.
   *
   * @param maxStreams how many streams may be open at once
   * @param mailbox how many events may wait for a stream before it's closed as fallen behind
   * @param onClosed told each stream's id as it closes, from whichever thread closes it
   */
  public Broadcaster(int maxStreams, int mailbox, Consumer<String> onClosed) {
    this(maxStreams, mailbox, onClosed, null);
  }

  /**
   * @param maxStreams how many streams may be open at once
   * @param mailbox how many events may wait for a stream before it's closed as fallen behind
   * @param onClosed told each stream's id as it closes, from whichever thread closes it
   * @param clock the robot's clock, in nanoseconds, the state compared at most every {@link
   *     #COMPARE_NANOS} on it; null to compare every loop
   */
  public Broadcaster(
      int maxStreams, int mailbox, Consumer<String> onClosed, @Nullable LongSupplier clock) {
    this.maxStreams = maxStreams;
    this.mailbox = mailbox;
    this.onClosed = onClosed;
    this.clock = clock;
  }

  /**
   * Opens a stream, which gets its snapshot at the next loop. Safe from any thread.
   *
   * <p>With {@link #MAX_STREAMS} open already, the one that wrote least lately is closed for it: a
   * page that has gone without its stream noticing (a laptop that slept, a dropped link) holds a
   * place until its next write fails, and a page that's here shouldn't wait for that.
   *
   * @param replaces the stream a page reconnecting had, closed now rather than when a write to it
   *     fails; null for none
   * @return empty when the broadcaster has closed
   */
  public synchronized Optional<Stream> open(@Nullable String replaces) {
    return open(replaces, Set.of());
  }

  /**
   * Opens a stream whose page shows these views, which decide the {@link Feed}s it's sent. Safe
   * from any thread.
   *
   * @param replaces the stream a page reconnecting had; null for none
   * @param views the views its page shows: a scene's id, say
   * @return empty when the broadcaster has closed
   */
  public synchronized Optional<Stream> open(@Nullable String replaces, Set<String> views) {
    if (closed) {
      return Optional.empty();
    }
    for (Stream stream : all()) {
      if (stream.id().equals(replaces)) {
        stream.close("its page reconnected");
      }
    }
    if (streams.size() + waiting.size() >= maxStreams) {
      all().stream()
          .min(Comparator.comparingLong(Stream::wroteNanos))
          .ifPresent(stalest -> stalest.close("another page needed its place"));
    }
    // Random, so no other page can guess it: a stream's id is what lets a request speak for its
    // page (keep a run going, drive a joystick, close the stream it replaces).
    Stream stream = new Stream(UUID.randomUUID().toString(), mailbox, this::closed);
    stream.show(views);
    waiting.add(stream);
    return Optional.of(stream);
  }

  /**
   * Says which views a stream's page shows now. Safe from any thread.
   *
   * @param stream the stream's id, as its snapshot named it
   * @return whether that stream is open
   */
  public boolean show(String stream, Set<String> views) {
    for (Stream open : all()) {
      if (open.id().equals(stream)) {
        open.show(views);
        return true;
      }
    }
    return false;
  }

  /** Whether any stream is open, so the loop has anyone to tell. Safe from any thread. */
  public boolean watched() {
    return !streams.isEmpty() || !waiting.isEmpty();
  }

  /** How many streams are open. Safe from any thread. */
  public int size() {
    return streams.size() + waiting.size();
  }

  /**
   * Compares the state at the next {@link #periodic}, however soon after the last comparison: for a
   * loop every page must hear of, such as one run while the simulator's clock is paused, which
   * stands still, so no comparison would come due. Call it from the robot loop.
   */
  public void compareNow() {
    compareNanos = Long.MIN_VALUE;
  }

  /**
   * Sends what changed to every stream, and a snapshot to each new one. Call it once per loop, from
   * the robot loop. With no stream open it does nothing, not even build the state.
   *
   * @param state the state now, section by section: see {@link Delta#patch}
   * @param run the run in progress, or the latest to end; null for none
   */
  public void periodic(Supplier<Map<String, Map<String, Object>>> state, @Nullable Run run) {
    periodic(state, run, List.of());
  }

  /**
   * Sends what changed to every stream, a snapshot to each new one, and each {@link Feed} to the
   * streams whose page shows its view. Call it once per loop, from the robot loop. With no stream
   * open it does nothing; a feed no page shows isn't asked anything.
   *
   * @param state the state now, section by section: see {@link Delta#patch}
   * @param run the run in progress, or the latest to end; null for none
   * @param feeds the parts of the state only some pages are sent
   */
  public void periodic(
      Supplier<Map<String, Map<String, Object>>> state,
      @Nullable Run run,
      List<? extends Feed> feeds) {
    if (!watched()) {
      return;
    }
    if (run != traced) {
      // A new run's section goes out this loop, ahead of its first samples: the page keeps samples
      // only for a run it knows, and drops those that come first.
      compareNow();
    }
    Map<String, @Nullable Object> patch = compareIfDue(state);
    // Each feed some page shows says what changed; one no page shows isn't asked.
    Set<String> shown = new HashSet<>();
    for (Stream stream : all()) {
      shown.addAll(stream.views());
    }
    Map<Feed, Map<String, @Nullable Object>> fed = new LinkedHashMap<>();
    for (Feed feed : feeds) {
      if (shown.contains(feed.view())) {
        Map<String, @Nullable Object> changed = feed.patch();
        if (!changed.isEmpty()) {
          fed.put(feed, changed);
        }
      }
    }
    // One patch event per stream, holding every change it's sent this loop; streams sent the same
    // feeds share it, so it's serialized once.
    Map<List<Feed>, Event> patches = new HashMap<>();
    for (Stream stream : streams) {
      Set<String> views = stream.views();
      List<Feed> carried = new ArrayList<>();
      for (Feed feed : fed.keySet()) {
        if (stream.following.contains(feed.view()) && views.contains(feed.view())) {
          carried.add(feed);
        }
      }
      if (!patch.isEmpty() || !carried.isEmpty()) {
        stream.offer(
            patches.computeIfAbsent(
                carried,
                feedsCarried -> new Event("patch", () -> withFeeds(patch, feedsCarried, fed))));
      }
      catchUp(stream, views, feeds);
    }
    Event samples = samples(run);
    if (samples != null) {
      send(samples);
    }
    if (!waiting.isEmpty()) {
      Map<String, Object> document = delta.document();
      Run tracing = traced;
      int[] upTo = sent.clone();
      for (Stream stream : waiting) {
        stream.offer(new Event("snapshot", () -> snapshot(stream.id(), document, tracing, upTo)));
        catchUp(stream, stream.views(), feeds);
        promote(stream);
      }
    }
  }

  /**
   * What changed in the state since it was last compared, if it's time to compare it again, or a
   * stream is waiting for its snapshot; else nothing, without building the state.
   */
  private Map<String, @Nullable Object> compareIfDue(
      Supplier<Map<String, Map<String, Object>>> state) {
    LongSupplier time = clock;
    if (time == null) {
      return delta.patch(state.get());
    }
    long now = time.getAsLong();
    if (now < compareNanos && waiting.isEmpty()) {
      return Map.of();
    }
    compareNanos = now + COMPARE_NANOS;
    return delta.patch(state.get());
  }

  /**
   * Sends a stream everything of each feed its page has started showing, and forgets those it no
   * longer shows, so showing one again starts over.
   */
  private static void catchUp(Stream stream, Set<String> views, List<? extends Feed> feeds) {
    Set<String> following = new HashSet<>();
    for (Feed feed : feeds) {
      if (!views.contains(feed.view())) {
        continue;
      }
      following.add(feed.view());
      if (!stream.following.contains(feed.view())) {
        Map<String, Object> whole = feed.sent();
        List<String> at = List.copyOf(feed.at());
        Map<String, Object> replace = new LinkedHashMap<>();
        replace.put("at", at);
        replace.put("value", whole);
        stream.offer(new Event("replace", () -> replace));
      }
    }
    stream.following = Set.copyOf(following);
  }

  /** The loop's patch, with each feed's changes nested where it goes in the page's state. */
  private static Map<String, @Nullable Object> withFeeds(
      Map<String, @Nullable Object> patch,
      List<Feed> carried,
      Map<Feed, Map<String, @Nullable Object>> fed) {
    Map<String, @Nullable Object> all = new LinkedHashMap<>(patch);
    for (Feed feed : carried) {
      nest(all, feed.at(), Objects.requireNonNull(fed.get(feed)));
    }
    return all;
  }

  /** Puts a value at a path in a patch, copying each map on the way so nothing shared changes. */
  private static void nest(Map<String, @Nullable Object> into, List<String> at, Object value) {
    Map<String, @Nullable Object> map = into;
    for (int i = 0; i < at.size() - 1; i++) {
      Map<String, @Nullable Object> next = new LinkedHashMap<>();
      if (map.get(at.get(i)) instanceof Map<?, ?> existing) {
        existing.forEach((key, each) -> next.put(String.valueOf(key), each));
      }
      map.put(at.get(i), next);
      map = next;
    }
    map.put(at.getLast(), value);
  }

  /** Closes every stream; none opens after. */
  @Override
  public void close() {
    List<Stream> all;
    synchronized (this) {
      closed = true;
      all = all();
    }
    all.forEach(stream -> stream.close("Pitwall stopped"));
  }

  private List<Stream> all() {
    List<Stream> all = new ArrayList<>(streams);
    all.addAll(waiting);
    return all;
  }

  private synchronized void promote(Stream stream) {
    waiting.remove(stream);
    if (stream.isOpen()) {
      streams.add(stream);
    }
  }

  private void closed(Stream stream) {
    synchronized (this) {
      streams.remove(stream);
      waiting.remove(stream);
    }
    onClosed.accept(stream.id());
  }

  private void send(Event event) {
    for (Stream stream : streams) {
      stream.offer(event);
    }
  }

  /** The run's samples since the last loop, as one event; null when there are none. */
  private @Nullable Event samples(@Nullable Run run) {
    if (run != traced) {
      traced = run;
      sent = new int[run == null ? 0 : run.trace().signals().size()];
    }
    if (run == null) {
      return null;
    }
    List<Trace.Signal> signals = run.trace().signals();
    int[] from = sent;
    int[] to = new int[signals.size()];
    boolean any = false;
    for (int i = 0; i < to.length; i++) {
      to[i] = run.trace().series(signals.get(i).name()).count();
      any |= to[i] > from[i];
    }
    sent = to;
    return any ? new Event("samples", () -> trace(run, from, to)) : null;
  }

  private static Object snapshot(
      String stream, Map<String, Object> document, @Nullable Run run, int[] upTo) {
    Map<String, Object> snapshot = new LinkedHashMap<>();
    snapshot.put("stream", stream);
    snapshot.put("document", document);
    if (run != null) {
      snapshot.put("trace", trace(run, new int[upTo.length], upTo));
    }
    return snapshot;
  }

  /**
   * A run's whole trace so far, as a snapshot carries it: {@code {"run", "signals": {name: {"t",
   * "v"}}}}, each sample's time in seconds from the run's start. Safe from any thread.
   */
  public static Map<String, Object> trace(Run run) {
    List<Trace.Signal> signals = run.trace().signals();
    int[] counts = new int[signals.size()];
    for (int i = 0; i < counts.length; i++) {
      counts[i] = run.trace().series(signals.get(i).name()).count();
    }
    return trace(run, new int[counts.length], counts);
  }

  /**
   * A run's samples from {@code from} up to {@code to}, signal by signal. What's counted in a
   * series never changes, so this reads it from any thread.
   */
  private static Map<String, Object> trace(Run run, int[] from, int[] to) {
    Map<String, Object> signals = new LinkedHashMap<>();
    List<Trace.Signal> all = run.trace().signals();
    long start = run.startNanos();
    for (int i = 0; i < all.size() && i < to.length; i++) {
      Series series = run.trace().series(all.get(i).name());
      int n = to[i] - from[i];
      if (n <= 0) {
        continue;
      }
      List<Double> times = new ArrayList<>(n);
      List<Double> values = new ArrayList<>(n);
      for (int k = from[i]; k < to[i]; k++) {
        times.add((series.time(k) - start) / 1e9);
        values.add(series.value(k));
      }
      Map<String, Object> samples = new LinkedHashMap<>();
      samples.put("t", times);
      samples.put("v", values);
      signals.put(all.get(i).name(), samples);
    }
    Map<String, Object> trace = new LinkedHashMap<>();
    trace.put("run", run.id());
    trace.put("signals", signals);
    return trace;
  }
}
