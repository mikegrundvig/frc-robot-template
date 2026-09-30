package com.michaelgrundvig.frc.pitwall.push;

import com.michaelgrundvig.frc.pitwall.experiment.Run;
import com.michaelgrundvig.frc.pitwall.experiment.Series;
import com.michaelgrundvig.frc.pitwall.experiment.Trace;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;

/**
 * Pushes the page's state to every open event stream, only when something changed.
 *
 * <p>Once per loop, and only while a stream is open, the robot loop hands over the state, section
 * by section, and the run in progress. What changed since the last loop goes to every stream as one
 * {@code patch} event (a JSON merge patch, from {@link Delta}), and the run's new samples as one
 * {@code samples} event. Nothing changed, nothing sent. A stream that has just opened gets a {@code
 * snapshot} first: the whole state as sent, the run's trace so far, and the stream's own id; the
 * patches after it apply to that.
 *
 * <p>The loop never touches a socket and never waits: it drops events into each stream's mailbox,
 * and each stream's own thread writes them (see {@link Stream}). Events are turned into JSON by the
 * first stream to write them, once, off the loop.
 */
public final class Broadcaster implements AutoCloseable {
  /** How many pages may have a stream open at once. */
  public static final int MAX_STREAMS = 8;

  /** How many events may wait for a stream before it counts as fallen behind: 2.5 s of loops. */
  public static final int MAILBOX = 128;

  private final int maxStreams;
  private final int mailbox;
  private final Consumer<String> onClosed;
  private final Delta delta = new Delta();

  /** Streams that have had their snapshot, and those still waiting for it. */
  private final List<Stream> streams = new CopyOnWriteArrayList<>();

  private final List<Stream> waiting = new CopyOnWriteArrayList<>();
  private int opened;
  private boolean closed;

  /** The run whose samples are being sent, and how many of each signal's have been. */
  private @Nullable Run traced;

  private int[] sent = new int[0];

  /**
   * @param maxStreams how many streams may be open at once
   * @param mailbox how many events may wait for a stream before it's closed as fallen behind
   * @param onClosed told each stream's id as it closes, from whichever thread closes it
   */
  public Broadcaster(int maxStreams, int mailbox, Consumer<String> onClosed) {
    this.maxStreams = maxStreams;
    this.mailbox = mailbox;
    this.onClosed = onClosed;
  }

  /**
   * Opens a stream, which gets its snapshot at the next loop. Safe from any thread.
   *
   * @return empty when {@link #MAX_STREAMS} are open already, or the broadcaster has closed
   */
  public synchronized Optional<Stream> open() {
    if (closed || streams.size() + waiting.size() >= maxStreams) {
      return Optional.empty();
    }
    opened++;
    Stream stream = new Stream(Integer.toString(opened), mailbox, this::closed);
    waiting.add(stream);
    return Optional.of(stream);
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
   * Sends what changed to every stream, and a snapshot to each new one. Call it once per loop, from
   * the robot loop. With no stream open it does nothing, not even build the state.
   *
   * @param state the state now, section by section: see {@link Delta#patch}
   * @param run the run in progress, or the latest to end; null for none
   */
  public void periodic(Supplier<Map<String, Map<String, Object>>> state, @Nullable Run run) {
    if (!watched()) {
      return;
    }
    Map<String, @Nullable Object> patch = delta.patch(state.get());
    if (!patch.isEmpty()) {
      send(new Event("patch", () -> patch));
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
        promote(stream);
      }
    }
  }

  /** Closes every stream; none opens after. */
  @Override
  public void close() {
    List<Stream> all;
    synchronized (this) {
      closed = true;
      all = new ArrayList<>(streams);
      all.addAll(waiting);
    }
    all.forEach(stream -> stream.close("Pitwall stopped"));
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
