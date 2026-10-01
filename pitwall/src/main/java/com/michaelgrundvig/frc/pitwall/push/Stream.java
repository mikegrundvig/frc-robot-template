package com.michaelgrundvig.frc.pitwall.push;

import java.time.Duration;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import org.jspecify.annotations.Nullable;

/**
 * One page's event stream, as the {@link Broadcaster} sees it: a bounded mailbox the robot loop
 * drops events into, and the stream's own thread takes them from and writes.
 *
 * <p>The loop never waits on a stream. A stream that falls so far behind that its mailbox fills (a
 * page on a Wi-Fi link that has stopped moving) is closed; the page reconnects and starts again
 * from a fresh snapshot. That keeps each stream's memory bounded without the loop ever deciding
 * which of a slow page's events matter. Closing a stream interrupts its thread, which frees it even
 * from a write stuck on a link that has stopped moving.
 */
public final class Stream {
  /** Stands in the mailbox for "closed", so a thread waiting on it wakes. */
  private static final Event CLOSED = new Event("closed", () -> "");

  private final String id;
  private final int capacity;
  private final ArrayBlockingQueue<Event> mailbox;
  private final Consumer<Stream> onClose;
  private volatile boolean open = true;
  private volatile String why = "";
  private volatile @Nullable Thread writer;

  /** When it last wrote, or opened if it hasn't: the stream to give up first is the stalest. */
  private volatile long wroteNanos = System.nanoTime();

  /** The views its page shows, which decide the {@link Feed}s it's sent. */
  private volatile Set<String> views = Set.of();

  /** The views it has been sent everything of, and since follows: the robot loop's alone. */
  Set<String> following = Set.of();

  /**
   * @param capacity how many events may wait before the stream counts as fallen behind
   * @param onClose told once, by whichever thread closes it
   */
  Stream(String id, int capacity, Consumer<Stream> onClose) {
    this.id = id;
    this.capacity = capacity;
    // One place more than the capacity, so "closed" always fits.
    mailbox = new ArrayBlockingQueue<>(capacity + 1);
    this.onClose = onClose;
  }

  /** Names this stream: the page sends it with each request, so the deadman knows who asks. */
  public String id() {
    return id;
  }

  /** Whether it's still open. Safe from any thread. */
  public boolean isOpen() {
    return open;
  }

  /** Why it closed; empty while open. */
  public String why() {
    return why;
  }

  /** The views its page shows: a {@link Feed} goes only to the streams that name its view. */
  public Set<String> views() {
    return views;
  }

  /**
   * Says which views its page shows now, from any thread. The next loop sends it everything of a
   * view it newly shows, and stops sending a view it no longer does.
   */
  public void show(Set<String> shown) {
    views = Set.copyOf(shown);
  }

  /** The stream's own thread takes it: closing the stream interrupts this thread. */
  public void writtenBy(Thread thread) {
    writer = thread;
    if (!open) {
      thread.interrupt();
    }
  }

  /** An event, or a heartbeat, went out. */
  public void wrote() {
    wroteNanos = System.nanoTime();
  }

  /** When it last wrote, on {@link System#nanoTime()}'s clock; when it opened, if it hasn't. */
  long wroteNanos() {
    return wroteNanos;
  }

  /**
   * The next event, waiting up to {@code idle} for one. Call it only from the stream's own thread.
   *
   * @return the event, or null when none came in time (send a heartbeat) or the stream closed
   *     (check {@link #isOpen()})
   * @throws InterruptedException if the thread is interrupted, as when the server stops
   */
  public @Nullable Event next(Duration idle) throws InterruptedException {
    if (!open) {
      return null;
    }
    Event event = mailbox.poll(idle.toNanos(), TimeUnit.NANOSECONDS);
    return event == CLOSED ? null : event;
  }

  /**
   * Closes it, if it isn't already, and says so. Safe from any thread.
   *
   * @param why what closed it, such as "the page went away"
   */
  public void close(String why) {
    synchronized (this) {
      if (!open) {
        return;
      }
      this.why = why;
      open = false;
    }
    mailbox.clear();
    mailbox.offer(CLOSED);
    // Its thread may be stuck writing to a page that has stopped answering: interrupting it
    // closes the connection under the write, and frees it.
    Thread thread = writer;
    if (thread != null && thread != Thread.currentThread()) {
      thread.interrupt();
    }
    onClose.accept(this);
  }

  /**
   * Drops an event in the mailbox, from the robot loop; a full mailbox closes the stream.
   *
   * @return whether it went in
   */
  boolean offer(Event event) {
    if (!open) {
      return false;
    }
    if (mailbox.size() >= capacity) {
      close("it fell " + capacity + " events behind");
      return false;
    }
    return mailbox.offer(event);
  }

  @Override
  public String toString() {
    return "stream " + id + (open ? "" : ", closed: " + why);
  }
}
