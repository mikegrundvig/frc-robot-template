package frc.robot.lib.timing;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.jspecify.annotations.Nullable;

/**
 * Times each robot loop, section by section, and keeps count of the recent loops that overran their
 * period.
 *
 * <p>Each loop: {@link #start} when it begins, {@link #mark} as each section ends (a section is
 * timed from the previous mark, or from the start), and {@link #finish} when it's done, which
 * reports the loop. A mark without a start times from the previous loop's last mark, so a loop run
 * without its start (a test calling one part of it) still reports something sensible.
 */
public final class LoopMonitor {
  private final long periodNanos;
  private final long[] recentLoops;
  private final List<String> names = new ArrayList<>();
  private final List<Long> durations = new ArrayList<>();
  private int next;
  private int filled;
  private long loopStart = Long.MIN_VALUE;
  private long lastMark = Long.MIN_VALUE;

  /** One section of a loop, and how long it took. */
  public record Section(String name, double seconds) {}

  /**
   * One loop, and the loops before it.
   *
   * @param seconds how long this loop took
   * @param overran whether it took longer than the period
   * @param sections each section, in the order they ran
   * @param overrunsLately how many of the recent loops overran, this one included
   * @param worstLatelySeconds the longest of the recent loops
   */
  public record Report(
      double seconds,
      boolean overran,
      List<Section> sections,
      int overrunsLately,
      double worstLatelySeconds) {
    /** The section that took longest this loop; null if the loop had none. */
    public @Nullable Section slowest() {
      Section slowest = null;
      for (Section section : sections) {
        if (slowest == null || section.seconds() > slowest.seconds()) {
          slowest = section;
        }
      }
      return slowest;
    }

    /** One line for the pit: how long loops take, and whether they keep up. */
    public String describe() {
      if (overrunsLately == 0) {
        return String.format(
            Locale.ROOT,
            "%.1f ms, worst %.1f ms lately",
            seconds * 1000,
            worstLatelySeconds * 1000);
      }
      return String.format(
          Locale.ROOT,
          "Overran %d %s lately, worst %.1f ms",
          overrunsLately,
          overrunsLately == 1 ? "time" : "times",
          worstLatelySeconds * 1000);
    }
  }

  /**
   * @param periodSeconds the loop's period: a loop that takes longer overran
   * @param recentLoops how many loops "lately" covers: 500 is ten seconds at 50 Hz
   */
  public LoopMonitor(double periodSeconds, int recentLoops) {
    if (!(periodSeconds > 0) || Double.isInfinite(periodSeconds)) {
      throw new IllegalArgumentException("a loop's period: " + periodSeconds);
    }
    if (recentLoops < 1) {
      throw new IllegalArgumentException("at least one recent loop: " + recentLoops);
    }
    periodNanos = Math.round(periodSeconds * 1e9);
    this.recentLoops = new long[recentLoops];
  }

  /** A loop begins. */
  public void start(long nowNanos) {
    loopStart = nowNanos;
    lastMark = nowNanos;
    names.clear();
    durations.clear();
  }

  /** A section of the loop just ended. */
  public void mark(String section, long nowNanos) {
    if (lastMark == Long.MIN_VALUE) {
      lastMark = nowNanos;
      loopStart = nowNanos;
    }
    if (loopStart == Long.MIN_VALUE) {
      loopStart = lastMark;
    }
    names.add(section);
    durations.add(Math.max(0, nowNanos - lastMark));
    lastMark = nowNanos;
  }

  /** The loop is done: how it went, and how the recent loops went. */
  public Report finish(long nowNanos) {
    long start = loopStart == Long.MIN_VALUE ? nowNanos : loopStart;
    long took = Math.max(0, nowNanos - start);
    recentLoops[next] = took;
    next = (next + 1) % recentLoops.length;
    filled = Math.min(filled + 1, recentLoops.length);
    int overruns = 0;
    long worst = 0;
    for (int i = 0; i < filled; i++) {
      overruns += recentLoops[i] > periodNanos ? 1 : 0;
      worst = Math.max(worst, recentLoops[i]);
    }
    List<Section> sections = new ArrayList<>(names.size());
    for (int i = 0; i < names.size(); i++) {
      sections.add(new Section(names.get(i), durations.get(i) / 1e9));
    }
    names.clear();
    durations.clear();
    loopStart = Long.MIN_VALUE;
    lastMark = nowNanos;
    return new Report(took / 1e9, took > periodNanos, List.copyOf(sections), overruns, worst / 1e9);
  }
}
