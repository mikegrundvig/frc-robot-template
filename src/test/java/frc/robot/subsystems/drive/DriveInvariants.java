package frc.robot.subsystems.drive;

import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.SchedulerEvent;

/**
 * What must hold of the drivetrain's moves every loop, whatever its drive team presses and whatever
 * fails: checked after each loop by {@code Monkey}. Every drive move ends with an outcome (arrived,
 * refused, timed out, or canceled by what took the drivetrain) by its deadline, enabled or not:
 * disabled, the robot doesn't move, and its move times out.
 */
public final class DriveInvariants {
  /** The drive moves running now, in the order they started. */
  private final Set<DriveMove> running = new LinkedHashSet<>();

  /** A move that ended without an outcome, said as it ended. */
  private @Nullable String endedWithout;

  /** The checks on the drivetrain's moves, watching its scheduler from now on. */
  public DriveInvariants(Scheduler scheduler) {
    scheduler.addEventListener(this::heard);
  }

  private void heard(SchedulerEvent event) {
    switch (event) {
      case SchedulerEvent.Mounted mounted when mounted.command() instanceof DriveMove move ->
          running.add(move);
      case SchedulerEvent.Completed completed
          when completed.command() instanceof DriveMove move -> {
        running.remove(move);
        if (move.outcome().isEmpty() && endedWithout == null) {
          endedWithout = move.name() + " ended with no outcome";
        }
      }
      case SchedulerEvent.Canceled canceled when canceled.command() instanceof DriveMove move ->
          running.remove(move);
      case SchedulerEvent.CompletedWithError error when error.command() instanceof DriveMove move ->
          running.remove(move);
      default -> {}
    }
  }

  /**
   * Checks the loop just run: the first thing found wrong, led by its check's name ("drive moves
   * end: ..."), or null when everything holds.
   *
   * @param now the loop's time on the robot's clock, in nanoseconds
   */
  public @Nullable String broken(long now) {
    String found = endedWithout;
    endedWithout = null;
    for (DriveMove move : running) {
      long deadline = move.deadlineNanos();
      if (found == null && deadline != Long.MAX_VALUE && now >= deadline) {
        found =
            String.format(
                Locale.ROOT,
                "%s is still running %.2f s past its deadline",
                move.name(),
                (now - deadline) / 1e9);
      }
    }
    return found == null ? null : "drive moves end: " + found;
  }
}
