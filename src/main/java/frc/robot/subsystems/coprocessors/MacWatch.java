package frc.robot.subsystems.coprocessors;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 * Tells two boards answering at one address from one board replaced by another, by the MAC each
 * answer carries. Two boards at one address take turns answering, as the robot's address table
 * flips between them: a MAC answers, another does, then the first again. A replacement answers
 * after the board it replaced and never gives way to it again. So an address is duplicated when,
 * within the window, a MAC answers again after another had answered.
 */
final class MacWatch {
  private record Answer(long nanos, String mac) {}

  private final long windowNanos;
  private final ArrayDeque<Answer> answers = new ArrayDeque<>();

  /**
   * @param windowSeconds how far back to look for answers taking turns
   */
  MacWatch(double windowSeconds) {
    windowNanos = Math.round(windowSeconds * 1e9);
  }

  /** An answer, from the board with this MAC, at this time on any steady clock. */
  void heard(long nanos, String mac) {
    if (mac.isEmpty()) {
      return;
    }
    answers.addLast(new Answer(nanos, mac));
    while (!answers.isEmpty() && answers.getFirst().nanos() < nanos - windowNanos) {
      answers.removeFirst();
    }
  }

  /**
   * The MACs taking turns at the address within the window, in the order they first answered; empty
   * while one board answers, or one replaced another.
   */
  List<String> takingTurns() {
    Set<String> ended = new HashSet<>();
    boolean tookTurns = false;
    String last = null;
    for (Answer answer : answers) {
      if (last != null && !last.equals(answer.mac())) {
        ended.add(last);
        tookTurns |= ended.contains(answer.mac());
      }
      last = answer.mac();
    }
    if (!tookTurns) {
      return List.of();
    }
    // Every MAC that answered in the window: the turns involve them all.
    Set<String> all = new LinkedHashSet<>();
    for (Answer answer : answers) {
      all.add(answer.mac());
    }
    return new ArrayList<>(all);
  }
}
