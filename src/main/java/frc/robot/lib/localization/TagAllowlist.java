package frc.robot.lib.localization;

import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;

/**
 * The tags a task trusts, and no others: while a robot lines up on a target, its cameras measure it
 * only from that target's tags. At an event the tags stand where the field's crew put them,
 * centimeters from the layout's numbers and from each other, and what the robot needs is to be
 * right relative to the target's own tags, not an average with tags across the field. 254 kept such
 * a list per task, and 5000 had a "hub tags only" switch.
 *
 * <p>Name the tags through the field model rather than by number, so the task works on any field
 * and for either alliance: a task lining up on a place takes the place's tags with {@code
 * vision.trustingOnly(hub, task)}, which finds them for the robot's alliance when the task starts
 * (bindings are made before the alliance is known).
 *
 * @param task what the tags are for, as the log names it: "Hub"
 * @param tagIds the tags it trusts: at least one
 */
public record TagAllowlist(String task, Set<Integer> tagIds) {
  public TagAllowlist {
    if (task.isBlank()) {
      throw new IllegalArgumentException("an allowlist names its task");
    }
    if (tagIds.isEmpty()) {
      throw new IllegalArgumentException(task + ": an allowlist trusts at least one tag");
    }
    tagIds = Set.copyOf(tagIds);
  }

  /** The tags a task trusts, in any order. */
  public static TagAllowlist of(String task, List<Integer> tagIds) {
    return new TagAllowlist(task, Set.copyOf(tagIds));
  }

  /** Whether the task trusts a tag. */
  public boolean allows(int tagId) {
    return tagIds.contains(tagId);
  }

  /** For the log: "Hub: 25, 26". */
  public String describe() {
    return task
        + ": "
        + new TreeSet<>(tagIds).stream().map(String::valueOf).collect(Collectors.joining(", "));
  }
}
