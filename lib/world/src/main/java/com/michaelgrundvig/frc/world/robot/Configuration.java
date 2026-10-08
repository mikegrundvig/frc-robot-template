package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.jspecify.annotations.Nullable;

/**
 * Several joints, each at one of its named positions or outputs, by a name of their own: a pose the
 * robot takes, such as "stowed" or "amp", or rollers run together, such as "feeding". It's partial:
 * it names only the joints it sets, and leaves the rest where they are, so the joints it names are
 * what a command moving to it requires.
 *
 * {@snippet class=frc.robot.examples.ExampleRobot region=configurations}
 *
 * <p>The robot's definition checks each against the rest of it: every joint one of the robot's,
 * none that follows another (its leader is set instead), every interlock obeyed with the joints it
 * doesn't name where they rest, and, where the links have shapes, the season's extension limit
 * ({@code SeasonLimits}).
 *
 * @param name what code, logs and Pitwall call it: unique among the robot's configurations
 * @param targets each joint it sets, at one of its named positions or outputs, each joint once
 */
@ConfigRecord
public record Configuration(String name, List<Condition.At> targets) {
  public Configuration {
    targets = List.copyOf(targets);
    List<Condition.At> set = targets;
    Checks.on(Configuration.class, name)
        .field("name", name, notBlank())
        .field("targets", set, Configuration::notOnceEach)
        .orThrow();
  }

  /** Starts describing a configuration, one named value at a time. */
  public static ConfigurationBuilder.NameStage builder() {
    return ConfigurationBuilder.builder();
  }

  /**
   * A configuration of joints at named places: {@code Configuration.of("feeding",
   * FEEDER.at(FEEDER_FEEDING), INDEXER.at(INDEXER_FEEDING))}.
   */
  public static Configuration of(String name, Condition.At... targets) {
    return builder().name(name).targets(List.of(targets)).build();
  }

  /** The joints it sets, in its order: what a command moving to it requires. */
  public List<Joint> joints() {
    return targets.stream().map(Condition.At::joint).toList();
  }

  @Override
  public String toString() {
    return name
        + " ("
        + targets.stream().map(Condition.At::toString).collect(Collectors.joining(", "))
        + ")";
  }

  /** Why targets aren't joints each set once, or null when they are. */
  private static @Nullable String notOnceEach(List<Condition.At> targets) {
    if (targets.isEmpty()) {
      return "sets no joint: name one at least";
    }
    Set<String> seen = new HashSet<>();
    List<String> twice = new ArrayList<>();
    for (Condition.At target : targets) {
      if (!seen.add(target.joint().label())) {
        twice.add(target.joint().label());
      }
    }
    return twice.isEmpty() ? null : "set each joint once, not " + String.join(", ", twice);
  }
}
