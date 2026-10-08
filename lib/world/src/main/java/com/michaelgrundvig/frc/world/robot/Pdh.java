package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Amps;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.wpilib.units.measure.Current;

/**
 * The robot's power distribution: REV's Power Distribution Hub, on one of Systemcore's CAN ports at
 * its CAN ID. Each of its channels feeds one device through a breaker or a fuse, and reads the
 * current that device draws. A device says its channel and breaker where its wiring is, {@code
 * TALON_FX.can(32).pdh(4, Amps.of(40))}, and the robot's definition checks them against it: each
 * channel one of its own, feeding one device, through a breaker no bigger than the channel takes.
 * The robot reads every channel once a loop, and each device's current by its name.
 *
 * <p>Its channels, as REV labels them and WPILib numbers them: 0 to 19 are high-current, each
 * taking a breaker up to 40 A; 20 to 22 are low-current, and 23 is the switchable one, each taking
 * a fuse or breaker up to 15 A (REV's PDH specifications and its labeled diagram; WPILib's {@code
 * PowerDistribution}, channels 0 to 23). CTRE's Power Distribution Panel isn't here: its channels'
 * ratings aren't confirmed yet.
 *
 * @param bus the CAN bus it's on: one of Systemcore's own, where WPILib reads it
 * @param id its CAN ID, 1 to 63: 1 unless it's been changed, where WPILib looks for a REV PDH by
 *     default
 */
@ConfigRecord
public record Pdh(CanBus bus, int id) {
  /** How many channels it has, numbered from 0: 24. */
  public static final int CHANNELS = 24;

  /** Its high-current channels are those below this one: 0 to 19. */
  private static final int LOW_CURRENT = 20;

  /** Its switchable channel, the last of its low-current ones. */
  private static final int SWITCHABLE = 23;

  public Pdh {
    Checks.on(Pdh.class)
        .field("id", id, between(1, 63))
        .check(
            "bus",
            () ->
                bus instanceof CanBus.Canivore
                    ? "a PDH is on Systemcore's own CAN, where WPILib reads it, not on a CANivore"
                    : null)
        .orThrow();
  }

  /** Starts describing a PDH, one named value at a time. */
  public static PdhBuilder.BusStage builder() {
    return PdhBuilder.builder();
  }

  /** A PDH on Systemcore's first CAN port, at an ID: {@code Pdh.can(1)}. */
  public static Pdh can(int id) {
    return can(CanBus.SYSTEMCORE, id);
  }

  /** A PDH on one of Systemcore's CAN ports, at an ID. */
  public static Pdh can(CanBus bus, int id) {
    return builder().bus(bus).id(id).build();
  }

  /**
   * A device's feed, where a builder takes one: {@code .driveFeed(Pdh.feed(0, Amps.of(40)))}. A
   * motor controller in a joint's block says its own, {@code TALON_FX.can(32).pdh(4, Amps.of(40))}.
   */
  public static Feed feed(int channel, Current breaker) {
    return PdhFeedBuilder.builder().channel(channel).breaker(breaker).build();
  }

  /**
   * The most a channel's breaker or fuse may be rated: 40 A on a high-current channel, 15 A on a
   * low-current one or the switchable one.
   */
  public static Current rating(int channel) {
    return Amps.of(channel < LOW_CURRENT ? 40 : 15);
  }

  /**
   * Which of the PDH's channels feeds one device, and through what.
   *
   * @param channel the channel, 0 to 23, as labeled on the PDH
   * @param breaker the breaker's or fuse's rating, as printed on it: no more than its channel takes
   */
  @ConfigRecord
  public record Feed(int channel, Current breaker) {
    public Feed {
      Checks.on(Feed.class)
          .field("channel", channel, nonNegative())
          .field("breaker", breaker, positive())
          .orThrow();
    }

    /** Starts describing a feed, one named value at a time. */
    public static PdhFeedBuilder.ChannelStage builder() {
      return PdhFeedBuilder.builder();
    }
  }

  /**
   * Why a robot's feeds don't fit its PDH, each about its device; nothing when they do: a feed with
   * no PDH declared; a channel the PDH hasn't; a breaker over its channel's rating; a channel
   * feeding two devices.
   *
   * @param pdh the robot's PDH, if it declares one
   * @param devices every device on the robot's CAN buses ({@code RobotDefinition.devices})
   */
  static List<Finding> problems(Optional<Pdh> pdh, List<CanDevice> devices) {
    List<CanDevice> fed = devices.stream().filter(device -> device.feed().isPresent()).toList();
    if (fed.isEmpty()) {
      return List.of();
    }
    if (pdh.isEmpty()) {
      return List.of(
          Finding.refused(
              fed.get(0).name(),
              String.join(", ", fed.stream().map(CanDevice::name).toList())
                  + (fed.size() == 1 ? " is" : " are")
                  + " fed from a PDH's channel, and the robot declares no PDH: give its definition"
                  + " one, .pdh(Pdh.can(1)) at its CAN ID"));
    }
    List<Finding> problems = new ArrayList<>();
    Map<Integer, String> owners = new HashMap<>();
    for (CanDevice device : fed) {
      Feed feed = device.feed().orElseThrow();
      int channel = feed.channel();
      if (channel >= CHANNELS) {
        problems.add(
            Finding.refused(
                device.name(),
                possessive(device.name())
                    + " channel, "
                    + channel
                    + ", isn't one of the PDH's: they're 0 to "
                    + (CHANNELS - 1)));
        continue;
      }
      Current rating = rating(channel);
      if (feed.breaker().gt(rating)) {
        problems.add(
            Finding.refused(
                device.name(),
                possessive(device.name())
                    + " "
                    + shown(feed.breaker())
                    + " breaker is over channel "
                    + channel
                    + "'s rating: "
                    + kind(channel)
                    + " takes up to "
                    + shown(rating)));
      }
      String other = owners.putIfAbsent(channel, device.name());
      if (other != null) {
        problems.add(
            Finding.refused(
                device.name(),
                "channel "
                    + channel
                    + " feeds "
                    + other
                    + " and "
                    + device.name()
                    + ": a channel feeds one device, so give each its own"));
      }
    }
    return problems;
  }

  /** What kind of channel it is, as a sentence names it. */
  private static String kind(int channel) {
    if (channel < LOW_CURRENT) {
      return "a high-current channel";
    }
    return channel == SWITCHABLE ? "the switchable channel" : "a low-current channel";
  }
}
