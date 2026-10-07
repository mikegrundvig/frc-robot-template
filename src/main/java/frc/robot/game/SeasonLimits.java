package frc.robot.game;

import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;

/**
 * What a season's game manual allows a robot to be: how big around, how tall at the start, how far
 * it may reach out, where its bumpers go, and how much it may weigh. The season's game class holds
 * its numbers; every robot's definition is checked against them in the tests ({@link #problems}),
 * its named configurations too. Where the robot reaches between its configurations, mid-move, isn't
 * checked: no guard reads the limits while the robot runs.
 *
 * @param framePerimeter the most the frame may measure around
 * @param startingHeight the tallest the robot may be in its starting configuration: every joint at
 *     its reference
 * @param extension the farthest any part may reach beyond the frame perimeter, seen from above
 * @param bumperZoneBottom the lowest the bumpers may sit, above the floor
 * @param bumperZoneTop the highest they may reach
 * @param weight the most the robot may weigh, bumpers on and battery off
 * @param battery the battery's mass, with its leads: what the weight leaves out of a robot weighed
 *     with it
 */
@ConfigRecord
public record SeasonLimits(
    Distance framePerimeter,
    Distance startingHeight,
    Distance extension,
    Distance bumperZoneBottom,
    Distance bumperZoneTop,
    Mass weight,
    Mass battery) {
  public SeasonLimits {
    Checks.on(SeasonLimits.class)
        .field("framePerimeter", framePerimeter, positive())
        .field("startingHeight", startingHeight, positive())
        .field("extension", extension, positive())
        .field("bumperZoneBottom", bumperZoneBottom, positive())
        .field("bumperZoneTop", bumperZoneTop, positive())
        .field("weight", weight, positive())
        .field("battery", battery, positive())
        .check(
            "bumperZoneTop",
            () ->
                bumperZoneTop.gt(bumperZoneBottom)
                    ? null
                    : shown(bumperZoneTop)
                        + " must be above the zone's bottom ("
                        + shown(bumperZoneBottom)
                        + ")")
        .orThrow();
  }

  /**
   * How far past a limit still counts as at it: a robot built to a limit exactly, in inches or
   * pounds, mustn't fail by the rounding of its conversion to meters or kilograms.
   */
  private static final double SLACK = 1e-9;

  /**
   * A robot not yet weighed: the most the season allows it, with its battery. What a robot's body
   * weighs until it's on a scale ({@code Body.weight}), flagged as not weighed in the robot's
   * missing-data report.
   */
  public Body.Weight unweighed() {
    return Body.Weight.unweighed(weight.plus(battery));
  }

  /** Starts describing a season's limits, one named value at a time. */
  public static SeasonLimitsBuilder.FramePerimeterStage builder() {
    return SeasonLimitsBuilder.builder();
  }

  /**
   * Everything about a robot's definition the rules don't allow, each in words; empty when it fits.
   * The robot is checked as it starts a match, every joint at its reference, and in each of its
   * configurations, the joints it doesn't name at their references, against the extension limit.
   * Only the links with shapes reach anywhere; collisions between links aren't checked yet.
   */
  public List<String> problems(RobotDefinition definition) {
    Body body = definition.body();
    RobotMath math = definition.math();
    Map<Joint, Double> stowed = math.references();
    List<String> problems = new ArrayList<>();
    if (body.framePerimeter().in(Meters) > framePerimeter.in(Meters) + SLACK) {
      problems.add(
          "its frame is "
              + shown(body.framePerimeter())
              + " around, more than the "
              + shown(framePerimeter)
              + " allowed");
    }
    double tallest = Math.max(body.height().in(Meters), math.top(stowed));
    if (tallest > startingHeight.in(Meters) + SLACK) {
      problems.add(
          String.format(
              Locale.ROOT,
              "it starts %.3f m tall, more than the %.3f m allowed",
              tallest,
              startingHeight.in(Meters)));
    }
    double reach = math.extension(stowed);
    if (reach > SLACK) {
      problems.add(
          String.format(
              Locale.ROOT,
              "it starts reaching %.3f m beyond its frame perimeter: it must start inside it",
              reach));
    }
    for (Configuration configuration : definition.configurations()) {
      double out = math.extension(positions(configuration));
      if (out > extension.in(Meters) + SLACK) {
        problems.add(
            String.format(
                Locale.ROOT,
                "configuration %s reaches %.3f m beyond its frame perimeter, more than the %.3f m"
                    + " allowed",
                configuration.name(),
                out,
                extension.in(Meters)));
      }
    }
    if (body.bumperBottom().in(Meters) < bumperZoneBottom.in(Meters) - SLACK
        || body.bumperTop().in(Meters) > bumperZoneTop.in(Meters) + SLACK) {
      problems.add(
          "its bumpers, "
              + shown(body.bumperBottom())
              + " to "
              + shown(body.bumperTop())
              + " up, aren't within the bumper zone, "
              + shown(bumperZoneBottom)
              + " to "
              + shown(bumperZoneTop));
    }
    Mass weighed = body.mass().minus(battery);
    if (weighed.in(Kilograms) > weight.in(Kilograms) + SLACK) {
      problems.add(
          "it weighs "
              + shown(weighed)
              + " without its battery, more than the "
              + shown(weight)
              + " allowed");
    }
    return problems;
  }

  /**
   * Where a configuration puts the joints it names at positions; the rest stay at their references,
   * and its outputs move nothing anywhere.
   */
  private static Map<Joint, Double> positions(Configuration configuration) {
    Map<Joint, Double> positions = new LinkedHashMap<>();
    for (Condition.At target : configuration.targets()) {
      if (!target.isOutput()) {
        positions.put(target.joint(), target.joint().position(target.name()).value());
      }
    }
    return positions;
  }
}
