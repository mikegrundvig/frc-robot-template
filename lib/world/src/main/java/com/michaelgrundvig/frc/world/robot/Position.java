package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * A place a joint goes, by name: an intake's "in", "run" and "out", an elevator's "clear". It's the
 * joint's own ({@code Joint.positions}), in the joint's units, radians for a revolute joint and
 * meters for a prismatic one, as its range and rest are, and inside its range. It knows which it
 * is, an angle or a length, so a length given a joint that turns, or an angle given one that
 * slides, is refused where the joint is built. Robot code moves to it by name, interlocks and
 * configurations name it, and logs and Pitwall show it.
 *
 * @param name what code, logs and Pitwall call it: unique among its joint's positions and outputs
 * @param kind an angle, for a joint that turns, or a length, for one that slides
 * @param value where, in the joint's units: radians or meters
 */
@ConfigRecord
public record Position(String name, Kind kind, double value) {
  /** What a position measures: how far a joint has turned, or how far it has slid. */
  public enum Kind {
    /** An angle, in radians: a revolute joint's. */
    ANGLE,
    /** A length, in meters: a prismatic joint's. */
    LENGTH;

    /** A joint's: an angle for one that turns, a length for one that slides. */
    static Kind of(Joint joint) {
      return of(joint instanceof Joint.Revolute);
    }

    /** An angle for a joint that turns, a length for one that slides. */
    static Kind of(boolean revolute) {
      return revolute ? ANGLE : LENGTH;
    }
  }

  public Position {
    Checks.on(Position.class, name)
        .field("name", name, notBlank())
        .field("value", value, finite())
        .orThrow();
  }

  /** Starts describing a position, one named value at a time. */
  public static PositionBuilder.NameStage builder() {
    return PositionBuilder.builder();
  }

  /** A revolute joint's position, in degrees as CAD and teams' code give them. */
  public static Position degrees(String name, double degrees) {
    return builder().name(name).kind(Kind.ANGLE).value(Math.toRadians(degrees)).build();
  }

  /** A revolute joint's position. */
  public static Position of(String name, Angle angle) {
    return builder().name(name).kind(Kind.ANGLE).value(angle.in(Radians)).build();
  }

  /** A prismatic joint's position. */
  public static Position of(String name, Distance distance) {
    return builder().name(name).kind(Kind.LENGTH).value(distance.in(Meters)).build();
  }
}
