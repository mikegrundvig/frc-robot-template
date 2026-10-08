package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.PITCH_UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.definition;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.encoder;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.gearing;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.link;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.point;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.prismatic;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.revolute;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;

import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * Where a joint is from one absolute encoder's reading, as a table ({@link RobotMath#fromAbsolute},
 * {@link RobotMath#absoluteWindow}): the window the encoder tells apart is its turn, centered on
 * the joint's range, and a reading is wrapped into it. A pivot from 0° to 135°, its encoder on its
 * axle, tells -112.5° to 247.5° apart; the same pivot, its encoder geared two to one down and
 * turning the other way, two of its turns, -292.5° to 427.5°; an elevator from 0 m to 1.2 m, its
 * encoder turning once for twelve turns of its 0.75 in drum, twelve of the drum's circumferences
 * centered on 0.6 m. Each is read at its stops, a little past each, just inside its window's edges
 * and just past them, where a reading is the other edge's, below its zero, below its window, and
 * past a whole turn. Exactly at an edge, a reading is either edge: they're one place, a turn apart.
 */
class AbsoluteReadingsTest {
  private static final Joint.Revolute PIVOT =
      revolute(
          "pivot",
          Part.CHASSIS,
          point(0.2, 0, 0.3),
          PITCH_UP,
          Optional.of(Range.degrees(0, 135)),
          0,
          link("arm", 2, point(0.3, 0, 0), point(0.4, 0.05, 0.05)));

  private static final Joint.Prismatic ELEVATOR =
      prismatic(
          "elevator",
          Part.CHASSIS,
          point(-0.2, 0, 0.1),
          UP,
          Range.meters(0, 1.2),
          link("carriage", 5, point(0, 0, 0.1), point(0.3, 0.3, 0.1)));

  /** The elevator's drum's circumference, in meters: its 0.75 in pitch radius. */
  private static final double DRUM = 2 * Math.PI * 0.75 * 0.0254;

  private static final Sensor.Encoder ON_AXLE = encoder("pivot on axle", PIVOT, 1, true);

  private static final Sensor.Encoder GEARED_BACK = encoder("pivot geared back", PIVOT, -0.5, true);

  private static final Sensor.Encoder ON_DRUM =
      encoder("elevator on drum", ELEVATOR, 1 / (12 * DRUM), true);

  private static RobotMath math(Joint joint, Sensor.Encoder encoder) {
    Drive drive =
        joint instanceof Joint.Prismatic slides
            ? MechanismFixtures.motors(slides, Motor.KRAKEN_X60, 2, gearing(5, 1), Inches.of(0.75))
            : MechanismFixtures.motors((Joint.Revolute) joint, Motor.KRAKEN_X60, 1, gearing(25, 1));
    return definition(List.of(joint), List.of(), List.of(drive), List.of(encoder)).math();
  }

  /** Each encoder's window, in degrees or meters, lower then upper. */
  static Stream<Arguments> windows() {
    return Stream.of(
        Arguments.of(ON_AXLE, -112.5, 247.5),
        Arguments.of(GEARED_BACK, -292.5, 427.5),
        Arguments.of(ON_DRUM, 0.6 - 6 * DRUM, 0.6 + 6 * DRUM));
  }

  @ParameterizedTest
  @MethodSource("windows")
  void itsWindowIsItsTurnCenteredOnTheRange(Sensor.Encoder encoder, double lower, double upper) {
    Range window = math(encoder.joint(), encoder).absoluteWindow(List.of(encoder));

    assertThat(shown(encoder, window.lower())).isCloseTo(lower, within(1e-9));
    assertThat(shown(encoder, window.upper())).isCloseTo(upper, within(1e-9));
  }

  /**
   * Readings in the encoder's turns, its offset applied and growing as the joint moves forward, and
   * where the joint is, in degrees or meters.
   */
  static Stream<Arguments> readings() {
    // The elevator: each of the encoder's turns is twelve of its drum's.
    double perTurn = 12 * DRUM;
    return Stream.of(
        Arguments.of(ON_AXLE, "at its lower stop", 0, 0),
        Arguments.of(ON_AXLE, "at its upper stop", 0.375, 135),
        Arguments.of(ON_AXLE, "a little below its lower stop", -0.01, -3.6),
        Arguments.of(ON_AXLE, "a little past its upper stop", 0.385, 138.6),
        Arguments.of(ON_AXLE, "just inside its window's lower edge", -0.312, -112.32),
        Arguments.of(ON_AXLE, "just below its window's lower edge", -0.313, 247.32),
        Arguments.of(ON_AXLE, "just inside its window's upper edge", 0.687, 247.32),
        Arguments.of(ON_AXLE, "just past its window's upper edge", 0.688, -112.32),
        Arguments.of(ON_AXLE, "below its zero", -0.2, -72),
        Arguments.of(ON_AXLE, "below its window", -0.35, 234),
        Arguments.of(ON_AXLE, "past a turn", 1.1, 36),
        Arguments.of(ON_AXLE, "past a turn below", -1.1, -36),
        Arguments.of(GEARED_BACK, "at its lower stop", 0, 0),
        Arguments.of(GEARED_BACK, "at its upper stop", 0.1875, 135),
        Arguments.of(GEARED_BACK, "a little below its lower stop", -0.005, -3.6),
        Arguments.of(GEARED_BACK, "a little past its upper stop", 0.1925, 138.6),
        Arguments.of(GEARED_BACK, "just inside its window's lower edge", -0.406, -292.32),
        Arguments.of(GEARED_BACK, "just past its window's upper edge", 0.594, -292.32),
        Arguments.of(GEARED_BACK, "below its zero", -0.3, -216),
        Arguments.of(GEARED_BACK, "below its window", -0.5, 360),
        Arguments.of(GEARED_BACK, "past a turn", 1.1, 72),
        Arguments.of(ON_DRUM, "at its lower stop", 0, 0),
        Arguments.of(ON_DRUM, "at its upper stop", 1.2 / perTurn, 1.2),
        Arguments.of(ON_DRUM, "a little below its lower stop", -0.01 / perTurn, -0.01),
        Arguments.of(ON_DRUM, "a little past its upper stop", 1.22 / perTurn, 1.22),
        Arguments.of(
            ON_DRUM,
            "just inside its window's lower edge",
            0.6 / perTurn - 0.5 + 0.001,
            0.6 - perTurn / 2 + 0.001 * perTurn),
        Arguments.of(
            ON_DRUM,
            "just past its window's upper edge",
            0.6 / perTurn + 0.5 + 0.001,
            0.6 - perTurn / 2 + 0.001 * perTurn),
        Arguments.of(ON_DRUM, "below its window", -0.5, perTurn / 2),
        Arguments.of(ON_DRUM, "past a turn", 1 + 0.3 / perTurn, 0.3));
  }

  @ParameterizedTest(name = "{0} {1}")
  @MethodSource("readings")
  void aReadingIsWhereTheJointIsInTheWindow(
      Sensor.Encoder encoder, String where, double turns, double expected) {
    double position = math(encoder.joint(), encoder).fromAbsolute(encoder, turns);

    assertThat(shown(encoder, position)).as(where).isCloseTo(expected, within(1e-9));
  }

  /** A position in degrees for a joint that turns, in meters for one that slides. */
  private static double shown(Sensor.Encoder encoder, double position) {
    return encoder.joint() instanceof Joint.Revolute ? Math.toDegrees(position) : position;
  }
}
