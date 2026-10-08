package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.PITCH_UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.definition;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.doubts;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.gearing;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.linear;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.link;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.point;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.prismatic;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.revolute;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatIllegalStateException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Volts;

import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * Named positions and outputs, configurations and interlocks, and the definition's checks of them:
 * on an intake (a pivot and the roller on its end) and a stacked arm (an arm on an elevator), the
 * design's own examples.
 */
class PlacesTest {
  static final Position PIVOT_IN = Position.degrees("in", 1.5);
  static final Position PIVOT_RUN = Position.degrees("run", 110);
  static final Position PIVOT_OUT = Position.degrees("out", 130);
  static final Output ROLLER_INTAKING = Output.rpm("intaking", 2000);
  static final Output ROLLER_OUTTAKING = Output.rpm("outtaking", -500);
  static final Position ELEVATOR_DOWN = Position.of("down", Inches.of(0));
  static final Position ELEVATOR_CLEAR = Position.of("clear", Inches.of(0.4 / 0.0254));
  static final Position ELEVATOR_TOP = Position.of("top", Inches.of(1.1 / 0.0254));
  static final Position ARM_STOWED = Position.of("stowed", Degrees.of(-20));
  static final Position ARM_LEVEL = Position.degrees("level", 0);
  static final Position ARM_UP = Position.degrees("up", 90);

  /** An intake's pivot, stowed in at 1.5°, run from 110°, out at 130°, its stops at 0° and 135°. */
  static final Joint.Revolute PIVOT =
      named(
          revolute(
              "pivot",
              Part.CHASSIS,
              point(0.25, 0, 0.3),
              PITCH_UP,
              Optional.of(Range.degrees(0, 135)),
              Math.toRadians(1.5),
              link("intake arm", 2, point(0.15, 0, 0), point(0.3, 0.4, 0.05))),
          List.of(PIVOT_IN, PIVOT_RUN, PIVOT_OUT),
          List.of());

  /** Its roller, through 3:1: a Kraken X60's 6000 rpm is 2000 of the roller's, all "intaking". */
  static final Joint.Revolute ROLLER =
      named(
          revolute(
              "roller",
              PIVOT,
              point(0.3, 0, 0),
              PITCH_UP,
              Optional.empty(),
              0,
              link("roller", 0.5, point(0, 0, 0), point(0.05, 0.4, 0.05))),
          List.of(),
          List.of(ROLLER_INTAKING, ROLLER_OUTTAKING));

  /** An elevator, 0 to 1.2 m: down, clear of the bumper at 0.4 m, and at the top. */
  static final Joint.Prismatic ELEVATOR =
      named(
          prismatic(
              "elevator",
              Part.CHASSIS,
              point(-0.1, 0, 0.1),
              UP,
              Range.meters(0, 1.2),
              link("carriage", 3, point(0, 0, 0.1), point(0.1, 0.4, 0.2))),
          List.of(ELEVATOR_DOWN, ELEVATOR_CLEAR, ELEVATOR_TOP),
          List.of());

  /** An arm on it, stowed pointing down and back at -20°, level at 0°, up at 90°. */
  static final Joint.Revolute ARM =
      named(
          revolute(
              "arm",
              ELEVATOR,
              point(0, 0, 0.2),
              PITCH_UP,
              Optional.of(Range.degrees(-30, 120)),
              Math.toRadians(-20),
              link("arm", 2, point(0.2, 0, 0), point(0.4, 0.05, 0.05))),
          List.of(ARM_STOWED, ARM_LEVEL, ARM_UP),
          List.of());

  static final List<Joint> JOINTS = List.of(PIVOT, ROLLER, ELEVATOR, ARM);

  static final List<Drive> DRIVES =
      List.of(
          MechanismFixtures.motors(PIVOT, Motor.KRAKEN_X60, 1, gearing(25, 1)),
          MechanismFixtures.motors(ROLLER, Motor.KRAKEN_X60, 1, gearing(3, 1)),
          MechanismFixtures.motors(ELEVATOR, Motor.KRAKEN_X60, 2, gearing(10, 1), Inches.of(1)),
          MechanismFixtures.motors(ARM, Motor.KRAKEN_X60, 1, gearing(60, 1)));

  static final Interlock ROLLER_ONLY_WHEN_OUT =
      Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN));

  /** The arm kept out of the bumper's band while the elevator is below 0.35 m, where it clears. */
  static final Interlock ARM_CLEARS_BUMPER =
      Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.atMost(Meters.of(0.35)));

  static final Configuration STOWED =
      Configuration.of("stowed", ELEVATOR.at(ELEVATOR_DOWN), ARM.at(ARM_STOWED));

  static final Configuration AMP =
      Configuration.of("amp", ELEVATOR.at(ELEVATOR_CLEAR), ARM.at(ARM_LEVEL));

  static final Configuration INTAKING =
      Configuration.of("intaking", PIVOT.at(PIVOT_OUT), ROLLER.at(ROLLER_INTAKING));

  @Test
  void aJointNamesItsPlacesInItsOwnUnits() {
    assertThat(PIVOT.position("run").value()).isCloseTo(Math.toRadians(110), within(1e-12));
    assertThat(ELEVATOR.position("clear").value()).isCloseTo(0.4, within(1e-12));
    assertThat(ROLLER.output("intaking").value())
        .as("radians a second")
        .isCloseTo(2000 * 2 * Math.PI / 60, within(1e-9));
    assertThat(Output.of("spin", RotationsPerSecond.of(1)).value())
        .isCloseTo(2 * Math.PI, within(1e-12));
    assertThat(Output.of("slide", MetersPerSecond.of(0.5)).value()).isEqualTo(0.5);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PIVOT.position("deployed"))
        .withMessage("pivot has no position named \"deployed\": it has in, run, out");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ROLLER.output("feeding"))
        .withMessage(
            "roller has no output named \"feeding\": it has intaking, outtaking, and stopped");
    assertThat(ROLLER.output("stopped")).as("every joint's, built in").isEqualTo(Output.STOPPED);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ELEVATOR.position("x"))
        .withMessageEndingWith("it has down, clear, top");
  }

  /**
   * A named position is inside its joint's range, as the robot's definition checks on the finished
   * joint, and each name is the joint's once.
   */
  @Test
  void aJointsPlacesAreInsideItsRangeAndNamedOnce() {
    Joint.Revolute pastItsStops =
        named(PIVOT, List.of(Position.degrees("in", 1.5), Position.degrees("up", 140)), List.of());
    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(List.of(pastItsStops), List.of(), List.of(), List.of()))
        .withMessage(
            "RobotDefinition[test].joints: pivot's up, 140°, is outside its hard stops, 0° to"
                + " 135°: it can't get there");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                named(
                    ELEVATOR,
                    List.of(Position.of("top", Inches.of(1))),
                    List.of(Output.of("top", MetersPerSecond.of(1)))))
        .withMessage(
            "Prismatic[elevator].outputs: it names top twice: each name is one position or output"
                + " of the joint");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> named(ROLLER, List.of(), List.of(Output.rpm("a", 1), Output.rpm("a", 2))))
        .withMessage(
            "Revolute[roller].outputs: it names a twice: each name is one position or output of the"
                + " joint");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> named(ROLLER, List.of(), List.of(Output.volts("stopped", 0))))
        .withMessage(
            "Revolute[roller].outputs: it names stopped, which every joint has built in (its"
                + " motors at neutral): name it something else");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Position.degrees(" ", 1))
        .withMessageContaining("Position[ ].name: must not be blank");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Output.rpm("fast", Double.POSITIVE_INFINITY))
        .withMessageContaining("Output[fast].value");
  }

  /**
   * A position knows whether it's an angle or a length, so one written for the wrong joint is
   * refused: a turning joint's in degrees put on a slide reads as meters otherwise, and a slide's
   * inches on a pivot as radians.
   */
  @Test
  void aPositionIsTheJointsKindOfPlace() {
    assertThat(PIVOT.position("run").kind()).isEqualTo(Position.Kind.ANGLE);
    assertThat(ELEVATOR.position("clear").kind()).isEqualTo(Position.Kind.LENGTH);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> named(PIVOT, List.of(Position.of("deep", Inches.of(1))), List.of()))
        .withMessage(
            "Revolute[pivot].positions: deep is a length, and the joint turns: give it an angle,"
                + " Position.degrees(...)");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> named(ELEVATOR, List.of(Position.degrees("level", 0)), List.of()))
        .withMessage(
            "Prismatic[elevator].positions: level is an angle, and the joint slides: give it a"
                + " length, Position.of(name, Inches.of(...))");
  }

  /**
   * A speed knows whether it's of turning or of sliding, as a position knows it's an angle or a
   * length: an rpm on an elevator would run in meters a second, and a speed along a slide on a
   * roller in radians a second. A voltage or a duty cycle is neither.
   */
  @Test
  void anOutputsSpeedIsTheJointsKind() {
    assertThat(ROLLER_INTAKING.motion()).contains(Position.Kind.ANGLE);
    assertThat(Output.of("creeping", MetersPerSecond.of(0.1)).motion())
        .contains(Position.Kind.LENGTH);
    assertThat(Output.volts("pushing", 6).motion()).isEmpty();
    assertThat(named(ELEVATOR, List.of(), List.of(Output.of("creeping", MetersPerSecond.of(0.1)))))
        .isNotNull();
    assertThatIllegalArgumentException()
        .isThrownBy(() -> named(ELEVATOR, List.of(), List.of(Output.rpm("creeping", 10))))
        .withMessage(
            "Prismatic[elevator].outputs: creeping, 10 rpm, is a speed of turning, and elevator"
                + " slides: give it a speed along it, Output.of(\"creeping\","
                + " InchesPerSecond.of(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> named(ROLLER, List.of(), List.of(Output.of("feeding", MetersPerSecond.of(3)))))
        .withMessage(
            "Revolute[roller].outputs: feeding, 3 m/s, is a speed along a slide, and roller turns:"
                + " give it a speed of turning, Output.rpm(\"feeding\", ...)");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> Output.builder().name("fast").kind(Output.Kind.VELOCITY).value(10).build())
        .withMessage(
            "Output[fast].motion: a speed is of turning or of sliding: give it with its unit,"
                + " Output.rpm(name, rpm) for a joint that turns, Output.of(name,"
                + " InchesPerSecond.of(...)) for one that slides");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Output.builder()
                    .name("push")
                    .kind(Output.Kind.VOLTAGE)
                    .value(6)
                    .motion(Position.Kind.ANGLE)
                    .build())
        .withMessage(
            "Output[push].motion: only a speed is of turning or of sliding, and this is a voltage");
  }

  /**
   * A range knows whether it's angles or lengths, as a position does: a joint refuses its hard
   * stops, or a region an interlock keeps it out of, of the other kind.
   */
  @Test
  void aRangeIsTheJointsKindOfStretch() {
    assertThat(Range.of(Degrees.of(0), Degrees.of(135))).isEqualTo(Range.degrees(0, 135));
    assertThat(Range.of(Inches.of(0), Inches.of(10)).kind()).isEqualTo(Position.Kind.LENGTH);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Joint.elevator("elevator", Part.CHASSIS, point(0, 0, 0))
                    .range(Range.degrees(0, 135))
                    .rest(Inches.of(0)))
        .withMessage(
            "Prismatic[elevator].range: its range is angles, and the joint slides: give it in"
                + " lengths, Range.meters(...) or Range.of(Inches.of(...), Inches.of(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Interlock.that(PIVOT)
                    .keepsOut(Range.meters(0, 1))
                    .when(ELEVATOR.atMost(Inches.of(10))))
        .withMessage(
            "the region is lengths, and the joint turns: give it in"
                + " angles, Range.degrees(...)");
  }

  /**
   * "At least" a position is from it on, toward the range's upper end; "at most" is up to it,
   * toward the lower; "between" includes both ends; "at" is the position exactly. None depends on
   * where the joint rests: a joint resting below its range's middle reads them as one resting above
   * it does.
   */
  @Test
  void conditionsAreRegionsOfAJointsTravel() {
    Position low = Position.degrees("low", -60);
    Joint.Revolute hanging =
        named(
            revolute(
                "hanging",
                Part.CHASSIS,
                point(0, 0, 0.5),
                PITCH_UP,
                Optional.of(Range.degrees(-90, 0)),
                0,
                link("hanging", 1, point(0.1, 0, 0), point(0.1, 0.1, 0.1))),
            List.of(low, Position.degrees("top", 0)),
            List.of());

    assertThat(PIVOT.atLeast(PIVOT_RUN).holdsAt(Math.toRadians(120))).isTrue();
    assertThat(PIVOT.atLeast(PIVOT_RUN).holdsAt(Math.toRadians(110))).isTrue();
    assertThat(PIVOT.atLeast(PIVOT_RUN).holdsAt(Math.toRadians(100))).isFalse();
    assertThat(PIVOT.atMost(PIVOT_RUN).holdsAt(Math.toRadians(100))).isTrue();
    assertThat(PIVOT.atMost(PIVOT_RUN).holdsAt(Math.toRadians(110))).isTrue();
    assertThat(PIVOT.atMost(PIVOT_RUN).holdsAt(Math.toRadians(111))).isFalse();
    assertThat(hanging.atMost(low).holdsAt(Math.toRadians(-70)))
        .as("below it, resting above it")
        .isTrue();
    assertThat(hanging.atMost(low).holdsAt(Math.toRadians(-50))).isFalse();
    assertThat(hanging.atLeast(low).holdsAt(Math.toRadians(-50))).isTrue();
    assertThat(PIVOT.between(PIVOT_OUT, PIVOT_RUN).holdsAt(Math.toRadians(120))).isTrue();
    assertThat(PIVOT.between(PIVOT_RUN, PIVOT_OUT).holdsAt(Math.toRadians(131))).isFalse();
    assertThat(PIVOT.at(PIVOT_OUT).holdsAt(Math.toRadians(130))).isTrue();
    assertThat(PIVOT.at(PIVOT_OUT).isOutput()).isFalse();
    assertThat(ROLLER.at(ROLLER_INTAKING).isOutput()).isTrue();
    assertThatIllegalStateException()
        .isThrownBy(() -> ROLLER.at(ROLLER_INTAKING).holdsAt(0))
        .withMessage("roller at intaking is an output, not a position");
    assertThat(ROLLER.at(Output.STOPPED).isOutput()).as("every joint's, built in").isTrue();
    assertThat(PIVOT.atLeast(PIVOT_RUN)).hasToString("pivot at least run");
    assertThat(ELEVATOR.atMost(ELEVATOR_CLEAR)).hasToString("elevator at most clear");
    assertThat(PIVOT.between(PIVOT_RUN, PIVOT_OUT)).hasToString("pivot between run and out");
    assertThat(ARM.at(ARM_LEVEL)).hasToString("arm at level");
  }

  /**
   * A condition on the joint's own places is checked as it's made: an edge at its lowest, a band
   * from a place to itself. One naming a place not the joint's own is made all the same, the place
   * kept as the code names it, and what uses it refuses it, saying whose it is: a gate, an
   * interlock or a configuration.
   */
  @Test
  void aConditionKeepsThePlacesItNamesForWhatUsesItToJudge() {
    RobotDefinition robot = robot(List.of(), List.of());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> ELEVATOR.atLeast(ELEVATOR_DOWN))
        .withMessage("down is elevator's lowest: it's at least that everywhere");
    assertThat(PIVOT.atLeast(Position.of("out", Degrees.of(130))).position())
        .as("its own out, written another way")
        .isCloseTo(Math.toRadians(130), within(1e-12));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PIVOT.between(PIVOT_RUN, PIVOT_RUN))
        .withMessage("a joint between run and itself is at it");
    assertThat(robot.whyNotOwn(PIVOT.between(PIVOT_RUN, PIVOT_OUT))).isNull();
    assertThat(robot.whyNotOwn(PIVOT.atMost(Position.degrees("deployed", 120))))
        .isEqualTo("pivot has no position deployed at 120°: it has in, run, out");
    assertThat(robot.whyNotOwn(PIVOT.atLeast(Position.of("out", Inches.of(5)))))
        .isEqualTo("pivot's out is an angle, and this out a length: another joint's");
    assertThat(robot.whyNotOwn(PIVOT.between(PIVOT_RUN, ARM_UP)))
        .isEqualTo("pivot has no position up at 90°: it has in, run, out");
    assertThat(PIVOT.between(PIVOT_RUN, ARM_UP)).hasToString("pivot between run and up");
    assertThat(robot.whyNotOwn(ROLLER.at(Output.rpm("feeding", 1000))))
        .isEqualTo("roller has no output feeding: it has intaking, outtaking, and stopped");
  }

  /**
   * Another joint's place of a name this one has, elsewhere, is refused wherever it's used, an
   * interlock, a configuration, a gate or a re-placing, as a move to it is: the arm's "out" at 60°
   * isn't the pivot's, whose out is at 130°. The definition refuses an interlock or a configuration
   * on it as it's built, saying whose it is. The pivot's own is the pivot's, at 130°, and so is any
   * out it has had.
   */
  @Test
  void anotherJointsPlaceOfTheSameNameIsRefusedInAnInterlockOrAConfiguration() {
    Position armOut = Position.degrees("out", 60);
    String why =
        "pivot's out is at 130°, and this out, at 60°, isn't one it has had: another joint's or"
            + " another robot's; give pivot's own";

    assertThat(robot(List.of(), List.of()).whyNotOwn(PIVOT.between(PIVOT_RUN, armOut)))
        .isEqualTo(why);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    List.of(Configuration.of("out", PIVOT.at(armOut), ROLLER.at(ROLLER_INTAKING))),
                    List.of(Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(armOut)))))
        .withMessageContaining("configurations: configuration out: " + why)
        .withMessageContaining(
            "interlocks: interlock \"roller runs only when pivot at least out\": " + why);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PIVOT.replaced(armOut, Degrees.of(128)))
        .withMessage(why);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ROLLER.replaced(Output.rpm("intaking", 1000), RPM.of(1800)))
        .withMessageStartingWith("roller's intaking runs at 2000 rpm, and this intaking, at 1000");
    assertThat(robot(List.of(), List.of()).whyNotOwn(ROLLER.at(Output.rpm("intaking", 1000))))
        .isEqualTo(
            "roller's intaking runs at 2000 rpm, and this intaking, at 1000 rpm, isn't one it has"
                + " had: another joint's or another robot's; give roller's own");
    Joint.Revolute practice = PIVOT.replaced(PIVOT_OUT, Degrees.of(125));
    assertThat(practice.atLeast(PIVOT_OUT).position())
        .as("the competition robot's constant, on its practice robot")
        .isCloseTo(Math.toRadians(125), within(1e-12));
    assertThat(practice.at(PIVOT_OUT)).isEqualTo(PIVOT.at(PIVOT_OUT));
    assertThat(practice.replaced(PIVOT_OUT, Degrees.of(120)).position("out"))
        .as("re-placed again, by the competition robot's constant")
        .isEqualTo(Position.of("out", Degrees.of(120)));
    assertThat(practice.atLeast(armOut).position())
        .as("kept as the code says, for what uses it to refuse")
        .isCloseTo(Math.toRadians(60), within(1e-12));
  }

  /**
   * A subsystem of a team's own finds its joint by its constant: on a practice robot, the practice
   * robot's own, with its places. A joint the robot hasn't is refused, saying so.
   */
  @Test
  void aRobotsOwnJointIsFoundByItsConstant() {
    RobotDefinition competition = robot(List.of(STOWED, AMP, INTAKING), interlocks());
    RobotDefinition practice = competition.with(PIVOT.replaced(PIVOT_RUN, Degrees.of(120)));

    assertThat(practice.own(PIVOT).position("run")).isEqualTo(Position.degrees("run", 120));
    assertThat(competition.own(PIVOT).position("run")).isEqualTo(PIVOT_RUN);
    Joint.Revolute wrist =
        revolute(
            "wrist",
            Part.CHASSIS,
            point(0, 0, 0.5),
            PITCH_UP,
            Optional.of(Range.degrees(0, 90)),
            0,
            link("wrist", 1, point(0.1, 0, 0), point(0.2, 0.1, 0.1)));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> practice.own(wrist))
        .withMessageEndingWith("has no joint wrist: add it to the definition's joints");
  }

  /**
   * A place is a name on its joint, its value the robot's: a practice robot's pivot that runs from
   * 120°, its configurations and interlocks written of the competition robot's constants, is
   * checked and enforced at its own places. One re-placed where it breaks an interlock is refused,
   * as one outside its range is, and one it has no name for.
   */
  @Test
  void aPracticeRobotsPlacesAreItsOwnWhereverItsCodeNamesThem() {
    RobotDefinition competition = robot(List.of(STOWED, AMP, INTAKING), interlocks());

    RobotDefinition practice = competition.with(PIVOT.replaced(PIVOT_RUN, Degrees.of(120)));

    Condition.AtLeast run = (Condition.AtLeast) practice.interlocks().getFirst().condition();
    assertThat(run).hasToString("pivot at least run");
    assertThat(run.position()).isCloseTo(Math.toRadians(120), within(1e-12));
    assertThat(practice.own(PIVOT.atLeast(PIVOT_RUN)))
        .as("a condition the program names, at the practice robot's own run")
        .isEqualTo(run);
    assertThat(practice.own(ROLLER.at(ROLLER_INTAKING))).isEqualTo(ROLLER.at(ROLLER_INTAKING));
    assertThat(run.joint().position("run").value())
        .as("the practice robot's own pivot")
        .isCloseTo(Math.toRadians(120), within(1e-12));
    assertThat(practice.configurations().get(2).targets().getFirst().joint().position("run"))
        .isEqualTo(Position.degrees("run", 120));
    assertThat(practice.configurations()).containsExactly(STOWED, AMP, INTAKING);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(PIVOT.replaced(PIVOT_RUN, Degrees.of(132))))
        .withMessageContaining(
            "configuration intaking runs roller, but roller runs only when pivot at least run");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(PIVOT.replaced(PIVOT_OUT, Degrees.of(140))))
        .withMessageContaining(
            "pivot's out, 140°, is outside its hard stops, 0° to 135°: it can't get there");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PIVOT.replaced(ARM_UP, Degrees.of(100)))
        .withMessage(
            "pivot has no position up to re-place: it has in, run, out. Its positions are what"
                + " its code names, the same on every robot of the design");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ROLLER.replaced(Output.volts("feeding", 4), Volts.of(5)))
        .withMessage(
            "roller has no output feeding to re-value: it has intaking, outtaking. Its outputs are"
                + " what its code names, the same on every robot of the design");
  }

  /**
   * A practice robot's stops are its own, and every check runs on them: a place outside them is
   * refused, and so is an interlock's edge.
   */
  @Test
  void aPracticeRobotsStopsAreChecked() {
    RobotDefinition competition = robot(List.of(STOWED, AMP, INTAKING), interlocks());

    RobotDefinition practice = competition.with(PIVOT.range(Range.degrees(0, 132)));

    assertThat(practice.joints().getFirst().range()).contains(Range.degrees(0, 132));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(ELEVATOR.range(Range.meters(0, 1))))
        .withMessageContaining(
            "elevator's top, 1.1 m, is outside its hard stops, 0 m to 1 m: it can't get there");
    Interlock clears = Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(125)));
    RobotDefinition clearing = robot(List.of(STOWED, AMP), List.of(clears, ARM_CLEARS_BUMPER));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                clearing.with(
                    PIVOT.replaced(PIVOT_OUT, Degrees.of(118)).range(Range.degrees(0, 120))))
        .withMessageContaining("125° is outside pivot's range, 0° to 120°");
  }

  /**
   * A practice robot's joint rests where it says, and every check runs on it: the interlocks as the
   * robot starts there, and its stops. One stowed past its stops can't start there.
   */
  @Test
  void aPracticeRobotsRestIsChecked() {
    RobotDefinition competition = robot(List.of(STOWED, AMP, INTAKING), interlocks());

    RobotDefinition practice = competition.with(PIVOT.rest(Degrees.of(5)));

    assertThat(practice.joints().getFirst().restPosition())
        .isCloseTo(Math.toRadians(5), within(1e-12));
    assertThat(competition.with(PIVOT.rest(Degrees.of(132))).joints().getFirst().restPosition())
        .as("stowed beyond its out, its run still from 110° on")
        .isCloseTo(Math.toRadians(132), within(1e-12));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(PIVOT.rest(Degrees.of(140))))
        .withMessageContaining(
            "pivot rests at 140°, outside its hard stops, 0° to 135°: it can't start there");
  }

  /**
   * An output re-valued keeps its name, and its new value says its kind: a practice robot's roller
   * that unjams at -1 V where the competition robot's runs back at a speed, or at a share of the
   * battery.
   */
  @Test
  void anOutputIsReValuedItsNameKept() {
    Joint.Revolute practice =
        ROLLER.replaced(ROLLER_OUTTAKING, Volts.of(-1)).replaced(ROLLER_INTAKING, RPM.of(1800));

    assertThat(practice).isEqualTo(ROLLER);
    assertThat(practice.output("outtaking")).isEqualTo(Output.volts("outtaking", -1));
    assertThat(practice.own(ROLLER_INTAKING).value())
        .isCloseTo(RPM.of(1800).in(RadiansPerSecond), within(1e-9));
    Output share = ROLLER.replaced(ROLLER_INTAKING, Percent.of(60)).output("intaking");
    assertThat(share.kind()).isEqualTo(Output.Kind.DUTY_CYCLE);
    assertThat(share.value()).isCloseTo(0.6, within(1e-12));
    assertThat(
            ELEVATOR
                .replaced(ELEVATOR_CLEAR, Meters.of(0.38))
                .outputs(Output.of("climbing", MetersPerSecond.of(0.5)))
                .replaced(Output.of("climbing", MetersPerSecond.of(0.5)), MetersPerSecond.of(0.4))
                .output("climbing"))
        .isEqualTo(Output.of("climbing", MetersPerSecond.of(0.4)));
    assertThat(ELEVATOR.replaced(ELEVATOR_CLEAR, Meters.of(0.38)).own(ELEVATOR_CLEAR).value())
        .isCloseTo(0.38, within(1e-12));
    assertThat(practice.formerOutputs())
        .as("what it ran at, the competition robot's")
        .containsExactly(ROLLER_OUTTAKING, ROLLER_INTAKING);
    assertThat(practice.own(ROLLER_OUTTAKING)).isEqualTo(Output.volts("outtaking", -1));
    assertThat(practice.whyNotOwn(Output.volts("outtaking", -2)))
        .isEqualTo(
            "roller's outtaking runs at -1 V, and this outtaking, at -2 V, isn't one it has had:"
                + " another joint's or another robot's; give roller's own");
    assertThat(
            ROLLER
                .replaced(ROLLER_INTAKING, Percent.of(60))
                .whyNotOwn(Output.dutyCycle("intaking", 0.5)))
        .startsWith("roller's intaking runs at 60% of the battery, and this intaking, at 50% of");
  }

  /** The design's stacked arm and intake, every interlock obeyed: the definition takes them. */
  @Test
  void aRobotsConfigurationsAndInterlocksAreChecked() {
    RobotDefinition robot = robot(List.of(STOWED, AMP, INTAKING), interlocks());

    assertThat(robot.configurations()).containsExactly(STOWED, AMP, INTAKING);
    assertThat(robot.interlocks()).containsExactly(ROLLER_ONLY_WHEN_OUT, ARM_CLEARS_BUMPER);
    assertThat(AMP.joints()).containsExactly(ELEVATOR, ARM);
    assertThat(AMP).hasToString("amp (elevator at clear, arm at level)");
    assertThat(ROLLER_ONLY_WHEN_OUT).hasToString("roller runs only when pivot at least run");
    assertThat(ARM_CLEARS_BUMPER)
        .hasToString("arm keeps out of -10° to 30° while elevator at most 0.35 m");
    assertThat(((Interlock.KeepsOut) ARM_CLEARS_BUMPER).keepsOut(Math.toRadians(30))).isFalse();
    assertThat(((Interlock.KeepsOut) ARM_CLEARS_BUMPER).keepsOut(Math.toRadians(29))).isTrue();
  }

  /**
   * A configuration names each joint once, at its named places, and obeys every interlock, with the
   * joints it doesn't name where they start.
   */
  @Test
  void aConfigurationObeysTheInterlocks() {
    Configuration rolling = Configuration.of("rolling", ROLLER.at(ROLLER_INTAKING));
    Configuration low = Configuration.of("low", ARM.at(ARM_LEVEL));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(List.of(rolling, low), interlocks()))
        .withMessageContaining(
            "RobotDefinition[test].configurations: configuration rolling runs roller, but roller"
                + " runs only when pivot at least run, and pivot is at 1.5°, where it rests")
        .withMessageContaining(
            "configuration low puts arm at 0°, but arm keeps out of -10° to 30° while elevator"
                + " at most 0.35 m, and elevator is at 0 m, where it rests");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Configuration.of("twice", ARM.at(ARM_LEVEL), ARM.at(ARM_UP)))
        .withMessage("Configuration[twice].targets: set each joint once, not arm");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Configuration.of("none"))
        .withMessage("Configuration[none].targets: sets no joint: name one at least");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(List.of(AMP, AMP), List.of()))
        .withMessage("RobotDefinition[test].configurations: two configurations are named amp");
    Configuration stopped =
        Configuration.of("stopped", ROLLER.at(Output.STOPPED), PIVOT.at(PIVOT_IN));
    assertThat(robot(List.of(stopped), interlocks()).configurations())
        .as("stopped doesn't run the roller, so the pivot may stay in")
        .containsExactly(stopped);
    Output pushing = Output.volts("pushing", 3);
    Configuration elsewhere =
        Configuration.of("elsewhere", named(ROLLER, List.of(), List.of(pushing)).at(pushing));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(List.of(elsewhere), interlocks()))
        .withMessageContaining(
            "configuration elsewhere's joint, roller, isn't one of the robot's joints: list it");
  }

  /**
   * An interlock restricts a joint by where another is: not itself, not by an output, not exactly
   * at one of its positions, and kept out of part of its range, never all of it or none.
   */
  @Test
  void anInterlockIsAboutAnotherJointsPlace() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Interlock.that(PIVOT).runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN)))
        .withMessageContaining("pivot can't be held by where it is itself");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Interlock.that(PIVOT).runsOnlyWhen(ROLLER.at(ROLLER_INTAKING)))
        .withMessageContaining(
            "an interlock speaks of where a joint is, or how fast it runs, and intaking is roller's output: say the speed it must reach, roller.upToSpeed(...)");
    // Judged against where the joint is measured, "at" a place would never hold.
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(PIVOT.at(PIVOT_OUT)))
        .withMessage(
            "pivot at out holds only with pivot exactly at out, 130°, which a measured position never is, so the interlock would never hold: say the edge where the parts clear, pivot.atLeast(Degrees.of(...)) or pivot.atMost(Degrees.of(...)), or a band around out, pivot.between(Degrees.of(...), Degrees.of(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Interlock.that(ARM).runsOnlyWhen(ELEVATOR.at(ELEVATOR_CLEAR)))
        .withMessageContaining(
            "elevator at clear holds only with elevator exactly at clear, 0.4 m, which a measured position never is, so the interlock would never hold: say the edge where the parts clear, elevator.atLeast(Inches.of(...)) or elevator.atMost(Inches.of(...)), or a band around clear, elevator.between(Inches.of(...), Inches.of(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Interlock.that(ARM)
                    .keepsOut(Range.degrees(130, 140))
                    .when(ELEVATOR.atMost(ELEVATOR_CLEAR)))
        .withMessage("130° to 140° is outside arm's range, -30° to 120°");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Interlock.that(ARM)
                    .keepsOut(Range.degrees(-40, 130))
                    .when(ELEVATOR.atMost(ELEVATOR_CLEAR)))
        .withMessage(
            "-40° to 130° is all of arm's range, -30° to 120°: it would have" + " nowhere to be");
  }

  /**
   * An edge is a number, as a clearance usually is, or one of the joint's places when it really is
   * one: judged the same, shown as it was written. A speed restricts by how fast another joint
   * runs, and joints holding each other through a speed are a deadlock all the same.
   */
  @Test
  void anEdgeIsANumberOrAPlaceAndASpeedIsHowFastAnotherRuns() {
    Condition.AtLeast byNumber = PIVOT.atLeast(Degrees.of(110));
    Condition.AtLeast byPlace = PIVOT.atLeast(PIVOT_RUN);
    Condition.AtMost low = ELEVATOR.atMost(Inches.of(10));
    Condition.Between band = ARM.between(Degrees.of(45), Degrees.of(0));
    Condition.UpToSpeed spun = ROLLER.upToSpeed(RPM.of(1500));

    assertThat(byNumber.holdsAt(Math.toRadians(111))).isTrue();
    assertThat(byNumber.holdsAt(Math.toRadians(109))).isFalse();
    assertThat(byPlace.holdsAt(Math.toRadians(111))).isTrue();
    assertThat(byNumber).hasToString("pivot at least 110°");
    assertThat(byPlace).hasToString("pivot at least run");
    assertThat(low.holdsAt(Inches.of(9).in(Meters))).isTrue();
    assertThat(low.holdsAt(Inches.of(11).in(Meters))).isFalse();
    assertThat(band.holdsAt(Math.toRadians(30))).isTrue();
    assertThat(band.holdsAt(Math.toRadians(50))).isFalse();
    assertThat(spun.holdsAt(RPM.of(1600).in(RadiansPerSecond))).isTrue();
    assertThat(spun.holdsAt(RPM.of(1400).in(RadiansPerSecond))).isFalse();
    assertThat(spun.toString()).startsWith("roller at least 1500");
    assertThat(Interlock.that(PIVOT).runsOnlyWhen(spun).other()).contains(ROLLER);
    assertThat(robot(List.of(), List.of(Interlock.that(PIVOT).runsOnlyWhen(spun))).interlocks())
        .hasSize(1);

    assertThatIllegalArgumentException()
        .isThrownBy(() -> PIVOT.atLeast(Degrees.of(150)))
        .withMessageContaining("150° is outside pivot's range, 0° to 135°");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ARM.between(Degrees.of(10), Degrees.of(10)))
        .withMessageContaining("is at it");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ROLLER.upToSpeed(RPM.of(-100)))
        .withMessageContaining("must be forward");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Interlock.that(ROLLER).runsOnlyWhen(ROLLER.upToSpeed(RPM.of(100))))
        .withMessageContaining("roller can't be held by how fast it runs itself");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    List.of(),
                    List.of(
                        Interlock.that(PIVOT).runsOnlyWhen(ROLLER.upToSpeed(RPM.of(100))),
                        Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(110))))))
        .withMessageContaining(
            "as the robot starts, it leaves pivot and roller holding each other by their"
                + " interlocks, so neither can free the other: a deadlock (pivot runs only when"
                + " roller at least 100 rpm, so pivot stays at 1.5°; roller runs only when pivot at"
                + " least 110°, so roller stays still)");
  }

  /**
   * At least or at most a clearance doesn't hang on where the joint rests: at least 110° is above
   * it, for a pivot resting in or one resting out at 120°. An end of the range, where one would
   * always hold, is refused, and the definition judges them as it does the rest.
   */
  @Test
  void atLeastAndAtMostDontHangOnWhereTheJointRests() {
    Joint.Revolute restingOut =
        JointRevoluteBuilder.builder(PIVOT).rest(Math.toRadians(120)).build();
    Condition.AtLeast clear = PIVOT.atLeast(Degrees.of(110));
    Condition.AtMost low = ELEVATOR.atMost(Inches.of(10));

    assertThat(clear.holdsAt(Math.toRadians(111))).isTrue();
    assertThat(clear.holdsAt(Math.toRadians(109))).isFalse();
    assertThat(restingOut.atLeast(Degrees.of(110)).holdsAt(Math.toRadians(111))).isTrue();
    assertThat(restingOut.atLeast(Degrees.of(110)).holdsAt(Math.toRadians(109))).isFalse();
    assertThat(low.holdsAt(Inches.of(9).in(Meters))).isTrue();
    assertThat(low.holdsAt(Inches.of(11).in(Meters))).isFalse();
    assertThat(clear).hasToString("pivot at least 110°");
    assertThat(low).hasToString("elevator at most 0.254 m");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PIVOT.atLeast(Degrees.of(0)))
        .withMessage("0° is pivot's lowest: it's at least that everywhere");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PIVOT.atMost(Degrees.of(135)))
        .withMessage("135° is pivot's highest: it's at most that everywhere");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ELEVATOR.atLeast(Meters.of(2)))
        .withMessage("2 m is outside elevator's range, 0 m to 1.2 m");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    List.of(),
                    List.of(
                        Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(130))),
                        Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atMost(Degrees.of(110))))))
        .withMessageContaining(
            "roller can never run: no position of pivot has pivot at least 130° and pivot at most"
                + " 110°");
  }

  /**
   * An interlock written in the restricted joint's block is the robot's as one in its list is: both
   * enforced and checked together, so a pair split between a block and the list can still be a
   * deadlock, and one written in both is refused. A practice robot of the same joints keeps them.
   */
  @Test
  void anInterlockInABlockIsTheRobotsAsOneInItsList() {
    Joint.Revolute arm = ARM.keepsOut(Range.degrees(40, 80)).when(ELEVATOR.atMost(Inches.of(10)));
    Interlock written =
        Interlock.that(ARM).keepsOut(Range.degrees(40, 80)).when(ELEVATOR.atMost(Inches.of(10)));
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(robot(List.of(), List.of(ROLLER_ONLY_WHEN_OUT)))
            .joints(List.of(PIVOT, ROLLER, ELEVATOR, arm))
            .build();

    assertThat(arm).isEqualTo(ARM);
    assertThat(arm.ownInterlocks()).containsExactly(written);
    assertThat(robot.interlocks()).containsExactly(ROLLER_ONLY_WHEN_OUT);
    assertThat(robot.everyInterlock()).containsExactly(written, ROLLER_ONLY_WHEN_OUT);
    assertThat(robot.with(ARM.control(MechanismFixtures.stowed(20))).everyInterlock())
        .as("re-tuned, it's the joint as written, without its block's interlocks")
        .containsExactly(ROLLER_ONLY_WHEN_OUT);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(robot)
                    .interlocks(List.of(ROLLER_ONLY_WHEN_OUT, written))
                    .build())
        .withMessageContaining(
            "is written twice: once is enough, in arm's block or the robot's interlocks");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(robot)
                    .interlocks(
                        List.of(Interlock.that(ELEVATOR).runsOnlyWhen(ARM.atLeast(Degrees.of(90)))))
                    .build())
        .withMessageContaining(
            "as the robot starts, it leaves arm and elevator holding each other by their"
                + " interlocks");
    assertThat(
            RobotDefinitionBuilder.builder(robot)
                .interlocks(
                    List.of(Interlock.that(ELEVATOR).runsOnlyWhen(ARM.atLeast(Degrees.of(0)))))
                .build()
                .everyInterlock())
        .as("the arm, kept below 40°, can still pass 0° and let the elevator go")
        .hasSize(2);
  }

  /**
   * Two joints that each run only when the other has moved, neither having moved as the robot
   * starts, hold each other there for good: a deadlock, refused, saying where each stays.
   */
  @Test
  void jointsMayNotHoldEachOtherWhereTheRobotStarts() {
    Interlock pivotWaitsOnArm = Interlock.that(PIVOT).runsOnlyWhen(ARM.atLeast(ARM_UP));
    Interlock armWaitsOnPivot = Interlock.that(ARM).runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(List.of(), List.of(pivotWaitsOnArm, armWaitsOnPivot)))
        .withMessage(
            "RobotDefinition[test].interlocks: as the robot starts, it leaves pivot and arm holding"
                + " each other by their interlocks, so neither can free the other: a deadlock"
                + " (pivot runs only when arm at least up, so pivot stays at 1.5°; arm runs only when"
                + " pivot at least run, so arm stays at -20°)");
  }

  /**
   * The design's stacked arm, kept out of the bumper's band while the elevator is low, and the
   * elevator kept up while the arm hangs below -25°: the two hold each other's joints on one side
   * only, since the arm, kept below the band, can still rise to -10°, out of "below -25°", and let
   * the elevator go. No state deadlocks them, so the pair is taken.
   */
  @Test
  void anArmAndItsElevatorKeptOutOfEachOthersWayAreNoDeadlock() {
    Interlock elevatorUpWhileArmDown =
        Interlock.that(ELEVATOR).keepsOut(Range.meters(0, 0.3)).when(ARM.atMost(Degrees.of(-25)));

    RobotDefinition robot =
        robot(List.of(STOWED, AMP), List.of(ARM_CLEARS_BUMPER, elevatorUpWhileArmDown));

    assertThat(robot.interlocks()).containsExactly(ARM_CLEARS_BUMPER, elevatorUpWhileArmDown);
  }

  /**
   * Kept out of each other where each is kept to: the arm kept out of the bumper's band while the
   * elevator is low, and the elevator kept low while the arm is at least 20°, which all of the
   * arm's travel above the band is. With the arm stowed below the band, nothing holds at the start,
   * but a configuration with the arm up and the elevator down puts each where only the other could
   * free it: refused, naming it.
   */
  @Test
  void aConfigurationMayNotPutJointsWhereTheyHoldEachOther() {
    Interlock elevatorLowWhileArmUp =
        Interlock.that(ELEVATOR).keepsOut(Range.meters(0.1, 0.6)).when(ARM.atLeast(Degrees.of(20)));
    Configuration reaching =
        Configuration.of("reaching", ELEVATOR.at(ELEVATOR_DOWN), ARM.at(ARM_UP));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(List.of(STOWED, reaching), List.of(ARM_CLEARS_BUMPER, elevatorLowWhileArmUp)))
        .withMessage(
            "RobotDefinition[test].configurations: configuration reaching leaves arm and elevator"
                + " holding each other by their interlocks, so neither can free the other: a"
                + " deadlock (arm keeps out of -10° to 30° while elevator at most 0.35 m, so arm"
                + " stays between 30° and 120°; elevator keeps out of 0.1 m to 0.6 m while arm at"
                + " least 20°, so elevator stays between 0 m and 0.1 m)");
    assertThat(
            robot(List.of(STOWED), List.of(ARM_CLEARS_BUMPER, elevatorLowWhileArmUp)).interlocks())
        .as("the robot starts with the arm below 20°, so the elevator may rise")
        .hasSize(2);
  }

  /**
   * The same pair kept out of each other the other way round, the arm stowed below the band and the
   * elevator kept low while the arm is below level: as the robot starts, the arm can't cross the
   * band while the elevator is low, nor the elevator rise while the arm is below level.
   */
  @Test
  void keepOutsMayNotHoldEachOtherWhereTheRobotStarts() {
    Interlock elevatorLowWhileArmLow =
        Interlock.that(ELEVATOR).keepsOut(Range.meters(0.1, 0.6)).when(ARM.atMost(Degrees.of(0)));
    Interlock armClearsBumper =
        Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.atMost(Inches.of(32)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(List.of(), List.of(elevatorLowWhileArmLow, armClearsBumper)))
        .withMessage(
            "RobotDefinition[test].interlocks: as the robot starts, it leaves elevator and arm"
                + " holding each other by their interlocks, so neither can free the other: a"
                + " deadlock (elevator keeps out of 0.1 m to 0.6 m while arm at most 0°, so elevator"
                + " stays between 0 m and 0.1 m; arm keeps out of -10° to 30° while elevator at most"
                + " 0.8128 m, so arm stays between -30° and -10°)");
  }

  /**
   * The same pair with the elevator's region reaching its bottom stop, where it rests: the elevator
   * starts inside it, which is doubted, not refused, as an elevator at its bottom kept out of its
   * lowest stretch while the intake is in is real. With no room below, it may leave upward, past
   * 0.3 m and on above 32 inches, freeing the arm: no deadlock. A configuration still may not put
   * it there.
   */
  @Test
  void aJointThatStartsInsideARegionAtItsEndMayOnlyLeaveByTheOtherEdge() {
    Interlock elevatorLowWhileArmLow =
        Interlock.that(ELEVATOR).keepsOut(Range.meters(0, 0.3)).when(ARM.atMost(Degrees.of(0)));
    Interlock armClearsBumper =
        Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.atMost(Inches.of(32)));

    RobotDefinition robot = robot(List.of(), List.of(elevatorLowWhileArmLow, armClearsBumper));

    assertThat(doubts(robot))
        .filteredOn(doubt -> doubt.startsWith("as the robot starts"))
        .containsExactly(
            "as the robot starts, it puts elevator at 0 m, but elevator keeps out of 0 m to 0.3 m"
                + " while arm at most 0°, and arm is at -20°, where it rests: elevator may only"
                + " leave it, by its edge at 0.3 m, until that changes");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    List.of(Configuration.of("down", ELEVATOR.at(ELEVATOR_DOWN))),
                    List.of(elevatorLowWhileArmLow, armClearsBumper)))
        .withMessage(
            "RobotDefinition[test].configurations: configuration down puts elevator at 0 m, but"
                + " elevator keeps out of 0 m to 0.3 m while arm at most 0°, and arm is at -20°,"
                + " where it rests");
  }

  /** A joint's conditions to run, all on one other joint, must all hold somewhere. */
  @Test
  void aJointsConditionsToRunCanAllHold() {
    Interlock out = Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(PIVOT_OUT));
    Interlock in = Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atMost(PIVOT_RUN));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(List.of(), List.of(out, in)))
        .withMessageContaining(
            "roller can never run: no position of pivot has pivot at least out and pivot at most"
                + " run");
    assertThat(
            robot(
                    List.of(),
                    List.of(
                        out,
                        Interlock.that(ROLLER).runsOnlyWhen(PIVOT.between(PIVOT_RUN, PIVOT_OUT))))
                .interlocks())
        .hasSize(2);
  }

  /** A named position that every place of the other joint keeps a joint out of is unreachable. */
  @Test
  void everyNamedPositionCanBeReached() {
    Interlock low =
        Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.atMost(ELEVATOR_CLEAR));
    Interlock high =
        Interlock.that(ARM).keepsOut(Range.degrees(-5, 10)).when(ELEVATOR.atLeast(ELEVATOR_CLEAR));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(List.of(), List.of(low, high)))
        .withMessage(
            "RobotDefinition[test].interlocks: arm can never reach level, 0°: no position of elevator"
                + " opens it (arm keeps out of -10° to 30° while elevator at most clear; arm keeps"
                + " out of -5° to 10° while elevator at least clear)");
  }

  /**
   * A joint resting inside a region it's kept out of as the robot starts is doubted: it may only
   * leave, by the nearer edge, until the region opens.
   */
  @Test
  void aJointThatStartsInsideARegionMayOnlyLeaveIt() {
    Interlock stowedInside =
        Interlock.that(ARM).keepsOut(Range.degrees(-25, 30)).when(ELEVATOR.atMost(ELEVATOR_CLEAR));

    assertThat(doubts(robot(List.of(), List.of(stowedInside))))
        .filteredOn(doubt -> doubt.startsWith("as the robot starts"))
        .containsExactly(
            "as the robot starts, it puts arm at -20°, but arm keeps out of -25° to 30° while"
                + " elevator at most clear, and elevator is at 0 m, where it rests: arm may only leave"
                + " it, by its edge at -25°, until that changes");
  }

  /**
   * A region that reaches its joint's range end keeps the joint off that end too, with no room past
   * it to be out: a slide whose region runs to its outer stop, put there by a configuration while
   * the region is closed, is refused, naming the interlock. On the robot, the slide headed out is
   * held at the region's inner edge, so it never gets there. Its inner edge is still out.
   */
  @Test
  void aRegionReachingItsJointsEndKeepsItOffThatEnd() {
    Position in = Position.of("in", Meters.of(0));
    Position half = Position.of("half", Meters.of(0.1));
    Position out = Position.of("out", Meters.of(0.3));
    Joint.Prismatic shuttle =
        named(
            prismatic(
                "shuttle",
                Part.CHASSIS,
                point(0.1, 0, 0.3),
                new Vector3(1, 0, 0),
                Range.meters(0, 0.3),
                link("shuttle", 1, point(0, 0, 0), point(0.1, 0.3, 0.05))),
            List.of(in, half, out),
            List.of());
    Interlock inWhileLow =
        Interlock.that(shuttle)
            .keepsOut(Range.meters(0.1, 0.3))
            .when(ELEVATOR.atMost(Meters.of(0.35)));
    List<Joint> joints = List.of(ELEVATOR, shuttle);

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(definition(joints, List.of(), List.of(), List.of()))
                    .configurations(
                        List.of(
                            Configuration.of(
                                "reaching", ELEVATOR.at(ELEVATOR_DOWN), shuttle.at(out))))
                    .interlocks(List.of(inWhileLow))
                    .build())
        .withMessage(
            "RobotDefinition[test].configurations: configuration reaching puts shuttle at 0.3 m,"
                + " but shuttle keeps out of 0.1 m to 0.3 m while elevator at most 0.35 m, and"
                + " elevator is at 0 m");
    assertThat(
            RobotDefinitionBuilder.builder(definition(joints, List.of(), List.of(), List.of()))
                .configurations(
                    List.of(
                        Configuration.of("near", ELEVATOR.at(ELEVATOR_DOWN), shuttle.at(half)),
                        Configuration.of("out", ELEVATOR.at(ELEVATOR_CLEAR), shuttle.at(out))))
                .interlocks(List.of(inWhileLow))
                .build()
                .configurations())
        .as("at the region's inner edge, or out with the region open")
        .hasSize(2);
  }

  /**
   * A named speed is within what the joint's motors turn it at: refused above it, noted above nine
   * tenths of it. A joint no motors turn can't be checked, which the missing-data report says.
   */
  @Test
  void aNamedSpeedIsWithinItsFreeSpeed() {
    Joint.Revolute tooFast =
        named(
            ROLLER, List.of(), List.of(Output.rpm("intaking", 5000), Output.rpm("feeding", 1900)));
    Joint.Revolute coasting =
        named(
            revolute(
                "coasting",
                Part.CHASSIS,
                point(0, 0, 0.2),
                PITCH_UP,
                Optional.empty(),
                0,
                link("coasting", 0.5, point(0, 0, 0), point(0.05, 0.3, 0.05))),
            List.of(),
            List.of(Output.rpm("spinning", 100)));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                definition(
                    List.of(PIVOT, tooFast),
                    List.of(),
                    List.of(MechanismFixtures.motors(tooFast, Motor.KRAKEN_X60, 1, gearing(3, 1))),
                    List.of()))
        .withMessage(
            "RobotDefinition[test].joints: roller's intaking, 5000 rpm, is 2.5 times its free"
                + " speed, 2000 rpm: its motors can't turn it that fast; give it a speed under that");
    assertThat(MechanismFixtures.doubts(robot(List.of(), List.of())))
        .containsExactly(
            "roller's intaking, 2000 rpm, is 100% of its free speed, 2000 rpm: a velocity loop has"
                + " little or nothing left to hold it under load");
    RobotDefinition unturned = definition(List.of(coasting), List.of(), List.of(), List.of());
    assertThat(MechanismFixtures.doubts(unturned)).isEmpty();
    assertThat(MechanismFixtures.notes(unturned))
        .contains(
            "coasting names outputs, and no motors turn it: its speeds and voltages aren't"
                + " checked against what its motors can do");
  }

  /**
   * A profile's cruise is within what the joint's motors turn it at, as its named speeds are: past
   * it, its moves would only lag their profile. The pivot's Kraken X60 turns it at 1440°/s through
   * 25:1; the elevator's at 9.6 m/s onto its 1 in drum through 10:1.
   */
  @Test
  void aProfileCruisesWithinItsFreeSpeed() {
    Joint.Revolute geared = PIVOT.motor(Motor.KRAKEN_X60, Gearing.of(25));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                definition(
                    List.of(
                        geared.control(
                            MechanismFixtures.stowed(30)
                                .withProfile(
                                    DegreesPerSecond.of(1800),
                                    DegreesPerSecondPerSecond.of(3600)))),
                    List.of(),
                    List.of(),
                    List.of()))
        .withMessage(
            "RobotDefinition[test].joints: pivot's profile cruises at 1800°/s, 1.25 times its free"
                + " speed, 1440°/s: its motors can't turn it that fast, so its moves would lag"
                + " their profile; give it a cruise under its free speed");
    assertThat(
            definition(
                    List.of(
                        geared.control(
                            MechanismFixtures.stowed(30)
                                .withProfile(
                                    DegreesPerSecond.of(1000),
                                    DegreesPerSecondPerSecond.of(3600)))),
                    List.of(),
                    List.of(),
                    List.of())
                .joints())
        .hasSize(1);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                definition(
                    List.of(
                        ELEVATOR
                            .motor(Motor.KRAKEN_X60, Gearing.of(10), Inches.of(1))
                            .control(
                                MechanismFixtures.stowed(60)
                                    .withProfile(
                                        MetersPerSecond.of(40), MetersPerSecondPerSecond.of(80)))),
                    List.of(),
                    List.of(),
                    List.of()))
        .withMessageContaining("elevator's profile cruises at 40 m/s");
    assertThatIllegalArgumentException()
        .as("a roller is said in rpm, as its outputs are")
        .isThrownBy(
            () ->
                definition(
                    List.of(
                        PIVOT,
                        ROLLER
                            .motor(Motor.KRAKEN_X60, Gearing.of(1))
                            .control(
                                MechanismControl.velocity(0.02)
                                    .withProfile(
                                        RPM.of(7500), RotationsPerSecondPerSecond.of(200)))),
                    List.of(),
                    List.of(),
                    List.of()))
        .withMessageContaining(
            "roller's profile cruises at 7500 rpm, 1.25 times its free speed, 6000 rpm:");
  }

  /**
   * A roller's bands are shares of the speed it's asked for, so bands given as angles, meant for a
   * joint held at positions, are refused on one; and a joint held at positions has its bands in
   * angles or lengths, so shares are refused on it.
   */
  @Test
  void aRollersBandsAreSharesOfItsSpeed() {
    Joint.Revolute banded =
        ROLLER
            .motor(Motor.KRAKEN_X60, Gearing.of(3))
            .control(MechanismControl.velocity(0.02).withBands(Degrees.of(2), Degrees.of(4)));
    Joint.Revolute shared =
        PIVOT
            .motor(Motor.KRAKEN_X60, Gearing.of(25))
            .control(MechanismFixtures.stowed(30).withBands(Percent.of(3), Percent.of(6)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(List.of(PIVOT, banded), List.of(), List.of(), List.of()))
        .withMessage(
            "RobotDefinition[test].joints: roller only runs at outputs, so its bands are shares of"
                + " the speed it's asked for, not angles: withBands(Percent.of(3), Percent.of(6)) is"
                + " 3% to arrive and 6% to stay");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(List.of(shared), List.of(), List.of(), List.of()))
        .withMessage(
            "RobotDefinition[test].joints: pivot is held at positions, so its bands are angles, not"
                + " shares of a speed: withBands(Degrees.of(2), Degrees.of(4)) is 2° to arrive and"
                + " 4° to stay");
  }

  /**
   * A named voltage is within the motors' nominal voltage, which the battery gives them; a duty
   * cycle is a share of it, from -1 to 1, by itself. Most teams run their rollers on one of these.
   */
  @Test
  void aNamedVoltageIsWithinWhatTheBatteryGives() {
    Joint.Revolute pushing =
        named(
            ROLLER,
            List.of(),
            List.of(Output.volts("pushing", 13), Output.dutyCycle("eject", -0.8)));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                definition(
                    List.of(PIVOT, pushing),
                    List.of(),
                    List.of(MechanismFixtures.motors(pushing, Motor.KRAKEN_X60, 1, gearing(3, 1))),
                    List.of()))
        .withMessage(
            "RobotDefinition[test].joints: roller's pushing, 13 V, is past its motors' nominal 12"
                + " V: the battery can't give it");
    assertThat(Output.of("hold", Volts.of(-6)))
        .isEqualTo(Output.builder().name("hold").kind(Output.Kind.VOLTAGE).value(-6).build());
    assertThat(Output.dutyCycle("eject", -0.8).runs()).isTrue();
    assertThat(Output.STOPPED.runs()).isFalse();
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Output.dutyCycle("flat out", 1.2))
        .withMessage(
            "Output[flat out].value: 1.2 must be a share of the battery's voltage, from -1 to 1");
  }

  /** A joint that follows another names nothing of its own, nor is it named: its leader is. */
  @Test
  void aFollowerIsNamedThroughItsLeader() {
    Joint.Prismatic stage =
        prismatic(
            "stage",
            Part.CHASSIS,
            point(-0.1, 0, 0.1),
            UP,
            Range.meters(0, 0.6),
            link("stage", 1, point(0, 0, 0.1), point(0.1, 0.4, 0.05)));
    Joint.Prismatic named = named(stage, List.of(Position.of("half", Inches.of(10))), List.of());
    Coupling.Linear follows = linear(ELEVATOR, stage, 0.5);

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                definition(
                    List.of(ELEVATOR, named),
                    List.of(linear(ELEVATOR, named, 0.5)),
                    List.of(),
                    List.of()))
        .withMessageContaining(
            "stage follows elevator, whose moves carry it: give its places to elevator, and none to"
                + " stage");
    RobotDefinition lifted =
        definition(List.of(ELEVATOR, stage), List.of(follows), List.of(), List.of());
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(lifted)
                    .interlocks(
                        List.of(
                            Interlock.that(stage)
                                .keepsOut(Range.meters(0.1, 0.2))
                                .when(ELEVATOR.atLeast(ELEVATOR_TOP))))
                    .build())
        .withMessage(
            "RobotDefinition[test].interlocks: interlock \"stage keeps out of 0.1 m to 0.2 m while elevator"
                + " at least top\" names stage, which follows elevator: name elevator instead");
  }

  /** The intake and stacked arm, with these configurations and interlocks. */
  private static RobotDefinition robot(
      List<Configuration> configurations, List<Interlock> interlocks) {
    return RobotDefinitionBuilder.builder(definition(JOINTS, List.of(), DRIVES, List.of()))
        .configurations(configurations)
        .interlocks(interlocks)
        .build();
  }

  private static List<Interlock> interlocks() {
    return List.of(ROLLER_ONLY_WHEN_OUT, ARM_CLEARS_BUMPER);
  }

  static Joint.Revolute named(
      Joint.Revolute joint, List<Position> positions, List<Output> outputs) {
    return JointRevoluteBuilder.builder(joint).positions(positions).outputs(outputs).build();
  }

  static Joint.Prismatic named(
      Joint.Prismatic joint, List<Position> positions, List<Output> outputs) {
    return JointPrismaticBuilder.builder(joint).positions(positions).outputs(outputs).build();
  }
}
