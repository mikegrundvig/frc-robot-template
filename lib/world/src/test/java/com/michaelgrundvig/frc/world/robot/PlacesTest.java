package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.PITCH_UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.definition;
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
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

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
      Interlock.that(ROLLER).runsOnlyWhen(PIVOT.past(PIVOT_RUN));

  static final Interlock ARM_CLEARS_BUMPER =
      Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.before(ELEVATOR_CLEAR));

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

  /** A named position is inside its joint's range, and each name is the joint's once. */
  @Test
  void aJointsPlacesAreInsideItsRangeAndNamedOnce() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                named(
                    PIVOT,
                    List.of(Position.degrees("in", 1.5), Position.degrees("up", 140)),
                    List.of()))
        .withMessage("Revolute[pivot].positions: up, 140°, must be inside its range (0° to 135°)");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                named(
                    ELEVATOR,
                    List.of(Position.of("top", Inches.of(60))),
                    List.of(Output.of("top", MetersPerSecond.of(1)))))
        .withMessageContaining("Prismatic[elevator].positions: top, 1.524 m, must be inside")
        .withMessageContaining("(0 m to 1.2 m)");
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
   * "Past" a position is from it on, away from the reference, whichever side of the reference it's
   * on; "before" is the rest; "between" includes both ends; "at" is the position exactly.
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

    assertThat(PIVOT.past(PIVOT_RUN).holdsAt(Math.toRadians(120))).isTrue();
    assertThat(PIVOT.past(PIVOT_RUN).holdsAt(Math.toRadians(110))).isTrue();
    assertThat(PIVOT.past(PIVOT_RUN).holdsAt(Math.toRadians(100))).isFalse();
    assertThat(PIVOT.before(PIVOT_RUN).holdsAt(Math.toRadians(100))).isTrue();
    assertThat(PIVOT.before(PIVOT_RUN).holdsAt(Math.toRadians(110))).isFalse();
    assertThat(hanging.past(low).holdsAt(Math.toRadians(-70))).as("below its reference").isTrue();
    assertThat(hanging.past(low).holdsAt(Math.toRadians(-50))).isFalse();
    assertThat(hanging.before(low).holdsAt(Math.toRadians(-50))).isTrue();
    assertThat(PIVOT.between(PIVOT_OUT, PIVOT_RUN).holdsAt(Math.toRadians(120))).isTrue();
    assertThat(PIVOT.between(PIVOT_RUN, PIVOT_OUT).holdsAt(Math.toRadians(131))).isFalse();
    assertThat(PIVOT.at(PIVOT_OUT).holdsAt(Math.toRadians(130))).isTrue();
    assertThat(PIVOT.at(PIVOT_OUT).isOutput()).isFalse();
    assertThat(ROLLER.at(ROLLER_INTAKING).isOutput()).isTrue();
    assertThatIllegalStateException()
        .isThrownBy(() -> ROLLER.at(ROLLER_INTAKING).holdsAt(0))
        .withMessage("roller at intaking is an output, not a position");
    assertThat(ROLLER.at(Output.STOPPED).isOutput()).as("every joint's, built in").isTrue();
    assertThat(PIVOT.past(PIVOT_RUN)).hasToString("pivot past run");
    assertThat(ELEVATOR.before(ELEVATOR_CLEAR)).hasToString("elevator before clear");
    assertThat(PIVOT.between(PIVOT_RUN, PIVOT_OUT)).hasToString("pivot between run and out");
    assertThat(ARM.at(ARM_LEVEL)).hasToString("arm at level");
  }

  @Test
  void aConditionNamesTheJointsOwnPlaces() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PIVOT.past(PIVOT_IN))
        .withMessage(
            "Past[pivot].position: in is pivot's reference: everywhere is past it, and nowhere"
                + " before it");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PIVOT.before(Position.degrees("deployed", 120)))
        .withMessage("pivot has no position deployed at 120°: it has in, run, out");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PIVOT.past(Position.degrees("out", 125)))
        .as("another out, not the pivot's own")
        .withMessage("pivot's out is at 130°, not 125°: another joint's out, or another robot's");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PIVOT.between(PIVOT_RUN, PIVOT_RUN))
        .withMessage("Between[pivot].to: a joint between run and itself is at it");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PIVOT.between(PIVOT_RUN, ARM_UP))
        .withMessage("pivot has no position up at 90°: it has in, run, out");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ROLLER.at(Output.rpm("feeding", 1000)))
        .withMessage("roller has no output feeding: it has intaking, outtaking, and stopped");
  }

  /** The design's stacked arm and intake, every interlock obeyed: the definition takes them. */
  @Test
  void aRobotsConfigurationsAndInterlocksAreChecked() {
    RobotDefinition robot = robot(List.of(STOWED, AMP, INTAKING), interlocks());

    assertThat(robot.configurations()).containsExactly(STOWED, AMP, INTAKING);
    assertThat(robot.interlocks()).containsExactly(ROLLER_ONLY_WHEN_OUT, ARM_CLEARS_BUMPER);
    assertThat(AMP.joints()).containsExactly(ELEVATOR, ARM);
    assertThat(AMP).hasToString("amp (elevator at clear, arm at level)");
    assertThat(ROLLER_ONLY_WHEN_OUT).hasToString("roller runs only when pivot past run");
    assertThat(ARM_CLEARS_BUMPER)
        .hasToString("arm keeps out of -10° to 30° while elevator before clear");
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
                + " runs only when pivot past run, and pivot is at 1.5°, its reference")
        .withMessageContaining(
            "configuration low puts arm at 0°, but arm keeps out of -10° to 30° while elevator"
                + " before clear, and elevator is at 0 m, its reference");
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
   * An interlock restricts a joint by where another is: not itself, not by an output, and kept out
   * of part of its range, never all of it or none.
   */
  @Test
  void anInterlockIsAboutAnotherJointsPlace() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Interlock.that(PIVOT).runsOnlyWhen(PIVOT.past(PIVOT_RUN)))
        .withMessageContaining("pivot can't be held by where it is itself");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Interlock.that(PIVOT).runsOnlyWhen(ROLLER.at(ROLLER_INTAKING)))
        .withMessageContaining(
            "an interlock speaks of where a joint is, and intaking is roller's output");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Interlock.that(ARM)
                    .keepsOut(Range.degrees(130, 140))
                    .when(ELEVATOR.before(ELEVATOR_CLEAR)))
        .withMessage("KeepsOut[arm].region: 130° to 140° is outside arm's range, -30° to 120°");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Interlock.that(ARM)
                    .keepsOut(Range.degrees(-40, 130))
                    .when(ELEVATOR.before(ELEVATOR_CLEAR)))
        .withMessage(
            "KeepsOut[arm].region: -40° to 130° is all of arm's range, -30° to 120°: it would have"
                + " nowhere to be");
  }

  /** Two joints held by each other's interlocks can't either move first. */
  @Test
  void jointsMayNotHoldEachOther() {
    Interlock pivotWaitsOnArm = Interlock.that(PIVOT).runsOnlyWhen(ARM.past(ARM_UP));
    Interlock armWaitsOnPivot = Interlock.that(ARM).runsOnlyWhen(PIVOT.past(PIVOT_RUN));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(List.of(), List.of(pivotWaitsOnArm, armWaitsOnPivot)))
        .withMessage(
            "RobotDefinition[test].interlocks: pivot and arm hold each other by their interlocks, so none"
                + " can move first: a deadlock (pivot runs only when arm past up; arm runs only"
                + " when pivot past run)");
  }

  /** A joint's conditions to run, all on one other joint, must all hold somewhere. */
  @Test
  void aJointsConditionsToRunCanAllHold() {
    Interlock out = Interlock.that(ROLLER).runsOnlyWhen(PIVOT.past(PIVOT_OUT));
    Interlock in = Interlock.that(ROLLER).runsOnlyWhen(PIVOT.before(PIVOT_RUN));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(List.of(), List.of(out, in)))
        .withMessageContaining(
            "roller can never run: no position of pivot has pivot past out and pivot before run");
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
        Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.before(ELEVATOR_CLEAR));
    Interlock high =
        Interlock.that(ARM).keepsOut(Range.degrees(-5, 10)).when(ELEVATOR.past(ELEVATOR_CLEAR));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(List.of(), List.of(low, high)))
        .withMessage(
            "RobotDefinition[test].interlocks: arm can never reach level, 0°: no position of elevator"
                + " opens it (arm keeps out of -10° to 30° while elevator before clear; arm keeps"
                + " out of -5° to 10° while elevator past clear)");
  }

  /** The robot starts with every joint at its reference, which every interlock must allow. */
  @Test
  void theRobotStartsObeyingTheInterlocks() {
    Interlock stowedInside =
        Interlock.that(ARM).keepsOut(Range.degrees(-25, 30)).when(ELEVATOR.before(ELEVATOR_CLEAR));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(List.of(), List.of(stowedInside)))
        .withMessage(
            "RobotDefinition[test].interlocks: as the robot starts, it puts arm at -20°, but arm keeps"
                + " out of -25° to 30° while elevator before clear, and elevator is at 0 m, its"
                + " reference");
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
                + " speed, 2000 rpm: its motors can't turn it that fast");
    assertThat(robot(List.of(), List.of()).notes())
        .containsExactly(
            "roller's intaking, 2000 rpm, is 100% of its free speed, 2000 rpm: a velocity loop has"
                + " little or nothing left to hold it under load");
    RobotDefinition unturned = definition(List.of(coasting), List.of(), List.of(), List.of());
    assertThat(unturned.notes()).isEmpty();
    assertThat(unturned.missing())
        .extracting(RobotDefinition.Missing::toString)
        .contains(
            "coasting names outputs, and no motors turn it: its speeds and voltages aren't"
                + " checked against what its motors can do");
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
        .withMessageContaining("stage follows elevator: its places are elevator's to name");
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
                                .when(ELEVATOR.past(ELEVATOR_TOP))))
                    .build())
        .withMessage(
            "RobotDefinition[test].interlocks: interlock \"stage keeps out of 0.1 m to 0.2 m while elevator"
                + " past top\" names stage, which follows elevator: name elevator instead");
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
