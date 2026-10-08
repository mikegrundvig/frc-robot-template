package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_FLEX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.tuple;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/**
 * The definition's checks of each joint's block against itself and the robot it's on: its control
 * against its range, its places and its sensors; its absolute encoder against its motors'
 * controllers; and where its switches are connected. Each a refusal as the definition is built,
 * wherever it's used, or, for what only the program needs, a refusal the program asks for.
 */
class BlocksTest {
  static final Position IN = Position.degrees("in", 1.5);
  static final Position RUN = Position.degrees("run", 110);
  static final Position OUT = Position.degrees("out", 130);
  static final Position DOWN = Position.of("down", Inches.of(0));
  static final Position TOP = Position.of("top", Inches.of(1.1 / 0.0254));
  static final Output INTAKING = Output.rpm("intaking", 1500);

  /** Held at positions with no absolute encoder: stowed by hand at power on. */
  static final MechanismControl HELD =
      MechanismControl.position(30).withHoming(MechanismControl.Homing.assumeAtRest());

  /** An intake's pivot, its stops at 0° and 135°: in at 1.5°, run from 110°, out at 130°. */
  static final Joint.Revolute PIVOT =
      Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.2, 0, 0.3), Joint.PITCH_UP)
          .range(Range.degrees(0, 135))
          .rest(IN)
          .positions(IN, RUN, OUT)
          .motor(Motor.KRAKEN_X60, Gearing.of(25))
          .control(HELD);

  /** Its roller, only run at its outputs. */
  static final Joint.Revolute ROLLER =
      Joint.roller("roller", PIVOT, new Translation3d(0.3, 0, 0))
          .outputs(INTAKING)
          .motor(Motor.KRAKEN_X60, Gearing.of(3))
          .control(MechanismControl.velocity(0.02));

  /** An elevator, 0 to 1.2 m, its top at 1.1 m. */
  static final Joint.Prismatic ELEVATOR =
      Joint.elevator("elevator", Part.CHASSIS, new Translation3d(-0.2, 0, 0.1))
          .range(Range.meters(0, 1.2))
          .rest(DOWN)
          .positions(DOWN, TOP)
          .motors(Motor.KRAKEN_X60, Gearing.of(10), Inches.of(1), 2)
          .control(
              MechanismControl.position(60).withHoming(MechanismControl.Homing.assumeAtRest()));

  /** A turret that turns freely, facing front. */
  static final Joint.Revolute TURRET =
      Joint.pivot("turret", Part.CHASSIS, new Translation3d(0, 0, 0.5), Vector3.UP)
          .positions(Position.degrees("front", 0))
          .motor(Motor.KRAKEN_X60, Gearing.of(50))
          .control(HELD);

  /**
   * Soft limits past the hard stops are refused, saying where both are; a named position outside
   * them too, since the joint is never driven there. Soft limits around every place are fine.
   */
  @Test
  void softLimitsAreInsideTheStopsAndAroundEveryPlace() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    PIVOT.retuned(
                        control ->
                            control.withSoftLimits(Range.of(Radians.of(-0.1), Radians.of(2.0))))))
        .withMessage(
            "RobotDefinition[test].joints: pivot's soft limits, -5.73° to 114.6°, must be inside"
                + " its hard stops, 0° to 135°\n"
                + "RobotDefinition[test].joints: pivot's out, 130°, is outside its soft limits,"
                + " -5.73° to 114.6°: it's never driven there");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    PIVOT.retuned(
                        control ->
                            control.withSoftLimits(Range.of(Radians.of(0), Radians.of(2.0))))))
        .withMessage(
            "RobotDefinition[test].joints: pivot's out, 130°, is outside its soft limits, 0° to"
                + " 114.6°: it's never driven there");
    assertThat(
            robot(
                    PIVOT.retuned(
                        control ->
                            control.withSoftLimits(Range.of(Radians.of(0), Radians.of(2.3)))))
                .joints())
        .hasSize(1);
  }

  /** A slide's limits read in meters. */
  @Test
  void aSlidesLimitsReadInMeters() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> robot(ELEVATOR.retuned(control -> control.withSoftLimits(Range.meters(0, 1.0)))))
        .withMessage(
            "RobotDefinition[test].joints: elevator's top, 1.1 m, is outside its soft limits, 0 m"
                + " to 1 m: it's never driven there");
  }

  /**
   * A homing recipe the joint can carry out: a roller has no position to find, and a joint that
   * turns freely no hard stop.
   */
  @Test
  void aHomingRecipeFitsItsJoint() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    PIVOT,
                    ROLLER.retuned(
                        control -> control.withHoming(MechanismControl.Homing.assumeAtRest()))))
        .withMessage(
            "RobotDefinition[test].joints: roller only runs at outputs: it has no position to"
                + " home");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    TURRET.retuned(
                        control ->
                            control.withHoming(
                                MechanismControl.Homing.toHardStop(
                                    Sensor.Switch.Limit.End.LOWER)))))
        .withMessage(
            "RobotDefinition[test].joints: turret turns freely: it has no hard stop to home to");
  }

  /**
   * A control is written for its joint's loop, by its factory: its kP is volts per that loop's
   * error, a position's or a speed's, so the other factory's is refused, saying the right one and
   * its kP's units in the joint's kind.
   */
  @Test
  void aControlIsWrittenForItsJointsLoop() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(PIVOT, ROLLER.retuned(old -> MechanismControl.position(30))))
        .withMessage(
            "RobotDefinition[test].joints: roller only runs at outputs (it has no range and no"
                + " named positions), and its control is MechanismControl.position(...), for a"
                + " joint held at positions: write MechanismControl.velocity(kP), its kP in volts"
                + " per radian per second of error, or per turn per second,"
                + " Volts.per(RotationsPerSecond).of(...)");
    MechanismControl.Homing stowed = MechanismControl.Homing.assumeAtRest();
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> robot(PIVOT.retuned(old -> MechanismControl.velocity(0.02).withHoming(stowed))))
        .withMessage(
            "RobotDefinition[test].joints: pivot is held at positions (it has a range or named"
                + " positions), and its control is MechanismControl.velocity(...), for a roller"
                + " run at speeds: write MechanismControl.position(kP), its kP in volts per radian"
                + " of error, or per turn, Volts.per(Rotation).of(...)");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> robot(ELEVATOR.retuned(old -> MechanismControl.velocity(60).withHoming(stowed))))
        .withMessageContaining(
            "write MechanismControl.position(kP), its kP in volts per meter of error, or per inch,"
                + " Volts.per(Inch).of(...)");
    assertThat(robot(PIVOT, ROLLER).joints()).hasSize(2);
  }

  /**
   * A joint held at positions knows where it is at power on: from its absolute encoder, its block's
   * or one of the robot's, read on a robot where its motors are wired; or by the recipe its control
   * names. Without either, it's refused, saying what to give.
   */
  @Test
  void aJointHeldAtPositionsHomesByItsEncoderOrARecipe() {
    MechanismControl unhomed = MechanismControl.position(30);

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(PIVOT.retuned(control -> unhomed)))
        .withMessage(
            "RobotDefinition[test].joints: pivot has no absolute encoder, so where it is at power"
                + " on is a guess: give its control a homing recipe, Homing.toHardStop,"
                + " Homing.toSwitch or Homing.assumeAtRest()");
    assertThat(robot(PIVOT.absoluteEncoder().retuned(control -> unhomed)).joints())
        .as("in the simulator alone, its encoder unwired")
        .hasSize(1);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    PIVOT.motorsOn(TALON_FX.can(32)).absoluteEncoder().retuned(control -> unhomed)))
        .withMessage(
            "RobotDefinition[test].joints: pivot homes by its absolute encoder, and its wiring"
                + " names no CANcoder, data port or Systemcore port to read it on: wire it in its"
                + " block, as a CANcoder on its TalonFX's bus,"
                + " .absoluteEncoder(CANCODER.can(...).offset(...)), or give its control a homing"
                + " recipe");
    assertThatIllegalArgumentException()
        .as("a SPARK reads no CANcoder: its data port, as a through-bore on a SPARK Flex")
        .isThrownBy(
            () ->
                robot(
                    unmotored(PIVOT)
                        .motor(Motor.NEO_VORTEX, Gearing.of(25), SPARK_FLEX.can(32))
                        .absoluteEncoder()
                        .retuned(control -> unhomed)))
        .withMessage(
            "RobotDefinition[test].joints: pivot homes by its absolute encoder, and its wiring"
                + " names no CANcoder, data port or Systemcore port to read it on: wire it in its"
                + " block, on its SPARK Flex's data port, .absoluteEncoder(DATA_PORT.offset(...)),"
                + " or give its control a homing recipe");
    assertThat(
            robot(
                    PIVOT
                        .motorsOn(TALON_FX.can(32))
                        .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.2)))
                        .retuned(control -> unhomed))
                .joints())
        .hasSize(1);
    assertThat(
            RobotDefinitionBuilder.builder(robot(PIVOT))
                .joints(List.of(PIVOT.retuned(control -> unhomed)))
                .sensors(List.of(Sensor.Encoder.absolute("pivot absolute", PIVOT, 8192)))
                .build()
                .joints())
        .as("one of the robot's sensors on it")
        .hasSize(1);
  }

  /**
   * A joint homed to a switch has one at that end whose position is known, its block's or one of
   * the robot's.
   */
  @Test
  void homingToASwitchNeedsOneWhosePositionIsKnown() {
    Joint.Revolute homing =
        PIVOT.retuned(
            control ->
                control.withHoming(
                    MechanismControl.Homing.toSwitch(Sensor.Switch.Limit.End.LOWER)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(homing))
        .withMessage(
            "RobotDefinition[test].joints: pivot homes to its lower limit switch, and its block"
                + " has none: give it one where it trips, .limitSwitch(End.LOWER, Degrees.of(...),"
                + " ...)");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(homing.limitSwitch(Sensor.Switch.Limit.End.LOWER)))
        .withMessage(
            "RobotDefinition[test].joints: pivot homes to pivot lower limit, whose position isn't"
                + " known: measure where it trips and give it, .limitSwitch(End.LOWER,"
                + " Degrees.of(...), ...), or home another way");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(robot(PIVOT))
                    .joints(List.of(homing))
                    .sensors(
                        List.of(
                            Sensor.Switch.limit(
                                "pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER)))
                    .build())
        .withMessage(
            "RobotDefinition[test].joints: pivot homes to pivot down, whose position isn't known:"
                + " measure where it trips and give it, .limitSwitch(End.LOWER, Degrees.of(...),"
                + " ...), or home another way");
    assertThat(robot(homing.limitSwitch(Sensor.Switch.Limit.End.LOWER, Degrees.of(0))).joints())
        .hasSize(1);
  }

  /**
   * A joint's loop closes on its absolute encoder only where the encoder tells it where it is
   * across its whole range: one held at positions, without a homing recipe, with a range, which the
   * definition checks one of the encoder's turns holds. A roller, a recipe and a joint that turns
   * freely are each refused, saying why; a SPARK's data port the same as a Systemcore port.
   */
  @Test
  void aLoopClosesOnAnEncoderOnlyWhereItTellsTheWholeRange() {
    MechanismWiring.PortEncoder closing = MechanismWiring.PortEncoder.dutyCycle(1).closesLoop();
    MechanismControl unhomed = MechanismControl.position(30);
    Joint.Revolute onSpark =
        unmotored(PIVOT)
            .motor(Motor.NEO, Gearing.of(25), SPARK_MAX.can(23))
            .absoluteEncoder(DATA_PORT.offset(Rotations.of(0)).closesLoop());

    assertThat(robot(PIVOT.absoluteEncoder(closing).retuned(control -> unhomed)).joints())
        .hasSize(1);
    assertThat(robot(onSpark.retuned(control -> unhomed)).joints()).hasSize(1);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(PIVOT.absoluteEncoder(closing)))
        .withMessage(
            "RobotDefinition[test].joints: pivot's loop closes on its absolute encoder, which tells"
                + " it where it is: drop its control's homing recipe");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(onSpark))
        .withMessageEndingWith("drop its control's homing recipe");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(PIVOT, ROLLER.absoluteEncoder(closing)))
        .withMessage(
            "RobotDefinition[test].joints: roller only runs at outputs, so its loop has no"
                + " position to close on its absolute encoder: drop .closesLoop() from its encoder's"
                + " wiring");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(TURRET.absoluteEncoder(closing).retuned(control -> unhomed)))
        .withMessage(
            "RobotDefinition[test].joints: turret turns freely, and its absolute encoder's reading"
                + " wraps each turn: its loop can't close on it yet, so let the encoder tell its"
                + " motor where it is instead");
  }

  /**
   * An elevator's absolute encoder turns with its drum, so one of its turns must cover the
   * elevator's travel: 1.2 m on a 1 in drum is 7.52 of the drum's turns, so one on the drum's shaft
   * is refused, saying the ratio that fits, and one geared nine to one down from it, 0.84 of a
   * turn, is taken, its loop closing on it too. A ten-turn potentiometer on the shaft covers it; a
   * five-turn one doesn't. Its readings must stay within a turn of 0, where it reads at the
   * elevator's zero, so one whose zero is far below its travel is refused, saying so. It needs its
   * block's motors to say the drum, and its stops to bound its travel. A turret's on its axle,
   * which turns more than once over its 400°, is refused the same way, in its own turns.
   */
  @Test
  void anElevatorsEncoderCoversItsTravelInOneTurn() {
    MechanismControl unhomed = MechanismControl.position(60);
    Driven.AbsoluteEncoder ninth = Driven.AbsoluteEncoder.of().ratio(1.0 / 9);

    assertThat(robot(ELEVATOR.absoluteEncoder(ninth).retuned(control -> unhomed)).joints())
        .hasSize(1);
    assertThat(
            robot(
                    ELEVATOR
                        .absoluteEncoder(
                            ninth.on(MechanismWiring.PortEncoder.dutyCycle(1).closesLoop()))
                        .retuned(control -> unhomed))
                .joints())
        .as("its loop closed on it")
        .hasSize(1);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(ELEVATOR.absoluteEncoder().retuned(control -> unhomed)))
        .withMessage(
            "RobotDefinition[test].joints: elevator's absolute encoder turns 7.519 times over its"
                + " travel, 0 m to 1.2 m: one of its turns must cover the travel, with a fortieth"
                + " of it to spare at each end, to tell where elevator is. Gear it to at most 0.126"
                + " of its turns for each turn of elevator's drum, or leave it out and give"
                + " elevator's control a homing recipe");
    assertThat(
            robot(
                    ELEVATOR
                        .absoluteEncoder(MechanismWiring.PortEncoder.analog(2, 10))
                        .retuned(control -> unhomed))
                .joints())
        .as("a ten-turn potentiometer on the drum's shaft")
        .hasSize(1);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    ELEVATOR
                        .absoluteEncoder(MechanismWiring.PortEncoder.analog(2, 5))
                        .retuned(control -> unhomed)))
        .withMessage(
            "RobotDefinition[test].joints: elevator's absolute encoder turns 7.519 times over its"
                + " travel, 0 m to 1.2 m: its signal's range, 5 of its turns, must cover the"
                + " travel, with a fortieth of it to spare at each end, to tell where elevator is."
                + " Gear it to at most 0.631 of its turns for each turn of elevator's drum, or leave"
                + " it out and give elevator's control a homing recipe");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    Joint.pivot("turret", Part.CHASSIS, new Translation3d(0, 0, 0.5), Vector3.UP)
                        .range(Range.degrees(-200, 200))
                        .rest(Degrees.of(0))
                        .motor(Motor.KRAKEN_X60, Gearing.of(50))
                        .absoluteEncoder()
                        .control(MechanismControl.position(20))))
        .withMessage(
            "RobotDefinition[test].joints: turret's absolute encoder turns 1.111 times over its"
                + " travel, -200° to 200°: one of its turns must cover the travel, with a fortieth"
                + " of it to spare at each end, to tell where turret is. Gear it to at most 0.854 of"
                + " its turns for each of turret's turns, or leave it out and give turret's control"
                + " a homing recipe");
    Joint.Prismatic fromTheFloor =
        Joint.elevator("elevator", Part.CHASSIS, new Translation3d(-0.2, 0, 0))
            .range(Range.meters(0.991, 1.804))
            .rest(Meters.of(0.991))
            .motors(Motor.KRAKEN_X60, Gearing.of(10), Inches.of(1), 2)
            .absoluteEncoder(ninth)
            .control(unhomed);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(fromTheFloor))
        .withMessage(
            "RobotDefinition[test].joints: elevator's absolute encoder reads 0.6899 to 1.256 turns"
                + " over its travel, 0.991 m to 1.804 m, counting from 0 at elevator's zero, and"
                + " must read within 0.975 turns of 0 all along it, where its wrap can be put. Gear"
                + " it to at most 0.0862 of its turns for each turn of elevator's drum, or move"
                + " elevator's zero, its origin, to a hard stop");
    // On its drum's shaft from the floor, both bound it, and the zero more: said, with the travel.
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    Joint.elevator("elevator", Part.CHASSIS, new Translation3d(-0.2, 0, 0))
                        .range(Range.meters(0.991, 1.804))
                        .rest(Meters.of(0.991))
                        .motors(Motor.KRAKEN_X60, Gearing.of(10), Inches.of(1), 2)
                        .absoluteEncoder()
                        .control(unhomed)))
        .withMessage(
            "RobotDefinition[test].joints: elevator's absolute encoder turns 5.094 times over its"
                + " travel, 0.991 m to 1.804 m: one of its turns must cover the travel, with a"
                + " fortieth of it to spare at each end, and it must read within 0.975 turns of 0"
                + " all along it, counting from 0 at elevator's zero, to tell where elevator is."
                + " Gear it to at most 0.0862 of its turns for each turn of elevator's drum, or"
                + " move elevator's zero, its origin, to a hard stop, or leave it out and give"
                + " elevator's control a homing recipe");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    Joint.elevator("lift", Part.CHASSIS, new Translation3d(0, 0, 0))
                        .range(Range.meters(0, 1))
                        .rest(Meters.of(0))
                        .absoluteEncoder(ninth)))
        .withMessage(
            "RobotDefinition[test].joints: lift's absolute encoder turns with its drum, and its"
                + " block has no motors to say the drum's pitch radius: give them, .motors(motor,"
                + " gearing, pitchRadius, ...)");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    Joint.slide("hopper", Part.CHASSIS, new Translation3d(0, 0, 0), Vector3.UP)
                        .motor(Motor.KRAKEN_X60, Gearing.of(10), Inches.of(1))
                        .absoluteEncoder(ninth)
                        .control(MechanismControl.velocity(0.02))))
        .withMessage(
            "RobotDefinition[test].joints: hopper has an absolute encoder and no hard stops, so"
                + " nothing says one of the encoder's turns covers its travel: give its range,"
                + " range(Range.meters(...))");
  }

  /** An encoder on its controller's data port needs a controller that has one: a TalonFX hasn't. */
  @Test
  void aTalonFxHasNoDataPort() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    PIVOT
                        .motorsOn(TALON_FX.can(23))
                        .absoluteEncoder(DATA_PORT.offset(Rotations.of(0)))
                        .retuned(control -> MechanismControl.position(30))))
        .withMessage(
            "RobotDefinition[test].joints: pivot's wiring names an encoder on its controller's"
                + " data port, and a TalonFX has none: wire it as a CANcoder");
  }

  /** A SPARK is on Systemcore's own CAN, never a CANivore, and reads no CANcoder. */
  @Test
  void aSparkIsOnSystemcoresCanAndReadsNoCancoder() {
    CanBus canivore = CanBus.canivore("upper");

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    unmotored(PIVOT).motor(Motor.NEO, Gearing.of(25), SPARK_MAX.can(canivore, 41))))
        .withMessage(
            "RobotDefinition[test].joints: pivot's SPARKs are on a CANivore: a SPARK is on"
                + " Systemcore's own CAN");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    unmotored(PIVOT)
                        .motor(Motor.NEO, Gearing.of(25), SPARK_MAX.can(41))
                        .absoluteEncoder(CANCODER.can(43).offset(Rotations.of(0)))))
        .withMessage(
            "RobotDefinition[test].joints: pivot's wiring names a CANcoder, which a SPARK doesn't"
                + " read: plug the encoder into the SPARK's data port, or give its control a homing"
                + " recipe");
  }

  /**
   * What the program needs that the definition allows: a control for each joint its motors turn,
   * whether they're its block's or a drive of the robot's, and for the drivetrain. Refusals the
   * program asks for as it's built; the definition itself, read by localization and the views,
   * takes them.
   */
  @Test
  void aJointWithMotorsAndTheDrivetrainNeedAControlToBeDriven() {
    Joint.Revolute uncontrolled =
        PIVOT.driven(DrivenBuilder.builder(PIVOT.driven()).control(Optional.empty()).build());
    Joint.Revolute bare =
        JointRevoluteBuilder.builder(uncontrolled)
            .label("bare")
            .link(Link.unseen("bare"))
            .driven(Driven.NONE)
            .build();
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(robot(uncontrolled, ELEVATOR))
            .joints(List.of(uncontrolled, ELEVATOR, bare))
            .drives(List.of(MechanismFixtures.motors(bare, Motor.KRAKEN_X60, 1, Gearing.of(10))))
            .build();

    assertThat(robot.findings())
        .filteredOn(Finding::refused)
        .extracting(Finding::subject, Finding::message)
        .containsExactly(
            tuple(
                "drivetrain",
                "the drivetrain has no control: give it its limits and gains,"
                    + " .control(DrivetrainControl.builder()...)"),
            tuple(
                "pivot",
                "pivot has motors and no control: give it .control(MechanismControl.position(kP)),"
                    + " or .velocity(kP) for one that only runs"),
            tuple(
                "bare",
                "bare has motors and no control: give it .control(MechanismControl.position(kP)),"
                    + " or .velocity(kP) for one that only runs"));
  }

  /**
   * One sensor on each input: two switches on one CANdi input are named, as is a joint's absolute
   * encoder on a Systemcore port a switch is on. A switch on a motor controller's limit input only
   * where it has one, on the joint whose switch it is: a Kraken's TalonFX has none, a joint whose
   * motors aren't wired has no controller, and a joint's limit switch on another's controller would
   * stop the other. A beam on a SPARK's limit input, or a limit switch on a Falcon's, is fine.
   */
  @Test
  void eachInputHasOneSensorAndAControllersLimitInputIsItsJoints() {
    Joint.Revolute onSpark = unmotored(PIVOT).motor(Motor.NEO, Gearing.of(25), SPARK_MAX.can(21));
    Joint.Revolute onKraken = ROLLER.motorsOn(TALON_FX.can(22));
    Sensor.Switch beam =
        Sensor.Switch.beam("claw beam", PIVOT)
            .on(
                SensorWiring.ControllerLimit.of(PIVOT, SensorWiring.Direction.FORWARD)
                    .normallyOpen());
    CanBus bus = CanBus.SYSTEMCORE;

    assertThat(
            RobotDefinitionBuilder.builder(robot(onSpark, onKraken))
                .sensors(
                    List.of(
                        beam,
                        Sensor.Switch.contact("cage", Part.CHASSIS)
                            .on(SensorWiring.port(4).normallyOpen())))
                .build()
                .sensors())
        .hasSize(2);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(robot(onSpark, onKraken))
                    .sensors(
                        List.of(
                            Sensor.Switch.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER)
                                .on(CANDI.can(bus, 31).s1().normallyClosed()),
                            Sensor.Switch.limit("pivot up", PIVOT, Sensor.Switch.Limit.End.UPPER)
                                .on(CANDI.can(bus, 31).s1().normallyClosed())))
                    .build())
        .withMessage(
            "RobotDefinition[test].sensors: pivot down and pivot up are on one input, CANdi 31's"
                + " S1: move one to an input of its own");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(robot(onSpark, onKraken, ELEVATOR))
                    .sensors(
                        List.of(
                            Sensor.Switch.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER)
                                .on(
                                    SensorWiring.ControllerLimit.of(
                                            onKraken, SensorWiring.Direction.REVERSE)
                                        .normallyClosed()),
                            Sensor.Switch.limit("pivot up", PIVOT, Sensor.Switch.Limit.End.UPPER)
                                .on(
                                    SensorWiring.ControllerLimit.of(
                                            ELEVATOR, SensorWiring.Direction.FORWARD)
                                        .normallyClosed())))
                    .build())
        .withMessage(
            "RobotDefinition[test].sensors: pivot down is pivot's limit switch, on roller's"
                + " controller, which would stop roller: wire it to pivot's\n"
                + "RobotDefinition[test].sensors: pivot down is on roller's TalonFX's limit input,"
                + " and a Kraken's TalonFX has none: put it on a CANdi\n"
                + "RobotDefinition[test].sensors: pivot up is pivot's limit switch, on elevator's"
                + " controller, which would stop elevator: wire it to pivot's\n"
                + "RobotDefinition[test].sensors: pivot up is on elevator's controller's limit"
                + " input, and elevator isn't wired: wire elevator's motors to their controllers, or"
                + " put pivot up on a Systemcore port or a CANdi");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(
                        robot(onSpark.absoluteEncoder(MechanismWiring.PortEncoder.dutyCycle(2))))
                    .sensors(
                        List.of(
                            Sensor.Switch.beam("claw beam", PIVOT)
                                .on(SensorWiring.port(2).normallyOpen())))
                    .build())
        .withMessage(
            "RobotDefinition[test].sensors: pivot absolute encoder and claw beam are on one input,"
                + " Systemcore's port 2: move one to an input of its own");
    assertThat(
            robot(
                    unmotored(PIVOT)
                        .motor(Motor.FALCON_500, Gearing.of(25), TALON_FX.can(24))
                        .limitSwitch(
                            Sensor.Switch.Limit.End.LOWER,
                            Degrees.of(0),
                            SensorWiring.Direction.REVERSE.normallyClosed()))
                .joints())
        .as("a Falcon's TalonFX has its limit inputs")
        .hasSize(1);
  }

  /** The test robot with these joints, each with its block. */
  private static RobotDefinition robot(Joint... joints) {
    List<Joint> all = new ArrayList<>(List.of(joints));
    return RobotDefinitionBuilder.builder(ConfigFixtures.definition(List.of())).joints(all).build();
  }
}
