package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Shape;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/**
 * A joint's block: everything about one mechanism in one place, its shape and places, its motors
 * and their wiring, its own sensors, and how the program drives it.
 */
class JointBlockTest {
  private static final Translation3d AXLE = new Translation3d(0.25, 0, 0.32);
  private static final Position IN = Position.degrees("in", 1.5);
  private static final Position OUT = Position.degrees("out", 130);
  private static final Output INTAKING = Output.rpm("intaking", 2000);
  private static final CanBus CANIVORE = CanBus.canivore("canivore");

  @Test
  void aPivotTurnsBetweenItsStopsFromWhereItRests() {
    Link arm = Link.box("arm", new Translation3d(0, 0, 0.2), new Translation3d(0.05, 0.5, 0.36));

    Joint.Revolute pivot =
        Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_DOWN)
            .range(Range.degrees(0, 135), IN)
            .carries(arm)
            .positions(IN, OUT);

    assertThat(pivot.origin().getTranslation()).isEqualTo(AXLE);
    assertThat(pivot.axis()).isEqualTo(Joint.PITCH_DOWN);
    assertThat(pivot.range()).contains(Range.degrees(0, 135));
    assertThat(pivot.reference()).isCloseTo(Math.toRadians(1.5), within(1e-12));
    assertThat(pivot.link()).isEqualTo(arm);
    assertThat(pivot.positions()).containsExactly(IN, OUT);
    assertThat(pivot.outputs()).isEmpty();
    assertThat(pivot.driven()).isEqualTo(Driven.NONE);
    assertThat(pivot.ownDrives()).isEmpty();
  }

  @Test
  void aRollerTurnsFreelyFromZeroAtItsOutputs() {
    Joint.Revolute roller = Joint.roller("intake", Part.CHASSIS, AXLE).outputs(INTAKING);

    assertThat(roller.range()).isEmpty();
    assertThat(roller.reference()).isZero();
    assertThat(roller.outputs()).containsExactly(INTAKING);
    assertThat(roller.link()).isEqualTo(Link.unseen("intake"));
    assertThat(roller.rests(Position.degrees("home", 10)).reference())
        .isCloseTo(Math.toRadians(10), within(1e-12));
  }

  @Test
  void anElevatorSlidesStraightUpOntoItsDrum() {
    Position clear = Position.of("clear", Inches.of(20));

    Joint.Prismatic elevator =
        Joint.elevator("elevator", Part.CHASSIS, AXLE)
            .range(Range.meters(0, 1.2), Meters.of(0))
            .positions(clear)
            .motors(
                Motor.KRAKEN_X60, Gearing.of(10), Inches.of(1), TALON_FX.can(5), TALON_FX.can(6))
            .limitSwitch(Sensor.Switch.Limit.End.LOWER, Meters.of(0));

    assertThat(elevator.axis()).isEqualTo(Joint.UP);
    assertThat(elevator.range()).contains(Range.meters(0, 1.2));
    assertThat(elevator.position("clear").value()).isCloseTo(0.508, within(1e-12));
    Drive drive = elevator.ownDrives().getFirst();
    assertThat(drive.transmission())
        .isEqualTo(
            Transmission.Drum.builder()
                .joint(elevator)
                .gearing(Gearing.of(10))
                .pitchRadius(Inches.of(1))
                .build());
    assertThat(((Actuator.Motors) drive.actuator()).count()).isEqualTo(2);
    assertThat(elevator.switchAt(Sensor.Switch.Limit.End.LOWER).label())
        .isEqualTo("elevator lower limit");
    assertThat(elevator.driven().wiring().orElseThrow().motors())
        .containsExactly(new MechanismWiring.Motor(5, false), new MechanismWiring.Motor(6, false));
  }

  /**
   * Its motors, unwired, run in the simulator: a drive under its label, and nothing to wire. On
   * controllers, the leader first, the joint's wiring is theirs, its absolute encoder read through
   * them.
   */
  @Test
  void itsMotorsAreItsOwnDrive() {
    Joint.Revolute sim =
        Joint.pivot("arm", Part.CHASSIS, AXLE, Joint.PITCH_UP)
            .range(Range.degrees(-30, 120), Degrees.of(-20))
            .motors(Motor.KRAKEN_X60, Gearing.of(60), 2);
    Joint.Revolute wired =
        sim.motors(
                Motor.KRAKEN_X60,
                Gearing.of(60),
                TALON_FX.can(CANIVORE, 20),
                TALON_FX.can(CANIVORE, 21).clockwise())
            .absoluteEncoder(
                4096,
                MechanismWiring.Cancoder.builder()
                    .id(22)
                    .offset(Rotations.of(0.12))
                    .clockwise(false)
                    .feedback(MechanismWiring.Feedback.FUSED)
                    .build());

    Drive drive = sim.ownDrives().getFirst();
    assertThat(drive.label()).isEqualTo("arm");
    assertThat(drive.actuator()).isEqualTo(new Actuator.Motors(Motor.KRAKEN_X60, 2));
    assertThat(drive.transmission())
        .isEqualTo(Transmission.Geared.builder().joint(sim).gearing(Gearing.of(60)).build());
    assertThat(sim.driven().wiring()).isEmpty();
    MechanismWiring wiring = wired.driven().wiring().orElseThrow();
    assertThat(wiring.canBus()).isEqualTo(CANIVORE);
    assertThat(wiring.controller()).isEqualTo(TALON_FX);
    assertThat(wiring.motors())
        .containsExactly(new MechanismWiring.Motor(20, false), new MechanismWiring.Motor(21, true));
    assertThat(wiring.cancoder().orElseThrow().id()).isEqualTo(22);
    Sensor.Encoder encoder = (Sensor.Encoder) wired.ownSensors().getFirst();
    assertThat(encoder.label()).isEqualTo("arm absolute encoder");
    assertThat(encoder.absolute()).isTrue();
    assertThat(encoder.countsPerTurn()).isEqualTo(4096);
    assertThat(encoder.joint()).isEqualTo(wired);
  }

  /**
   * An absolute encoder is on the axle or geared, and read any of three ways: a CANcoder through
   * the joint's TalonFX, its SPARK's data port, or a Systemcore port the robot reads, which needs
   * no motors wired.
   */
  @Test
  void anAbsoluteEncoderIsGearedAndReadAnyOfThreeWays() {
    Joint.Revolute turret =
        Joint.roller("turret", Part.CHASSIS, AXLE, Vector3.UP)
            .absoluteEncoder(Driven.AbsoluteEncoder.of(4096).ratio(92.0 / 18));
    Joint.Revolute pivot =
        Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_UP)
            .range(Range.degrees(0, 90), Degrees.of(0))
            .absoluteEncoder(
                1024,
                MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0.82)).inverted());
    MechanismWiring.PortEncoder pot =
        MechanismWiring.PortEncoder.analog(2, 10).offset(Rotations.of(-2.5));
    Joint.Revolute winch =
        Joint.roller("winch", Part.CHASSIS, AXLE)
            .absoluteEncoder(Driven.AbsoluteEncoder.of(1024).ratio(4).on(pot));

    Sensor.Encoder geared = (Sensor.Encoder) turret.ownSensors().getFirst();
    assertThat(geared.terms().getFirst().ratio()).isEqualTo(92.0 / 18);
    assertThat(geared.absolute()).isTrue();
    MechanismWiring.PortEncoder port =
        pivot.driven().absoluteEncoder().orElseThrow().port().orElseThrow();
    assertThat(port.signal()).isEqualTo(MechanismWiring.PortEncoder.Signal.DUTY_CYCLE);
    assertThat(port.read(0.25)).isCloseTo(0.57, within(1e-12));
    assertThat(port.raw(port.read(0.25))).isCloseTo(0.25, within(1e-12));
    // A ten-turn pot reads in tenths of its range: its sensor is geared a tenth as much.
    assertThat(pot.read(0.5)).isCloseTo(0.25, within(1e-12));
    assertThat(pot.raw(1.1)).isCloseTo(0.35, within(1e-12));
    Sensor.Encoder slow = (Sensor.Encoder) winch.ownSensors().getFirst();
    assertThat(slow.terms().getFirst().ratio()).isCloseTo(0.4, within(1e-12));
    assertThat(slow.countsPerTurn()).isEqualTo(10240);
    assertThat(pivot.driven().wiring()).isEmpty();
    assertThat(pivot.driven().absoluteEncoder().orElseThrow().readByController()).isFalse();
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Driven.AbsoluteEncoder.of(1024)
                    .on(MechanismWiring.PortEncoder.dutyCycle(0))
                    .on(
                        MechanismWiring.DataPortEncoder.builder()
                            .offset(Rotations.of(0))
                            .reversed(false)
                            .build()))
        .withMessageContaining("it's read one way");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Driven.AbsoluteEncoder.of(1024).ratio(0))
        .withMessageContaining("must be a ratio");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Driven.AbsoluteEncoder.of(1024).ratio(-2))
        .withMessageContaining("which way it turns is its wiring's");
    assertThatIllegalArgumentException().isThrownBy(() -> MechanismWiring.PortEncoder.dutyCycle(6));
    assertThatIllegalArgumentException().isThrownBy(() -> MechanismWiring.PortEncoder.analog(0, 0));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> MechanismWiring.PortEncoder.dutyCycle(0).offset(Rotations.of(1.5)))
        .withMessageContaining("more than its range");
  }

  /**
   * Its limit switches are its own sensors, named for its ends, each carrying its wiring: on a
   * port, or on its own controller's pin, which names the joint.
   */
  @Test
  void itsLimitSwitchesAreItsOwnSensors() {
    Joint.Revolute pivot =
        Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_DOWN)
            .range(Range.degrees(0, 135), IN)
            .limitSwitch(Sensor.Switch.Limit.End.LOWER, Degrees.of(0))
            .limitSwitch(
                Sensor.Switch.Limit.End.UPPER,
                Degrees.of(135),
                SensorWiring.Direction.FORWARD,
                SensorWiring.Polarity.NORMALLY_CLOSED);

    Sensor.Switch lower = pivot.switchAt(Sensor.Switch.Limit.End.LOWER);
    Sensor.Switch upper = pivot.switchAt(Sensor.Switch.Limit.End.UPPER);
    assertThat(lower.label()).isEqualTo("pivot lower limit");
    assertThat(lower.wiring()).isEmpty();
    assertThat(((Sensor.Switch.Limit) lower.kind()).position()).contains(0.0);
    assertThat(upper.wiring())
        .contains(
            new SensorWiring.Connection(
                new SensorWiring.ControllerLimit("pivot", SensorWiring.Direction.FORWARD),
                SensorWiring.Polarity.NORMALLY_CLOSED));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                pivot.limitSwitch(
                    Sensor.Switch.Limit.End.LOWER,
                    Degrees.of(0),
                    new SensorWiring.Port(1),
                    SensorWiring.Polarity.NORMALLY_OPEN))
        .withMessageContaining("two limit switches at its lower end");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Joint.roller("roller", Part.CHASSIS, AXLE).switchAt(Sensor.Switch.Limit.End.LOWER))
        .withMessage("roller has no limit switch at its lower end");
    Joint.Prismatic unknown =
        Joint.elevator("climber", Part.CHASSIS, AXLE)
            .range(Range.meters(0, 0.5), Meters.of(0))
            .limitSwitch(Sensor.Switch.Limit.End.LOWER)
            .limitSwitch(
                Sensor.Switch.Limit.End.UPPER,
                new SensorWiring.Port(2),
                SensorWiring.Polarity.NORMALLY_CLOSED);
    Sensor.Switch.Limit lowest =
        (Sensor.Switch.Limit) unknown.switchAt(Sensor.Switch.Limit.End.LOWER).kind();
    assertThat(lowest.position()).as("where it trips isn't known").isEmpty();
    assertThat(unknown.switchAt(Sensor.Switch.Limit.End.UPPER).wiring()).isPresent();
  }

  /** A hood's servos push between the chassis and its link, each a drive under its own label. */
  @Test
  void itsServosPushBetweenItsParentAndItsLink() {
    Actuator.Servo linear =
        Actuator.Servo.builder().from(0.168).to(0.268).force(50).speed(0.032).build();
    Joint.Revolute hood =
        Joint.pivot("hood", Part.CHASSIS, AXLE, Joint.PITCH_UP)
            .range(Range.degrees(0, 40), Degrees.of(0))
            .servo(
                "hood left",
                linear,
                new Translation3d(0.054, 0.1, -0.233),
                new Translation3d(0.054, 0.1, 0));

    Drive left = hood.ownDrives().getFirst();
    assertThat(left.label()).isEqualTo("hood left");
    assertThat(left.transmission())
        .isEqualTo(
            Transmission.Between.builder()
                .a(Part.CHASSIS)
                .onA(AXLE.plus(new Translation3d(0.054, 0.1, -0.233)))
                .b(hood)
                .onB(new Translation3d(0.054, 0.1, 0))
                .build());
  }

  /** How it's driven isn't part of what it is: a re-tuned or re-wired joint is the same joint. */
  @Test
  void howItsDrivenIsntWhatItIs() {
    Joint.Revolute pivot =
        Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_DOWN)
            .range(Range.degrees(0, 135), IN)
            .motor(Motor.KRAKEN_X60, Gearing.of(25))
            .control(MechanismControl.position(30));
    Joint.Revolute practice =
        pivot
            .motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(33))
            .control(MechanismControl.position(25));

    assertThat(practice).isEqualTo(pivot).hasSameHashCodeAs(pivot);
    assertThat(practice.driven()).isNotEqualTo(pivot.driven());
    assertThat(practice.driven().control().orElseThrow().gains().kP()).isEqualTo(25);
    assertThat(pivot.range(Range.degrees(0, 120), IN)).isNotEqualTo(pivot);
  }

  @Test
  void itRefusesWhatCantBeWired() {
    Joint.Revolute arm =
        Joint.pivot("arm", Part.CHASSIS, AXLE, Joint.PITCH_UP)
            .range(Range.degrees(0, 90), Degrees.of(0));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> arm.motors(Motor.KRAKEN_X60, Gearing.of(60), TALON_FX.can(20), SPARK_MAX.can(21)))
        .withMessageContaining("a joint's motors are on one kind of controller, on one bus");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> arm.motor(Motor.NEO, Gearing.of(60), TALON_FX.can(20)))
        .withMessageContaining("a TalonFX doesn't drive NEO");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                arm.absoluteEncoder(
                    1024,
                    MechanismWiring.DataPortEncoder.builder()
                        .offset(Rotations.of(0))
                        .reversed(false)
                        .build()))
        .withMessageContaining("its absolute encoder is wired, and its motors aren't");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                arm.driven(
                    DrivenBuilder.builder(Driven.NONE)
                        .motors(
                            Optional.of(
                                new Driven.Motors(
                                    Motor.KRAKEN_X60,
                                    1,
                                    Gearing.of(60),
                                    Optional.of(Inches.of(1)),
                                    List.of())))
                        .build()))
        .withMessageContaining("a joint that turns has no drum");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Joint.elevator("elevator", Part.CHASSIS, AXLE)
                    .driven(
                        DrivenBuilder.builder(Driven.NONE)
                            .absoluteEncoder(Optional.of(Driven.AbsoluteEncoder.of(1024)))
                            .build()))
        .withMessageContaining("one that slides has none");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                new Driven.Motors(
                    Motor.KRAKEN_X60, 2, Gearing.of(1), Optional.empty(), List.of(TALON_FX.can(1))))
        .withMessageContaining("names 1 controllers for 2 motors");
  }

  /**
   * What real robots' code needed: a roller and a slide on any axis, a rest with no stops, a second
   * robot's gearing with its controllers kept, a switch of unknown position on the joint's own pin,
   * and a rest given in degrees landing on a stop given in degrees.
   */
  @Test
  void theBlockTakesWhatRealRobotsNeeded() {
    Joint.Revolute spindexer = Joint.roller("spindexer", Part.CHASSIS, AXLE, Vector3.UP);
    Joint.Prismatic hopper = Joint.slide("hopper", Part.CHASSIS, AXLE, new Vector3(1, 0, 0));
    Joint.Revolute shoulder =
        Joint.pivot("shoulder", Part.CHASSIS, AXLE, Joint.PITCH_UP).rests(Degrees.of(104.4));
    Joint.Prismatic held = hopper.rests(Meters.of(0.1));
    Joint.Revolute shooter =
        Joint.roller("shooter", Part.CHASSIS, AXLE)
            .motors(Motor.KRAKEN_X60, Gearing.of(1), TALON_FX.can(1), TALON_FX.can(2).clockwise());
    Joint.Revolute regeared = shooter.gearing(Gearing.of(1.5));
    Joint.Prismatic topped =
        Joint.elevator("elevator", Part.CHASSIS, AXLE)
            .range(Range.meters(0, 1.2), Meters.of(0))
            .limitSwitch(
                Sensor.Switch.Limit.End.UPPER,
                SensorWiring.Direction.FORWARD,
                SensorWiring.Polarity.NORMALLY_CLOSED);

    assertThat(spindexer.axis()).isEqualTo(Vector3.UP);
    assertThat(hopper.axis()).isEqualTo(new Vector3(1, 0, 0));
    assertThat(shoulder.reference()).isCloseTo(Math.toRadians(104.4), within(1e-12));
    assertThat(held.reference()).isEqualTo(0.1);
    assertThat(regeared.driven().motors().orElseThrow().gearing()).isEqualTo(Gearing.of(1.5));
    assertThat(regeared.driven().wiring()).isEqualTo(shooter.driven().wiring());
    assertThat(topped.switchAt(Sensor.Switch.Limit.End.UPPER).wiring())
        .contains(
            new SensorWiring.Connection(
                SensorWiring.ControllerLimit.of(topped, SensorWiring.Direction.FORWARD),
                SensorWiring.Polarity.NORMALLY_CLOSED));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> spindexer.gearing(Gearing.of(2)))
        .withMessage("spindexer has no motors to gear: give it .motor(...) first");
    assertThat(
            Joint.pivot("tilt", Part.CHASSIS, AXLE, Joint.PITCH_UP)
                .range(Range.degrees(12, 90), Degrees.of(12))
                .reference())
        .as("at its stop, though 12° converts a step apart two ways")
        .isCloseTo(Math.toRadians(12), within(1e-12));
  }

  /** A place of the same name with another value is refused saying the values differ. */
  @Test
  void anotherRobotsPlaceIsRefusedNamingItsValue() {
    Position climb = Position.of("climb", Meters.of(0.0508));
    Joint.Prismatic climber =
        Joint.elevator("climber", Part.CHASSIS, AXLE)
            .range(Range.meters(0, 0.3), Meters.of(0))
            .positions(climb);

    assertThat(climber.whyNotOwn(Position.of("climb", Meters.of(0.0762))))
        .isEqualTo(
            "climber's climb is at 0.0508 m, not 0.0762 m: another joint's climb, or another"
                + " robot's");
    assertThat(climber.whyNotOwn(climb)).isNull();
  }

  @Test
  void aLinkIsUnweighedUnlessItsMassIsGiven() {
    Translation3d center = new Translation3d(0.1, 0, 0.2);
    Translation3d size = new Translation3d(0.2, 0.4, 0.05);

    Link drawn = Link.box("arm", center, size);
    Link weighed = Link.weighed("arm", Kilograms.of(3), center, size);

    assertThat(drawn.mass()).isEqualTo(MassProperties.NONE);
    assertThat(drawn.shapes()).hasSize(1);
    assertThat(((Shape.Box) drawn.shapes().get(0)).material()).isEqualTo(Material.ALUMINUM);
    assertThat(Link.unseen("arm").shapes()).isEmpty();
    assertThat(weighed.mass().mass().in(Kilograms)).isEqualTo(3);
    assertThat(weighed.mass().cg()).isEqualTo(center);
    assertThat(weighed.shapes()).isEqualTo(drawn.shapes());
  }

  @Test
  void aGearingLosesNothingUntilMeasured() {
    assertThat(Gearing.of(25)).isEqualTo(new Gearing(25, 1));
    assertThat(List.of(Joint.PITCH_UP, Joint.PITCH_DOWN)).doesNotContain(Joint.UP);
  }
}
