package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Inch;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.InchesPerSecond;
import static org.wpilib.units.Units.InchesPerSecondPerSecond;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Rotation;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

/** How a mechanism is driven is checked when it's written down. */
class MechanismControlTest {
  @Test
  void seededGainsLeaveTheFeedforwardToTheDefinition() {
    MechanismControl.Gains gains = MechanismControl.Gains.seeded();

    assertThat(gains.kV()).isEmpty();
    assertThat(gains.kA()).isEmpty();
    assertThat(gains.kG()).isEmpty();
    assertThat(gains.kP()).isZero();
  }

  @Test
  void aGainIsNeverNegativeButKgMayBe() {
    assertThatThrownBy(
            () ->
                MechanismControl.Gains.builder()
                    .kP(-1)
                    .kI(0)
                    .kD(0)
                    .kS(0)
                    .kV(-0.1)
                    .kG(Double.NaN)
                    .build())
        .hasMessage(
            "Gains.kP: -1 must be zero or more\n"
                + "Gains.kV: -0.1 must be zero or more\n"
                + "Gains.kG: NaN must be a finite number");
    assertThat(
            MechanismControl.Gains.builder()
                .kP(0)
                .kI(0)
                .kD(0)
                .kS(0)
                .kG(-0.4) // a counterbalance stronger than the arm
                .build()
                .kG())
        .contains(-0.4);
  }

  @Test
  void softLimitsGoUp() {
    assertThatThrownBy(() -> MechanismControl.SoftLimits.builder().lower(1).upper(1).build())
        .hasMessage("SoftLimits.upper: 1 must be above the lower limit (1)");
  }

  @Test
  void aProfileMoves() {
    assertThatThrownBy(
            () -> MechanismControl.Profile.builder().cruiseVelocity(0).acceleration(-1).build())
        .hasMessage(
            "Profile.cruiseVelocity: 0 must be positive\nProfile.acceleration: -1 must be positive");
  }

  @Test
  void theSupplyLimitStepsDownWithinPhoenixsFiveSeconds() {
    assertThatThrownBy(
            () ->
                MechanismControl.CurrentLimits.builder()
                    .stator(Amps.of(40))
                    .supply(Amps.of(40))
                    .supplyLower(Amps.of(30))
                    .supplyLowerTime(Seconds.of(6))
                    .build())
        .hasMessage("CurrentLimits.supplyLowerTime: 6 s must be at most 5 s");
  }

  /** In a line: the feedforward seeded, the default limits, braking; a flywheel coasts. */
  @Test
  void aJointsControlStartsFromTheDefaults() {
    MechanismControl arm = MechanismControl.position(30);
    MechanismControl roller = MechanismControl.velocity(0.02);

    assertThat(arm.gains().kP()).isEqualTo(30);
    assertThat(arm.gains().kV()).as("seeded from the definition").isEmpty();
    assertThat(arm.gains().kG()).isEmpty();
    assertThat(arm.profile()).isEmpty();
    assertThat(arm.currentLimits()).isEqualTo(MechanismControl.CurrentLimits.DEFAULTS);
    assertThat(arm.neutralMode()).isEqualTo(MechanismControl.NeutralMode.BRAKE);
    assertThat(arm.homing()).isEmpty();
    assertThat(roller.neutralMode()).isEqualTo(MechanismControl.NeutralMode.BRAKE);
    assertThat(roller.coasting().neutralMode()).isEqualTo(MechanismControl.NeutralMode.COAST);
    assertThat(MechanismControl.CurrentLimits.DEFAULTS.stator()).isEqualTo(Amps.of(40));
  }

  /**
   * Each change sets its one value and leaves the rest: every value the pit tunes and SysId
   * measures has one.
   */
  @Test
  void eachChangeSetsOneValue() {
    MechanismControl arm =
        MechanismControl.position(30)
            .withKi(0.1)
            .withKd(1)
            .withKs(0.2)
            .withKv(2.1)
            .withKa(0.05)
            .withProfile(RadiansPerSecond.of(8), RadiansPerSecondPerSecond.of(30))
            .withSoftLimits(Range.of(Radians.of(0), Radians.of(2)))
            .withStatorLimit(Amps.of(60))
            .withSupplyLimit(Amps.of(50))
            .withBands(Radians.of(0.02), Radians.of(0.05))
            .withHoming(MechanismControl.Homing.assumeAtRest());

    assertThat(arm.gains().kP()).isEqualTo(30);
    assertThat(arm.gains().kI()).isEqualTo(0.1);
    assertThat(arm.gains().kD()).isEqualTo(1);
    assertThat(arm.gains().kS()).isEqualTo(0.2);
    assertThat(arm.gains().kV()).contains(2.1);
    assertThat(arm.gains().kA()).contains(0.05);
    assertThat(arm.gains().kind()).as("plain numbers, in the joint's units").isEmpty();
    assertThat(arm.profile().orElseThrow().cruiseVelocity()).isEqualTo(8);
    assertThat(arm.profile().orElseThrow().acceleration()).isEqualTo(30);
    assertThat(arm.softLimits().orElseThrow().upper()).isEqualTo(2);
    assertThat(arm.currentLimits().stator()).isEqualTo(Amps.of(60));
    assertThat(arm.currentLimits().supply()).isEqualTo(Amps.of(50));
    assertThat(arm.currentLimits().supplyLower()).isEqualTo(Amps.of(40));
    assertThat(arm.bands().orElseThrow().enter()).isEqualTo(0.02);
    assertThat(arm.bands().orElseThrow().stay()).isEqualTo(0.05);
    assertThat(MechanismControl.velocity(0.02).withBands(Percent.of(3), Percent.of(6)).bands())
        .as("a roller's, shares of its speed")
        .contains(MechanismControl.Bands.builder().enter(0.03).stay(0.06).build());
    assertThatIllegalArgumentException()
        .isThrownBy(() -> arm.withBands(Degrees.of(4), Degrees.of(2)))
        .withMessage(
            "a band to stay within, 2°, is at least the band to arrive within, 4°: give 4° or more,"
                + " so it leaves only past where it arrived");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> arm.withBands(Percent.of(0), Percent.of(6)))
        .withMessage("a band to arrive within is more than none, not 0%");
    assertThat(arm.homing()).contains(MechanismControl.Homing.assumeAtRest());
  }

  /**
   * A value given with its unit is held in the joint's, and remembers whether it's an angle or a
   * length: a kP per turn of the joint, as Phoenix's slot holds it, is 2π times its kP per radian;
   * an elevator's per inch, 0.0254 times its kP per meter.
   */
  @Test
  void aValueGivenWithItsUnitIsHeldInTheJoints() {
    double turn = 2 * Math.PI;
    MechanismControl arm =
        MechanismControl.position(Volts.per(Rotation).of(turn * 30))
            .withKd(Volts.per(RotationsPerSecond).of(turn))
            .withKv(Volts.per(RotationsPerSecond).of(turn * 2))
            .withKa(Volts.per(RotationsPerSecondPerSecond).of(turn * 0.05))
            .withProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(360))
            .withSoftLimits(Range.degrees(5, 130))
            .withBands(Degrees.of(1), Degrees.of(2));
    MechanismControl elevator =
        MechanismControl.position(Volts.per(Inch).of(2.54))
            .withProfile(InchesPerSecond.of(50), InchesPerSecondPerSecond.of(100))
            .withSoftLimits(Range.of(Inches.of(1), Inches.of(40)))
            .withBands(Inches.of(0.5), Inches.of(1));
    MechanismControl roller =
        MechanismControl.velocity(Volts.per(RotationsPerSecond).of(turn * 0.02));

    assertThat(arm.gains().kP()).isCloseTo(30, within(1e-9));
    assertThat(arm.gains().kD()).isCloseTo(1, within(1e-9));
    assertThat(arm.gains().kV().orElseThrow()).isCloseTo(2, within(1e-9));
    assertThat(arm.gains().kA().orElseThrow()).isCloseTo(0.05, within(1e-9));
    assertThat(arm.gains().kind()).contains(Position.Kind.ANGLE);
    assertThat(arm.profile().orElseThrow().cruiseVelocity()).isCloseTo(Math.PI, within(1e-9));
    assertThat(arm.profile().orElseThrow().acceleration()).isCloseTo(2 * Math.PI, within(1e-9));
    assertThat(arm.profile().orElseThrow().kind()).contains(Position.Kind.ANGLE);
    assertThat(arm.softLimits().orElseThrow().upper()).isCloseTo(Math.toRadians(130), within(1e-9));
    assertThat(arm.softLimits().orElseThrow().kind()).contains(Position.Kind.ANGLE);
    assertThat(arm.bands().orElseThrow().stay()).isCloseTo(Math.toRadians(2), within(1e-9));
    assertThat(arm.bands().orElseThrow().kind()).contains(Position.Kind.ANGLE);
    assertThat(elevator.gains().kP()).isCloseTo(100, within(1e-9));
    assertThat(elevator.gains().kind()).contains(Position.Kind.LENGTH);
    assertThat(elevator.profile().orElseThrow().cruiseVelocity()).isCloseTo(1.27, within(1e-9));
    assertThat(elevator.softLimits().orElseThrow().lower()).isCloseTo(0.0254, within(1e-9));
    assertThat(elevator.bands().orElseThrow().enter()).isCloseTo(0.0127, within(1e-9));
    assertThat(elevator.bands().orElseThrow().kind()).contains(Position.Kind.LENGTH);
    assertThat(roller.gains().kP()).isCloseTo(0.02, within(1e-9));
    assertThat(roller.gains().kind()).contains(Position.Kind.ANGLE);
  }

  /**
   * A gain given with its unit is per what it takes, a position, a speed or an acceleration, and
   * per the same kind as the control's other gains: one control drives one joint.
   */
  @Test
  void aGainIsPerWhatItTakes() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> MechanismControl.position(Volts.per(RotationsPerSecond).of(1)))
        .withMessage(
            "Gains.kP: volts per Rotation per Second can't be one: give it per an angle,"
                + " Volts.per(Rotation), or a length, Volts.per(Meter)");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> MechanismControl.velocity(Volts.per(Rotation).of(1)))
        .withMessage(
            "Gains.kP: volts per Rotation can't be one: give it per a speed,"
                + " Volts.per(RotationsPerSecond) or Volts.per(MetersPerSecond)");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> MechanismControl.position(30).withKa(Volts.per(Seconds).of(1)))
        .withMessage(
            "Gains.kA: volts per Second can't be one: give it per an acceleration,"
                + " Volts.per(RotationsPerSecondPerSecond) or"
                + " Volts.per(MetersPerSecondPerSecond)");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                MechanismControl.position(Volts.per(Rotation).of(188))
                    .withKd(Volts.per(MetersPerSecond).of(1)))
        .withMessage(
            "Gains.kD: it's per a length, and the other gains are per an angle: a control drives"
                + " one joint, which turns or slides");
    assertThat(
            MechanismControl.velocity(0.02)
                .withKd(Volts.per(RotationsPerSecondPerSecond).of(0.001))
                .gains()
                .kind())
        .as("a joint run at speeds takes a kD per acceleration")
        .contains(Position.Kind.ANGLE);
  }

  /**
   * A control remembers the loop its factory was written for, and a gain that's per its loop's
   * error, kP and kD, given with its unit, is per what that loop takes: kP per a position or a
   * speed, kD per a speed or an acceleration.
   */
  @Test
  void aControlRemembersItsLoop() {
    double turn = 2 * Math.PI;
    assertThat(MechanismControl.position(30).loop()).isEqualTo(MechanismControl.Loop.POSITION);
    assertThat(MechanismControl.position(Volts.per(Rotation).of(188)).loop())
        .isEqualTo(MechanismControl.Loop.POSITION);
    assertThat(MechanismControl.velocity(0.02).loop()).isEqualTo(MechanismControl.Loop.VELOCITY);
    assertThat(MechanismControl.velocity(Volts.per(RotationsPerSecond).of(0.12)).loop())
        .isEqualTo(MechanismControl.Loop.VELOCITY);
    assertThat(MechanismControl.position(30).withKp(25).gains().kP()).isEqualTo(25);
    assertThat(MechanismControl.position(30).withKp(Volts.per(Rotation).of(turn * 25)).gains())
        .extracting(MechanismControl.Gains::kP, MechanismControl.Gains::kind)
        .containsExactly(25.0, Optional.of(Position.Kind.ANGLE));
    assertThat(
            MechanismControl.velocity(0.02)
                .withKp(Volts.per(RotationsPerSecond).of(turn * 0.03))
                .gains()
                .kP())
        .isCloseTo(0.03, within(1e-12));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> MechanismControl.velocity(0.02).withKp(Volts.per(Rotation).of(1)))
        .withMessage(
            "Gains.kP: volts per Rotation can't be one in a control for speeds: give it per a"
                + " speed, Volts.per(RotationsPerSecond) or Volts.per(MetersPerSecond)");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                MechanismControl.position(30).withKd(Volts.per(RotationsPerSecondPerSecond).of(1)))
        .withMessageStartingWith("Gains.kD: volts per ")
        .withMessageEndingWith(
            " can't be one in a control for positions: give it per a speed,"
                + " Volts.per(RotationsPerSecond) or Volts.per(MetersPerSecond)");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> MechanismControl.velocity(0.02).withKd(Volts.per(RotationsPerSecond).of(1)))
        .withMessage(
            "Gains.kD: volts per Rotation per Second can't be one in a control for speeds: give it"
                + " per an acceleration, Volts.per(RotationsPerSecondPerSecond) or"
                + " Volts.per(MetersPerSecondPerSecond)");
  }

  /**
   * A joint refuses a control whose values were given as the other kind, each saying how to give
   * it: an elevator's soft limits in degrees, an arm's profile in inches per second.
   */
  @Test
  void aJointRefusesAControlOfTheOtherKind() {
    Joint.Prismatic elevator =
        MechanismFixtures.prismatic(
            "elevator",
            Part.CHASSIS,
            MechanismFixtures.point(0, 0, 0.1),
            MechanismFixtures.UP,
            Range.meters(0, 1.2),
            Link.unseen("carriage"));
    Joint.Revolute arm =
        MechanismFixtures.revolute(
            "arm",
            Part.CHASSIS,
            MechanismFixtures.point(0, 0, 0.5),
            MechanismFixtures.PITCH_UP,
            Optional.of(Range.degrees(0, 135)),
            0,
            Link.unseen("arm"));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                elevator.control(
                    MechanismControl.position(Volts.per(Rotation).of(188))
                        .withSoftLimits(Range.degrees(5, 130))))
        .withMessage(
            "Prismatic[elevator].driven.control: its gains are per angle, and the joint slides:"
                + " give them per length, Volts.per(Meter) or Volts.per(Inch); a gain per turn of"
                + " its drum is that per the drum's circumference; its soft limits are angles, and"
                + " the joint slides: give them in lengths, Range.meters(...) or"
                + " Range.of(Inches.of(...), Inches.of(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                arm.control(
                    MechanismControl.position(30)
                        .withProfile(InchesPerSecond.of(50), InchesPerSecondPerSecond.of(100))
                        .withBands(Inches.of(1), Inches.of(2))))
        .withMessage(
            "Revolute[arm].driven.control: its profile is in lengths, and the joint turns: give it"
                + " in angles, DegreesPerSecond.of(...) and DegreesPerSecondPerSecond.of(...); its"
                + " bands are lengths, and the joint turns: give them in angles, Degrees.of(...)");
    assertThat(
            arm.control(MechanismControl.position(30).withSoftLimits(Range.degrees(5, 130)))
                .driven()
                .control())
        .isPresent();
  }

  /**
   * A tuned kG, alone, or with where an arm's weight pulls hardest: each change sets its own, and
   * kG alone leaves the angle.
   */
  @Test
  void aTunedKgMaySayWhereTheWeightPullsHardest() {
    MechanismControl level = MechanismControl.position(30).withKg(0.4);
    MechanismControl hanging = level.withKg(0.5, Degrees.of(-90));

    assertThat(level.gains().kG()).contains(0.4);
    assertThat(level.gains().gravityAngle()).isEmpty();
    assertThat(hanging.gains().kG()).contains(0.5);
    assertThat(hanging.gains().gravityAngle()).contains(Degrees.of(-90));
    assertThat(hanging.withKg(0).gains().gravityAngle()).contains(Degrees.of(-90));
    assertThat(hanging.gains().kP()).isEqualTo(30);
    assertThatThrownBy(() -> level.withKg(0.4, Degrees.of(Double.NaN)))
        .hasMessageStartingWith("Gains.gravityAngle: ")
        .hasMessageEndingWith("must be a finite number");
  }

  /**
   * A homing's timeout left to its joint is twice the time its model crosses its travel at its
   * push, and half a second more: an elevator's range, at its free speed scaled by the push over 12
   * V; a turret that turns freely, a turn; a slide with no stops, from its lowest place to its
   * highest. One given is kept as it is, and must be positive.
   */
  @Test
  void aHomingsTimeoutLeftToItsJointComesFromItsTravel() {
    Joint.Prismatic elevator =
        MechanismFixtures.prismatic(
            "elevator",
            Part.CHASSIS,
            MechanismFixtures.point(0, 0, 0.1),
            MechanismFixtures.UP,
            Range.meters(0, 1.2),
            Link.unseen("carriage"));
    Joint.Revolute turret =
        MechanismFixtures.revolute(
            "turret",
            Part.CHASSIS,
            MechanismFixtures.point(0, 0, 0.5),
            MechanismFixtures.UP,
            Optional.empty(),
            0,
            Link.unseen("turret"));
    Joint.Prismatic slide =
        JointPrismaticBuilder.builder(elevator)
            .label("slide")
            .range(Optional.empty())
            .rest(0.2)
            .link(Link.unseen("slide"))
            .positions(
                List.of(Position.of("in", Meters.of(0.1)), Position.of("out", Meters.of(0.5))))
            .build();
    RobotMath math =
        MechanismFixtures.definition(
                List.of(elevator, turret, slide),
                List.of(),
                List.of(
                    MechanismFixtures.motors(
                        elevator,
                        Motor.KRAKEN_X60,
                        1,
                        MechanismFixtures.gearing(10, 1),
                        Inches.of(1)),
                    MechanismFixtures.motors(
                        turret, Motor.KRAKEN_X60, 1, MechanismFixtures.gearing(50, 1)),
                    MechanismFixtures.motors(
                        slide,
                        Motor.KRAKEN_X60,
                        1,
                        MechanismFixtures.gearing(10, 1),
                        Inches.of(1))),
                List.of())
            .math();
    DCMotor kraken = Motor.KRAKEN_X60.model();
    double lifting = kraken.freeSpeed / 10 * Inches.of(1).in(Meters) * 2 / 12; // m/s at 2 V
    double turning = kraken.freeSpeed / 50 * 1 / 12; // rad/s at 1 V

    double elevatorTravel = MechanismControl.Homing.travelSeconds(math, elevator, 2);
    var derived =
        (MechanismControl.Homing.ToHardStop)
            MechanismControl.Homing.toHardStop(Sensor.Switch.Limit.End.LOWER);
    var given =
        (MechanismControl.Homing.ToSwitch)
            MechanismControl.Homing.toSwitch(Sensor.Switch.Limit.End.UPPER, Seconds.of(5));

    assertThat(elevatorTravel).isCloseTo(1.2 / lifting, within(1e-9));
    assertThat(derived.timeout()).isEmpty();
    assertThat(derived.secondsOn(math, elevator)).isCloseTo(2 * elevatorTravel + 0.5, within(1e-9));
    assertThat(MechanismControl.Homing.travelSeconds(math, turret, 1))
        .isCloseTo(2 * Math.PI / turning, within(1e-9));
    assertThat(MechanismControl.Homing.travelSeconds(math, slide, 2))
        .isCloseTo(0.4 / lifting, within(1e-9));
    assertThat(given.secondsOn(math, elevator)).isEqualTo(5);
    assertThatThrownBy(
            () ->
                MechanismControl.Homing.ToSwitch.builder()
                    .end(Sensor.Switch.Limit.End.LOWER)
                    .volts(2)
                    .timeout(Seconds.of(0))
                    .build())
        .hasMessage("ToSwitch.timeout: 0 s must be positive");
  }

  /**
   * When a motor controller reads as drawing nothing on its PDH channel is its joint's control's:
   * the template's 2 A, 0.5 A and half a second unless it says otherwise, a channel drawing nothing
   * always under what its leader draws.
   */
  @Test
  void aDeadChannelIsTheControlsToSay() {
    MechanismControl control = MechanismControl.position(30);
    MechanismControl tuned = control.withDeadChannel(Amps.of(3), Amps.of(1), Seconds.of(0.25));

    assertThat(control.deadChannel()).isEqualTo(MechanismControl.DeadChannel.DEFAULTS);
    assertThat(MechanismControl.DeadChannel.DEFAULTS.drawing()).isEqualTo(Amps.of(2));
    assertThat(MechanismControl.DeadChannel.DEFAULTS.dead()).isEqualTo(Amps.of(0.5));
    assertThat(MechanismControl.DeadChannel.DEFAULTS.time()).isEqualTo(Seconds.of(0.5));
    assertThat(tuned.deadChannel().drawing()).isEqualTo(Amps.of(3));
    assertThat(tuned.deadChannel().dead()).isEqualTo(Amps.of(1));
    assertThat(tuned.deadChannel().time()).isEqualTo(Seconds.of(0.25));
    assertThatThrownBy(() -> control.withDeadChannel(Amps.of(1), Amps.of(1), Seconds.of(0.5)))
        .hasMessage(
            "DeadChannel.dead: 1 A must be under what the joint's leader draws, 1 A: a motor"
                + " drawing as much as its leader isn't dead");
    assertThatThrownBy(() -> control.withDeadChannel(Amps.of(2), Amps.of(0.5), Seconds.of(0)))
        .hasMessage("DeadChannel.time: 0 s must be positive");
  }

  /**
   * How far a joint's absolute encoder and its motor's count may disagree is given with its unit
   * and held in the joint's, remembering its kind: the joint refuses one of the other kind, saying
   * how to write it, and none is more than none.
   */
  @Test
  void anEncoderGapIsGivenWithItsUnit() {
    MechanismControl arm = MechanismControl.position(30).withEncoderGap(Degrees.of(8));
    MechanismControl elevator = MechanismControl.position(60).withEncoderGap(Inches.of(1));

    assertThat(arm.encoderGap().orElseThrow().margin()).isCloseTo(Math.toRadians(8), within(1e-12));
    assertThat(arm.encoderGap().orElseThrow().kind()).isEqualTo(Position.Kind.ANGLE);
    assertThat(elevator.encoderGap().orElseThrow().margin()).isCloseTo(0.0254, within(1e-12));
    assertThat(MechanismControl.position(30).encoderGap()).as("the template's").isEmpty();
    assertThat(elevator.ofTheOtherKind(Position.Kind.ANGLE))
        .isEqualTo(
            "its encoder gap is a length, and the joint turns: give it in an angle,"
                + " withEncoderGap(Degrees.of(...))");
    assertThat(arm.ofTheOtherKind(Position.Kind.LENGTH))
        .isEqualTo(
            "its encoder gap is an angle, and the joint slides: give it in a length,"
                + " withEncoderGap(Inches.of(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> arm.withEncoderGap(Degrees.of(0)))
        .withMessage("an encoder gap is more than none, not 0°");
  }
}
