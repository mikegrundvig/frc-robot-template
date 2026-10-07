package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

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
                    .kV(Optional.of(-0.1))
                    .kA(Optional.empty())
                    .kG(Optional.of(Double.NaN))
                    .gravityAngle(Optional.empty())
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
                .kV(Optional.empty())
                .kA(Optional.empty())
                .kG(Optional.of(-0.4)) // a counterbalance stronger than the arm
                .gravityAngle(Optional.empty())
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
            () ->
                MechanismControl.Profile.builder()
                    .cruiseVelocity(0)
                    .acceleration(1)
                    .jerk(-1)
                    .build())
        .hasMessage(
            "Profile.cruiseVelocity: 0 must be positive\nProfile.jerk: -1 must be zero or more");
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

  /** Each change sets its one value and leaves the rest. */
  @Test
  void eachChangeSetsOneValue() {
    MechanismControl arm =
        MechanismControl.position(30)
            .withKd(1)
            .withProfile(8, 30)
            .withSoftLimits(0, 2)
            .withStatorLimit(Amps.of(60))
            .withHoming(MechanismControl.Homing.assumeAtReference());

    assertThat(arm.gains().kP()).isEqualTo(30);
    assertThat(arm.gains().kD()).isEqualTo(1);
    assertThat(arm.profile().orElseThrow().cruiseVelocity()).isEqualTo(8);
    assertThat(arm.profile().orElseThrow().acceleration()).isEqualTo(30);
    assertThat(arm.softLimits().orElseThrow().upper()).isEqualTo(2);
    assertThat(arm.currentLimits().stator()).isEqualTo(Amps.of(60));
    assertThat(arm.currentLimits().supply()).isEqualTo(Amps.of(60));
    assertThat(arm.homing()).contains(MechanismControl.Homing.assumeAtReference());
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
            .reference(0.2)
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
            MechanismControl.Homing.toHardStop(MechanismControl.Homing.Stop.LOWER);
    var given =
        (MechanismControl.Homing.ToSwitch)
            MechanismControl.Homing.toSwitch(MechanismControl.Homing.Stop.UPPER, Seconds.of(5));

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
                    .end(MechanismControl.Homing.Stop.LOWER)
                    .volts(2)
                    .timeout(Optional.of(Seconds.of(0)))
                    .build())
        .hasMessage("ToSwitch.timeout: 0 s must be positive");
  }
}
