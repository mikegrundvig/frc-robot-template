package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANRANGE;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/**
 * The PDH is the robot's, and each motor controller says which of its channels feeds it where it's
 * wired: the definition checks them against each other when it's written down, and says what it
 * can't check.
 */
class PdhTest {
  private static final Translation3d AXLE = new Translation3d(0.25, 0, 0.32);
  private static final Output INTAKING = Output.rpm("intaking", 2000);
  private static final CanBus CANIVORE = CanBus.canivore("canivore");

  /** A roller on two TalonFXs, each fed as written. */
  private static Joint.Revolute roller(
      MechanismWiring.Device leader, MechanismWiring.Device other) {
    return Joint.roller("roller", Part.CHASSIS, AXLE)
        .outputs(INTAKING)
        .motors(Motor.KRAKEN_X60, Gearing.of(3), leader, other);
  }

  private static final Joint.Revolute FED =
      roller(
          TALON_FX.can(31).pdh(4, Amps.of(40)), TALON_FX.can(32).clockwise().pdh(5, Amps.of(30)));

  /** The test robot with a PDH, if given, and these joints. */
  private static RobotDefinition robot(Optional<Pdh> pdh, Joint... joints) {
    return RobotDefinitionBuilder.builder(ConfigFixtures.definition(List.of()))
        .pdh(pdh)
        .joints(List.of(joints))
        .build();
  }

  private static SwerveWiring.ModuleWiring module(
      int first, Optional<Pdh.Feed> driveFeed, Optional<Pdh.Feed> steerFeed) {
    return SwerveWiring.ModuleWiring.builder()
        .driveId(first)
        .steerId(first + 1)
        .encoderOffset(Rotations.of(0))
        .driveInverted(false)
        .steerInverted(false)
        .encoderInverted(false)
        .encoderId(first)
        .driveFeed(driveFeed)
        .steerFeed(steerFeed)
        .build();
  }

  /** The test drivetrain, wired: its front left module fed on channels 0 and 1, the rest not. */
  private static DrivetrainGeometry wiredDrivetrain() {
    return ConfigFixtures.drivetrain()
        .wired(
            SwerveWiring.builder()
                .canBus(CanBus.SYSTEMCORE)
                .driveController(TALON_FX)
                .steerController(TALON_FX)
                .gyroId(9)
                .gyroYawScale(1.0)
                .modules(
                    List.of(
                        module(
                            1,
                            Optional.of(Pdh.feed(0, Amps.of(40))),
                            Optional.of(Pdh.feed(1, Amps.of(40)))),
                        module(3, Optional.empty(), Optional.empty()),
                        module(5, Optional.empty(), Optional.empty()),
                        module(7, Optional.empty(), Optional.empty())))
                .build());
  }

  /**
   * REV's PDH: 20 high-current channels, 0 to 19, up to 40 A; three low-current, 20 to 22, and the
   * switchable one, 23, each up to 15 A (REV's specifications and labeled diagram).
   */
  @Test
  void aRevPdhHasTwentyHighCurrentChannelsThreeLowAndASwitchableOne() {
    assertThat(Pdh.CHANNELS).isEqualTo(24);
    assertThat(Pdh.rating(0)).isEqualTo(Amps.of(40));
    assertThat(Pdh.rating(19)).isEqualTo(Amps.of(40));
    assertThat(Pdh.rating(20)).isEqualTo(Amps.of(15));
    assertThat(Pdh.rating(23)).isEqualTo(Amps.of(15));
  }

  /** WPILib reads a REV PDH at CAN IDs 1 to 63, on Systemcore's own CAN, never a CANivore. */
  @Test
  void itsOnSystemcoresOwnCanAtAnIdFromOneTo63() {
    Pdh pdh = Pdh.can(1);

    assertThat(pdh.bus()).isEqualTo(CanBus.SYSTEMCORE);
    assertThat(pdh.id()).isEqualTo(1);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Pdh.can(0))
        .withMessage("Pdh.id: 0 must be from 1 to 63");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Pdh.can(CANIVORE, 1))
        .withMessage(
            "Pdh.bus: a PDH is on Systemcore's own CAN, where WPILib reads it, not on a CANivore");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Pdh.feed(-1, Amps.of(40)))
        .withMessage("Feed.channel: -1 must be zero or more");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Pdh.feed(4, Amps.of(0)))
        .withMessage("Feed.breaker: 0 A must be positive");
  }

  /** A controller's feed is one call where it's wired, in any order with its direction. */
  @Test
  void aControllerSaysItsFeedWhereItsWired() {
    MechanismWiring.Device device = SPARK_MAX.can(CANIVORE, 41).pdh(7, Amps.of(30)).clockwise();

    assertThat(device.feed()).contains(Pdh.feed(7, Amps.of(30)));
    assertThat(device.turnsClockwise()).isTrue();
    assertThat(device.bus()).isEqualTo(CANIVORE);
    assertThat(device.id()).isEqualTo(41);
    assertThat(TALON_FX.can(32).feed()).isEmpty();
  }

  /**
   * One list of every device on the robot's buses, each once, worked out from where each is
   * written: the PDH, the drivetrain's, each joint's, and its sensors' CANdi and CANrange, a CANdi
   * whose two inputs are wired listed once.
   */
  @Test
  void theRobotListsEveryDeviceOnItsBusesOnce() {
    Joint.Revolute pivot =
        Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_DOWN)
            .range(Range.degrees(0, 135))
            .rest(Position.degrees("in", 1.5))
            .motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(CANIVORE, 20).pdh(6, Amps.of(40)))
            .absoluteEncoder(CANCODER.can(CANIVORE, 21).offset(Rotations.of(0)).remote())
            .limitSwitch(
                Sensor.Switch.Limit.End.LOWER, CANDI.can(CANIVORE, 22).s1().normallyClosed())
            .limitSwitch(
                Sensor.Switch.Limit.End.UPPER, CANDI.can(CANIVORE, 22).s2().normallyClosed());
    Sensor.Rangefinder hopper =
        Sensor.Rangefinder.of("hopper range", Part.CHASSIS, Meters.of(1), Degrees.of(20))
            .on(CANRANGE.can(CanBus.SYSTEMCORE, 30));
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(robot(Optional.of(Pdh.can(1)), pivot, FED))
            .drivetrain(wiredDrivetrain())
            .sensors(List.of(hopper))
            .build();

    assertThat(robot.devices())
        .extracting(CanDevice::name)
        .containsExactly(
            "the PDH",
            "the gyro",
            "front-left drive",
            "front-left steering",
            "front-left CANcoder",
            "front-right drive",
            "front-right steering",
            "front-right CANcoder",
            "back-left drive",
            "back-left steering",
            "back-left CANcoder",
            "back-right drive",
            "back-right steering",
            "back-right CANcoder",
            "pivot motor 20",
            "pivot CANcoder",
            "roller motor 31",
            "roller motor 32",
            "CANdi 22",
            "hopper range");
    assertThat(robot.devices().get(14))
        .isEqualTo(
            new CanDevice(
                "pivot motor 20",
                CanDevice.Kind.TALON_FX,
                CANIVORE,
                20,
                Optional.of(Pdh.feed(6, Amps.of(40)))));
    assertThat(robot.devices().get(15).bus()).isEqualTo(CANIVORE); // on its motors' bus
    assertThat(robot.devices().get(2).feed()).contains(Pdh.feed(0, Amps.of(40)));
    assertThat(robot.devices().get(19).kind()).isEqualTo(CanDevice.Kind.CANRANGE);
  }

  /** One breaker, one device: two controllers on one channel is a wiring mistake. */
  @Test
  void aChannelFeedsOneDevice() {
    Joint.Revolute twice =
        roller(TALON_FX.can(31).pdh(4, Amps.of(40)), TALON_FX.can(32).pdh(4, Amps.of(40)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(Optional.of(Pdh.can(1)), twice))
        .withMessage(
            "RobotDefinition[test].pdh: channel 4 feeds roller motor 31 and roller motor 32: a"
                + " channel feeds one device, so give each its own");
  }

  /**
   * A channel is one of the PDH's 24, and its breaker no bigger than it takes: 40 A on a
   * high-current channel, 15 A on a low-current one.
   */
  @Test
  void aChannelIsThePdhsAndItsBreakerNoBiggerThanItTakes() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    Optional.of(Pdh.can(1)),
                    roller(
                        TALON_FX.can(31).pdh(24, Amps.of(40)),
                        TALON_FX.can(32).pdh(19, Amps.of(50)))))
        .withMessage(
            "RobotDefinition[test].pdh: roller motor 31's channel, 24, isn't one of the PDH's:"
                + " they're 0 to 23\nRobotDefinition[test].pdh: roller motor 32's 50 A breaker is"
                + " over channel 19's rating: a high-current channel takes up to 40 A");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    Optional.of(Pdh.can(1)),
                    roller(
                        TALON_FX.can(31).pdh(22, Amps.of(20)),
                        TALON_FX.can(32).pdh(23, Amps.of(20)))))
        .withMessage(
            "RobotDefinition[test].pdh: roller motor 31's 20 A breaker is over channel 22's"
                + " rating: a low-current channel takes up to 15 A\nRobotDefinition[test].pdh:"
                + " roller motor 32's 20 A breaker is over channel 23's rating: the switchable"
                + " channel takes up to 15 A");
    assertThat(
            robot(
                    Optional.of(Pdh.can(1)),
                    roller(
                        TALON_FX.can(31).pdh(19, Amps.of(40)),
                        TALON_FX.can(32).pdh(20, Amps.of(15))))
                .devices())
        .hasSize(3);
  }

  /** A device fed from a channel needs the PDH it's on, the drivetrain's included. */
  @Test
  void aFeedNeedsAPdh() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(Optional.empty(), FED))
        .withMessage(
            "RobotDefinition[test].pdh: roller motor 31, roller motor 32 are fed from a PDH's"
                + " channel, and the robot declares no PDH: give its definition one,"
                + " .pdh(Pdh.can(1)) at its CAN ID");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(robot(Optional.empty()))
                    .drivetrain(wiredDrivetrain())
                    .build())
        .withMessageStartingWith(
            "RobotDefinition[test].pdh: front-left drive, front-left steering are fed");
  }

  /**
   * With a PDH declared, a wired motor controller that doesn't say its channel is a doubt, not a
   * refusal: once for the drivetrain, naming those it lacks, and once for each joint. A joint that
   * runs in the simulator alone has nothing to feed.
   */
  @Test
  void withAPdhAWiredControllerWithNoChannelIsADoubt() {
    Joint.Revolute half = roller(TALON_FX.can(31).pdh(4, Amps.of(40)), TALON_FX.can(32));
    Joint.Revolute none = roller(TALON_FX.can(41), TALON_FX.can(42));
    Joint.Revolute simulated =
        Joint.roller("simulated", Part.CHASSIS, AXLE)
            .outputs(INTAKING)
            .motor(Motor.KRAKEN_X60, Gearing.of(3));
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(robot(Optional.of(Pdh.can(1)), half, simulated))
            .drivetrain(wiredDrivetrain())
            .build();

    assertThat(MechanismFixtures.notes(robot))
        .filteredOn(note -> note.contains("PDH channel"))
        .containsExactly(
            "the drivetrain's front-right drive, front-right steering, back-left drive, back-left"
                + " steering, back-right drive, back-right steering have no PDH channel: which"
                + " channel feeds each, and through what breaker, isn't checked: give each"
                + " module's wiring its feeds, .driveFeed(Pdh.feed(channel, breaker)) and"
                + " .steerFeed(...)",
            "roller's motor controller 32 has no PDH channel: which channel feeds each, and"
                + " through what breaker, isn't checked: give each its own where it's wired,"
                + " .pdh(channel, breaker)");
    assertThat(robot.findings())
        .filteredOn(finding -> finding.message().contains("PDH channel"))
        .allMatch(finding -> finding.scope() == Finding.Scope.ROBOT);
    assertThat(MechanismFixtures.notes(robot(Optional.of(Pdh.can(1)), none)))
        .anyMatch(
            note -> note.startsWith("roller's motor controllers 41, 42 have no PDH channel:"));
  }

  /** Without a PDH declared, nothing is fed, and nothing is said about it. */
  @Test
  void withoutAPdhNothingIsSaidOfFeeds() {
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(
                robot(Optional.empty(), roller(TALON_FX.can(31), TALON_FX.can(32))))
            .drivetrain(
                ConfigFixtures.drivetrain()
                    .wired(
                        SwerveWiring.builder()
                            .canBus(CanBus.SYSTEMCORE)
                            .driveController(TALON_FX)
                            .steerController(TALON_FX)
                            .gyroId(9)
                            .gyroYawScale(1.0)
                            .modules(
                                List.of(
                                    module(1, Optional.empty(), Optional.empty()),
                                    module(3, Optional.empty(), Optional.empty()),
                                    module(5, Optional.empty(), Optional.empty()),
                                    module(7, Optional.empty(), Optional.empty())))
                            .build()))
            .build();

    assertThat(robot.findings()).noneMatch(finding -> finding.message().contains("PDH"));
    assertThat(robot.devices()).noneMatch(device -> device.feed().isPresent());
    assertThat(
            MechanismFixtures.notes(RobotDefinitionBuilder.builder(robot).pdh(Pdh.can(1)).build()))
        .anyMatch(
            note ->
                note.startsWith("none of the drivetrain's motor controllers has a PDH channel:"));
  }
}
