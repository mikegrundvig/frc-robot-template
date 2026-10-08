package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.body;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.drivetrain;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.gearing;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.modules;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

class DrivetrainGeometryTest {
  @Test
  void keepsItsSettings() {
    DrivetrainGeometry drivetrain = drivetrain();

    assertThat(drivetrain.corners())
        .extracting(DrivetrainGeometry.Corner::module)
        .containsOnly(modules());

    DrivetrainGeometry.ModuleType modules = drivetrain.module(0);
    assertThat(modules.driveMotor()).isEqualTo(Motor.KRAKEN_X60);
    assertThat(modules.steerMotor()).isEqualTo(Motor.KRAKEN_X44);
    assertThat(modules.wheelRadius().in(Inches)).isCloseTo(2, within(1e-9));
    assertThat(modules.driveGearing()).isEqualTo(gearing(6.75));
    assertThat(modules.steerGearing()).isEqualTo(gearing(12.8));
    assertThat(modules.wheelCoefficientOfFriction()).contains(1.2);
    assertThat(modules.driveInertia().orElseThrow().in(KilogramSquareMeters)).isEqualTo(0.001);
    assertThat(modules.steerInertia().orElseThrow().in(KilogramSquareMeters)).isEqualTo(0.004);
  }

  /**
   * A module type that leaves its tread's grip and its inertias out takes their defaults:
   * PathPlanner's 1.2, and the template's estimates, 0.001 and 0.004 kg m². Its joints' discs turn
   * as hard as those, and a measured tread beside it still slips first at its own grip.
   */
  @Test
  void aModuleTypeLeftUnmeasuredTakesTheDefaults() {
    DrivetrainGeometry.ModuleType unmeasured =
        changeModules()
            .wheelCoefficientOfFriction(Optional.empty())
            .driveInertia(Optional.empty())
            .steerInertia(Optional.empty())
            .build();
    DrivetrainGeometry.ModuleJoints corner = unmeasured.joints("here", new Translation2d(0.3, 0.2));
    DrivetrainGeometry worn =
        DrivetrainGeometry.builder()
            .corners(
                List.of(
                    corner(unmeasured, new Translation2d(0.3, 0.2)),
                    corner(unmeasured, new Translation2d(0.3, -0.2)),
                    corner(
                        changeModules().wheelCoefficientOfFriction(1.0).build(),
                        new Translation2d(-0.3, 0.2)),
                    corner(unmeasured, new Translation2d(-0.3, -0.2))))
            .build();

    assertThat(unmeasured.wheelCoefficientOfFrictionOrDefault()).isEqualTo(1.2);
    assertThat(unmeasured.driveInertiaOrDefault().in(KilogramSquareMeters)).isEqualTo(0.001);
    assertThat(unmeasured.steerInertiaOrDefault().in(KilogramSquareMeters)).isEqualTo(0.004);
    assertThat(corner.wheel().link().mass().inertia().iyy()).isCloseTo(0.001, within(1e-12));
    assertThat(corner.steer().link().mass().inertia().izz()).isCloseTo(0.004, within(1e-12));
    assertThat(
            DrivetrainGeometry.inset(unmeasured, body(), Inches.of(2.625))
                .wheelCoefficientOfFriction())
        .isEqualTo(1.2);
    assertThat(worn.wheelCoefficientOfFriction()).as("the measured tread's, least").isEqualTo(1.0);
    assertThat(modules().wheelCoefficientOfFrictionOrDefault()).as("as measured").isEqualTo(1.2);
  }

  @Test
  void freeSpeedComesFromTheMotorGearingAndWheel() {
    // A Kraken X60 turns 6000 rpm free: 100 rotations per second, divided by the gearing, times
    // the wheel's circumference.
    double expected = 6000.0 / 60 / 6.75 * 2 * Math.PI * Inches.of(2).in(Meters);

    assertThat(modules().freeSpeed().in(MetersPerSecond)).isCloseTo(expected, within(1e-9));
    assertThat(modules().idealDriveKv()).isCloseTo(12 / expected, within(1e-9));
    assertThat(drivetrain().freeSpeed().in(MetersPerSecond)).isCloseTo(expected, within(1e-9));
  }

  /** The drive and the steering are each one motor through their own gearing. */
  @Test
  void eachWheelIsDrivenAndTurnedByOneMotor() {
    assertThat(modules().driveMotors())
        .isEqualTo(Actuator.Motors.builder().motor(Motor.KRAKEN_X60).count(1).build());
    assertThat(modules().steerMotors().motor()).isEqualTo(Motor.KRAKEN_X44);
    assertThat(modules().steerMotors().count()).isEqualTo(1);
  }

  @Test
  void wheelsSitInFromTheFrameCorners() {
    // A 28 x 24 in frame with wheels 2.625 in in from its edges.
    double x = Inches.of(14 - 2.625).in(Meters);
    double y = Inches.of(12 - 2.625).in(Meters);

    assertThat(drivetrain().moduleTranslations())
        .as("front left, front right, back left, back right")
        .containsExactly(
            new Translation2d(x, y),
            new Translation2d(x, -y),
            new Translation2d(-x, y),
            new Translation2d(-x, -y));
    assertThat(
            DrivetrainGeometry.uniform(modules(), Meters.of(x), Meters.of(y)).moduleTranslations())
        .as("the same, as Tuner X's swerve project gives them")
        .containsExactly(drivetrain().moduleTranslations());
  }

  /**
   * Each corner its own type, where its wheel sits: MK5i in front and MK5n behind, their steering
   * geared apart. The slowest module sets the drivetrain's free speed, the slipperiest tread its
   * grip.
   */
  @Test
  void eachCornerHasItsOwnModule() {
    DrivetrainGeometry.ModuleType back =
        DrivetrainGeometryModuleTypeBuilder.builder(modules())
            .steerGearing(gearing(26.09))
            .driveGearing(gearing(7.5))
            .wheelCoefficientOfFriction(1.0)
            .build();
    List<DrivetrainGeometry.Corner> corners = new ArrayList<>(drivetrain().corners());
    corners.set(2, corner(back, corners.get(2).position()));
    corners.set(3, corner(back, corners.get(3).position()));
    DrivetrainGeometry mixed = DrivetrainGeometry.builder().corners(corners).build();

    assertThat(mixed.module(0)).isEqualTo(modules());
    assertThat(mixed.module(3)).isEqualTo(back);
    assertThat(
            mixed
                .joints()
                .get(3)
                .steering()
                .transmission()
                .shaftRadiansPer(mixed.joints().get(3).steer()))
        .isEqualTo(26.09);
    assertThat(mixed.freeSpeed()).isEqualTo(back.freeSpeed());
    assertThat(mixed.wheelCoefficientOfFriction()).isEqualTo(1.0);
  }

  @Test
  void wheelsMustFitInsideTheFrameEachInItsCorner() {
    // The frame is 28 in long and 24 in wide: a 12 in inset meets in the middle side to side.
    DrivetrainGeometry wide = DrivetrainGeometry.uniform(modules(), Inches.of(11), Inches.of(13));

    assertThat(wide.whyWheelsDontFit(body()))
        .contains("wheels outside the frame (28 in by 24 in): front-left's");
    assertThat(drivetrain().whyWheelsDontFit(body())).isNull();
    assertRejected(
        "DrivetrainGeometry.corners",
        () -> DrivetrainGeometry.inset(modules(), body(), Inches.of(12)));
    List<DrivetrainGeometry.Corner> swapped = new ArrayList<>(drivetrain().corners());
    swapped.set(0, drivetrain().corners().get(1));
    swapped.set(1, drivetrain().corners().get(0));
    assertRejected(
        "DrivetrainGeometry.corners", () -> DrivetrainGeometry.builder().corners(swapped).build());
    assertRejected(
        "DrivetrainGeometry.corners: needs four",
        () -> DrivetrainGeometry.builder().corners(swapped.subList(0, 3)).build());
  }

  @Test
  void rejectsImpossibleModuleValues() {
    assertRejected(
        "ModuleType.wheelRadius", () -> changeModules().wheelRadius(Inches.of(0)).build());
    assertRejected(
        "ModuleType.wheelCoefficientOfFriction",
        () -> changeModules().wheelCoefficientOfFriction(Double.NaN).build());
    assertRejected(
        "ModuleType.driveInertia",
        () -> changeModules().driveInertia(KilogramSquareMeters.of(0)).build());
    assertRejected(
        "ModuleType.steerInertia",
        () -> changeModules().steerInertia(KilogramSquareMeters.of(-0.004)).build());
    assertRejected("ModuleType.couplingRatio", () -> changeModules().couplingRatio(-1).build());
    assertRejected("Corner.position", () -> corner(modules(), new Translation2d(Double.NaN, 0.2)));
  }

  private static DrivetrainGeometryModuleTypeBuilder changeModules() {
    return DrivetrainGeometryModuleTypeBuilder.builder(modules());
  }

  private static DrivetrainGeometry.Corner corner(
      DrivetrainGeometry.ModuleType module, Translation2d position) {
    return DrivetrainGeometry.Corner.builder().module(module).position(position).build();
  }

  /**
   * Without a coupling the drive motor turns only its wheel; with one, the module's steering too.
   * Each module is a disc as hard to turn as the module's inertia says.
   */
  @Test
  void eachCornersDriveMotorTurnsItsWheelAndThroughTheCouplingItsSteering() {
    DrivetrainGeometry.ModuleJoints uncoupled = drivetrain().joints().get(1);
    DrivetrainGeometry.ModuleJoints coupled =
        DrivetrainGeometryModuleTypeBuilder.builder(modules())
            .couplingRatio(3.5)
            .build()
            .joints("here", new Translation2d(0.3, 0.2));

    assertThat(uncoupled.label()).isEqualTo("front-right");
    assertThat(uncoupled.drive().transmission().joints()).containsExactly(uncoupled.wheel());
    assertThat(uncoupled.steering().transmission().joints()).containsExactly(uncoupled.steer());
    assertThat(uncoupled.wheel().parent()).isSameAs(uncoupled.steer());
    assertThat(coupled.drive().transmission().joints())
        .containsExactly(coupled.wheel(), coupled.steer());
    assertThat(coupled.drive().transmission().shaftRadiansPer(coupled.steer())).isEqualTo(3.5);
    assertThat(coupled.wheel().link().mass().inertia().iyy())
        .isCloseTo(modules().driveInertiaOrDefault().in(KilogramSquareMeters), within(1e-12));
    assertThat(coupled.steer().link().mass().inertia().izz())
        .isCloseTo(modules().steerInertiaOrDefault().in(KilogramSquareMeters), within(1e-12));
  }

  /**
   * A drivetrain's wiring must drive its definition's motors: REV's MAXSwerve, a Vortex driving and
   * a NEO 550 steering, on SPARKs; the template's own Krakens on TalonFXs, and not the other way
   * round, saying what does. SPARKs are on Systemcore's own CAN, never a CANivore.
   */
  @Test
  void aSwervesWiringMustDriveItsMotors() {
    DrivetrainGeometry ours = drivetrain();
    DrivetrainGeometry.ModuleType maxSwerve =
        DrivetrainGeometryModuleTypeBuilder.builder(ours.corners().get(0).module())
            .driveMotor(Motor.NEO_VORTEX)
            .steerMotor(Motor.NEO_550)
            .build();
    DrivetrainGeometry rev =
        DrivetrainGeometryBuilder.builder(ours)
            .corners(
                ours.corners().stream()
                    .map(
                        corner ->
                            DrivetrainGeometryCornerBuilder.builder(corner)
                                .module(maxSwerve)
                                .build())
                    .toList())
            .build();
    SwerveWiring talons =
        wiring(MechanismWiring.Controller.TALON_FX, MechanismWiring.Controller.TALON_FX, true);
    SwerveWiring sparks =
        wiring(MechanismWiring.Controller.SPARK_FLEX, MechanismWiring.Controller.SPARK_MAX, false);

    assertThat(ours.wired(talons).wiring()).contains(talons);
    assertThat(rev.wired(sparks).wiring()).contains(sparks);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> rev.wired(talons))
        .withMessageContaining(
            "its drive's NEO Vortex is on a TalonFX, which doesn't drive one: a TalonFXS or a"
                + " SPARK MAX or a SPARK Flex does; its steering's NEO 550 is on a TalonFX, which"
                + " doesn't drive one: a TalonFXS or a SPARK MAX or a SPARK Flex does");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                rev.wired(
                    SwerveWiringBuilder.builder(sparks)
                        .canBus(CanBus.canivore("canivore"))
                        .build()))
        .withMessageContaining("its SPARKs are on a CANivore: a SPARK is on Systemcore's own CAN");
  }

  private static SwerveWiring wiring(
      MechanismWiring.Controller drive, MechanismWiring.Controller steer, boolean cancoders) {
    List<SwerveWiring.ModuleWiring> modules = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      modules.add(
          SwerveWiring.ModuleWiring.builder()
              .driveId(1 + 2 * i)
              .steerId(2 + 2 * i)
              .encoderOffset(Rotations.of(0))
              .driveInverted(false)
              .steerInverted(true)
              .encoderInverted(false)
              .encoderId(cancoders ? Optional.of(20 + i) : Optional.empty())
              .build());
    }
    return SwerveWiring.builder()
        .canBus(CanBus.port(CanBus.Systemcore.CAN_S0))
        .driveController(drive)
        .steerController(steer)
        .gyroId(30)
        .gyroYawScale(1.0)
        .modules(modules)
        .build();
  }

  private static void assertRejected(String field, ThrowingCallable build) {
    assertThatIllegalArgumentException().isThrownBy(build).withMessageContaining(field);
  }
}
