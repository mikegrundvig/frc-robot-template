package frc.robot.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatNoException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryCornerBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryModuleTypeBuilder;
import com.michaelgrundvig.frc.world.robot.Motor;
import frc.robot.Robots;
import frc.robot.lib.wiring.CanBus;
import frc.robot.lib.wiring.MechanismWiring;
import frc.robot.lib.wiring.SwerveWiring;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.imu.OnboardIMU;

class DrivetrainControlTest {
  @Test
  void keepsItsSettings() {
    DrivetrainControl control = control();

    assertThat(control.limits()).isEqualTo(limits());
    assertThat(control.gains()).isEqualTo(gains());

    DrivetrainControl.Limits limits = control.limits();
    assertThat(limits.driveStatorCurrent().in(Amps)).isCloseTo(40, within(1e-9));
    assertThat(limits.driveSupplyCurrent().in(Amps)).isCloseTo(60, within(1e-9));
    assertThat(limits.driveSupplyLowerCurrent().in(Amps)).isCloseTo(35, within(1e-9));
    assertThat(limits.driveSupplyLowerTime().in(Seconds)).isCloseTo(0.5, within(1e-9));
    assertThat(limits.steerStatorCurrent().in(Amps)).isCloseTo(30, within(1e-9));
    assertThat(limits.maxLinearSpeed().in(MetersPerSecond)).isCloseTo(4, within(1e-9));
    assertThat(limits.maxAngularSpeed().in(RotationsPerSecond)).isCloseTo(1, within(1e-9));

    DrivetrainControl.Gains gains = control.gains();
    assertThat(gains.driveKs()).isEqualTo(0.1);
    assertThat(gains.driveKv()).isEqualTo(2.5);
    assertThat(gains.driveKp()).isEqualTo(2.0);
    assertThat(gains.steerKp()).isEqualTo(16.0);
    assertThat(gains.steerKd()).isEqualTo(0.08);
    assertThat(gains.skewLookahead().in(Seconds)).isCloseTo(0.03, within(1e-9));
  }

  /** The template's drivetrain reaches 4.73 m/s free: 4 m/s leaves room, 5 m/s doesn't. */
  @Test
  void topSpeedCannotExceedTheFreeSpeed() {
    var drivetrain = Robots.SIM.definition().drivetrain();
    var tooFast =
        DrivetrainControlBuilder.builder(control())
            .limits(changeLimits().maxLinearSpeed(MetersPerSecond.of(5)).build())
            .build();

    assertThat(control().whyTooFastFor(drivetrain)).isNull();
    assertThat(tooFast.whyTooFastFor(drivetrain))
        .isEqualTo("5 m/s is faster than the drivetrain's free speed of 4.729 m/s");
  }

  @Test
  void limitsMustBePositive() {
    assertRejected(
        "Limits.driveStatorCurrent", () -> changeLimits().driveStatorCurrent(Amps.of(0)).build());
    assertRejected(
        "Limits.driveSupplyCurrent", () -> changeLimits().driveSupplyCurrent(Amps.of(-60)).build());
    assertRejected(
        "Limits.driveSupplyLowerCurrent",
        () -> changeLimits().driveSupplyLowerCurrent(Amps.of(0)).build());
    assertRejected(
        "Limits.steerStatorCurrent",
        () -> changeLimits().steerStatorCurrent(Amps.of(Double.NaN)).build());
    assertRejected(
        "Limits.maxLinearSpeed",
        () -> changeLimits().maxLinearSpeed(MetersPerSecond.of(0)).build());
    assertRejected(
        "Limits.maxAngularSpeed",
        () ->
            changeLimits()
                .maxAngularSpeed(RotationsPerSecond.of(Double.POSITIVE_INFINITY))
                .build());
  }

  /**
   * Phoenix steps the drive's supply limit down after a time from just over zero to 5 s. At zero it
   * never steps down, which the template says by setting the lower current to the supply limit
   * instead, where it shows.
   */
  @Test
  void theStepDownTakesFromJustOverZeroToFiveSeconds() {
    assertRejected(
        "Limits.driveSupplyLowerTime",
        () -> changeLimits().driveSupplyLowerTime(Seconds.of(0)).build());
    assertRejected(
        "Limits.driveSupplyLowerTime",
        () -> changeLimits().driveSupplyLowerTime(Seconds.of(5.1)).build());
    assertThatNoException()
        .isThrownBy(() -> changeLimits().driveSupplyLowerTime(Seconds.of(5)).build());
    assertThatNoException()
        .as("no step-down, said where it shows")
        .isThrownBy(() -> changeLimits().driveSupplyLowerCurrent(Amps.of(60)).build());
  }

  @Test
  void gainsMayBeZeroButNotNegative() {
    assertThatNoException()
        .isThrownBy(
            () ->
                DrivetrainControl.Gains.builder()
                    .driveKs(0)
                    .driveKv(0)
                    .driveKp(0)
                    .steerKp(0)
                    .steerKd(0)
                    .skewLookahead(Seconds.of(0))
                    .build());
    assertRejected("Gains.driveKs", () -> changeGains().driveKs(-0.1).build());
    assertRejected("Gains.driveKv", () -> changeGains().driveKv(Double.NaN).build());
    assertRejected("Gains.driveKp", () -> changeGains().driveKp(-1).build());
    assertRejected("Gains.steerKp", () -> changeGains().steerKp(-1).build());
    assertRejected("Gains.steerKd", () -> changeGains().steerKd(-1).build());
    assertRejected(
        "Gains.skewLookahead", () -> changeGains().skewLookahead(Seconds.of(-0.01)).build());
    assertRejected(
        "Gains.skewLookahead", () -> changeGains().skewLookahead(Seconds.of(Double.NaN)).build());
  }

  private static DrivetrainControl.Limits limits() {
    return DrivetrainControl.Limits.builder()
        .driveStatorCurrent(Amps.of(40))
        .driveSupplyCurrent(Amps.of(60))
        .driveSupplyLowerCurrent(Amps.of(35))
        .driveSupplyLowerTime(Seconds.of(0.5))
        .steerStatorCurrent(Amps.of(30))
        .maxLinearSpeed(MetersPerSecond.of(4))
        .maxAngularSpeed(RotationsPerSecond.of(1))
        .build();
  }

  private static DrivetrainControl.Gains gains() {
    return DrivetrainControl.Gains.builder()
        .driveKs(0.1)
        .driveKv(2.5)
        .driveKp(2.0)
        .steerKp(16.0)
        .steerKd(0.08)
        .skewLookahead(Seconds.of(0.03))
        .build();
  }

  private static DrivetrainControl control() {
    return DrivetrainControl.builder().limits(limits()).gains(gains()).build();
  }

  private static DrivetrainControlLimitsBuilder changeLimits() {
    return DrivetrainControlLimitsBuilder.builder(limits());
  }

  private static DrivetrainControlGainsBuilder changeGains() {
    return DrivetrainControlGainsBuilder.builder(gains());
  }

  private static void assertRejected(String field, ThrowingCallable build) {
    assertThatIllegalArgumentException().isThrownBy(build).withMessageStartingWith(field);
  }

  /**
   * A drivetrain's wiring must drive its definition's motors: REV's MAXSwerve, a Vortex driving and
   * a NEO 550 steering, on SPARKs; the template's own Krakens on TalonFXs, and not the other way
   * round.
   */
  @Test
  void aSwervesWiringMustDriveItsMotors() {
    DrivetrainGeometry ours = Robots.SIM.definition().drivetrain();
    DrivetrainGeometry.ModuleType maxSwerve =
        DrivetrainGeometryModuleTypeBuilder.builder(ours.corners().get(0).module())
            .driveMotor(Motor.NEO_VORTEX)
            .steerMotor(Motor.NEO_550)
            .build();
    DrivetrainGeometry rev =
        new DrivetrainGeometry(
            ours.corners().stream()
                .map(
                    corner ->
                        DrivetrainGeometryCornerBuilder.builder(corner).module(maxSwerve).build())
                .toList());
    SwerveWiring talons =
        wiring(MechanismWiring.Controller.TALON_FX, MechanismWiring.Controller.TALON_FX, true);
    SwerveWiring sparks =
        wiring(MechanismWiring.Controller.SPARK_FLEX, MechanismWiring.Controller.SPARK_MAX, false);

    assertThat(DrivetrainControl.whyNotDriven(ours, talons)).isNull();
    assertThat(DrivetrainControl.whyNotDriven(rev, sparks)).isNull();
    assertThat(DrivetrainControl.whyNotDriven(rev, talons))
        .isEqualTo(
            "the drivetrain's wiring: its drive's NEO Vortex isn't driven by a TalonFX; its"
                + " steering's NEO 550 isn't driven by a TalonFX");
  }

  private static SwerveWiring wiring(
      MechanismWiring.Controller drive, MechanismWiring.Controller steer, boolean cancoders) {
    List<SwerveWiring.ModuleWiring> modules = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      modules.add(
          SwerveWiring.ModuleWiring.builder()
              .driveId(1 + 2 * i)
              .steerId(2 + 2 * i)
              .encoderId(cancoders ? Optional.of(20 + i) : Optional.empty())
              .encoderOffset(Rotations.of(0))
              .driveInverted(false)
              .steerInverted(true)
              .encoderInverted(false)
              .build());
    }
    return SwerveWiring.builder()
        .canBus(CanBus.port(CANPort.CAN_S0))
        .driveController(drive)
        .steerController(steer)
        .gyroId(30)
        .gyroYawScale(1.0)
        .modules(modules)
        .systemcoreMount(OnboardIMU.MountOrientation.FLAT)
        .build();
  }
}
