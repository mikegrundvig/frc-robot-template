package frc.robot.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robots;
import frc.robot.SimTestSupport;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.config.SwerveWiring;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * The CTRE IO against Phoenix's own simulated devices: set what a sensor reads, and check what the
 * robot code gets. Conversions, the coupling, and the odometry thread; the control loops belong to
 * the devices.
 */
class PhoenixIOTest {
  private static final CANBus BUS = new CANBus("drivetrain");
  private static final SwerveConfig DRIVETRAIN = Robots.SIM.drivetrain();
  private static final double COUPLING = 3.5;

  private final PhoenixOdometryThread odometry = new PhoenixOdometryThread();

  @BeforeAll
  static void initHal() {
    SimTestSupport.initHal();
  }

  @AfterEach
  void tearDown() {
    odometry.interrupt();
  }

  private static SwerveWiring.ModuleWiring wiring(int drive, int steer, int encoder) {
    return SwerveWiring.ModuleWiring.builder()
        .driveId(drive)
        .steerId(steer)
        .encoderId(encoder)
        .encoderOffset(Rotations.of(0))
        .driveInverted(false)
        .steerInverted(false)
        .encoderInverted(false)
        .build();
  }

  /** Waits up to two seconds, reading again every few milliseconds, for something to be true. */
  private static boolean eventually(Runnable read, BooleanSupplier done)
      throws InterruptedException {
    for (int i = 0; i < 200; i++) {
      read.run();
      if (done.getAsBoolean()) {
        return true;
      }
      Thread.sleep(10);
    }
    return false;
  }

  @Test
  void theWheelTurnsByItsGearingLessWhatTurningTheModulePutThere() throws InterruptedException {
    ModuleIOTalonFX io =
        new ModuleIOTalonFX(wiring(1, 2, 3), BUS, DRIVETRAIN.modules(), COUPLING, odometry);
    io.configure(DRIVETRAIN.gains(), DRIVETRAIN.limits());
    odometry.start();
    double gearing = DRIVETRAIN.modules().driveGearing();

    new TalonFX(1, BUS).getSimState().setRawRotorPosition(2 * gearing); // two wheel turns' worth
    new CANcoder(3, BUS).getSimState().setRawPosition(0.25); // the module turned a quarter
    // The steering motor passes the CANcoder's angle on a moment after the CANcoder reports it, so
    // the readings settle over a few frames.
    double expected = 2 * Math.PI * (2 - COUPLING * 0.25 / gearing);
    ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    boolean settled =
        eventually(
            () -> io.updateInputs(inputs),
            () -> {
              int last = inputs.odometryDrivePositionsRad.length - 1;
              return last >= 0
                  && Math.abs(inputs.drivePositionRad - expected) < 0.01
                  && Math.abs(inputs.odometryDrivePositionsRad[last] - expected) < 0.01
                  && Math.abs(inputs.odometrySteerAnglesRad[last] - Math.PI / 2) < 0.01;
            });

    assertThat(settled).as("the readings settle where the gearing and coupling put them").isTrue();
    assertThat(inputs.steerAngle.getDegrees()).isCloseTo(90, within(0.5));
    assertThat(inputs.odometryTimestampsNanos).hasSameSizeAs(inputs.odometryDrivePositionsRad);
  }

  @Test
  void theCouplingIsTheWheelTurnsTheModulesTurnPutsOnTheDriveEncoder() {
    // A quarter turn of the module with the wheel still turns the drive motor 3.5 / 4 times.
    assertThat(ModuleIOTalonFX.wheelRotations(3.5 * 0.25 / 6.75, 0.25, 3.5, 6.75))
        .isCloseTo(0, within(1e-12));
    assertThat(ModuleIOTalonFX.wheelRotations(2, 0, 3.5, 6.75)).isEqualTo(2);
  }

  @Test
  void theGyroPassesItsAnglesThroughInWpilibsConvention() throws InterruptedException {
    GyroIOPigeon2 io = new GyroIOPigeon2(20, BUS, 1.0, odometry);
    odometry.start();

    var pigeon = new Pigeon2(20, BUS).getSimState();
    pigeon.setRawYaw(90);
    pigeon.setPitch(-5); // nose up
    pigeon.setRoll(3);
    GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();
    boolean read =
        eventually(
            () -> io.updateInputs(inputs),
            () ->
                Math.abs(inputs.yaw.getDegrees() - 90) < 0.1
                    && Math.abs(Math.toDegrees(inputs.rollRad) - 3) < 0.1
                    && inputs.odometryYawsRad.length > 0);

    assertThat(read).as("the simulated gyro answered").isTrue();
    assertThat(Math.toDegrees(inputs.pitchRad)).isCloseTo(-5, within(0.1));
    assertThat(inputs.odometryYawsRad[inputs.odometryYawsRad.length - 1])
        .isCloseTo(Math.PI / 2, within(0.002));
  }
}
