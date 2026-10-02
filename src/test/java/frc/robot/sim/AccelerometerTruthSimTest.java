package frc.robot.sim;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.simulation.SensorErrorsAccelErrorsBuilder;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.ImuMountBuilder;
import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The truth suite's accelerometer, as the slip check needs it: with its errors off it reads the
 * specific force physics gives where it sits (gravity's share on a tilt, the turn swinging it
 * around the center, the robot speeding up); with them on, the statistics configured; it clips
 * exactly at its range, and takes a new sample every 10 ms.
 */
@ExtendWith(RobotSim.class)
class AccelerometerTruthSimTest {
  private static final double G = ImuReadings.STANDARD_GRAVITY;

  /** Sampled at 100 Hz, but nothing else: no filter, noise, bias, mount error, or range. */
  private static final SensorErrors.AccelErrors EXACT_SAMPLED =
      SensorErrorsAccelErrorsBuilder.builder(SensorErrors.AccelErrors.IDEAL).sampleHz(100).build();

  /** Tilted and standing still, it reads gravity's share along its tipped axes, exactly. */
  @Test
  void tiltedItReadsGravitysShare() {
    TruthRig rig = rig(EXACT_SAMPLED, Translation2d.ZERO);
    rig.simulation.tilt(Math.toRadians(-10), Math.toRadians(5)); // nose up, left side up
    rig.loops(1);

    GyroIO.GyroIOInputs read = read(rig);
    assertThat(read.accelerationXMetersPerSecSq)
        .isCloseTo(G * Math.sin(Math.toRadians(10)), within(1e-9));
    assertThat(read.accelerationYMetersPerSecSq)
        .isCloseTo(G * Math.sin(Math.toRadians(5)) * Math.cos(Math.toRadians(10)), within(1e-9));
  }

  /**
   * Mounted 20 cm ahead of the center and 10 cm left, on a robot spinning steadily in place, it
   * reads the centripetal pull toward the center, ω² times its distance. It samples the robot's
   * velocity a step apart, so it reads the step's average: its size short by (ωΔt)²/24 and its
   * direction turned back by half a step's turn, ωΔt/2; physics' instant within those.
   */
  @Test
  void offTheCenterOfASpinningRobotItReadsTheCentripetalPull() {
    Translation2d place = new Translation2d(0.2, 0.1);
    TruthRig rig = rig(EXACT_SAMPLED, place);
    double omega = 3.0;
    for (int i = 0; i < 4; i++) {
      Translation2d wheel = rig.wheels[i];
      rig.modules[i].setSteerAngle(wheel.getAngle().orElseThrow().plus(Rotation2d.CCW_90DEG));
      double speed = omega * wheel.getNorm();
      rig.modules[i].setDriveVelocity(
          speed / rig.wheelRadius(), Robots.SIM.drivetrain().gains().driveKv() * speed);
    }
    rig.run(2); // up to speed, and steady

    rig.loops(1);
    double rate = rig.simulation.yawRateRadiansPerSecond();
    GyroIO.GyroIOInputs inputs = read(rig);
    Translation2d read =
        new Translation2d(inputs.accelerationXMetersPerSecSq, inputs.accelerationYMetersPerSecSq);
    Translation2d pull = place.times(-rate * rate);
    double halfStep = rate * TruthRig.STEP_SECONDS / 2;

    assertThat(rate).as("spinning").isCloseTo(omega, within(0.05));
    assertThat(read.getNorm())
        .isCloseTo(pull.getNorm(), within(pull.getNorm() * (halfStep * halfStep / 6 + 1e-6)));
    assertThat(read.getAngle().orElseThrow().minus(pull.getAngle().orElseThrow()).getRadians())
        .isCloseTo(-halfStep, within(1e-4));
  }

  /**
   * Speeding up straight ahead at its drive motors' stator limit, it reads how fast the robot's
   * speed changed over its last step: the robot's own acceleration, within 0.1% (it changes that
   * little within a loop).
   */
  @Test
  void speedingUpItReadsTheRobotsAcceleration() {
    TruthRig rig = rig(EXACT_SAMPLED, Translation2d.ZERO);
    rig.driveVolts(12);
    rig.run(0.1);

    double before = rig.velocity().getX();
    rig.loops(1);
    double acceleration = (rig.velocity().getX() - before) / 0.02;

    assertThat(read(rig).accelerationXMetersPerSecSq)
        .isCloseTo(acceleration, within(1e-3 * acceleration));
  }

  /**
   * With a Pigeon 2's noise and bias, and nothing else, standing still: its readings average to the
   * bias (within three standard errors) and spread as its noise (within 10%, three standard errors
   * of a spread from 500 readings).
   */
  @Test
  void itsReadingsSpreadAsItsNoiseAroundItsBias() {
    SensorErrors.AccelErrors noisy =
        SensorErrorsAccelErrorsBuilder.builder(EXACT_SAMPLED)
            .noise(0.3)
            .biasForward(0.08)
            .biasLeft(-0.05)
            .build();
    TruthRig rig = rig(noisy, Translation2d.ZERO);
    int n = 500;
    double sum = 0;
    double sumSquares = 0;
    double sumLeft = 0;
    for (int i = 0; i < n; i++) {
      rig.loops(1);
      GyroIO.GyroIOInputs read = read(rig);
      sum += read.accelerationXMetersPerSecSq;
      sumSquares += read.accelerationXMetersPerSecSq * read.accelerationXMetersPerSecSq;
      sumLeft += read.accelerationYMetersPerSecSq;
    }
    double mean = sum / n;
    double spread = Math.sqrt(sumSquares / n - mean * mean);

    assertThat(mean).isCloseTo(0.08, within(3 * 0.3 / Math.sqrt(n)));
    assertThat(sumLeft / n).isCloseTo(-0.05, within(3 * 0.3 / Math.sqrt(n)));
    assertThat(spread).isCloseTo(0.3, within(0.03));
  }

  /** A 4 g hit reads exactly its range, 2 g, as a Pigeon 2's does. */
  @Test
  void aHitPastItsRangeReadsItsRangeExactly() {
    SensorErrors.AccelErrors ranged =
        SensorErrorsAccelErrorsBuilder.builder(EXACT_SAMPLED).rangeMetersPerSecSq(2 * G).build();
    TruthRig rig = rig(ranged, Translation2d.ZERO);
    rig.loops(1);

    rig.simulation.hit(new Transform2d(0.3, 0, Rotation2d.ZERO));
    rig.loops(1);

    assertThat(read(rig).accelerationXMetersPerSecSq).isEqualTo(2 * G);
  }

  /** At 100 Hz, its reading changes every 10 ms, and holds between: 100 times a second. */
  @Test
  void itTakesANewSampleEvery10Milliseconds() {
    SensorErrors.AccelErrors noisy =
        SensorErrorsAccelErrorsBuilder.builder(EXACT_SAMPLED).noise(0.3).build();
    TruthRig rig = rig(noisy, Translation2d.ZERO);
    AccelerometerSim part =
        new AccelerometerSim(rig.simulation, noisy, Robots.SIM.definition().imus().gyro(), 7);

    int changes = 0;
    Translation2d last = part.read();
    for (int step = 0; step < 250; step++) {
      part.step(TruthRig.STEP_SECONDS);
      Translation2d now = part.read();
      changes += now.equals(last) ? 0 : 1;
      last = now;
    }

    assertThat(changes).isEqualTo(100);
  }

  /** The template's robot, its gyro's accelerometer reading like this, mounted here. */
  private static TruthRig rig(SensorErrors.AccelErrors accelerometer, Translation2d place) {
    ImuMount mount =
        ImuMountBuilder.builder(Robots.SIM.definition().imus().gyro())
            .forward(Meters.of(place.getX()))
            .left(Meters.of(place.getY()))
            .build();
    return new TruthRig(
        Robots.SIM.definition().body(),
        Robots.SIM.definition().drivetrain().modules().wheelCoefficientOfFriction(),
        SensorErrors.NONE.withAccelerometer(accelerometer),
        mount);
  }

  private static GyroIO.GyroIOInputs read(TruthRig rig) {
    GyroIO.GyroIOInputs inputs = new GyroIO.GyroIOInputs();
    rig.simulation.gyro().updateInputs(inputs);
    for (ModuleIO module : rig.modules) {
      module.updateInputs(new ModuleIO.ModuleIOInputs());
    }
    return inputs;
  }
}
