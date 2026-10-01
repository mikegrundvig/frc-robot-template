package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.Random;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;

/** Wheel readings in, the robot's motion (and how well the wheels agree on it) out. */
class KinematicsModelTest {
  private static final Translation2d[] MODULES = {
    new Translation2d(0.3, 0.3),
    new Translation2d(0.3, -0.3),
    new Translation2d(-0.3, 0.3),
    new Translation2d(-0.3, -0.3)
  };
  private final SwerveKinematicsModel swerve = new SwerveKinematicsModel(MODULES);

  /** The wheels' readings after a rigid motion, from readings of zero, all pointing one way. */
  private static OdometrySample afterRigidMotion(double dx, double dy, double dtheta) {
    double[] distances = new double[4];
    double[] angles = new double[4];
    for (int i = 0; i < 4; i++) {
      double travelX = dx - MODULES[i].getY() * dtheta;
      double travelY = dy + MODULES[i].getX() * dtheta;
      distances[i] = Math.hypot(travelX, travelY);
      angles[i] = Math.atan2(travelY, travelX);
    }
    return new OdometrySample(1, distances, angles);
  }

  private static OdometrySample startingFrom(OdometrySample after) {
    double[] angles = new double[4];
    for (int i = 0; i < 4; i++) {
      angles[i] = after.angleRadians(i);
    }
    return new OdometrySample(0, new double[4], angles);
  }

  private BodyMotion motionOf(double dx, double dy, double dtheta) {
    OdometrySample after = afterRigidMotion(dx, dy, dtheta);
    return swerve.motion(startingFrom(after), after);
  }

  @Test
  void aRigidMotionComesBackExactlyWithNoResidual() {
    for (double[] motion :
        new double[][] {{0.1, 0, 0}, {0.05, 0.08, 0}, {0, 0, 0.1}, {0.05, 0.02, 0.03}}) {
      BodyMotion found = motionOf(motion[0], motion[1], motion[2]);

      assertThat(found.dxMeters()).isCloseTo(motion[0], within(1e-12));
      assertThat(found.dyMeters()).isCloseTo(motion[1], within(1e-12));
      assertThat(found.dthetaRadians()).isCloseTo(motion[2], within(1e-12));
      assertThat(found.residualMeters()).isCloseTo(0.0, within(1e-12));
    }
  }

  /**
   * A wheel that spins without the robot moving to match disagrees with every rigid motion, and the
   * residual says how far that put the motion off: no more, no less.
   */
  @Test
  void aSlippingWheelShowsAsAResidualAsLargeAsTheErrorItCauses() {
    OdometrySample after = afterRigidMotion(0.1, 0, 0);
    double[] distances = new double[4];
    double[] angles = new double[4];
    for (int i = 0; i < 4; i++) {
      distances[i] = after.distanceMeters(i);
      angles[i] = after.angleRadians(i);
    }
    distances[0] += 0.04; // the front left wheel spun 4 cm more than the robot moved
    OdometrySample slipped = new OdometrySample(1, distances, angles);

    BodyMotion found = swerve.motion(startingFrom(after), slipped);

    assertThat(found.dxMeters()).as("the others outvote it").isCloseTo(0.11, within(1e-12));
    assertThat(found.residualMeters())
        .as("the centimeter it put the motion off")
        .isCloseTo(0.01, within(1e-12));
    assertThat(found.oneWheelMeters())
        .as("all of it that one wheel's")
        .isCloseTo(found.residualMeters(), within(1e-12));
  }

  /**
   * Wheels that each miss a little (scrub, noise, each reaching a new speed at its own pace) leave
   * every wheel a part of the misses: little of them is counted as one wheel's, a quarter on
   * average. One wheel slipping ten times as far as that, among them, is mostly its own.
   */
  @Test
  void missesSpreadOverTheWheelsAreLittleOfThemOneWheels() {
    Random random = new Random(11);
    double spreadShares = 0;
    double slipShares = 0;
    int trials = 400;
    for (int trial = 0; trial < trials; trial++) {
      double[] angles = new double[4];
      double[] noisy = new double[4];
      double[] slipping = new double[4];
      for (int i = 0; i < 4; i++) {
        angles[i] = random.nextGaussian() * 0.01; // pointing ahead, give or take half a degree
        noisy[i] = 0.02 + random.nextGaussian() * 0.001;
        slipping[i] = noisy[i];
      }
      slipping[trial % 4] += 0.01;
      OdometrySample start = new OdometrySample(0, new double[4], angles);
      BodyMotion spread = swerve.motion(start, new OdometrySample(1, noisy, angles));
      BodyMotion slipped = swerve.motion(start, new OdometrySample(1, slipping, angles));
      spreadShares += spread.oneWheelMeters() / spread.residualMeters();
      slipShares += slipped.oneWheelMeters() / slipped.residualMeters();
    }

    assertThat(spreadShares / trials).as("spread").isBetween(0.1, 0.4);
    assertThat(slipShares / trials).as("one wheel slipping").isGreaterThan(0.8);
  }

  /** With two modules, neither can be told from the other: all of it counts as one wheel's. */
  @Test
  void withTwoModulesItAllCountsAsOneWheels() {
    SwerveKinematicsModel two =
        new SwerveKinematicsModel(new Translation2d(0.3, 0), new Translation2d(-0.3, 0));
    double[] angles = {0, 0};

    BodyMotion found =
        two.motion(
            new OdometrySample(0, new double[2], angles),
            new OdometrySample(1, new double[] {0.1, 0.12}, angles));

    assertThat(found.residualMeters()).isPositive();
    assertThat(found.oneWheelMeters()).isEqualTo(found.residualMeters());
  }

  /**
   * Whichever way a wheel slips, on a rectangular drivetrain too, its residual is within about a
   * tenth of the error it causes.
   */
  @Test
  void onARectangleTheResidualIsCloseToTheError() {
    Translation2d[] rectangle = {
      new Translation2d(0.35, 0.2),
      new Translation2d(0.35, -0.2),
      new Translation2d(-0.35, 0.2),
      new Translation2d(-0.35, -0.2)
    };
    SwerveKinematicsModel rectangular = new SwerveKinematicsModel(rectangle);
    double[] angles = new double[4];
    for (int k = 0; k < 12; k++) {
      double direction = k * Math.PI / 12;
      for (int slipping = 0; slipping < 4; slipping++) {
        double[] distances = new double[4];
        double[] pointing = angles.clone();
        pointing[slipping] = direction;
        distances[slipping] = 0.04; // standing still, one wheel spins 4 cm
        BodyMotion found =
            rectangular.motion(
                new OdometrySample(0, new double[4], pointing),
                new OdometrySample(1, distances, pointing));

        double error = Math.hypot(found.dxMeters(), found.dyMeters());
        assertThat(found.residualMeters()).isCloseTo(error, within(0.11 * error));
        assertThat(found.oneWheelMeters()).isCloseTo(found.residualMeters(), within(1e-9));
      }
    }
  }

  /** A module steering during the interval traveled, on average, along its middle angle. */
  @Test
  void aModuleSteeringDuringTheStepTravelsAlongItsMiddleAngle() {
    double[] start = {0, 0, 0, 0};
    double[] angles0 = {0, 0, 0, 0};
    double[] angles1 = new double[4];
    double[] distances = new double[4];
    for (int i = 0; i < 4; i++) {
      angles1[i] = Math.toRadians(20);
      distances[i] = 0.1;
    }

    BodyMotion found =
        swerve.motion(
            new OdometrySample(0, start, angles0), new OdometrySample(1, distances, angles1));

    assertThat(Math.toDegrees(Math.atan2(found.dyMeters(), found.dxMeters())))
        .isCloseTo(10.0, within(1e-9));
  }

  @Test
  void itAgreesWithWpilibsOdometryWhenTheModulesHoldTheirAngles() {
    SwerveDriveKinematics wpilib = new SwerveDriveKinematics(MODULES);
    Random random = new Random(7);
    for (int trial = 0; trial < 20; trial++) {
      double[] distances = new double[4];
      double[] angles = new double[4];
      SwerveModulePosition[] start = new SwerveModulePosition[4];
      SwerveModulePosition[] end = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        distances[i] = random.nextGaussian() * 0.05;
        angles[i] = random.nextDouble() * 2 * Math.PI - Math.PI;
        start[i] = new SwerveModulePosition(0, new Rotation2d(angles[i]));
        end[i] = new SwerveModulePosition(distances[i], new Rotation2d(angles[i]));
      }

      BodyMotion ours =
          swerve.motion(
              new OdometrySample(0, new double[4], angles),
              new OdometrySample(1, distances, angles));
      Twist2d theirs = wpilib.toTwist2d(start, end);

      assertThat(ours.dxMeters()).isCloseTo(theirs.dx, within(1e-12));
      assertThat(ours.dyMeters()).isCloseTo(theirs.dy, within(1e-12));
      assertThat(ours.dthetaRadians()).isCloseTo(theirs.dtheta, within(1e-12));
    }
  }

  @Test
  void swerveNeedsModulesThatCanFixAMotion() {
    assertThatThrownBy(() -> new SwerveKinematicsModel(new Translation2d(0.3, 0)))
        .hasMessageContaining("at least 2");
    assertThatThrownBy(
            () -> new SwerveKinematicsModel(new Translation2d(0.3, 0), new Translation2d(0.3, 0)))
        .hasMessageContaining("one point");
    OdometrySample two = new OdometrySample(0, new double[2], new double[2]);
    assertThatThrownBy(() -> swerve.motion(two, two)).hasMessageContaining("for 4 modules");
  }

  @Test
  void aTankDriveGoesForwardOnItsAverageAndTurnsOnItsDifference() {
    DifferentialKinematicsModel tank = new DifferentialKinematicsModel(0.6);
    OdometrySample start = new OdometrySample(0, new double[] {0, 0}, new double[] {0, 0});

    BodyMotion straight =
        tank.motion(start, new OdometrySample(1, new double[] {1, 1}, new double[] {0, 0}));
    BodyMotion spin =
        tank.motion(start, new OdometrySample(1, new double[] {-0.3, 0.3}, new double[] {0, 0}));

    assertThat(straight).isEqualTo(new BodyMotion(1, 0, 0, 0));
    assertThat(spin.dxMeters()).isZero();
    assertThat(spin.dthetaRadians()).isCloseTo(1.0, within(1e-12));
    assertThatThrownBy(() -> new DifferentialKinematicsModel(0)).hasMessageContaining("track");
    OdometrySample four = new OdometrySample(0, new double[4], new double[4]);
    assertThatThrownBy(() -> tank.motion(four, four)).hasMessageContaining("2 wheels");
  }

  @Test
  void aSingularMatrixHasNoInverse() {
    double[] out = new double[9];

    assertThat(Matrices.invert3(new double[] {1, 2, 3, 2, 4, 6, 0, 0, 1}, out)).isFalse();
    assertThat(Matrices.invert3(new double[] {2, 0, 0, 0, 4, 0, 0, 0, 5}, out)).isTrue();
    assertThat(out[4]).isEqualTo(0.25);
  }
}
