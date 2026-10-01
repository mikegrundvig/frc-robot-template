package frc.robot.lib.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The filter's math, fed by hand: what it does with the errors it considers but never estimates.
 * Every number here can be worked out on paper.
 */
class PoseFilterTest {
  private static final double TURN = Math.toRadians(0.25);
  private static final double SHIFT = 0.01;

  private static final double HEADING_SHIFT = Math.toRadians(1);

  /**
   * A gyro whose scale may be off by 0.3%, and cameras turned 0.25°, their positions shifted 1 cm
   * and their headings 1°.
   */
  private static final PoseFilter.Considered CONSIDERED =
      new PoseFilter.Considered(0.003, 0.005, TURN, SHIFT, HEADING_SHIFT, 1.0, Math.toRadians(45));

  /** Nothing considered: every error is new each time, the textbook filter. */
  private static final PoseFilter.Considered TEXTBOOK =
      new PoseFilter.Considered(0, 0, 0, 0, 0, 1.0, Math.toRadians(45));

  private static PoseFilter placed(
      PoseFilter.Considered considered, double meters, double radians) {
    PoseFilter filter = new PoseFilter(considered);
    filter.set(
        new PoseReset(0, Pose2d.ZERO, PoseCovariance.fromStdDevs(meters, meters, radians), "x"), 0);
    return filter;
  }

  /** A position solved with the gyro's heading, from tags this far off. */
  private static VisionMeasurement fix(Translation2d position, Translation2d toTags, double noise) {
    return new VisionMeasurement(
        1,
        new Pose2d(position, Rotation2d.ZERO),
        PoseCovariance.fromStdDevs(noise, noise, Double.POSITIVE_INFINITY),
        toTags,
        "front",
        List.of(26),
        VisionMeasurement.Solver.HEADING_CONSTRAINED);
  }

  /** A measurement of the heading too, from a multi-tag solve. */
  private static VisionMeasurement withHeading(Rotation2d heading, double noise, double radians) {
    return new VisionMeasurement(
        1,
        new Pose2d(Translation2d.ZERO, heading),
        PoseCovariance.fromStdDevs(noise, noise, radians),
        Translation2d.ZERO,
        "front",
        List.of(25, 26),
        VisionMeasurement.Solver.MULTI_TAG);
  }

  private static void apply(PoseFilter filter, VisionMeasurement measurement, int camera) {
    filter.correct(Objects.requireNonNull(filter.innovation(measurement, 0, camera)));
  }

  @Test
  void withNothingConsideredItIsTheTextbookKalmanUpdate() {
    PoseFilter filter = placed(TEXTBOOK, 0.1, 0.01);

    apply(filter, fix(new Translation2d(0.2, 0), Translation2d.ZERO, 0.1), 0);

    // Equal trust: halfway, and the variance halved.
    assertThat(filter.x()).isCloseTo(0.1, within(1e-12));
    assertThat(filter.covariance().xx()).isCloseTo(0.005, within(1e-12));
    assertThat(filter.covariance().heading()).isCloseTo(1e-4, within(1e-15));
  }

  /**
   * However many images one camera takes of the same tags, its shift is in every one: the position
   * can't be surer than it along the tags, nor than its shift and turn together square to them.
   */
  @Test
  void oneCameraCantPlaceTheRobotBetterThanItsRepeatedErrorsAllow() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0); // the heading exactly known
    for (int i = 0; i < 2000; i++) {
      apply(filter, fix(Translation2d.ZERO, new Translation2d(3, 0), 0.05), 0);
    }

    PoseCovariance covariance = filter.covariance();
    assertThat(covariance.xStdDev()).isCloseTo(SHIFT, within(0.02 * SHIFT));
    double square = Math.hypot(SHIFT, 3 * TURN); // 1.65 cm
    assertThat(covariance.yStdDev()).isCloseTo(square, within(0.02 * square));
  }

  /** Two cameras, each with its own shift, together do better than either: by √2. */
  @Test
  void twoCamerasEachWithItsOwnErrorsDoBetter() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0);
    for (int i = 0; i < 2000; i++) {
      apply(filter, fix(Translation2d.ZERO, new Translation2d(3, 0), 0.05), i % 2);
    }

    assertThat(filter.covariance().xStdDev()).isCloseTo(SHIFT / Math.sqrt(2), within(0.02 * SHIFT));
  }

  /**
   * A camera without a place among the considered errors (beyond {@link PoseFilter#MAX_CAMERAS})
   * counts each image's error as new: it averages down as the textbook says.
   */
  @Test
  void aCameraWithoutAPlaceCountsEachImageAsNew() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0);
    for (int i = 0; i < 2500; i++) {
      apply(filter, fix(Translation2d.ZERO, new Translation2d(3, 0), 0.05), -1);
    }

    assertThat(filter.covariance().xStdDev()).isCloseTo(0.001, within(0.0001));
  }

  /**
   * Driving on, the camera sees its tags from elsewhere and its shift is a new one: its images
   * count again, and the position gets surer than standing still allowed.
   */
  @Test
  void asTheRobotMovesACamerasShiftBecomesANewOne() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0);
    for (int i = 0; i < 2000; i++) {
      apply(filter, fix(Translation2d.ZERO, new Translation2d(3, 0), 0.05), 0);
    }
    double standing = filter.covariance().xStdDev();

    // Two meters straight ahead, sure of every millimeter: the shift is mostly forgotten.
    filter.predict(0, 0, 2, 0, 0, 0, YawSource.GYRO);
    for (int i = 0; i < 2000; i++) {
      apply(filter, fix(new Translation2d(2, 0), new Translation2d(3, 0), 0.05), 0);
    }

    // Even knowing the shift had changed exactly by e^-2, nothing could do better than averaging
    // old and new: the shift times √((1 + e^-2) / 2), 0.75 of it.
    assertThat(standing).isCloseTo(SHIFT, within(0.02 * SHIFT));
    assertThat(filter.covariance().xStdDev()).isLessThan(0.85 * standing);
  }

  /**
   * A gyro's scale error grows the heading's in proportion to the turn, the same each turn: a turn
   * one way and back again leaves no doubt from it at all, where a random walk would leave √2.
   */
  @Test
  void theGyrosScaleErrorIsTheSameEveryTurn() {
    PoseFilter filter = placed(CONSIDERED, 0.01, 0);

    filter.predict(0, 2 * Math.PI, 0, 0, 0, 0, YawSource.GYRO);
    double afterATurn = filter.covariance().headingStdDev();
    filter.predict(2 * Math.PI, 2 * Math.PI, 0, 0, 0, 0, YawSource.GYRO);
    double afterTwo = filter.covariance().headingStdDev();
    filter.predict(4 * Math.PI, -4 * Math.PI, 0, 0, 0, 0, YawSource.GYRO);

    assertThat(afterATurn).isCloseTo(0.003 * 2 * Math.PI, within(1e-12));
    assertThat(afterTwo).as("twice as far off").isCloseTo(2 * afterATurn, within(1e-12));
    assertThat(filter.covariance().headingStdDev()).as("back where it started").isLessThan(1e-9);
  }

  /**
   * Each IMU's scale is its own: three turns one way on the gyro, then three back on the backup IMU
   * while the gyro is lost, leave the heading as unsure as both scales over three turns each, not
   * back where it started as one scale's turns there and back would.
   */
  @Test
  void eachImusScaleIsItsOwn() {
    PoseFilter filter = placed(CONSIDERED, 0.01, 0);

    filter.predict(0, 6 * Math.PI, 0, 0, 0, 0, YawSource.GYRO);
    filter.predict(6 * Math.PI, -6 * Math.PI, 0, 0, 0, 0, YawSource.BACKUP);

    double gyro = 0.003 * 6 * Math.PI;
    double backup = 0.005 * 6 * Math.PI;
    assertThat(filter.covariance().headingStdDev())
        .isCloseTo(Math.hypot(gyro, backup), within(1e-12));
  }

  /** Turned by the wheels while the gyro is lost, the gyro's scale has nothing to do with it. */
  @Test
  void theWheelsTurningLeaveTheGyrosScaleOut() {
    PoseFilter filter = placed(CONSIDERED, 0.01, 0);

    filter.predict(0, 2 * Math.PI, 0, 0, 0, 0, YawSource.WHEELS);

    assertThat(filter.covariance().headingStdDev()).isZero();
  }

  /**
   * A camera's headings share its turn on its mount and its shift: however many there are, they
   * can't make the heading surer than those two together.
   */
  @Test
  void oneCamerasHeadingsCantBeatItsTurnAndShift() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0.1);
    for (int i = 0; i < 2000; i++) {
      apply(filter, withHeading(Rotation2d.ZERO, 0.5, 0.05), 0);
    }

    // A little above it: the images' own noise, 0.05 rad over 2,000 of them, adds in squares.
    double shared = Math.hypot(TURN, HEADING_SHIFT);
    assertThat(filter.covariance().headingStdDev()).isCloseTo(shared, within(0.03 * shared));
  }

  @Test
  void aMeasurementWithHeadingTurnsTheHeading() {
    PoseFilter filter = placed(TEXTBOOK, 1.0, 0.1);

    apply(filter, withHeading(new Rotation2d(0.1), 1.0, 0.1), 0);

    assertThat(filter.offset()).isCloseTo(0.05, within(1e-9));
  }

  /** What the history keeps, it gives back exactly: the covariance and every relation. */
  @Test
  void theUncertaintyComesBackAsKept() {
    PoseFilter filter = placed(CONSIDERED, 0.1, 0.02);
    filter.predict(0, 1, 1, 0.5, 0.001, 0.0001, YawSource.GYRO);
    apply(filter, fix(new Translation2d(1, 0.6), new Translation2d(2, 1), 0.05), 3);
    double[] kept = new double[PoseFilter.UNCERTAINTY];
    filter.copyUncertaintyTo(kept, 0);
    PoseFilter other = new PoseFilter(CONSIDERED);

    other.restore(filter.x(), filter.y(), filter.offset(), kept, 0);
    apply(filter, fix(new Translation2d(1.1, 0.5), new Translation2d(2, 1), 0.05), 3);
    apply(other, fix(new Translation2d(1.1, 0.5), new Translation2d(2, 1), 0.05), 3);

    assertThat(other.covariance()).isEqualTo(filter.covariance());
    assertThat(other.x()).isEqualTo(filter.x());
  }

  /**
   * Set afresh, a position or a heading has nothing to do with what came before: the next camera
   * fix weighs only its own errors against it.
   */
  @Test
  void aResetForgetsHowTheEstimateRelatedToTheConsideredErrors() {
    PoseFilter filter = placed(CONSIDERED, 1.0, 0);
    for (int i = 0; i < 2000; i++) {
      apply(filter, fix(Translation2d.ZERO, new Translation2d(3, 0), 0.05), 0);
    }
    filter.shift(0, 0, 0.0001);
    PoseFilter fresh = placed(CONSIDERED, 0.01, 0);

    apply(filter, fix(new Translation2d(0.01, 0), new Translation2d(3, 0), 0.05), 0);
    apply(fresh, fix(new Translation2d(0.01, 0), new Translation2d(3, 0), 0.05), 0);

    assertThat(filter.x()).isCloseTo(fresh.x(), within(1e-12));
    assertThat(filter.covariance().xx()).isCloseTo(fresh.covariance().xx(), within(1e-15));
  }
}
