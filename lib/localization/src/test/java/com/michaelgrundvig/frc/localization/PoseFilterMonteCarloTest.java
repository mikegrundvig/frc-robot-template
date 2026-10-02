package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.Objects;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * The filter against a world that errs exactly as it claims: every error it considers drawn at the
 * spread it considers, every image's noise at the spread the image claims. Its NEES must then
 * average its number of dimensions, 2 for position and 1 for heading, at every moment: above, the
 * bookkeeping of how the estimate relates to the considered errors is wrong somewhere; below, it
 * claims less than it knows.
 *
 * <p>The world: four cameras on mounts turned and shifted a little, each seeing tags 3.5 m off, and
 * a gyro whose scale is off.
 */
class PoseFilterMonteCarloTest {
  private static final double DT = 0.02;
  private static final double SCALE = 1.0 / 360;
  private static final double TURN = Math.toRadians(0.25);
  private static final double SHIFT = 0.01;
  private static final double HEADING_SHIFT = Math.toRadians(2);
  private static final double SHIFT_METERS = 1.0;
  private static final double SHIFT_RADIANS = Math.toRadians(45);
  private static final double NOISE = 0.02;
  private static final double HEADING_NOISE = Math.toRadians(3);
  private static final double TRANSLATION_PER_RADIAN = 0.02;
  private static final double HEADING_PER_RADIAN = Math.toRadians(0.1) / Math.sqrt(2 * Math.PI);

  /** Where each camera looks, from the robot's heading, and how far its tags are. */
  private static final double[] LOOKS = {Math.toRadians(23), Math.toRadians(-23), Math.PI, Math.PI};

  private static final double DISTANCE = 3.5;

  /** One world: the truth, what it drew, and the filter following it. */
  private static final class World {
    final SplittableRandom random;
    final PoseFilter filter =
        new PoseFilter(
            new PoseFilter.Considered(
                SCALE, 2 * SCALE, TURN, SHIFT, HEADING_SHIFT, SHIFT_METERS, SHIFT_RADIANS));
    double x;
    double y;
    double heading;
    double yaw; // the gyro's
    final double scale;
    final double[] turn = new double[LOOKS.length];
    final double[][] shift = new double[LOOKS.length][3];

    World(long seed) {
      random = new SplittableRandom(seed);
      scale = SCALE * random.nextGaussian();
      for (int c = 0; c < LOOKS.length; c++) {
        turn[c] = TURN * random.nextGaussian();
        shift[c][0] = SHIFT * random.nextGaussian();
        shift[c][1] = SHIFT * random.nextGaussian();
        shift[c][2] = HEADING_SHIFT * random.nextGaussian();
      }
      double placed = 0.05;
      double placedHeading = Math.toRadians(2);
      x = 3 + placed * random.nextGaussian();
      y = 4 + placed * random.nextGaussian();
      heading = placedHeading * random.nextGaussian();
      filter.set(
          new PoseReset(
              0,
              new Pose2d(3, 4, Rotation2d.ZERO),
              PoseCovariance.fromStdDevs(placed, placed, placedHeading),
              "placed"),
          yaw);
    }

    /** One loop: turning at this rate, then every camera's image, with its heading or without. */
    void loop(double rate, boolean headings) {
      double trueTurn = rate * DT;
      double read = (1 + scale) * trueTurn;
      double positionNoise = TRANSLATION_PER_RADIAN * TRANSLATION_PER_RADIAN * Math.abs(read);
      double headingNoise = HEADING_PER_RADIAN * HEADING_PER_RADIAN * Math.abs(read);
      double turned = read + Math.sqrt(headingNoise) * random.nextGaussian();
      filter.predict(yaw, turned, 0, 0, positionNoise, headingNoise, YawSource.GYRO);
      yaw += turned;
      heading += trueTurn;
      x += Math.sqrt(positionNoise) * random.nextGaussian();
      y += Math.sqrt(positionNoise) * random.nextGaussian();
      // What a camera's images share fades as it turns, to a fresh one.
      double kept = Math.exp(-Math.abs(turned) / SHIFT_RADIANS);
      double fresh = Math.sqrt(1 - kept * kept);
      for (int c = 0; c < LOOKS.length; c++) {
        shift[c][0] = kept * shift[c][0] + fresh * SHIFT * random.nextGaussian();
        shift[c][1] = kept * shift[c][1] + fresh * SHIFT * random.nextGaussian();
        shift[c][2] = kept * shift[c][2] + fresh * HEADING_SHIFT * random.nextGaussian();
      }
      for (int c = 0; c < LOOKS.length; c++) {
        image(c, headings);
      }
    }

    /** A camera's image: its tags placed through the estimate's heading, turned by its own turn. */
    private void image(int camera, boolean withHeading) {
      double estimated = yaw + filter.offset();
      double toTags = heading + LOOKS[camera];
      double vx = DISTANCE * Math.cos(LOOKS[camera]);
      double vy = DISTANCE * Math.sin(LOOKS[camera]);
      double solvedWith = estimated - turn[camera];
      double tagX = x + DISTANCE * Math.cos(toTags);
      double tagY = y + DISTANCE * Math.sin(toTags);
      double fixX =
          tagX - (Math.cos(solvedWith) * vx - Math.sin(solvedWith) * vy) + shift[camera][0];
      double fixY =
          tagY - (Math.sin(solvedWith) * vx + Math.cos(solvedWith) * vy) + shift[camera][1];
      fixX += NOISE * random.nextGaussian();
      fixY += NOISE * random.nextGaussian();
      double measuredHeading =
          withHeading
              ? heading + turn[camera] + shift[camera][2] + HEADING_NOISE * random.nextGaussian()
              : estimated;
      VisionMeasurement measurement =
          new VisionMeasurement(
              1,
              new Pose2d(fixX, fixY, new Rotation2d(measuredHeading)),
              PoseCovariance.fromStdDevs(
                  NOISE, NOISE, withHeading ? HEADING_NOISE : Double.POSITIVE_INFINITY),
              new VisionMeasurement.Lever(
                  new Rotation2d(estimated),
                  new Translation2d(
                      DISTANCE * Math.cos(estimated + LOOKS[camera]),
                      DISTANCE * Math.sin(estimated + LOOKS[camera]))),
              "camera " + camera,
              List.of(camera),
              VisionMeasurement.Solver.HEADING_CONSTRAINED);
      filter.correct(Objects.requireNonNull(filter.innovation(measurement, yaw, camera)));
    }

    double positionNees() {
      PoseCovariance p = filter.covariance();
      double ex = filter.x() - x;
      double ey = filter.y() - y;
      double determinant = p.xx() * p.yy() - p.xy() * p.xy();
      return (p.yy() * ex * ex - 2 * p.xy() * ex * ey + p.xx() * ey * ey) / determinant;
    }

    double headingNees() {
      double error = MathUtil.angleModulus(yaw + filter.offset() - heading);
      return error * error / filter.covariance().heading();
    }
  }

  /**
   * Standing with headings, two turns in place on the gyro, standing with headings again: each
   * camera's turn and shift, and the gyro's scale, as considered.
   */
  @Test
  void standingTurningAndStandingAgainItsNeesAveragesItsDimensions() {
    // Each world's draws last all its loops, so the worlds are the sample: a thousand put the mean
    // NEES within about 0.06 of 2, and 0.05 of 1.
    int worlds = 1000;
    int[] phases = {150, 300, 150};
    double[] rates = {0, 4 * Math.PI / (phases[1] * DT), 0};
    double[][] position = new double[phases.length][2];
    double[][] heading = new double[phases.length][2];
    for (int w = 0; w < worlds; w++) {
      World world = new World(w * 31L + 7);
      for (int phase = 0; phase < phases.length; phase++) {
        for (int i = 0; i < phases[phase]; i++) {
          world.loop(rates[phase], rates[phase] == 0);
          position[phase][0] += world.positionNees();
          position[phase][1]++;
          heading[phase][0] += world.headingNees();
          heading[phase][1]++;
        }
      }
    }
    for (int phase = 0; phase < phases.length; phase++) {
      double p = position[phase][0] / position[phase][1];
      double h = heading[phase][0] / heading[phase][1];
      assertThat(p).as("position NEES, phase %d", phase).isBetween(1.7, 2.3);
      assertThat(h).as("heading NEES, phase %d", phase).isBetween(0.8, 1.2);
    }
  }
}
