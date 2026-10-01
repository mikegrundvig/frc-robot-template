package frc.robot.lib.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;
import org.wpilib.math.kinematics.ChassisVelocities;

/** The wheels against the IMU, loop by loop: do they slide over the floor together? */
class SlipCheckTest {
  private static final double LOOP = 0.02;
  private static final double REACH = 0.4;
  private static final SlipCheck.Settings SETTINGS = SlipCheck.Settings.DEFAULTS;

  /** The tolerance for turning, in radians. */
  private static final double TURN_TOLERANCE = Math.toRadians(SETTINGS.turnToleranceDegrees());

  /** What the accelerometer didn't feel fades this much a loop. */
  private static final double FADE = Math.exp(-LOOP / SETTINGS.speedMemorySeconds());

  /** A difference of this much a loop, for this many loops, as the speed check carries it. */
  private static double carried(double perLoop, int loops) {
    double sum = 0;
    for (int i = 0; i < loops; i++) {
      sum = sum * FADE + perLoop;
    }
    return sum;
  }

  /**
   * A robot moving on the field, and what its wheels and IMU read of it, loop by loop. The truth is
   * its velocity on the field and its heading; the wheels can roll faster than it moves, and turn
   * it further than it turns.
   */
  private static final class Robot {
    final SlipCheck check = new SlipCheck(SETTINGS, REACH);
    double heading = 0.3;
    double fieldVx;
    double fieldVy;

    /** How much faster than the robot the wheels roll, in the field's frame. */
    double slideX;

    double slideY;

    /**
     * One loop: the robot speeds up by this much on the field and turns this far; the wheels turn
     * it further by this much.
     */
    Robot loop(double speedUpX, double speedUpY, double turn, double wheelsTurnMore) {
      double before = heading;
      heading += turn;
      fieldVx += speedUpX;
      fieldVy += speedUpY;
      double cos = Math.cos(heading);
      double sin = Math.sin(heading);
      double wheelsX = fieldVx + slideX;
      double wheelsY = fieldVy + slideY;
      ChassisVelocities wheels =
          new ChassisVelocities(cos * wheelsX + sin * wheelsY, -sin * wheelsX + cos * wheelsY, 0);
      double ax = speedUpX / LOOP;
      double ay = speedUpY / LOOP;
      check.update(
          LOOP,
          turn + wheelsTurnMore,
          heading - before,
          heading,
          wheels,
          cos * ax + sin * ay,
          -sin * ax + cos * ay);
      return this;
    }

    Robot loops(int count, double speedUpX, double speedUpY, double turn, double wheelsTurnMore) {
      for (int i = 0; i < count; i++) {
        loop(speedUpX, speedUpY, turn, wheelsTurnMore);
      }
      return this;
    }

    Robot slide(double x, double y) {
      slideX = x;
      slideY = y;
      return this;
    }
  }

  @Test
  void wheelsThatAgreeWithTheImuArentSliding() {
    // Speeding up hard down the field while turning: 4.4 m/s², a turn a second.
    Robot robot = new Robot().loops(50, 4.4 * LOOP, 0, 2 * Math.PI * LOOP, 0);

    assertThat(robot.check.slipMetersPerSecond()).isCloseTo(0, within(1e-9));
  }

  /**
   * The wheels turn the robot further than the gyro does: they slid around its center. Beyond the
   * tolerance, at the wheels' reach, over the window, that's how fast.
   */
  @Test
  void aTurnTheGyroDoesntSeeIsASlide() {
    Robot robot = new Robot().loops(20, 0, 0, 0, 0.002); // 0.1 rad/s more than the gyro

    double apart = 0.002 * 13; // the fewest loops that cover a quarter second
    assertThat(robot.check.turnApartRadians()).isCloseTo(apart, within(1e-12));
    assertThat(robot.check.turnSlipMetersPerSecond())
        .isCloseTo((apart - TURN_TOLERANCE) * REACH / 0.25, within(1e-9));
    assertThat(robot.check.speedSlipMetersPerSecond()).isZero();
    Robot faster = new Robot().loops(20, 0, 0, 0, 0.004);
    assertThat(faster.check.slipMetersPerSecond())
        .as("the doubt grows with the slide")
        .isGreaterThan(2 * robot.check.slipMetersPerSecond());
  }

  @Test
  void aTurnWithinTheToleranceIsNoSlide() {
    // Wheels worn unevenly: they turn the robot's reckoning a little as it drives straight...
    Robot straight = new Robot().loops(20, 0, 0, 0, TURN_TOLERANCE / 14);
    // ... and a little more than it turns.
    Robot turning = new Robot().loops(20, 0, 0, 0.1, 0.004);

    assertThat(straight.check.slipMetersPerSecond()).isZero();
    assertThat(turning.check.slipMetersPerSecond()).isZero();
  }

  /**
   * The wheels speed up faster than the accelerometer feels the robot speed up: they spin, and how
   * much faster they roll than the robot moves builds up as long as they do.
   */
  @Test
  void aSpeedUpTheAccelerometerDoesntFeelIsASlide() {
    Robot robot = new Robot();
    // Half the wheels' acceleration takes: the robot speeds up at 2.2 m/s², the wheels at 4.4.
    double[] slip = new double[3];
    for (int i = 1; i <= 30; i++) {
      robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0);
      if (i % 10 == 0) {
        slip[i / 10 - 1] = robot.check.speedSlipMetersPerSecond();
      }
    }

    // The first loop only starts the comparison: 29 loops of 4.4 cm/s each, fading.
    assertThat(robot.check.slideMetersPerSecond()).isCloseTo(carried(2.2 * LOOP, 29), within(1e-9));
    assertThat(robot.check.speedSlipMetersPerSecond())
        .isCloseTo(
            carried(2.2 * LOOP, 29) - SETTINGS.speedToleranceMetersPerSecond(), within(1e-9));
    assertThat(slip).as("growing as the slide does").isSorted();
    assertThat(slip[2]).isGreaterThan(slip[1] + 0.2);
    assertThat(robot.check.turnSlipMetersPerSecond()).isZero();
  }

  /** Once the robot catches up with its spinning wheels, the accelerometer feels that: no slide. */
  @Test
  void whenTheRobotCatchesUpTheSlideIsOver() {
    Robot robot = new Robot();
    for (int i = 1; i <= 30; i++) {
      robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0);
    }
    assertThat(robot.check.speedSlipMetersPerSecond()).isPositive();

    // The wheels hold their speed; the robot speeds up to it, at 6 m/s², in a fifth of a second.
    double behind = robot.slideX;
    for (int i = 1; i <= 11; i++) {
      double catchUp = Math.min(6 * LOOP, robot.slideX);
      robot.slide(robot.slideX - catchUp, 0).loop(catchUp, 0, 0, 0);
    }

    assertThat(behind).isCloseTo(1.32, within(1e-9));
    assertThat(robot.slideX).isCloseTo(0, within(1e-12));
    // What it carried faded while the robot fell behind, and the catching up didn't: a little of
    // it is left over, the other way, until that fades too.
    assertThat(robot.check.speedSlipMetersPerSecond()).isLessThan(0.05);
  }

  /** Pushed sideways while the wheels stand still: the IMU feels the push get going. */
  @Test
  void aPushTheWheelsDontSeeIsASlide() {
    Robot slow = new Robot().loops(5, 0, 0, 0, 0); // standing
    Robot fast = new Robot().loops(5, 0, 0, 0, 0);
    for (int i = 1; i <= 10; i++) {
      // 3 m/s² sideways for a fifth of a second; the wheels roll none of it.
      slow.slide(0, -1.5 * LOOP * i).loop(0, 1.5 * LOOP, 0, 0);
      fast.slide(0, -3 * LOOP * i).loop(0, 3 * LOOP, 0, 0);
    }

    // 0.3 m/s in all, within the accelerometer's tolerance; 0.6 m/s, past it.
    assertThat(slow.check.speedSlipMetersPerSecond()).isZero();
    assertThat(fast.check.speedSlipMetersPerSecond())
        .isCloseTo(carried(0.06, 10) - SETTINGS.speedToleranceMetersPerSecond(), within(1e-9));
  }

  /** A slide that turns and one that speeds up, at once: the root of their squares. */
  @Test
  void theTwoAddAsTheRootOfTheirSquares() {
    Robot robot = new Robot();
    for (int i = 1; i <= 20; i++) {
      robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0.002);
    }

    assertThat(robot.check.slipMetersPerSecond())
        .isCloseTo(
            Math.hypot(
                robot.check.turnSlipMetersPerSecond(), robot.check.speedSlipMetersPerSecond()),
            within(1e-12));
    assertThat(robot.check.turnSlipMetersPerSecond()).isPositive();
    assertThat(robot.check.speedSlipMetersPerSecond()).isPositive();
  }

  /** A turn out of the window no longer counts: the doubt it added stays, and no more is added. */
  @Test
  void aTurnThatEndedLeavesTheWindow() {
    Robot robot = new Robot().loops(5, 0, 0, 0, 0.01).loops(14, 0, 0, 0, 0);

    assertThat(robot.check.slipMetersPerSecond()).isZero();
  }

  @Test
  void aMissingReadingStartsTheComparisonOver() {
    Robot robot = new Robot().loops(20, 0, 0, 0, 0.004);
    for (int i = 1; i <= 20; i++) {
      robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0.004);
    }
    assertThat(robot.check.turnSlipMetersPerSecond()).isPositive();
    assertThat(robot.check.speedSlipMetersPerSecond()).isPositive();

    // No IMU turn: the turn starts over. No acceleration (a jolt, say): the speed starts over.
    robot.check.update(LOOP, 0, Double.NaN, 0, new ChassisVelocities(), Double.NaN, 0);
    assertThat(robot.check.slipMetersPerSecond()).isZero();
    robot.check.update(LOOP, 0.1, 0, 0, new ChassisVelocities(1, 0, 0), 0, 0);
    assertThat(robot.check.speedSlipMetersPerSecond())
        .as("one loop is only the comparison's start")
        .isZero();

    robot.loops(20, 0, 0, 0, 0.004);
    robot.check.reset();
    assertThat(robot.check.slipMetersPerSecond()).isZero();
  }

  @Test
  void settingsMustMakeSense() {
    assertThatThrownBy(() -> SlipCheckSettingsBuilder.builder(SETTINGS).windowSeconds(0).build())
        .hasMessage("Settings.windowSeconds: 0 must be positive");
    assertThatThrownBy(
            () ->
                SlipCheckSettingsBuilder.builder(SETTINGS)
                    .speedToleranceMetersPerSecond(-1)
                    .build())
        .hasMessage("Settings.speedToleranceMetersPerSecond: -1 must be zero or more");
    assertThatThrownBy(
            () -> SlipCheckSettingsBuilder.builder(SETTINGS).speedMemorySeconds(0).build())
        .hasMessage("Settings.speedMemorySeconds: 0 must be positive");
    assertThatThrownBy(() -> new SlipCheck(SETTINGS, 0)).hasMessageContaining("reach");
    SlipCheck check = new SlipCheck(SETTINGS, REACH);
    assertThatThrownBy(() -> check.update(0, 0, 0, 0, new ChassisVelocities(), 0, 0))
        .hasMessageContaining("takes time");
  }
}
