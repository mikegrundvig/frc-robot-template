package frc.robot.lib.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * The wheels against the IMU, loop by loop: do they slide over the floor together, and was that a
 * hit? Each test drives a robot whose truth it knows (how it moves on the field, and how much
 * faster than that its wheels roll) and checks what the slip check says against that truth.
 */
class SlipCheckTest {
  private static final double LOOP = 0.02;
  private static final double REACH = 0.4;
  private static final SlipCheck.Settings SETTINGS = SlipCheck.Settings.DEFAULTS;
  private static final double TOLERANCE = SETTINGS.speedToleranceMetersPerSecond();

  /**
   * A robot moving on the field, and what its wheels and IMU read of it, loop by loop. The truth is
   * its velocity on the field and its heading; its wheels can roll faster than it moves, and turn
   * it further than it turns. Its IMU sits where it's placed, and is swung around the center as the
   * robot turns.
   */
  private static final class Robot {
    final SlipCheck check;
    final Translation2d imu;
    double heading = 0.3;
    double rate;
    double fieldVx;
    double fieldVy;

    /** How much faster than the robot the wheels roll, in the field's frame. */
    double slideX;

    double slideY;

    /** What the accelerometer reads, turned this far from the robot's axes. */
    double imuTurned;

    Robot() {
      this(Translation2d.ZERO, true);
    }

    Robot(Translation2d imu, boolean checked) {
      this.imu = imu;
      check = new SlipCheck(SETTINGS, REACH, checked);
    }

    /** The IMU's velocity on the field: the robot's, and its swing around the center. */
    Translation2d imuVelocity() {
      Translation2d swing = new Translation2d(-rate * imu.getY(), rate * imu.getX());
      return new Translation2d(fieldVx, fieldVy).plus(swing.rotateBy(new Rotation2d(heading)));
    }

    /**
     * One loop: the robot speeds up by this much on the field and turns at this rate by its end;
     * the wheels turn it further by this much.
     */
    Robot loop(double speedUpX, double speedUpY, double newRate, double wheelsTurnMore) {
      Translation2d before = imuVelocity();
      double turn = (rate + newRate) / 2 * LOOP;
      heading += turn;
      rate = newRate;
      fieldVx += speedUpX;
      fieldVy += speedUpY;
      // What the IMU felt over the loop, along the robot's axes.
      Translation2d felt =
          imuVelocity().minus(before).div(LOOP).rotateBy(new Rotation2d(-heading - imuTurned));
      Translation2d wheels =
          new Translation2d(fieldVx + slideX, fieldVy + slideY).rotateBy(new Rotation2d(-heading));
      check.update(
          new SlipCheck.Sample(
              LOOP,
              turn + wheelsTurnMore,
              turn,
              heading,
              rate,
              imu,
              new ChassisVelocities(wheels.getX(), wheels.getY(), rate),
              felt.getX(),
              felt.getY()));
      return this;
    }

    Robot loops(int count, double speedUpX, double speedUpY, double rate, double wheelsTurnMore) {
      for (int i = 0; i < count; i++) {
        loop(speedUpX, speedUpY, rate, wheelsTurnMore);
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
    Robot robot = new Robot().loops(50, 4.4 * LOOP, 0, 2 * Math.PI, 0);

    assertThat(robot.check.slipMetersPerSecond()).isZero();
    assertThat(robot.check.slideMetersPerSecond()).isLessThan(1e-9);
    assertThat(robot.check.collisionShare()).isZero();
  }

  /**
   * An IMU 0.25 m from the center is swung around it as the robot turns, and feels that: the
   * wheels' velocity is compared where it sits, so a robot spinning up to a turn a second as it
   * drives is no slide.
   */
  @Test
  void anImuOffTheCenterFeelsTheTurnSwingItAndIsNoSlide() {
    Robot robot = new Robot(new Translation2d(0.2, 0.15), true);
    for (int i = 1; i <= 50; i++) {
      robot.loop(2.0 * LOOP, 0, 2 * Math.PI * Math.min(1, i / 25.0), 0);
    }

    assertThat(robot.check.slideMetersPerSecond()).isLessThan(1e-9);
    assertThat(robot.check.slipMetersPerSecond()).isZero();
  }

  /**
   * The wheels turn the robot further than the gyro does: they slid around its center. The check
   * says how fast, at the wheels' reach, less what its tolerance allows.
   */
  @Test
  void aTurnTheGyroDoesntSeeIsASlideAtTheWheels() {
    double rate = 0.1; // rad/s more than the gyro: 4 cm/s at the wheels
    Robot slow = new Robot().loops(20, 0, 0, 0, rate * LOOP);
    Robot fast = new Robot().loops(20, 0, 0, 0, 4 * rate * LOOP);

    // The tolerance, spread over the quarter second it looks back, at the wheels.
    double allowed = Math.toRadians(SETTINGS.turnToleranceDegrees()) * REACH / 0.25;
    assertThat(slow.check.turnSlipMetersPerSecond())
        .as("the slide at the wheels, less what the tolerance allows")
        .isBetween(rate * REACH - allowed, rate * REACH);
    assertThat(fast.check.turnSlipMetersPerSecond() - slow.check.turnSlipMetersPerSecond())
        .as("four times the slide, and its doubt grows with it")
        .isCloseTo(3 * rate * REACH, within(1e-9));
    assertThat(slow.check.speedSlipMetersPerSecond()).isZero();
  }

  @Test
  void aTurnWithinTheToleranceIsNoSlide() {
    double tolerance = Math.toRadians(SETTINGS.turnToleranceDegrees());
    // Wheels worn unevenly: they turn the robot's reckoning a little as it drives straight...
    Robot straight = new Robot().loops(20, 0, 0, 0, tolerance / 14);
    // ... and a little more than it turns.
    Robot turning = new Robot().loops(20, 0, 0, 5, 0.004);

    assertThat(straight.check.slipMetersPerSecond()).isZero();
    assertThat(turning.check.slipMetersPerSecond()).isZero();
  }

  /**
   * Half the wheels' acceleration takes: the robot speeds up at 2.2 m/s², its spinning wheels at
   * 4.4. They come to roll ever faster than it moves, and the check follows them.
   */
  @Test
  void wheelsSpinningInALaunchAreASlide() {
    Robot robot = new Robot();
    double[] slip = new double[3];
    for (int i = 1; i <= 30; i++) {
      robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0);
      if (i % 10 == 0) {
        slip[i / 10 - 1] = robot.check.speedSlipMetersPerSecond();
      }
    }

    double sliding = robot.slideX; // 1.32 m/s faster than the robot moves
    assertThat(slip).as("growing as the slide does").isSorted();
    assertThat(robot.check.slideMetersPerSecond()).isBetween(0.7 * sliding, sliding);
    assertThat(robot.check.speedSlipMetersPerSecond())
        .as("past the tolerance, all of it")
        .isEqualTo(robot.check.slideMetersPerSecond());
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
    for (int i = 1; i <= 11; i++) {
      double catchUp = Math.min(6 * LOOP, robot.slideX);
      robot.slide(robot.slideX - catchUp, 0).loop(catchUp, 0, 0, 0);
    }

    assertThat(robot.slideX).isCloseTo(0, within(1e-12));
    assertThat(robot.check.speedSlipMetersPerSecond()).isZero();
    assertThat(robot.check.slideMetersPerSecond())
        .as("what was let go while it slid, at the bias's rate, is under the tolerance")
        .isLessThan(TOLERANCE);
  }

  /** Pushed sideways while the wheels stand still: the IMU feels the push get going. */
  @Test
  void aPushTheWheelsDontSeeIsASlide() {
    Robot slow = new Robot().loops(5, 0, 0, 0, 0); // standing
    Robot fast = new Robot().loops(5, 0, 0, 0, 0);
    for (int i = 1; i <= 10; i++) {
      // 1.5 and 3 m/s² sideways for a fifth of a second; the wheels roll none of it.
      slow.slide(0, -1.5 * LOOP * i).loop(0, 1.5 * LOOP, 0, 0);
      fast.slide(0, -3 * LOOP * i).loop(0, 3 * LOOP, 0, 0);
    }

    // 0.3 m/s in all, within the accelerometer's tolerance; 0.6 m/s, past it.
    assertThat(slow.check.speedSlipMetersPerSecond()).isZero();
    assertThat(fast.check.slideMetersPerSecond()).isBetween(0.8 * 0.6, 0.6);
    assertThat(fast.check.speedSlipMetersPerSecond()).isPositive();
  }

  /**
   * A push that holds at a steady speed: the accelerometer felt it get going, and nothing since,
   * but the slide stays seen, let go only as fast as a bias could have added it. When the push lets
   * go, the accelerometer feels the robot slow to its wheels' pace, and the slide is over.
   */
  @Test
  void aSteadySlideStaysSeenUntilItStops() {
    Robot robot = new Robot().loops(5, 0, 0, 0, 0);
    for (int i = 1; i <= 100; i++) {
      // To 1 m/s sideways at 3 m/s², then held there: two seconds in all. The wheels roll none of
      // it.
      double speedUp = Math.min(3 * LOOP, 1 - robot.fieldVy);
      robot.slide(0, -(robot.fieldVy + speedUp)).loop(0, speedUp, 0, 0);
    }
    double held = robot.check.slideMetersPerSecond();
    double slipping = robot.check.slipMetersPerSecond();
    for (int i = 1; i <= 25; i++) {
      double slowDown = Math.min(3 * LOOP, robot.fieldVy);
      robot.slide(0, -(robot.fieldVy - slowDown)).loop(0, -slowDown, 0, 0);
    }

    // Let go at the bias's rate: 0.5 m/s of the 1 m/s in two seconds.
    assertThat(held).as("still seen, two seconds on").isGreaterThan(0.45);
    assertThat(slipping).isPositive();
    assertThat(robot.check.slipMetersPerSecond()).as("stopped").isZero();
  }

  /** A slide that turns and one that speeds up, at once: the root of their squares. */
  @Test
  void theTwoAddAsTheRootOfTheirSquares() {
    Robot robot = new Robot();
    for (int i = 1; i <= 20; i++) {
      robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0.002);
    }

    assertThat(robot.check.turnSlipMetersPerSecond()).isPositive();
    assertThat(robot.check.speedSlipMetersPerSecond()).isPositive();
    assertThat(robot.check.slipMetersPerSecond())
        .isCloseTo(
            Math.hypot(
                robot.check.turnSlipMetersPerSecond(), robot.check.speedSlipMetersPerSecond()),
            within(1e-12));
  }

  /** A turn out of the window no longer counts: the doubt it added stays, and no more is added. */
  @Test
  void aTurnThatEndedLeavesTheWindow() {
    Robot robot = new Robot().loops(5, 0, 0, 0, 0.01).loops(14, 0, 0, 0, 0);

    assertThat(robot.check.slipMetersPerSecond()).isZero();
  }

  /**
   * An accelerometer turned a quarter turn from the robot's axes reads every change of speed
   * sideways. Not yet checked, it isn't compared, and a robot speeding up hard isn't taken for
   * sliding; checked, the same reading would be.
   */
  @Test
  void anUncheckedAccelerometerIsntComparedWithTheWheels() {
    Robot unchecked = new Robot(Translation2d.ZERO, false);
    Robot checked = new Robot();
    unchecked.imuTurned = Math.PI / 2;
    checked.imuTurned = Math.PI / 2;

    unchecked.loops(30, 4.4 * LOOP, 0, 0, 0);
    checked.loops(30, 4.4 * LOOP, 0, 0, 0);

    assertThat(unchecked.check.slipMetersPerSecond()).isZero();
    assertThat(checked.check.speedSlipMetersPerSecond()).as("as it would be").isPositive();
  }

  /**
   * A hit: the IMU feels 18 m/s² that the wheels don't. A robot stopping as hard is felt by both.
   * And one felt at the accelerometer's range is past what it can say: a jolt, whatever the wheels
   * did.
   */
  @Test
  void aJoltTheWheelsDontExplainIsACollision() {
    Robot hit = new Robot().loops(10, 0, 0, 0, 0);
    Robot stopping = new Robot();
    stopping.fieldVx = 2;
    stopping.loops(10, 0, 0, 0, 0);
    Robot slammed = new Robot();
    slammed.fieldVx = 2;
    slammed.loops(10, 0, 0, 0, 0);

    hit.slide(-0.36, 0).loop(0.36, 0, 0, 0); // shoved to 0.36 m/s in a loop: 18 m/s²
    stopping.loop(-0.36, 0, 0, 0); // slows as fast, its wheels with it
    slammed.loop(-0.8, 0, 0, 0); // 40 m/s², past the range, its wheels with it

    assertThat(hit.check.collisionShare()).isOne();
    assertThat(stopping.check.collisionShare()).isCloseTo(0, within(1e-12));
    assertThat(slammed.check.collisionShare()).isOne();
    assertThat(hit.check.speedSlipMetersPerSecond()).as("the collision's doubt covers it").isZero();
  }

  /**
   * Hit from the side as it stops hard: the IMU feels 17 m/s², past the limit. The wheels explain
   * the 12 m/s² it slows along its way, not the 12 sideways, which is under the limit on its own:
   * still a share of a collision, as large as it is against the limit. Only a share, the speed
   * comparison goes on with it, so a slide it starts is still seen.
   */
  @Test
  void aHitTheWheelsPartlyExplainIsAShareOfACollision() {
    Robot robot = new Robot();
    robot.fieldVx = 2;
    robot.loops(10, 0, 0, 0, 0);

    robot.slide(0, -0.24).loop(-0.24, 0.24, 0, 0);

    assertThat(robot.check.collisionShare()).isCloseTo(12 / 15.0, within(1e-9));
    assertThat(robot.check.slideMetersPerSecond())
        .as("the speed comparison keeps the sideways 0.24 m/s the wheels didn't see")
        .isCloseTo(0.24, within(0.01));
  }

  /** Without a checked accelerometer, a jolt's size alone decides. */
  @Test
  void anUncheckedAccelerometersJoltIsJudgedByItsSize() {
    Robot stopping = new Robot(Translation2d.ZERO, false);
    stopping.fieldVx = 2;
    stopping.loops(10, 0, 0, 0, 0);

    stopping.loop(-0.36, 0, 0, 0);

    assertThat(stopping.check.collisionShare()).isOne();
  }

  @Test
  void aMissingReadingStartsTheComparisonOver() {
    Robot robot = new Robot().loops(20, 0, 0, 0, 0.004);
    for (int i = 1; i <= 20; i++) {
      robot.slide(2.2 * LOOP * i, 0).loop(2.2 * LOOP, 0, 0, 0.004);
    }
    assertThat(robot.check.turnSlipMetersPerSecond()).isPositive();
    assertThat(robot.check.speedSlipMetersPerSecond()).isPositive();

    robot.check.startOver(Double.NaN, Double.NaN);
    assertThat(robot.check.slipMetersPerSecond()).isZero();
    assertThat(robot.check.slideMetersPerSecond()).isZero();
    assertThat(robot.check.collisionShare()).isZero();

    // Starting over, one loop is only the comparison's start.
    robot.loop(2.2 * LOOP, 0, 0, 0.004);
    assertThat(robot.check.speedSlipMetersPerSecond()).isZero();
    robot.check.startOver(40, 0);
    assertThat(robot.check.collisionShare()).as("a jolt, by its size").isOne();
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
    assertThatThrownBy(() -> new SlipCheck(SETTINGS, 0, true)).hasMessageContaining("reach");
    assertThatThrownBy(
            () ->
                new SlipCheck.Sample(
                    0, 0, 0, 0, 0, Translation2d.ZERO, new ChassisVelocities(), 0, 0))
        .hasMessageContaining("takes time");
  }
}
