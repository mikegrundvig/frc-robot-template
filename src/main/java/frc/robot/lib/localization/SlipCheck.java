package frc.robot.lib.localization;

import static frc.robot.lib.config.Rules.nonNegative;
import static frc.robot.lib.config.Rules.positive;

import frc.robot.lib.config.Checks;
import frc.robot.lib.config.ConfigRecord;
import java.util.ArrayDeque;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Compares the wheels with the IMU, to catch the wheels sliding over the floor together, which
 * their disagreement with each other (the kinematic residual) can't show, and to tell a collision
 * from hard driving:
 *
 * <ul>
 *   <li><b>The turn.</b> Over a sliding window, the gyro and the wheels should turn the robot
 *       alike. Beyond the gyro's noise and a share of the turn (worn tread turns the wheels'
 *       reckoning a little more or less than the robot), the difference is how far the wheels slid
 *       around the robot's center: at their reach, over the window, how fast. A tank drive's two
 *       sides always agree with each other, so on one this is the only check.
 *   <li><b>The speed.</b> The wheels' velocity, carried to where the IMU sits (an IMU off the
 *       center is swung around it as the robot turns), should change as the accelerometer feels it
 *       change. What it doesn't feel, added up loop by loop, is how much faster or slower than the
 *       robot the wheels roll: wheels spinning in a launch, or a push they don't see. It's let go
 *       no faster than the most the accelerometer's bias could add, so a bias can't build up, while
 *       a slide that holds at a steady speed stays seen; when it stops (the robot catches up with
 *       its wheels, or the push lets go), the accelerometer feels that too, and it's gone. Beyond
 *       the accelerometer's noise, that's how fast the wheels slide. Only on a robot whose
 *       accelerometer is known to read along its axes ({@link ImuPlacement#accelerometerChecked}).
 *   <li><b>A jolt.</b> The accelerometer feeling more than {@link Settings#joltMetersPerSecSq} is a
 *       collision, as far as the wheels' own change of velocity doesn't explain it (a robot
 *       stopping hard is felt by both, a hit only by the IMU): what they don't explain, against the
 *       limit, is its share of one. A reading at the accelerometer's range is a whole one, whatever
 *       the wheels did, and without a checked accelerometer its size alone decides. The speed
 *       comparison starts over after a whole one: the collision's doubt covers it. A share of one
 *       stays in it as well, so a slide it started is still seen.
 * </ul>
 *
 * <p>The turn and the speed each say how fast the wheels may be sliding over the floor; together,
 * the root of the sum of their squares, as a slide's turn at the wheels and its push add. The doubt
 * it adds is the distance the wheels may have slid, so it grows with the evidence.
 *
 * <p>Neither sees a slide that neither turns the robot nor changes its speed (wheels spinning at a
 * steady speed, wheels in the air): only the cameras find that. Nor does either ask whether the
 * drive motors are at their current limit: that says the motors push as hard as they may, not that
 * the wheels slip, and a limit set below the tread's grip keeps them from it.
 */
public final class SlipCheck {
  /**
   * How closely the wheels must agree with the IMU.
   *
   * @param windowSeconds how far back the turn's comparison looks: long enough for the gyro's noise
   *     to average out, short enough that a slide shows while it happens
   * @param turnToleranceDegrees how far apart the wheels and the gyro may turn over the window,
   *     however little it turned: the gyro's noise, and wheels worn unevenly, which turn the
   *     wheels' reckoning as the robot drives straight
   * @param turnScaleTolerance and this share of the turning on top: worn tread turns the wheels'
   *     reckoning a little further than the robot, the gyro's scale a little less
   * @param speedBiasMetersPerSecSq the most the accelerometer's bias may be: what it didn't feel is
   *     let go at this rate and no faster, so a bias up to it never builds up, and a slide stays
   *     seen until it stops
   * @param speedToleranceMetersPerSecond how far the wheels' speed and the accelerometer's
   *     reckoning of the robot's may drift apart without a slide: its noise, and its lag as the
   *     robot's acceleration changes
   * @param joltMetersPerSecSq an acceleration past this is a collision, as far as the wheels don't
   *     explain it: what they don't, against this, is its share of one
   * @param accelerometerRangeMetersPerSecSq the most the accelerometer reports on each axis: a
   *     reading at it is a jolt past what it can say, whatever the wheels did
   */
  @ConfigRecord
  public record Settings(
      double windowSeconds,
      double turnToleranceDegrees,
      double turnScaleTolerance,
      double speedBiasMetersPerSecSq,
      double speedToleranceMetersPerSecond,
      double joltMetersPerSecSq,
      double accelerometerRangeMetersPerSecSq) {
    /**
     * The turn over a quarter second, within half a degree and 5% of it; the speed let go at 0.15
     * m/s² and within 0.3 m/s; a jolt past 15 m/s², under a Pigeon 2's ±2 g range.
     */
    public static final Settings DEFAULTS =
        builder()
            .windowSeconds(0.25)
            .turnToleranceDegrees(0.5)
            .turnScaleTolerance(0.05)
            .speedBiasMetersPerSecSq(0.15)
            .speedToleranceMetersPerSecond(0.3)
            .joltMetersPerSecSq(15)
            .accelerometerRangeMetersPerSecSq(2 * ImuReadings.STANDARD_GRAVITY)
            .build();

    public Settings {
      Checks.on(Settings.class)
          .field("windowSeconds", windowSeconds, positive())
          .field("turnToleranceDegrees", turnToleranceDegrees, nonNegative())
          .field("turnScaleTolerance", turnScaleTolerance, nonNegative())
          .field("speedBiasMetersPerSecSq", speedBiasMetersPerSecSq, nonNegative())
          .field("speedToleranceMetersPerSecond", speedToleranceMetersPerSecond, nonNegative())
          .field("joltMetersPerSecSq", joltMetersPerSecSq, positive())
          .field("accelerometerRangeMetersPerSecSq", accelerometerRangeMetersPerSecSq, positive())
          .check(
              "joltMetersPerSecSq",
              () ->
                  joltMetersPerSecSq < accelerometerRangeMetersPerSecSq
                      ? null
                      : joltMetersPerSecSq
                          + " must be under the accelerometer's range ("
                          + accelerometerRangeMetersPerSecSq
                          + "), or a jolt along one axis can never pass it")
          .orThrow();
    }

    /** Starts building the settings, one named value at a time. */
    public static SlipCheckSettingsBuilder.WindowSecondsStage builder() {
      return SlipCheckSettingsBuilder.builder();
    }
  }

  /**
   * One loop of what the wheels and one IMU read. A part not a number starts its comparison over.
   *
   * @param seconds how long the loop was: since the last sample's wheel reading
   * @param wheelTurnRadians how far the wheels turned the robot over the loop
   * @param imuTurnRadians how far the IMU turned it over the same readings; not a number when this
   *     IMU didn't read both ends of them
   * @param yawRadians the IMU's yaw at the loop's end, which holds still while the robot turns: the
   *     frame the speed is compared in. Not a number without an IMU.
   * @param yawRateRadiansPerSecond how fast the IMU turns at the loop's end, counterclockwise; not
   *     a number if it can't say, and the wheels' turn rate stands in
   * @param imuPlace where the IMU sits, from the robot's center
   * @param wheels the robot's velocity as its wheels measure it, robot-relative, at the loop's end
   * @param forwardAccelerationMetersPerSecSq the acceleration the IMU felt over the loop along the
   *     robot's forward axis, without gravity's share; not a number when it can't say
   * @param leftAccelerationMetersPerSecSq the same, along the robot's left axis
   */
  public record Sample(
      double seconds,
      double wheelTurnRadians,
      double imuTurnRadians,
      double yawRadians,
      double yawRateRadiansPerSecond,
      Translation2d imuPlace,
      ChassisVelocities wheels,
      double forwardAccelerationMetersPerSecSq,
      double leftAccelerationMetersPerSecSq) {
    public Sample {
      if (!(seconds > 0) || Double.isInfinite(seconds)) {
        throw new IllegalArgumentException("a loop takes time: " + seconds);
      }
    }
  }

  /** One loop's turns: how far the wheels, and the IMU, turned the robot over it. */
  private record Turn(double seconds, double wheels, double imu) {}

  private final double windowSeconds;
  private final double turnToleranceRadians;
  private final double turnScaleTolerance;
  private final double speedBias;
  private final double speedTolerance;
  private final double joltLimit;
  private final double saturation;
  private final double reachMeters;
  private final boolean accelerometerChecked;

  private final ArrayDeque<Turn> turns = new ArrayDeque<>();
  private double turnsSeconds;

  /**
   * The wheels' velocity carried to the IMU, at the last loop's end, in the IMU's fixed frame; not
   * a number at first.
   */
  private double lastVelocityX = Double.NaN;

  private double lastVelocityY = Double.NaN;

  /** How much faster than the robot the wheels roll, as the accelerometer reckons it. */
  private double slideX;

  private double slideY;

  private double turnApart;
  private double turnSlip;
  private double speedSlip;
  private double collision;

  /**
   * @param settings how closely the wheels must agree with the IMU
   * @param wheelReachMeters how far the wheels sit from the robot's center: a turn they see and the
   *     gyro doesn't, times this, is how far they slid
   * @param accelerometerChecked whether the accelerometer is known to read along the robot's axes
   *     ({@link ImuPlacement#accelerometerChecked}): without it, the speed isn't compared, and a
   *     jolt is judged by its size alone
   */
  public SlipCheck(Settings settings, double wheelReachMeters, boolean accelerometerChecked) {
    if (!(wheelReachMeters > 0) || Double.isInfinite(wheelReachMeters)) {
      throw new IllegalArgumentException("the wheels' reach must be positive: " + wheelReachMeters);
    }
    windowSeconds = settings.windowSeconds();
    turnToleranceRadians = Math.toRadians(settings.turnToleranceDegrees());
    turnScaleTolerance = settings.turnScaleTolerance();
    speedBias = settings.speedBiasMetersPerSecSq();
    speedTolerance = settings.speedToleranceMetersPerSecond();
    joltLimit = settings.joltMetersPerSecSq();
    // A reading within a hair of the range is at it.
    saturation = 0.99 * settings.accelerometerRangeMetersPerSecSq();
    reachMeters = wheelReachMeters;
    this.accelerometerChecked = accelerometerChecked;
  }

  /** One loop's readings. */
  public void update(Sample sample) {
    compareTurns(sample.seconds(), sample.wheelTurnRadians(), sample.imuTurnRadians());
    compareSpeeds(sample);
  }

  /**
   * A loop without a sample to compare: no wheel readings, or no IMU, judged by the size of what an
   * IMU felt alone. Both comparisons start over.
   *
   * @param forwardAccelerationMetersPerSecSq what an IMU felt along the robot's forward axis,
   *     without gravity's share, or not a number without one
   * @param leftAccelerationMetersPerSecSq the same, along its left axis
   */
  public void startOver(
      double forwardAccelerationMetersPerSecSq, double leftAccelerationMetersPerSecSq) {
    turns.clear();
    turnsSeconds = 0;
    turnApart = 0;
    turnSlip = 0;
    forgetSpeeds();
    lastVelocityX = Double.NaN;
    lastVelocityY = Double.NaN;
    collision =
        joltBySize(forwardAccelerationMetersPerSecSq, leftAccelerationMetersPerSecSq) ? 1 : 0;
  }

  /** Whether an acceleration is a jolt by its size alone: past the limit, or at the range. */
  private boolean joltBySize(double forward, double left) {
    return Math.hypot(forward, left) > joltLimit || saturated(forward, left);
  }

  /** Whether the accelerometer read at its range on either axis: past what it can say. */
  private boolean saturated(double forward, double left) {
    return Math.abs(forward) >= saturation || Math.abs(left) >= saturation;
  }

  /**
   * How far the wheels turned the robot beyond the gyro over the window, in radians, either way:
   * before the tolerances, for tuning them.
   */
  public double turnApartRadians() {
    return turnApart;
  }

  /**
   * How much faster or slower than the robot the wheels roll, as the accelerometer reckons it, in
   * meters per second: before the tolerance, for tuning it.
   */
  public double slideMetersPerSecond() {
    return Math.hypot(slideX, slideY);
  }

  /** How fast the wheels may be sliding, as the gyro sees them turning, in meters per second. */
  public double turnSlipMetersPerSecond() {
    return turnSlip;
  }

  /**
   * How fast the wheels may be sliding, as the accelerometer sees their speed, in meters per
   * second.
   */
  public double speedSlipMetersPerSecond() {
    return speedSlip;
  }

  /** How fast the wheels may be sliding over the floor, in meters per second: zero if they grip. */
  public double slipMetersPerSecond() {
    return Math.hypot(turnSlip, speedSlip);
  }

  /**
   * How much of a collision the last loop's IMU felt, from none to a whole one: a jolt the wheels
   * don't explain past the limit, or one at the accelerometer's range, is a whole one; a jolt past
   * the limit the wheels partly explain, what they don't against the limit.
   */
  public double collisionShare() {
    return collision;
  }

  private void forgetSpeeds() {
    slideX = 0;
    slideY = 0;
    speedSlip = 0;
  }

  private void compareTurns(double seconds, double wheelTurn, double imuTurn) {
    if (!Double.isFinite(wheelTurn) || !Double.isFinite(imuTurn)) {
      turns.clear();
      turnsSeconds = 0;
      turnApart = 0;
      turnSlip = 0;
      return;
    }
    turns.addLast(new Turn(seconds, wheelTurn, imuTurn));
    turnsSeconds += seconds;
    // The fewest loops that cover the window.
    while (turns.size() > 1 && turnsSeconds - turns.getFirst().seconds() >= windowSeconds) {
      turnsSeconds -= turns.removeFirst().seconds();
    }
    double apart = 0;
    double turned = 0;
    for (Turn turn : turns) {
      apart += turn.wheels() - turn.imu();
      turned += turn.imu();
    }
    turnApart = Math.abs(apart);
    double beyond = turnApart - turnToleranceRadians - turnScaleTolerance * Math.abs(turned);
    // How fast, over the loops the window holds.
    turnSlip = beyond > 0 ? beyond * reachMeters / turnsSeconds : 0;
  }

  private void compareSpeeds(Sample sample) {
    double forward = sample.forwardAccelerationMetersPerSecSq();
    double left = sample.leftAccelerationMetersPerSecSq();
    double yaw = sample.yawRadians();
    ChassisVelocities wheels = sample.wheels();
    if (!Double.isFinite(yaw) || !Double.isFinite(wheels.vx) || !Double.isFinite(wheels.vy)) {
      startOver(forward, left);
      return;
    }
    // The wheels' velocity where the IMU sits: the robot's, plus its turn swinging the IMU around.
    double rate =
        Double.isFinite(sample.yawRateRadiansPerSecond())
            ? sample.yawRateRadiansPerSecond()
            : wheels.omega;
    Translation2d place = sample.imuPlace();
    double atImuX = wheels.vx - rate * place.getY();
    double atImuY = wheels.vy + rate * place.getX();
    double cos = Math.cos(yaw);
    double sin = Math.sin(yaw);
    double velocityX = cos * atImuX - sin * atImuY;
    double velocityY = sin * atImuX + cos * atImuY;
    double feltX = cos * forward - sin * left;
    double feltY = sin * forward + cos * left;
    double seconds = sample.seconds();
    boolean before = Double.isFinite(lastVelocityX);
    collision = joltBySize(forward, left) ? 1 : 0;
    if (collision > 0 && accelerometerChecked && before && !saturated(forward, left)) {
      // What the wheels felt too is driving; what they don't explain, the hit's share.
      double unexplained =
          Math.hypot(
              feltX - (velocityX - lastVelocityX) / seconds,
              feltY - (velocityY - lastVelocityY) / seconds);
      collision = Math.min(1, unexplained / joltLimit);
    }
    if (!accelerometerChecked
        || collision >= 1
        || !Double.isFinite(feltX)
        || !Double.isFinite(feltY)) {
      forgetSpeeds();
    } else if (before) {
      // How the wheels' velocity changed over the loop, less what the accelerometer felt...
      slideX += velocityX - lastVelocityX - feltX * seconds;
      slideY += velocityY - lastVelocityY - feltY * seconds;
      // ... let go toward none at the most a bias could have added, and no faster.
      double size = Math.hypot(slideX, slideY);
      double kept = size > speedBias * seconds ? 1 - speedBias * seconds / size : 0;
      slideX *= kept;
      slideY *= kept;
      speedSlip = Math.max(0, size * kept - speedTolerance);
    }
    lastVelocityX = velocityX;
    lastVelocityY = velocityY;
  }
}
