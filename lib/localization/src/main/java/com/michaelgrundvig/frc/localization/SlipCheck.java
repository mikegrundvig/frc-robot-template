package com.michaelgrundvig.frc.localization;

import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import java.util.ArrayDeque;
import org.wpilib.math.geometry.Translation3d;
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
 *       its wheels, or the push lets go), the accelerometer feels that too, and it's gone. Once
 *       past the accelerometer's noise, all of it is how fast the wheels slide: the tolerance tells
 *       a slide from the noise, it isn't a share of every slide. Whenever the wheels and the IMU
 *       say the robot is standing still, what the accelerometer reads is its bias, which is taken
 *       off what it reads after; and no slide is counted faster than the robot could go. An
 *       accelerometer not yet known to read along the robot's axes ({@link
 *       ImuMount#accelerometerChecked}) is compared by size, not direction: the wheels' change of
 *       velocity and what it felt, each added up on its own, add up alike whichever way it's
 *       turned, and what one holds past the other is a slide.
 *   <li><b>A jolt.</b> What the accelerometer feels that the wheels' own change of velocity doesn't
 *       explain (a robot stopping hard is felt by both, a hit only by the IMU) is a collision as
 *       far as it's past the tread's grip, which is the most the floor can explain (the robot
 *       catching up with wheels that got ahead of it): a whole one at {@link
 *       Settings#joltMetersPerSecSq}, a share of one between. A reading at the accelerometer's
 *       range is a whole one, whatever the wheels did, and without a checked accelerometer the
 *       jolt's size alone decides. The speed comparison starts over after a whole one: the
 *       collision's doubt covers it. A share of one stays in it as well, so a slide it started is
 *       still seen.
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
   *     robot's acceleration changes. Past it, the whole of it is the slide's speed.
   * @param stillMetersPerSecond the robot is standing still when its wheels roll slower than this,
   *     and turn it slower than this at their reach, and the IMU turns it slower than this too
   * @param stillMetersPerSecSq and the accelerometer reads within this of its bias: further, the
   *     robot is being moved (a push getting going), not standing
   * @param biasSeconds how long the accelerometer's reading, standing still, is averaged over for
   *     its bias: long enough to average out its noise, short enough to learn it before the robot
   *     drives
   * @param maxSlideMetersPerSecond the fastest the wheels and the robot can be sliding apart: past
   *     it, what the speed comparison holds is no slide (a bias it hasn't learned, adding up), and
   *     it counts no faster
   * @param joltMetersPerSecSq what the IMU feels and the wheels don't explain, past this, is a
   *     whole collision; past the tread's grip and short of this, a share of one
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
      double stillMetersPerSecond,
      double stillMetersPerSecSq,
      double biasSeconds,
      double maxSlideMetersPerSecond,
      double joltMetersPerSecSq,
      double accelerometerRangeMetersPerSecSq) {
    /**
     * The turn over a quarter second, within half a degree and 5% of it; the speed let go at 0.15
     * m/s² and within 0.3 m/s, its bias learned over half a second still, and no slide past 5 m/s;
     * a jolt past 15 m/s², under a Pigeon 2's ±2 g range.
     */
    public static final Settings DEFAULTS =
        builder()
            .windowSeconds(0.25)
            .turnToleranceDegrees(0.5)
            .turnScaleTolerance(0.05)
            .speedBiasMetersPerSecSq(0.15)
            .speedToleranceMetersPerSecond(0.3)
            .stillMetersPerSecond(0.05)
            .stillMetersPerSecSq(1)
            .biasSeconds(0.5)
            .maxSlideMetersPerSecond(5)
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
          .field("stillMetersPerSecond", stillMetersPerSecond, positive())
          .field("stillMetersPerSecSq", stillMetersPerSecSq, positive())
          .field("biasSeconds", biasSeconds, positive())
          .field("maxSlideMetersPerSecond", maxSlideMetersPerSecond, positive())
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
   * @param imuPosition where the IMU sits, from the robot's origin (its center, on the floor)
   * @param accelerometerChecked whether this IMU's accelerometer is known to read along the robot's
   *     axes ({@link ImuMount#accelerometerChecked}): without it, speeds are compared by size, not
   *     direction, and a jolt is judged by its size alone
   * @param wheels the robot's velocity as its wheels measure it, robot-relative, at the loop's end
   * @param forwardAccelerationMetersPerSecSq the acceleration the IMU felt over the loop along the
   *     robot's forward axis, without gravity's share; not a number when it can't say
   * @param leftAccelerationMetersPerSecSq the same, along the robot's left axis
   * @param tiltRadians how far the robot is tilted from level, by the IMU's attitude: an
   *     accelerometer not yet checked may have gravity's share as the robot tilts taken off the
   *     wrong axis. Not a number, it's taken as level.
   */
  public record Sample(
      double seconds,
      double wheelTurnRadians,
      double imuTurnRadians,
      double yawRadians,
      double yawRateRadiansPerSecond,
      Translation3d imuPosition,
      boolean accelerometerChecked,
      ChassisVelocities wheels,
      double forwardAccelerationMetersPerSecSq,
      double leftAccelerationMetersPerSecSq,
      double tiltRadians) {
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
  private final double stillSpeed;
  private final double stillAcceleration;
  private final double biasSeconds;
  private final double maxSlide;
  private final double joltLimit;
  private final double grip;
  private final double saturation;
  private final double reachMeters;

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

  /**
   * Not known to read along the robot's axes: how far the wheels' velocity has changed past what
   * the accelerometer felt, and how far what it felt has changed past the wheels, on the field. One
   * of the two is always none.
   */
  private double aheadX;

  private double aheadY;
  private double movedX;
  private double movedY;

  private double turnApart;
  private double turnSlip;
  private double speedSlip;
  private double collision;
  private double unexplained;

  /** The accelerometer's bias, along the robot's axes, as learned standing still. */
  private double biasForward;

  private double biasLeft;

  /** How long the robot has been standing still. */
  private double stillSeconds;

  /** How long since the wheels were last seen sliding. */
  private double sinceSlide = Double.POSITIVE_INFINITY;

  /**
   * @param settings how closely the wheels must agree with the IMU
   * @param wheelReachMeters how far the wheels sit from the robot's center: a turn they see and the
   *     gyro doesn't, times this, is how far they slid
   * @param gripMetersPerSecSq the most the tread's grip can change the robot's velocity: its
   *     friction times g. Felt by the IMU and unexplained by the wheels, an acceleration up to it
   *     may be the floor bringing the robot to its wheels' speed, and is no collision.
   */
  public SlipCheck(Settings settings, double wheelReachMeters, double gripMetersPerSecSq) {
    if (!(wheelReachMeters > 0) || Double.isInfinite(wheelReachMeters)) {
      throw new IllegalArgumentException("the wheels' reach must be positive: " + wheelReachMeters);
    }
    windowSeconds = settings.windowSeconds();
    turnToleranceRadians = Math.toRadians(settings.turnToleranceDegrees());
    turnScaleTolerance = settings.turnScaleTolerance();
    speedBias = settings.speedBiasMetersPerSecSq();
    speedTolerance = settings.speedToleranceMetersPerSecond();
    stillSpeed = settings.stillMetersPerSecond();
    stillAcceleration = settings.stillMetersPerSecSq();
    biasSeconds = settings.biasSeconds();
    maxSlide = settings.maxSlideMetersPerSecond();
    joltLimit = settings.joltMetersPerSecSq();
    if (!(gripMetersPerSecSq > 0) || !(gripMetersPerSecSq < joltLimit)) {
      throw new IllegalArgumentException(
          "the tread's grip must be under the collision's jolt ("
              + joltLimit
              + " m/s²): "
              + gripMetersPerSecSq);
    }
    grip = gripMetersPerSecSq;
    // A reading within a hair of the range is at it.
    saturation = 0.99 * settings.accelerometerRangeMetersPerSecSq();
    reachMeters = wheelReachMeters;
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
    // Another IMU may read next, with a bias of its own.
    biasForward = 0;
    biasLeft = 0;
    stillSeconds = 0;
    sinceSlide = Double.POSITIVE_INFINITY;
    collision =
        joltBySize(forwardAccelerationMetersPerSecSq, leftAccelerationMetersPerSecSq) ? 1 : 0;
  }

  /**
   * Learns the accelerometer's bias while the robot stands still: its wheels and its IMU say it
   * isn't moving or turning, and has been for the turn's window, and the accelerometer reads near
   * what it did. Standing still, it feels nothing but its bias.
   */
  private void learnBias(Sample sample) {
    ChassisVelocities wheels = sample.wheels();
    double rate =
        Double.isFinite(sample.yawRateRadiansPerSecond()) ? sample.yawRateRadiansPerSecond() : 0;
    double forward = sample.forwardAccelerationMetersPerSecSq();
    double left = sample.leftAccelerationMetersPerSecSq();
    boolean still =
        Math.hypot(wheels.vx, wheels.vy) < stillSpeed
            && Math.abs(wheels.omega) * reachMeters < stillSpeed
            && Math.abs(rate) * reachMeters < stillSpeed
            && Double.isFinite(forward)
            && Double.isFinite(left)
            && Math.hypot(forward - biasForward, left - biasLeft) < stillAcceleration;
    stillSeconds = still ? stillSeconds + sample.seconds() : 0;
    if (stillSeconds >= windowSeconds) {
      double share = 1 - Math.exp(-sample.seconds() / biasSeconds);
      biasForward += (forward - biasForward) * share;
      biasLeft += (left - biasLeft) * share;
    }
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
    return Math.hypot(slideX, slideY) + Math.hypot(aheadX, aheadY) + Math.hypot(movedX, movedY);
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
   * What the last loop's IMU felt that the wheels' own change of velocity doesn't explain, in m/s²:
   * zero when they explain it, or it felt less than they changed, or its accelerometer isn't
   * checked. For tuning the collision's limits.
   */
  public double unexplainedMetersPerSecSq() {
    return unexplained;
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
    aheadX = 0;
    aheadY = 0;
    movedX = 0;
    movedY = 0;
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
    double yaw = sample.yawRadians();
    ChassisVelocities wheels = sample.wheels();
    if (!Double.isFinite(yaw) || !Double.isFinite(wheels.vx) || !Double.isFinite(wheels.vy)) {
      startOver(
          sample.forwardAccelerationMetersPerSecSq(), sample.leftAccelerationMetersPerSecSq());
      return;
    }
    learnBias(sample);
    double forward = sample.forwardAccelerationMetersPerSecSq() - biasForward;
    double left = sample.leftAccelerationMetersPerSecSq() - biasLeft;
    // The wheels' velocity where the IMU sits: the robot's, plus its turn swinging the IMU around,
    // the turn's rate crossed with the IMU's position. Only the yaw rate is read, which swings it
    // level whatever its height: a robot pitching or rolling would swing a high IMU too.
    double rate =
        Double.isFinite(sample.yawRateRadiansPerSecond())
            ? sample.yawRateRadiansPerSecond()
            : wheels.omega;
    Translation3d position = sample.imuPosition();
    double atImuX = wheels.vx - rate * position.getY();
    double atImuY = wheels.vy + rate * position.getX();
    double cos = Math.cos(yaw);
    double sin = Math.sin(yaw);
    double velocityX = cos * atImuX - sin * atImuY;
    double velocityY = sin * atImuX + cos * atImuY;
    double feltX = cos * forward - sin * left;
    double feltY = sin * forward + cos * left;
    double seconds = sample.seconds();
    boolean before = Double.isFinite(lastVelocityX);
    collision = joltBySize(forward, left) ? 1 : 0;
    boolean checked = sample.accelerometerChecked();
    unexplained = 0;
    if (checked && before && Double.isFinite(feltX) && Double.isFinite(feltY)) {
      double wheelsX = (velocityX - lastVelocityX) / seconds;
      double wheelsY = (velocityY - lastVelocityY) / seconds;
      if (sinceSlide < windowSeconds) {
        // Sliding, or just done (what the IMU feels of the robot catching up lags a little), the
        // wheels' change of velocity says nothing of the robot's: all it felt.
        unexplained = Math.hypot(feltX, feltY);
      } else if (Math.hypot(feltX, feltY) >= Math.hypot(wheelsX, wheelsY)) {
        // What the IMU felt that the wheels don't explain. Felt less than the wheels' own change,
        // it's the wheels changing faster than the robot can follow: a slide, not a hit.
        unexplained = Math.hypot(feltX - wheelsX, feltY - wheelsY);
      }
    }
    if (checked && before && !saturated(forward, left)) {
      // Past the grip, the floor can't explain it: a share of a collision, up to a whole one.
      collision = Math.clamp((unexplained - grip) / (joltLimit - grip), 0.0, 1.0);
    }
    if (collision >= 1 || !Double.isFinite(feltX) || !Double.isFinite(feltY)) {
      forgetSpeeds();
    } else if (before && checked) {
      // How the wheels' velocity changed over the loop, less what the accelerometer felt...
      slideX += velocityX - lastVelocityX - feltX * seconds;
      slideY += velocityY - lastVelocityY - feltY * seconds;
      // ... let go toward none at the most a bias could have added, and no faster.
      double size = Math.hypot(slideX, slideY);
      double kept = kept(size, speedBias * seconds);
      slideX *= kept;
      slideY *= kept;
      speedSlip = slip(size * kept);
    } else if (before) {
      compareSizes(
          velocityX - lastVelocityX,
          velocityY - lastVelocityY,
          feltX * seconds,
          feltY * seconds,
          (speedBias + 2 * ImuReadings.STANDARD_GRAVITY * Math.sin(tilt(sample))) * seconds);
    }
    lastVelocityX = velocityX;
    lastVelocityY = velocityY;
    sinceSlide = speedSlip > 0 || turnSlip > 0 ? 0 : sinceSlide + seconds;
  }

  /** The robot's tilt, from level, or level if the IMU can't say. */
  private static double tilt(Sample sample) {
    return Double.isFinite(sample.tiltRadians()) ? Math.abs(sample.tiltRadians()) : 0;
  }

  /**
   * An accelerometer not known to read along the robot's axes compares sizes, not directions: what
   * it feels, turned on its mount, is the robot's every change of velocity turned the same way, so
   * the two add up to the same size whichever way it's turned. What both have added up alike is
   * driving, and is let go; what one holds past the other is a slide: the wheels getting ahead of
   * the robot, or the robot moved without them. A slide's size is at least that. Its bias, and
   * gravity's share as the robot tilts, which the robot's attitude may take off the wrong axis of a
   * mount not yet checked (at most twice the share), are let go of too.
   *
   * @param wheelsX how the wheels' velocity changed over the loop, on the field
   * @param wheelsY and along y
   * @param feltX what the accelerometer felt over the loop, as a change of velocity on the field
   * @param feltY and along y
   * @param letGo how much of a slide to let go of this loop
   */
  private void compareSizes(
      double wheelsX, double wheelsY, double feltX, double feltY, double letGo) {
    aheadX += wheelsX;
    aheadY += wheelsY;
    movedX += feltX;
    movedY += feltY;
    double ahead = Math.hypot(aheadX, aheadY);
    double moved = Math.hypot(movedX, movedY);
    double alike = Math.min(ahead, moved);
    double aheadKept = ahead > 0 ? (ahead - alike) / ahead : 0;
    double movedKept = moved > 0 ? (moved - alike) / moved : 0;
    aheadX *= aheadKept;
    aheadY *= aheadKept;
    movedX *= movedKept;
    movedY *= movedKept;
    // One of the two is now none: the other is the slide.
    double aheadSize = ahead - alike;
    double movedSize = moved - alike;
    double aheadLetGo = kept(aheadSize, letGo);
    double movedLetGo = kept(movedSize, letGo);
    aheadX *= aheadLetGo;
    aheadY *= aheadLetGo;
    movedX *= movedLetGo;
    movedY *= movedLetGo;
    speedSlip = slip(Math.max(aheadSize * aheadLetGo, movedSize * movedLetGo));
  }

  /**
   * How much of a slide of this size to keep, letting go of up to this much of it toward none, and
   * counting none faster than the robot can go: past that, what's held is a bias not yet learned,
   * adding up.
   */
  private double kept(double size, double letGo) {
    double held = Math.min(Math.max(size - letGo, 0), maxSlide);
    return size > 0 ? held / size : 0;
  }

  /**
   * How fast the wheels slide, from the slide held: past the tolerance, all of it, so the doubt
   * keeps up with the slide from where it's seen; within it, none.
   */
  private double slip(double held) {
    return held > speedTolerance ? held : 0;
  }
}
