package frc.robot.lib.localization;

import static frc.robot.lib.config.Rules.nonNegative;
import static frc.robot.lib.config.Rules.positive;

import frc.robot.lib.config.Checks;
import frc.robot.lib.config.ConfigRecord;
import java.util.ArrayDeque;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Compares the wheels with the IMU, to catch the wheels sliding over the floor together, which
 * their disagreement with each other (the kinematic residual) can't show:
 *
 * <ul>
 *   <li><b>The turn.</b> Over a sliding window, the gyro and the wheels should turn the robot
 *       alike. Beyond the gyro's noise and a share of the turn (worn tread turns the wheels'
 *       reckoning a little more or less than the robot), the difference is how far the wheels slid
 *       around the robot's center: at their reach, over the window, how fast. A tank drive's two
 *       sides always agree with each other, so on one this is the only check.
 *   <li><b>The speed.</b> The wheels' velocity should change as the accelerometer feels the robot's
 *       change. What it doesn't feel, added up loop by loop, is how much faster or slower than the
 *       robot the wheels roll: wheels spinning in a launch, or a push they don't see. It fades as
 *       it's carried, so the accelerometer's bias can't build up; when the robot catches up with
 *       its wheels, the accelerometer feels that too, and it's gone. Beyond the accelerometer's
 *       noise, that's how fast the wheels slide.
 * </ul>
 *
 * <p>Each says how fast the wheels may be sliding over the floor; together, the root of the sum of
 * their squares, as a slide's turn at the wheels and its push add. The doubt it adds is the
 * distance the wheels may have slid, so it grows with the evidence: a little past the tolerances, a
 * little doubt.
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
   * @param speedMemorySeconds how long the speed's comparison remembers what the accelerometer
   *     didn't feel: it fades by e over this, so a bias in the accelerometer adds up to no more
   *     than the bias times this
   * @param speedToleranceMetersPerSecond how far the wheels' speed and the accelerometer's
   *     reckoning of the robot's may drift apart without a slide: its noise and bias over the
   *     memory, and the wheels' own errors (worn tread reads a little fast)
   */
  @ConfigRecord
  public record Settings(
      double windowSeconds,
      double turnToleranceDegrees,
      double turnScaleTolerance,
      double speedMemorySeconds,
      double speedToleranceMetersPerSecond) {
    /**
     * The turn over a quarter second, within half a degree and 5% of it; the speed remembered for a
     * second, within 0.4 m/s.
     */
    public static final Settings DEFAULTS =
        builder()
            .windowSeconds(0.25)
            .turnToleranceDegrees(0.5)
            .turnScaleTolerance(0.05)
            .speedMemorySeconds(1.0)
            .speedToleranceMetersPerSecond(0.4)
            .build();

    public Settings {
      Checks.on(Settings.class)
          .field("windowSeconds", windowSeconds, positive())
          .field("turnToleranceDegrees", turnToleranceDegrees, nonNegative())
          .field("turnScaleTolerance", turnScaleTolerance, nonNegative())
          .field("speedMemorySeconds", speedMemorySeconds, positive())
          .field("speedToleranceMetersPerSecond", speedToleranceMetersPerSecond, nonNegative())
          .orThrow();
    }

    /** Starts building the settings, one named value at a time. */
    public static SlipCheckSettingsBuilder.WindowSecondsStage builder() {
      return SlipCheckSettingsBuilder.builder();
    }
  }

  /** One loop's turns: how far the wheels, and the IMU, turned the robot over it. */
  private record Turn(double seconds, double wheels, double imu) {}

  private final double windowSeconds;
  private final double turnToleranceRadians;
  private final double turnScaleTolerance;
  private final double speedMemorySeconds;
  private final double speedTolerance;
  private final double reachMeters;

  private final ArrayDeque<Turn> turns = new ArrayDeque<>();
  private double turnsSeconds;

  /**
   * The wheels' velocity at the last loop's end, in the IMU's fixed frame; not a number at first.
   */
  private double lastVelocityX = Double.NaN;

  private double lastVelocityY = Double.NaN;

  /** How much faster than the robot the wheels roll, as the accelerometer reckons it. */
  private double slideX;

  private double slideY;

  private double turnApart;
  private double turnSlip;
  private double speedSlip;

  /**
   * @param settings how closely the wheels must agree with the IMU
   * @param wheelReachMeters how far the wheels sit from the robot's center: a turn they see and the
   *     gyro doesn't, times this, is how far they slid
   */
  public SlipCheck(Settings settings, double wheelReachMeters) {
    if (!(wheelReachMeters > 0) || Double.isInfinite(wheelReachMeters)) {
      throw new IllegalArgumentException("the wheels' reach must be positive: " + wheelReachMeters);
    }
    windowSeconds = settings.windowSeconds();
    turnToleranceRadians = Math.toRadians(settings.turnToleranceDegrees());
    turnScaleTolerance = settings.turnScaleTolerance();
    speedMemorySeconds = settings.speedMemorySeconds();
    speedTolerance = settings.speedToleranceMetersPerSecond();
    reachMeters = wheelReachMeters;
  }

  /**
   * One loop of readings. A comparison missing a reading starts over, and says nothing until it has
   * two loops to compare.
   *
   * @param seconds how long the loop was: since the last update
   * @param wheelTurnRadians how far the wheels turned the robot over the loop
   * @param imuTurnRadians how far the IMU turned it over the same readings; not a number when no
   *     IMU read both ends of them
   * @param yawRadians the IMU's yaw at the loop's end, which holds still while the robot turns: the
   *     frame the speed is compared in. Not a number without an IMU.
   * @param wheels the robot's velocity as its wheels measure it, robot-relative, at the loop's end
   * @param forwardAccelerationMetersPerSecSq the robot's acceleration over the loop as the IMU felt
   *     it, along the robot's forward axis, without gravity's share; not a number when it can't
   *     say, or felt a jolt beyond driving (a collision, whose own doubt covers it)
   * @param leftAccelerationMetersPerSecSq the same, along the robot's left axis
   */
  public void update(
      double seconds,
      double wheelTurnRadians,
      double imuTurnRadians,
      double yawRadians,
      ChassisVelocities wheels,
      double forwardAccelerationMetersPerSecSq,
      double leftAccelerationMetersPerSecSq) {
    if (!(seconds > 0) || Double.isInfinite(seconds)) {
      throw new IllegalArgumentException("a loop takes time: " + seconds);
    }
    compareTurns(seconds, wheelTurnRadians, imuTurnRadians);
    compareSpeeds(
        seconds,
        yawRadians,
        wheels,
        forwardAccelerationMetersPerSecSq,
        leftAccelerationMetersPerSecSq);
  }

  /** Starts both comparisons over: the IMU was lost, or changed. */
  public void reset() {
    turns.clear();
    turnsSeconds = 0;
    turnApart = 0;
    turnSlip = 0;
    forgetSpeeds();
  }

  private void forgetSpeeds() {
    lastVelocityX = Double.NaN;
    lastVelocityY = Double.NaN;
    slideX = 0;
    slideY = 0;
    speedSlip = 0;
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
    turnSlip = beyond > 0 ? beyond * reachMeters / windowSeconds : 0;
  }

  private void compareSpeeds(
      double seconds,
      double yaw,
      ChassisVelocities wheels,
      double forwardAcceleration,
      double leftAcceleration) {
    if (!Double.isFinite(yaw)
        || !Double.isFinite(forwardAcceleration)
        || !Double.isFinite(leftAcceleration)
        || !Double.isFinite(wheels.vx)
        || !Double.isFinite(wheels.vy)) {
      forgetSpeeds();
      return;
    }
    double cos = Math.cos(yaw);
    double sin = Math.sin(yaw);
    double velocityX = cos * wheels.vx - sin * wheels.vy;
    double velocityY = sin * wheels.vx + cos * wheels.vy;
    if (Double.isFinite(lastVelocityX)) {
      // How the wheels' velocity changed over the loop, less what the accelerometer felt.
      double fade = Math.exp(-seconds / speedMemorySeconds);
      slideX =
          slideX * fade
              + velocityX
              - lastVelocityX
              - (cos * forwardAcceleration - sin * leftAcceleration) * seconds;
      slideY =
          slideY * fade
              + velocityY
              - lastVelocityY
              - (sin * forwardAcceleration + cos * leftAcceleration) * seconds;
    }
    lastVelocityX = velocityX;
    lastVelocityY = velocityY;
    speedSlip = Math.max(0, Math.hypot(slideX, slideY) - speedTolerance);
  }
}
