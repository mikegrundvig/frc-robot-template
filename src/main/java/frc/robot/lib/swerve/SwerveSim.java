package frc.robot.lib.swerve;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import frc.robot.lib.config.Body;
import frc.robot.lib.config.SwerveConfig;
import java.util.Arrays;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.MathUtil;

/**
 * Physics simulation of a swerve robot on carpet, detailed enough to answer "will this brown out or
 * lose traction?" rather than just "does the code run?".
 *
 * <p>Modeled: DC motor electrical behavior (back-EMF, resistance), stator and supply current
 * limits, the motor controllers' onboard control loops (run at the physics rate, like the real
 * ones), battery voltage sag under load, tire friction limited by each wheel's normal force (so
 * wheels can slip, and slip corrupts odometry just as on a real robot), and quasi-static weight
 * transfer when accelerating (so the lightly loaded wheels slip first).
 *
 * <p>Not modeled: walls and other robots, carpet direction and wear, drivetrain friction and
 * efficiency losses, motor heating, bumpers flexing, and the controller's brownout behavior (tests
 * detect a brownout rather than simulate its consequences). Constants marked "estimate" should be
 * replaced by measured values.
 *
 * <p>The robot's facts (mass, gearing, motors, wheel positions) come from its configuration. Each
 * module's controller settings (gains and current limits) are whatever was last {@link
 * Module#configure configured}, as on real motor controllers, so tuned values reach the simulation
 * the same way they reach a robot.
 *
 * <p>Call {@link #update(double, boolean)} once per robot loop, before the drive reads its inputs.
 * Each instance is independent (no shared global state), so many can run in parallel.
 */
public final class SwerveSim {
  /**
   * Battery model.
   *
   * @param nominalVolts open-circuit voltage
   * @param resistanceOhms internal plus wiring resistance
   * @param otherLoadAmps what everything but the drivetrain draws
   */
  public record Battery(double nominalVolts, double resistanceOhms, double otherLoadAmps) {
    /** A healthy, charged FRC battery with typical wiring. Estimate. */
    public static final Battery TYPICAL = new Battery(12.6, 0.020, 3.0);
  }

  static final double SUBSTEP_SEC = 0.0002;

  /** The battery's voltage is solved to within this each substep. */
  private static final double BATTERY_TOLERANCE_VOLTS = 1e-9;

  /** A backstop for the battery solver, which normally needs two or three steps. */
  private static final int MAX_BATTERY_SOLVER_STEPS = 100;

  /** Below this slip speed, friction ramps linearly (a common stabilizing approximation). */
  private static final double SLIP_REGULARIZATION_MPS = 0.1;

  /** Wheel plus reflected motor rotor inertia, at the wheel. Estimate. */
  private static final double WHEEL_INERTIA_KG_M2 = 0.0027;

  /** Rotating module assembly about its steer axis. Estimate. */
  private static final double STEER_INERTIA_KG_M2 = 0.004;

  private static final double GRAVITY = 9.81;

  private enum DriveMode {
    OFF,
    VELOCITY,
    VOLTAGE
  }

  /** Extremes over every physics substep since the last {@link #resetStats()}. */
  public static final class Stats {
    public double minBatteryVolts = Double.POSITIVE_INFINITY;
    public double maxBatteryVolts = Double.NEGATIVE_INFINITY;
    public double peakTotalSupplyAmps = 0.0;
    public double peakDriveStatorAmps = 0.0;
    public double maxSlipSpeedMps = 0.0;
    public final double[] slipSeconds = new double[4];
    public double minNormalForceNewtons = Double.POSITIVE_INFINITY;
    public double maxSpeedMps = 0.0;
    public double seconds = 0.0;

    /** Fraction of time the worst wheel spent sliding. */
    public double worstSlipFraction() {
      double worst = 0.0;
      for (double s : slipSeconds) {
        worst = Math.max(worst, s);
      }
      return seconds > 0 ? worst / seconds : 0.0;
    }
  }

  /**
   * One simulated module: its wheel, its steering, and the two motor controllers that drive them.
   * Robot code commands and reads it as it would real hardware.
   */
  public static final class Module {
    private final Translation2d offset;
    private double wheelOmega;
    private double wheelAngle;
    private double steerAngle;
    private double steerOmega;

    private DriveMode mode = DriveMode.OFF;
    private double velocitySetpoint;
    private double feedforwardVolts;
    private double openLoopVolts;
    private boolean steering;
    private double steerSetpoint;
    private boolean connected = true;

    // The motor controllers' settings.
    private double driveKp;
    private double steerKp;
    private double steerKd;
    private double driveStatorLimitAmps;
    private double driveSupplyLimitAmps;
    private double steerStatorLimitAmps;

    private double driveVolts;
    private double driveStatorAmps;
    private double driveSupplyAmps;
    private double steerVolts;
    private double steerStatorAmps;
    private double slipSpeed;
    private double normalForce;

    Module(Translation2d offset, SwerveConfig drivetrain) {
      this.offset = offset;
      configure(drivetrain.gains(), drivetrain.limits());
    }

    /** Sets the motor controllers' gains and current limits, as robot code configures them. */
    public void configure(SwerveConfig.Gains gains, SwerveConfig.Limits limits) {
      driveKp = gains.driveKp();
      steerKp = gains.steerKp();
      steerKd = gains.steerKd();
      driveStatorLimitAmps = limits.driveStatorCurrent().in(Amps);
      driveSupplyLimitAmps = limits.driveSupplyCurrent().in(Amps);
      steerStatorLimitAmps = limits.steerStatorCurrent().in(Amps);
    }

    /** Closed-loop wheel velocity on the drive controller, plus the robot's feedforward. */
    public void setDriveVelocity(double wheelRadPerSec, double feedforwardVolts) {
      mode = DriveMode.VELOCITY;
      velocitySetpoint = wheelRadPerSec;
      this.feedforwardVolts = feedforwardVolts;
    }

    /** Open-loop drive voltage, for characterization. */
    public void setDriveVoltage(double volts) {
      mode = DriveMode.VOLTAGE;
      openLoopVolts = volts;
    }

    /** Closed-loop steering angle on the steer controller. */
    public void setSteerAngle(Rotation2d angle) {
      steering = true;
      steerSetpoint = angle.getRadians();
    }

    /** Both motors off. */
    public void stop() {
      mode = DriveMode.OFF;
      steering = false;
    }

    public boolean connected() {
      return connected;
    }

    /** Wheel rotation since the start, in radians at the wheel. */
    public double drivePositionRad() {
      return wheelAngle;
    }

    public double driveVelocityRadPerSec() {
      return wheelOmega;
    }

    public double driveAppliedVolts() {
      return driveVolts;
    }

    public double driveStatorAmps() {
      return Math.abs(driveStatorAmps);
    }

    public double driveSupplyAmps() {
      return driveSupplyAmps;
    }

    /** From the absolute encoder: valid at power-on, no homing needed. */
    public Rotation2d steerAngle() {
      return new Rotation2d(steerAngle);
    }

    public double steerVelocityRadPerSec() {
      return steerOmega;
    }

    public double steerAppliedVolts() {
      return steerVolts;
    }

    public double steerStatorAmps() {
      return Math.abs(steerStatorAmps);
    }
  }

  private final Battery battery;
  private final DCMotor driveMotor;
  private final DCMotor steerMotor;
  private final Module[] modules = new Module[4];
  private final double massKg;
  private final double yawMomentOfInertiaKgM2;
  private final double cgHeightMeters;
  private final double wheelBaseMeters;
  private final double trackWidthMeters;
  private final double wheelRadiusMeters;
  private final double driveGearing;
  private final double steerGearing;
  private final double mu;

  private double x;
  private double y;
  private double theta;
  private double vx;
  private double vy;
  private double omega;
  private double forwardAccel;
  private double leftAccel;

  private double batteryVolts;
  private double totalSupplyAmps;

  /** What each module's motor controllers are asked for in the substep being simulated. */
  private final double[] driveCommands = new double[4];

  private final double[] steerCommands = new double[4];

  private final Stats stats = new Stats();

  /**
   * @param body the robot's mass and how it is distributed
   * @param drivetrain its drivetrain; the modules' controllers start with its gains and limits
   * @param startPose where the robot starts on the field
   */
  public SwerveSim(Body body, SwerveConfig drivetrain, Battery battery, Pose2d startPose) {
    this.battery = battery;
    SwerveConfig.Modules hardware = drivetrain.modules();
    driveMotor = hardware.driveMotor().model();
    steerMotor = hardware.steerMotor().model();
    massKg = body.mass().in(Kilograms);
    yawMomentOfInertiaKgM2 = body.yawMomentOfInertia().in(KilogramSquareMeters);
    cgHeightMeters = body.cgHeight().in(Meters);
    wheelRadiusMeters = hardware.wheelRadius().in(Meters);
    driveGearing = hardware.driveGearing();
    steerGearing = hardware.steerGearing();
    mu = hardware.wheelCoefficientOfFriction();
    Translation2d[] offsets = drivetrain.moduleTranslations(body);
    wheelBaseMeters = 2 * offsets[0].getX();
    trackWidthMeters = 2 * offsets[0].getY();
    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(offsets[i], drivetrain);
    }
    x = startPose.getX();
    y = startPose.getY();
    theta = startPose.getRotation().getRadians();
    batteryVolts = battery.nominalVolts();
  }

  /**
   * The simulated hardware of module {@code index}: front left, front right, back left, back right.
   */
  public Module module(int index) {
    return modules[index];
  }

  /**
   * Advance physics by one robot loop. {@code enabled} is the robot's enable state: when disabled,
   * motor controllers output nothing (drive motors brake).
   */
  public void update(double dtSec, boolean enabled) {
    int steps = (int) Math.round(dtSec / SUBSTEP_SEC);
    for (int i = 0; i < steps; i++) {
      substep(SUBSTEP_SEC, enabled);
    }
  }

  private void substep(double h, boolean enabled) {
    double cos = Math.cos(theta);
    double sin = Math.sin(theta);
    double forceX = 0.0;
    double forceY = 0.0;
    double torque = 0.0;
    double supplyAmps = battery.otherLoadAmps();

    for (int index = 0; index < modules.length; index++) {
      Module m = modules[index];
      boolean powered = enabled && m.connected;

      // Steer: position loop on the motor controller.
      steerCommands[index] = 0.0;
      if (powered && m.steering) {
        double error = MathUtil.angleModulus(m.steerSetpoint - m.steerAngle);
        steerCommands[index] = m.steerKp * error - m.steerKd * m.steerOmega;
      }

      // Drive: velocity loop on the motor controller, or open loop. Unpowered motors are in brake
      // mode: zero volts, so back-EMF brakes the wheel.
      driveCommands[index] = 0.0;
      if (powered) {
        driveCommands[index] =
            switch (m.mode) {
              case VELOCITY ->
                  m.feedforwardVolts
                      + m.driveKp * (m.velocitySetpoint - m.wheelOmega) * wheelRadiusMeters;
              case VOLTAGE -> m.openLoopVolts;
              case OFF -> 0.0;
            };
      }
    }
    batteryVolts = loadedBatteryVolts();

    for (int index = 0; index < modules.length; index++) {
      Module m = modules[index];
      double[] steer =
          motor(
              steerMotor,
              steerCommands[index],
              m.steerOmega * steerGearing,
              m.steerStatorLimitAmps,
              Double.POSITIVE_INFINITY,
              batteryVolts);
      m.steerStatorAmps = steer[0];
      m.steerVolts = steer[2];
      double steerTorque = steerMotor.Kt * steer[0] * steerGearing;
      m.steerOmega += steerTorque / STEER_INERTIA_KG_M2 * h;
      m.steerAngle = MathUtil.angleModulus(m.steerAngle + m.steerOmega * h);

      double[] drive =
          motor(
              driveMotor,
              driveCommands[index],
              m.wheelOmega * driveGearing,
              m.driveStatorLimitAmps,
              m.driveSupplyLimitAmps,
              batteryVolts);
      m.driveStatorAmps = drive[0];
      m.driveSupplyAmps = drive[1];
      m.driveVolts = drive[2];
      supplyAmps += drive[1] + steer[1];
      double wheelTorque = driveMotor.Kt * drive[0] * driveGearing;

      // Normal force: a quarter of the weight, shifted toward the back when accelerating forward
      // and toward the right when accelerating left.
      double normal = massKg * GRAVITY / 4.0;
      double pitchShift = massKg * forwardAccel * cgHeightMeters / wheelBaseMeters / 2.0;
      double rollShift = massKg * leftAccel * cgHeightMeters / trackWidthMeters / 2.0;
      normal += m.offset.getX() > 0 ? -pitchShift : pitchShift;
      normal += m.offset.getY() > 0 ? -rollShift : rollShift;
      m.normalForce = normal;
      normal = Math.max(0.0, normal);

      // Tire: the contact patch slides when the wheel surface and the ground disagree. Friction
      // opposes the sliding and can never exceed mu * N.
      double offsetX = m.offset.getX() * cos - m.offset.getY() * sin;
      double offsetY = m.offset.getX() * sin + m.offset.getY() * cos;
      double contactVx = vx - omega * offsetY;
      double contactVy = vy + omega * offsetX;
      double headingCos = Math.cos(theta + m.steerAngle);
      double headingSin = Math.sin(theta + m.steerAngle);
      double slipX = contactVx - m.wheelOmega * wheelRadiusMeters * headingCos;
      double slipY = contactVy - m.wheelOmega * wheelRadiusMeters * headingSin;
      double slip = Math.hypot(slipX, slipY);
      double maxFriction = mu * normal;
      double scale =
          slip <= SLIP_REGULARIZATION_MPS
              ? maxFriction / SLIP_REGULARIZATION_MPS
              : maxFriction / slip;
      double fx = -slipX * scale;
      double fy = -slipY * scale;
      m.slipSpeed = slip;

      // The ground's push along the wheel's direction resists the wheel's spin.
      double longitudinal = fx * headingCos + fy * headingSin;
      m.wheelOmega += (wheelTorque - longitudinal * wheelRadiusMeters) / WHEEL_INERTIA_KG_M2 * h;
      m.wheelAngle += m.wheelOmega * h;

      forceX += fx;
      forceY += fy;
      torque += offsetX * fy - offsetY * fx;

      if (slip > SLIP_REGULARIZATION_MPS) {
        stats.slipSeconds[index] += h;
      }
      stats.maxSlipSpeedMps = Math.max(stats.maxSlipSpeedMps, slip);
      stats.minNormalForceNewtons = Math.min(stats.minNormalForceNewtons, m.normalForce);
      stats.peakDriveStatorAmps = Math.max(stats.peakDriveStatorAmps, Math.abs(m.driveStatorAmps));
    }

    double ax = forceX / massKg;
    double ay = forceY / massKg;
    vx += ax * h;
    vy += ay * h;
    omega += torque / yawMomentOfInertiaKgM2 * h;
    x += vx * h;
    y += vy * h;
    theta = MathUtil.angleModulus(theta + omega * h);
    forwardAccel = ax * cos + ay * sin;
    leftAccel = -ax * sin + ay * cos;

    totalSupplyAmps = supplyAmps;

    stats.seconds += h;
    stats.minBatteryVolts = Math.min(stats.minBatteryVolts, batteryVolts);
    stats.maxBatteryVolts = Math.max(stats.maxBatteryVolts, batteryVolts);
    stats.peakTotalSupplyAmps = Math.max(stats.peakTotalSupplyAmps, supplyAmps);
    stats.maxSpeedMps = Math.max(stats.maxSpeedMps, Math.hypot(vx, vy));
  }

  public Stats stats() {
    return stats;
  }

  public void resetStats() {
    Stats fresh = new Stats();
    stats.minBatteryVolts = fresh.minBatteryVolts;
    stats.maxBatteryVolts = fresh.maxBatteryVolts;
    stats.peakTotalSupplyAmps = 0.0;
    stats.peakDriveStatorAmps = 0.0;
    stats.maxSlipSpeedMps = 0.0;
    Arrays.fill(stats.slipSeconds, 0.0);
    stats.minNormalForceNewtons = fresh.minNormalForceNewtons;
    stats.maxSpeedMps = 0.0;
    stats.seconds = 0.0;
  }

  /**
   * The battery's voltage this substep: where its sag (its resistance times the current drawn)
   * matches what the motor controllers draw at that voltage. Solved, not carried over from the last
   * substep: a controller at full output draws more the higher the voltage, so feeding one
   * substep's current into the next substep's voltage sets the two chasing each other into an
   * oscillation.
   */
  private double loadedBatteryVolts() {
    // Guess too high and the battery has voltage to spare; too low and the load pulls it lower
    // still. Bracket the answer, starting from the last substep's voltage.
    double lo = batteryVolts;
    double loExcess = excessVolts(lo);
    if (loExcess == 0) {
      return lo;
    }
    double hi = lo;
    double hiExcess = loExcess;
    for (double widen = 0.01; lo > 0 && loExcess > 0; widen *= 2) {
      lo = Math.max(0.0, lo - widen);
      loExcess = excessVolts(lo);
    }
    for (double widen = 0.01; hiExcess < 0; widen *= 2) {
      hi += widen;
      hiExcess = excessVolts(hi);
    }

    // Then close in on where the straight line between the ends crosses zero (regula falsi). Over
    // a small bracket the excess is close to straight, so this takes a step or two. Halving the
    // weight of an end that stays put twice (the Illinois variant) keeps a bend from stalling it.
    int keptEnd = 0; // -1 lo, +1 hi
    for (int step = 0; step < MAX_BATTERY_SOLVER_STEPS; step++) {
      double volts = (lo * hiExcess - hi * loExcess) / (hiExcess - loExcess);
      double excess = excessVolts(volts);
      if (Math.abs(excess) < BATTERY_TOLERANCE_VOLTS || hi - lo < BATTERY_TOLERANCE_VOLTS) {
        return volts;
      }
      if (excess < 0) {
        lo = volts;
        loExcess = excess;
        if (keptEnd == 1) {
          hiExcess /= 2;
        }
        keptEnd = 1;
      } else {
        hi = volts;
        hiExcess = excess;
        if (keptEnd == -1) {
          loExcess /= 2;
        }
        keptEnd = -1;
      }
    }
    return (lo + hi) / 2;
  }

  /** How far {@code busVolts} is above what the battery gives with the load it would carry. */
  private double excessVolts(double busVolts) {
    double amps = battery.otherLoadAmps();
    for (int index = 0; index < modules.length; index++) {
      Module m = modules[index];
      double steerOmega = m.steerOmega * steerGearing;
      double steer =
          statorAmps(
              steerMotor,
              steerCommands[index],
              steerOmega,
              m.steerStatorLimitAmps,
              Double.POSITIVE_INFINITY,
              busVolts);
      double driveOmega = m.wheelOmega * driveGearing;
      double drive =
          statorAmps(
              driveMotor,
              driveCommands[index],
              driveOmega,
              m.driveStatorLimitAmps,
              m.driveSupplyLimitAmps,
              busVolts);
      amps +=
          supplyAmps(steerMotor, steer, steerOmega, busVolts)
              + supplyAmps(driveMotor, drive, driveOmega, busVolts);
    }
    return busVolts - (battery.nominalVolts() - battery.resistanceOhms() * amps);
  }

  /**
   * A DC motor behind a current-limited motor controller, on a supply at {@code busVolts}.
   *
   * @return {stator current, supply current, applied volts}
   */
  private static double[] motor(
      DCMotor motor,
      double commandVolts,
      double motorOmega,
      double statorLimit,
      double supplyLimit,
      double busVolts) {
    double stator = statorAmps(motor, commandVolts, motorOmega, statorLimit, supplyLimit, busVolts);
    return new double[] {
      stator,
      supplyAmps(motor, stator, motorOmega, busVolts),
      stator * motor.R + motorOmega / motor.Kv
    };
  }

  /** The motor's current: what the command drives, within the stator and then supply limits. */
  private static double statorAmps(
      DCMotor motor,
      double commandVolts,
      double motorOmega,
      double statorLimit,
      double supplyLimit,
      double busVolts) {
    double volts = Math.clamp(commandVolts, -busVolts, busVolts);
    double backEmf = motorOmega / motor.Kv;
    double stator = Math.clamp((volts - backEmf) / motor.R, -statorLimit, statorLimit);
    if (Math.abs(supplyAmps(motor, stator, motorOmega, busVolts)) > supplyLimit) {
      stator = supplyLimitedStator(motor.R, backEmf, stator, supplyLimit, busVolts);
    }
    return stator;
  }

  /** The current drawn from the supply: the motor's electrical power over the supply's voltage. */
  private static double supplyAmps(
      DCMotor motor, double stator, double motorOmega, double busVolts) {
    return busVolts > 0 ? (stator * motor.R + motorOmega / motor.Kv) * stator / busVolts : 0.0;
  }

  /**
   * The largest stator current, in the direction of {@code stator}, whose supply current is at the
   * limit. Supply current is the power the motor takes over the battery's voltage, (I R + E) I / V
   * for stator current I and back-EMF E. It grows faster than I, so scaling I by limit / supply
   * stops short of the limit: solve the quadratic instead.
   *
   * <p>Braking current flowing back into the battery is limited the same way, an assumption still
   * to check against a real controller's logs. A braking motor reaches that limit first.
   */
  private static double supplyLimitedStator(
      double resistance, double backEmf, double stator, double supplyLimit, double volts) {
    double along = Math.signum(stator) * backEmf; // back-EMF in the direction the current flows
    double limitTerm = 4 * resistance * supplyLimit * volts;
    double braking = along * along - limitTerm;
    double amps =
        along < 0 && braking >= 0
            ? (-along - Math.sqrt(braking)) / (2 * resistance)
            : (-along + Math.sqrt(along * along + limitTerm)) / (2 * resistance);
    return Math.copySign(Math.min(amps, Math.abs(stator)), stator);
  }

  /** The gyro's heading: the true heading, as a perfect gyro would report it. */
  public Rotation2d yaw() {
    return new Rotation2d(theta);
  }

  public double yawVelocityRadPerSec() {
    return omega;
  }

  /** Simulate a module dropping off the CAN bus (both motors unpowered, readings disconnected). */
  public void setModuleConnected(int index, boolean connected) {
    modules[index].connected = connected;
  }

  // ---- Ground truth, for tests and visualization. Robot code never sees these. ----

  public Pose2d truePose() {
    return new Pose2d(x, y, new Rotation2d(theta));
  }

  /** True field-relative velocity. */
  public ChassisVelocities trueFieldVelocities() {
    return new ChassisVelocities(vx, vy, omega);
  }

  public double batteryVolts() {
    return batteryVolts;
  }

  public double totalSupplyAmps() {
    return totalSupplyAmps;
  }

  public double wheelSlipSpeed(int index) {
    return modules[index].slipSpeed;
  }

  /** Normal force before clamping at zero: negative means that wheel would lift. */
  public double wheelNormalForce(int index) {
    return modules[index].normalForce;
  }

  public double driveStatorAmps(int index) {
    return Math.abs(modules[index].driveStatorAmps);
  }
}
