package frc.robot.sim;

import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import frc.robot.lib.config.Body;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.field.Terrain;
import frc.robot.lib.localization.BodyMotion;
import frc.robot.lib.localization.ImuReadings;
import frc.robot.lib.localization.OdometrySample;
import frc.robot.lib.localization.SwerveKinematicsModel;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;

/**
 * The simulated drivetrain as it really moves: four modules' motors, and the robot's true pose on
 * the field. Its sensors report that truth with {@link SensorErrors}, as real sensors would, so the
 * pose the robot code works out can drift from where the robot really is, and localization is
 * tested against that gap instead of a perfect world.
 *
 * <p>Call {@link #step} once per loop, before the robot code reads its sensors. It runs the physics
 * at 250 Hz and takes the modules' and gyro's readings at each of those steps, as a robot's
 * high-rate odometry does. The robot moves as its wheels roll, at their true radius and true angle;
 * it doesn't slip on its own, since traction isn't modeled yet (see docs/stand-ins.md), but faults
 * can be made: {@link #spin} slips a wheel (and a module can keep spinning faster than the floor),
 * {@link #shove} moves the robot the way a push does, at once, and {@link #push} as one does over
 * time, {@link #hit} the way a collision does, and {@link #tilt} with {@link #grip} lifts it onto a
 * bump.
 *
 * <p>Each wheel carries an equal share of the robot's configured mass ({@code Body.mass}), as
 * inertia on its drive motor ({@link ModuleIOSim#driveInertiaKgM2}), so the robot speeds up and
 * slows down as a robot of that mass does under its drive motors' torque. What that leaves out
 * (weight transfer, the battery, the robot's own yaw inertia, the modules pushing on one another
 * through the frame) is in docs/stand-ins.md.
 */
public final class DriveSimulation {
  /** The backup IMU's noise is drawn this far from the gyro's seed: no seed's gyro shares it. */
  private static final long BACKUP_SEED_OFFSET = 1L << 32;

  /** Physics steps, and odometry readings, per robot loop: 250 Hz at a 50 Hz loop. */
  static final int STEPS_PER_LOOP = 5;

  /** A hard hit's jolt, as an IMU feels it: 4 g. */
  static final double HIT_JOLT_METERS_PER_SECOND_SQUARED = 4 * ImuReadings.STANDARD_GRAVITY;

  /**
   * How quickly a push gets the robot sliding, and lets it stop, in m/s²: about a third of g, as
   * one robot driving into another starts it moving.
   */
  static final double PUSH_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0;

  private final ModuleIOSim[] modules = new ModuleIOSim[4];
  private final GyroIOSim gyro;
  private final GyroIOSim backupGyro;
  private final double[] trueRadiusMeters = new double[4];

  /** Each wheel's true radius over the configured one, now and as the robot was built. */
  private final double[] treadScale = new double[4];

  private final double[] builtTreadScale = new double[4];
  private final double configuredRadiusMeters;
  private final SwerveKinematicsModel kinematics;
  private OdometrySample lastTrue;
  private Pose2d truePose = Pose2d.ZERO;
  private double yawTurnedRadians;
  private double yawRateRadiansPerSecond;
  private double elapsedSeconds;

  private double grip = 1;
  private double pitchRadians;
  private double rollRadians;

  // The ground under the robot: the terrain, and the plane through its wheels' contact points.
  private final Translation2d[] moduleLocations;
  private final double[] planeFit = new double[9];
  private Terrain terrain = Terrain.FLAT;
  private boolean tiltedByHand;
  private double heightMeters;
  private double riseForward;
  private double riseLeft;

  // The robot's motion on the field, for what its IMU feels: its velocity over the last physics
  // step, and how that changed over the last loop.
  private Translation2d lastVelocity = Translation2d.ZERO;
  private Translation2d loopAcceleration = Translation2d.ZERO;

  /** A push the wheels don't see, in the field's frame: the velocity it's headed for, and now. */
  private Translation2d pushTarget = Translation2d.ZERO;

  private Translation2d pushVelocity = Translation2d.ZERO;

  /** A hit's jolt, in the robot's frame, until the IMU reads it. */
  private Translation2d jolt = Translation2d.ZERO;

  /**
   * On flat ground until given its terrain ({@link #setTerrain}).
   *
   * @param body the robot's frame, which places the wheels, and its mass, which they carry
   * @param drivetrain the drivetrain as configured: the code's belief about its modules
   * @param errors how the real wheels and sensors differ from the configuration
   */
  public DriveSimulation(Body body, SwerveConfig drivetrain, SensorErrors errors) {
    SwerveConfig.Modules config = drivetrain.modules();
    Translation2d[] moduleLocations = drivetrain.moduleTranslations(body);
    kinematics = new SwerveKinematicsModel(moduleLocations);
    this.moduleLocations = moduleLocations.clone();
    planeFit(moduleLocations, planeFit);
    configuredRadiusMeters = config.wheelRadius().in(Meters);
    for (int i = 0; i < 4; i++) {
      // Each module's CANcoder sends on its own clock: here a quarter period after the last, to the
      // steering loop's millisecond (0, 2, 5 and 7 ms into its cycle, at 100 Hz).
      modules[i] =
          new ModuleIOSim(
              config,
              body.mass().in(Kilograms) / modules.length,
              Math.toRadians(errors.steerOffsetDegrees().get(i)),
              i / 4.0);
      treadScale[i] = errors.wheelRadiusScale().get(i);
      builtTreadScale[i] = treadScale[i];
      trueRadiusMeters[i] = configuredRadiusMeters * treadScale[i];
    }
    // Each IMU's noise from its own stream of the seed: the backup's isn't the next seed's gyro.
    gyro = new GyroIOSim(this, errors.gyro(), errors.seed());
    backupGyro = new GyroIOSim(this, errors.backupGyro(), errors.seed() + BACKUP_SEED_OFFSET);
    lastTrue = trueWheels();
  }

  /** The modules' motor controllers and encoders, for the drivetrain. */
  public ModuleIO[] modules() {
    return new ModuleIO[] {modules[0], modules[1], modules[2], modules[3]};
  }

  /** The robot's gyro. */
  public GyroIO gyro() {
    return gyro;
  }

  /** The backup IMU (Systemcore's own), read once a loop. */
  public GyroIO backupGyro() {
    return backupGyro;
  }

  /**
   * The ground the robot drives on: it rides over the bumps, tilted by the ground under its wheels,
   * and its wheels' travel moves it at the slope's cosine. Undoes a tilt made by hand.
   */
  public void setTerrain(Terrain terrain) {
    this.terrain = terrain;
    tiltedByHand = false;
    settle();
  }

  /** The ground the robot drives on. */
  public Terrain terrain() {
    return terrain;
  }

  /** Unplugs the gyro, or plugs it back in. */
  public void disconnectGyro(boolean disconnected) {
    gyro.setConnected(!disconnected);
  }

  /** Freezes the gyro: it keeps reporting, the same yaw every time. */
  public void freezeGyro(boolean frozen) {
    gyro.setFrozen(frozen);
  }

  /** Unplugs the backup IMU, or plugs it back in. */
  public void disconnectBackupGyro(boolean disconnected) {
    backupGyro.setConnected(!disconnected);
  }

  /** Freezes the backup IMU: it keeps reporting, the same yaw every time. */
  public void freezeBackupGyro(boolean frozen) {
    backupGyro.setFrozen(frozen);
  }

  /**
   * Wears one wheel's tread from now on: its true radius over the configured one, 0.97 for tread
   * worn 3%. The robot moves that much less than its encoder counts.
   *
   * @param module front left, front right, back left, back right: 0 to 3
   */
  public void wearTread(int module, double scale) {
    if (!(scale > 0)) {
      throw new IllegalArgumentException("a wheel's radius scale is positive: " + scale);
    }
    treadScale[module] = scale;
    trueRadiusMeters[module] = configuredRadiusMeters * scale;
    // The wheels' travel so far was at the old radius: start counting again from here.
    lastTrue = trueWheels();
  }

  /** A wheel's true radius over the configured one. */
  double treadScale(int module) {
    return treadScale[module];
  }

  /** A wheel's true radius over the configured one, as the robot was built. */
  double builtTreadScale(int module) {
    return builtTreadScale[module];
  }

  /** The simulated gyro, for its faults. */
  GyroIOSim gyroSim() {
    return gyro;
  }

  /** The simulated backup IMU, for its faults. */
  GyroIOSim backupGyroSim() {
    return backupGyro;
  }

  /**
   * A simulated module, for its faults.
   *
   * @param module front left, front right, back left, back right: 0 to 3
   */
  ModuleIOSim moduleSim(int module) {
    return modules[module];
  }

  /**
   * Runs the physics through one loop, and takes the sensors' readings along the way.
   *
   * @param seconds the loop's length
   * @param nowNanos the robot clock at the loop's end: the last reading's time
   */
  public void step(double seconds, long nowNanos) {
    if (!(seconds > 0)) {
      throw new IllegalArgumentException("a step takes time: " + seconds);
    }
    double dt = seconds / STEPS_PER_LOOP;
    long dtNanos = Math.round(dt * 1e9);
    Translation2d loopStartVelocity = lastVelocity;
    for (int remaining = STEPS_PER_LOOP - 1; remaining >= 0; remaining--) {
      for (ModuleIOSim module : modules) {
        module.step(dt);
      }
      OdometrySample wheels = trueWheels();
      BodyMotion motion = kinematics.motion(lastTrue, wheels);
      Translation2d before = truePose.getTranslation();
      // Only the gripping share of the wheels' rolling moves the robot, and rolling up or down a
      // slope moves it less across the field.
      double travel = Math.hypot(motion.dxMeters(), motion.dyMeters());
      double rise =
          travel > 0
              ? (riseForward * motion.dxMeters() + riseLeft * motion.dyMeters()) / travel
              : 0;
      double level = grip / Math.sqrt(1 + rise * rise);
      truePose =
          truePose.plus(
              new Twist2d(
                      level * motion.dxMeters(),
                      level * motion.dyMeters(),
                      grip * motion.dthetaRadians())
                  .exp());
      pushVelocity =
          toward(pushVelocity, pushTarget, PUSH_ACCELERATION_METERS_PER_SECOND_SQUARED * dt);
      truePose =
          new Pose2d(
              truePose.getTranslation().plus(pushVelocity.times(dt)), truePose.getRotation());
      settle();
      yawTurnedRadians += grip * motion.dthetaRadians();
      yawRateRadiansPerSecond = grip * motion.dthetaRadians() / dt;
      lastVelocity = truePose.getTranslation().minus(before).div(dt);
      lastTrue = wheels;
      elapsedSeconds += dt;
      long readingTime = nowNanos - remaining * dtNanos;
      for (ModuleIOSim module : modules) {
        module.sample(readingTime);
      }
      gyro.sample(readingTime);
      if (remaining == 0) {
        backupGyro.sample(readingTime);
      }
    }
    loopAcceleration = lastVelocity.minus(loopStartVelocity).div(seconds);
  }

  /** A velocity moved toward another by at most so much. */
  private static Translation2d toward(Translation2d from, Translation2d to, double most) {
    Translation2d change = to.minus(from);
    double size = change.getNorm();
    return size <= most ? to : from.plus(change.times(most / size));
  }

  /** Where the robot really is. */
  public Pose2d truePose() {
    return truePose;
  }

  /** Where the robot really is, raised and tilted as it is: what its cameras see from. */
  public Pose3d truePose3d() {
    return new Pose3d(
        truePose.getX(),
        truePose.getY(),
        heightMeters,
        new Rotation3d(rollRadians, pitchRadians, truePose.getRotation().getRadians()));
  }

  /**
   * Sets the robot on the ground where it is: the plane through the ground under its wheels gives
   * its height and, unless tilted by hand, its attitude.
   */
  private void settle() {
    double sum = 0;
    double sumX = 0;
    double sumY = 0;
    for (Translation2d wheel : moduleLocations) {
      double height =
          terrain.heightAt(truePose.getTranslation().plus(wheel.rotateBy(truePose.getRotation())));
      sum += height;
      sumX += wheel.getX() * height;
      sumY += wheel.getY() * height;
    }
    // Least squares: height = a + (rise forward) x + (rise left) y, in the robot's frame.
    heightMeters = planeFit[0] * sum + planeFit[1] * sumX + planeFit[2] * sumY;
    riseForward = planeFit[3] * sum + planeFit[4] * sumX + planeFit[5] * sumY;
    riseLeft = planeFit[6] * sum + planeFit[7] * sumX + planeFit[8] * sumY;
    if (!tiltedByHand) {
      // WPILib's convention: nose up is a negative pitch, the left side up a positive roll.
      pitchRadians = -Math.atan(riseForward);
      rollRadians = Math.atan(riseLeft * Math.cos(pitchRadians));
    }
  }

  /** The inverse of the plane fit's normal equations for these wheels, row by row. */
  private static void planeFit(Translation2d[] wheels, double[] inverse) {
    double n = wheels.length;
    double sx = 0;
    double sy = 0;
    double sxx = 0;
    double sxy = 0;
    double syy = 0;
    for (Translation2d wheel : wheels) {
      sx += wheel.getX();
      sy += wheel.getY();
      sxx += wheel.getX() * wheel.getX();
      sxy += wheel.getX() * wheel.getY();
      syy += wheel.getY() * wheel.getY();
    }
    double[] m = {n, sx, sy, sx, sxx, sxy, sy, sxy, syy};
    double det =
        m[0] * (m[4] * m[8] - m[5] * m[7])
            - m[1] * (m[3] * m[8] - m[5] * m[6])
            + m[2] * (m[3] * m[7] - m[4] * m[6]);
    inverse[0] = (m[4] * m[8] - m[5] * m[7]) / det;
    inverse[1] = (m[2] * m[7] - m[1] * m[8]) / det;
    inverse[2] = (m[1] * m[5] - m[2] * m[4]) / det;
    inverse[3] = (m[5] * m[6] - m[3] * m[8]) / det;
    inverse[4] = (m[0] * m[8] - m[2] * m[6]) / det;
    inverse[5] = (m[2] * m[3] - m[0] * m[5]) / det;
    inverse[6] = (m[3] * m[7] - m[4] * m[6]) / det;
    inverse[7] = (m[1] * m[6] - m[0] * m[7]) / det;
    inverse[8] = (m[0] * m[4] - m[1] * m[3]) / det;
  }

  /**
   * Puts the robot somewhere, as the drive team does: its wheels don't turn, but its gyro feels any
   * turn, as a real one would.
   */
  public void place(Pose2d pose) {
    yawTurnedRadians += pose.getRotation().minus(truePose.getRotation()).getRadians();
    truePose = pose;
    settle();
  }

  /**
   * Moves the robot without its wheels turning, as another robot's push or a collision does:
   * odometry can't see it, and only the gyro sees any turn. A shove is in the robot's own frame.
   */
  public void shove(Transform2d shove) {
    place(truePose.plus(shove));
  }

  /**
   * Pushes the robot at this velocity, in its own frame as it faces now, as another robot pushing
   * it does: its wheels skid, never seeing it, while its IMU feels the push get going, and stop, at
   * {@link #PUSH_ACCELERATION_METERS_PER_SECOND_SQUARED}. Zero stops it.
   */
  public void push(Translation2d robotRelativeVelocity) {
    pushTarget = robotRelativeVelocity.rotateBy(truePose.getRotation());
  }

  /** Whether a push is on. Once it's stopped, the robot still slows to its wheels' pace. */
  boolean isPushed() {
    return pushTarget.getNorm() > 0;
  }

  /**
   * Moves the robot as a collision does ({@link #shove}), with the jolt a hit gives: its IMU feels
   * {@link #HIT_JOLT_METERS_PER_SECOND_SQUARED} the way it was knocked.
   */
  public void hit(Transform2d shove) {
    Rotation2d direction = shove.getTranslation().getAngle().orElse(Rotation2d.ZERO);
    jolt = new Translation2d(HIT_JOLT_METERS_PER_SECOND_SQUARED, direction);
    shove(shove);
  }

  /**
   * Tilts the robot by hand, as a wheel up on a game piece does, whatever the terrain under it
   * says: its IMU reads the attitude. Level again with zeros; back on the terrain with {@link
   * #setTerrain}.
   */
  public void tilt(double pitchRadians, double rollRadians) {
    tiltedByHand = true;
    this.pitchRadians = pitchRadians;
    this.rollRadians = rollRadians;
  }

  /** Whether it's tilted by hand, whatever the ground under it says. */
  boolean isTiltedByHand() {
    return tiltedByHand;
  }

  /** How much of its wheels' rolling moves the robot: 1 with full grip. */
  double grip() {
    return grip;
  }

  /**
   * How much of its wheels' rolling moves the robot: 1 on carpet, less with wheels in the air on a
   * bump or spinning on a slick spot. The encoders count all of it.
   */
  public void grip(double fraction) {
    if (!(fraction >= 0 && fraction <= 1)) {
      throw new IllegalArgumentException("a share of the wheels' rolling: " + fraction);
    }
    grip = fraction;
  }

  /**
   * Spins one wheel without the robot moving: a slip, which its encoder counts and the floor never
   * sees. Odometry catches it as the wheels disagreeing.
   *
   * @param module front left, front right, back left, back right: 0 to 3
   */
  public void spin(int module, double radians) {
    modules[module].spin(radians);
  }

  /**
   * From now on one wheel spins this much faster than the floor moves it, in meters per second: a
   * wheel losing its grip, which its encoder counts and the floor never sees. Zero grips again.
   *
   * @param module front left, front right, back left, back right: 0 to 3
   */
  public void spinAt(int module, double metersPerSecond) {
    modules[module].spinAt(metersPerSecond);
  }

  /** Every true turn since the start, driving or placed: what a gyro integrates. */
  double yawTurnedRadians() {
    return yawTurnedRadians;
  }

  double yawRateRadiansPerSecond() {
    return yawRateRadiansPerSecond;
  }

  double elapsedSeconds() {
    return elapsedSeconds;
  }

  double pitchRadians() {
    return pitchRadians;
  }

  double rollRadians() {
    return rollRadians;
  }

  /**
   * What the IMU feels, in the robot's frame: the robot's acceleration over the last loop, as an
   * IMU's filter reads it once a loop, gravity's share as the robot tilts, and the jolt of any hit
   * since it last read. The simulated robot follows its wheels without slipping, so where it turns
   * or stops harder than a real robot's grip allows (a module steering at speed sends it the new
   * way at once), its IMU feels that too: what the simulated robot does, not what a real one would,
   * so the wheels and the IMU agree as they would on a robot that really moved so.
   */
  Translation2d feltAcceleration() {
    Translation2d driving = loopAcceleration.rotateBy(truePose.getRotation().unaryMinus());
    // WPILib's attitude: a positive pitch is nose down, a positive roll the left side up. The IMU
    // feels the floor holding it up, tipped along the robot's axes.
    Translation2d gravity =
        new Translation2d(
            -ImuReadings.STANDARD_GRAVITY * Math.sin(pitchRadians),
            ImuReadings.STANDARD_GRAVITY * Math.sin(rollRadians) * Math.cos(pitchRadians));
    Translation2d felt = driving.plus(gravity).plus(jolt);
    jolt = Translation2d.ZERO;
    return felt;
  }

  /** The wheels as they truly rolled: the rotation that moved the robot, at the true radius. */
  private OdometrySample trueWheels() {
    double[] distances = new double[4];
    double[] angles = new double[4];
    for (int i = 0; i < 4; i++) {
      distances[i] = modules[i].wheelRotationRadians() * trueRadiusMeters[i];
      Rotation2d angle = modules[i].trueSteerAngle();
      angles[i] = angle.getRadians();
    }
    return new OdometrySample(0, distances, angles);
  }
}
