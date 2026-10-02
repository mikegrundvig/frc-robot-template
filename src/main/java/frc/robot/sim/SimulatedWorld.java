package frc.robot.sim;

import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.coprocessor.table.Computer;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Imus;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.subsystems.coprocessors.CoprocessorIO;
import frc.robot.subsystems.coprocessors.CoprocessorIOSim;
import frc.robot.subsystems.coprocessors.Coprocessors;
import frc.robot.subsystems.coprocessors.PhotonVisionIO;
import frc.robot.subsystems.coprocessors.PhotonVisionIOSim;
import frc.robot.subsystems.drive.DriveHardware;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.VisionSim;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform3d;

/**
 * The world the simulated robot drives in: the drivetrain's physics, which knows where the robot
 * really is, and PhotonVision's simulated cameras, rendering what they'd see from there. The
 * robot's code sees it only through its IO interfaces ({@link #drivetrainHardware}, {@link
 * #cameras}, {@link #coprocessors}), as it would see real hardware.
 *
 * <p>The robot's wheels and sensors differ from its configuration as a real robot's do ({@link
 * SensorErrors}), so its estimate drifts from the truth, and localization is tested against that
 * gap, not against a perfect world.
 *
 * <p>Its parts can fail while it runs: {@link Faults} declares each one, and what can go wrong with
 * it, to Pitwall's simulator controls ({@link #addTo}), where the page and the scenarios apply
 * them.
 */
public final class SimulatedWorld implements AutoCloseable {
  private final FieldModel field;
  private final DriveSimulation drivetrain;
  private final VisionSim vision;
  private final CameraIOSim[] cameras;

  /** The coprocessors' simulated health agents, once the robot asks for them. */
  private final List<CoprocessorIOSim> coprocessors = new ArrayList<>();

  /**
   * @param field the field, with its tags and terrain
   * @param drivetrain the drivetrain as defined: its modules, and where its wheels sit
   * @param body the robot's frame, which places the wheels, and its mass, which they carry
   * @param cameras the robot's cameras, as configured
   * @param errors how the robot's wheels, sensors, and camera mounts differ from its configuration
   */
  public SimulatedWorld(
      FieldModel field,
      DrivetrainGeometry drivetrain,
      Body body,
      List<CameraConfig> cameras,
      SensorErrors errors) {
    this(field, drivetrain, body, cameras, Imus.UNCHECKED, errors);
  }

  /**
   * @param field the field, with its tags and terrain
   * @param drivetrain the drivetrain as defined: its modules, and where its wheels sit
   * @param body the robot's frame, which places the wheels, and its mass, which they carry
   * @param cameras the robot's cameras, as configured
   * @param imus where the robot's definition mounts its IMUs
   * @param errors how the robot's wheels, sensors, and camera mounts differ from its configuration
   */
  public SimulatedWorld(
      FieldModel field,
      DrivetrainGeometry drivetrain,
      Body body,
      List<CameraConfig> cameras,
      Imus imus,
      SensorErrors errors) {
    this.field = field;
    this.drivetrain = new DriveSimulation(body, drivetrain, errors, imus);
    this.drivetrain.setTerrain(field.terrain());
    vision = new VisionSim(field, errors.tagErrors(), errors.seed());
    this.cameras = new CameraIOSim[cameras.size()];
    for (int i = 0; i < cameras.size(); i++) {
      CameraConfig camera = cameras.get(i);
      Transform3d mountError = errors.cameraMountError(camera.name());
      this.cameras[i] =
          new CameraIOSim(camera.name(), vision.camera(camera, mountError), vision, mountError);
    }
  }

  /**
   * Declares the world's parts that can fail, and what can go wrong with each, to Pitwall's
   * simulator controls: the cameras, the gyro and the backup IMU, each module, and the robot itself
   * ({@link Faults}).
   */
  public void addTo(Components components) {
    Faults.declare(this, components);
  }

  /** The drivetrain's simulated modules and IMUs, for the drivetrain to read. */
  public DriveHardware drivetrainHardware() {
    return DriveHardware.builder()
        .modules(List.of(drivetrain.modules()))
        .gyro(drivetrain.gyro())
        .backupImu(drivetrain.backupGyro())
        .odometryLock(new ReentrantLock())
        .build();
  }

  /** The simulated cameras, in the robot's configured order. */
  public CameraIO[] cameras() {
    return cameras.clone();
  }

  /**
   * The coprocessors' simulated health agents, one for each computer in the table, in its order:
   * made the first time they're asked for, and the same ones after.
   *
   * @param table the table of coprocessors compiled into the program
   * @param settings how often they're asked
   */
  public CoprocessorIO[] coprocessors(CompiledTable table, Coprocessors.Settings settings) {
    if (coprocessors.isEmpty()) {
      for (Computer computer : table.table().computers()) {
        coprocessors.add(new CoprocessorIOSim(table, computer, settings));
      }
    }
    return coprocessors.toArray(CoprocessorIO[]::new);
  }

  /** PhotonVision's version, as the simulated coprocessors publish it. */
  public PhotonVisionIO photonVision() {
    return new PhotonVisionIOSim(coprocessors);
  }

  /** The coprocessors' simulated agents, for their faults. */
  List<CoprocessorIOSim> coprocessorSims() {
    return List.copyOf(coprocessors);
  }

  /** The cameras' names, in the robot's configured order. */
  public List<String> cameraNames() {
    return Arrays.stream(cameras).map(CameraIOSim::name).toList();
  }

  /** A simulated camera, for its faults. */
  CameraIOSim camera(int index) {
    return cameras[index];
  }

  /** The field the robot drives on. */
  FieldModel field() {
    return field;
  }

  /** The cameras' simulation, which places the field's tags: for their faults. */
  VisionSim vision() {
    return vision;
  }

  /** The drive team puts the robot here. */
  public void place(Pose2d pose) {
    drivetrain.place(pose);
  }

  /**
   * One loop: the world moves on, and the cameras see where the robot now is. Before the robot
   * reads its sensors, as on a robot between loops.
   */
  public void step(long nowNanos) {
    drivetrain.step(RobotLoop.PERIOD_SECONDS, nowNanos);
    vision.update(drivetrain.truePose3d());
    for (CoprocessorIOSim coprocessor : coprocessors) {
      coprocessor.step(nowNanos);
    }
  }

  /** Where the robot really is. */
  public Pose2d truePose() {
    return drivetrain.truePose();
  }

  /**
   * Logs where the robot really is, and how far the estimate is from it: what localization has to
   * close. In 3D too, raised and tilted as the robot really is, for AdvantageScope's ghost.
   */
  public void log(Pose2d estimated) {
    Pose2d truth = drivetrain.truePose();
    Logger.recordOutput("Sim/TruePose", truth);
    Logger.recordOutput("Sim/TruePose3d", drivetrain.truePose3d());
    Logger.recordOutput(
        "Sim/PoseErrorMeters", truth.getTranslation().getDistance(estimated.getTranslation()));
    Logger.recordOutput(
        "Sim/HeadingErrorDegrees", estimated.getRotation().minus(truth.getRotation()).getDegrees());
  }

  /** The drivetrain's physics: for tests to shove, tilt, or trip it. */
  public DriveSimulation drivetrain() {
    return drivetrain;
  }

  @Override
  public void close() {
    vision.close();
  }

  /** How many cameras it has. */
  int cameraCount() {
    return cameras.length;
  }
}
