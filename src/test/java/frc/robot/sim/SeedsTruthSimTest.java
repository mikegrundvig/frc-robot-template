package frc.robot.sim;

import static frc.robot.sim.KnownGap.Check.stillOpen;
import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.vision.VisionSim;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * The truth suite's seeds: a simulated run is a function of its seed, so its random streams must be
 * its own, apart from one another and from every other seed's, and the run must come out the same
 * to the bit however it's run.
 */
@ExtendWith(RobotSim.class)
class SeedsTruthSimTest {
  /** The seeds a study runs: the consistency study's twelve, and a scenario lab's hundred. */
  private static final int SEEDS = 100;

  /** The template's robot's cameras. */
  private static final int CAMERAS = Robots.SIM.definition().cameras().size();

  /**
   * Every random stream, over a study's seeds, starts from a seed of its own: no two draw the same
   * numbers. Today the streams are offsets from the world's seed, and they overlap: a seed's gyro
   * draws what another seed's gyro's accelerometer does, and seed 1's gyro what its first camera
   * does.
   */
  @Test
  @KnownGap("each stream is an offset from the world's seed, not drawn from the seed and its name")
  void everyStreamOfEverySeedIsItsOwn() {
    Map<Long, String> streams = new LinkedHashMap<>();
    List<String> shared = new ArrayList<>();
    for (long seed = 1; seed <= SEEDS; seed++) {
      Map<String, Long> world = streams(seed);
      for (Map.Entry<String, Long> stream : world.entrySet()) {
        String name = "seed " + seed + "'s " + stream.getKey();
        String before = streams.putIfAbsent(stream.getValue(), name);
        if (before != null) {
          shared.add(name + " = " + before);
        }
      }
    }

    stillOpen(
        "every stream its own",
        shared.isEmpty(),
        shared.size() == 202
            && shared.contains("seed 1's first camera = seed 1's gyro")
            && shared.contains("seed 2's gyro = seed 1's gyro's accelerometer"),
        shared.size()
            + " streams shared, the first "
            + shared.subList(0, Math.min(4, shared.size())));
  }

  /**
   * Streams from seeds of their own draw uncorrelated numbers: the first 10,000 normal draws of
   * every pair a world's seeds give (neighbors included, a seed and the next), correlated within
   * three standard errors, 3 / √n.
   */
  @Test
  void streamsFromTheirOwnSeedsAreUncorrelated() {
    int n = 10_000;
    List<double[]> draws = new ArrayList<>();
    for (long seed : new java.util.TreeSet<>(streams(1).values())) {
      Random random = new Random(seed);
      double[] values = new double[n];
      for (int i = 0; i < n; i++) {
        values[i] = random.nextGaussian();
      }
      draws.add(values);
    }
    for (int a = 0; a < draws.size(); a++) {
      for (int b = a + 1; b < draws.size(); b++) {
        assertThat(Math.abs(correlation(draws.get(a), draws.get(b))))
            .as("streams %d and %d", a, b)
            .isLessThan(3 / Math.sqrt(n));
      }
    }
  }

  /**
   * The same seed gives the same run, to the bit: repeated, and run on eight threads at once, each
   * its own world.
   */
  @Test
  void aRunIsTheSameToTheBitAloneRepeatedAndBesideOthers() throws Exception {
    List<Long> alone = fingerprint(3);
    assertThat(fingerprint(3)).as("repeated").isEqualTo(alone);
    assertThat(fingerprint(4)).as("another seed").isNotEqualTo(alone);

    ExecutorService threads = Executors.newFixedThreadPool(8);
    try {
      List<Future<List<Long>>> runs = new ArrayList<>();
      for (int i = 0; i < 8; i++) {
        runs.add(threads.submit(() -> fingerprint(3)));
      }
      for (Future<List<Long>> run : runs) {
        assertThat(run.get()).as("beside seven others").isEqualTo(alone);
      }
    } finally {
      threads.shutdownNow();
    }
  }

  /** A world's random streams by name, from its seed. */
  private static Map<String, Long> streams(long seed) {
    Map<String, Long> streams = new LinkedHashMap<>();
    streams.put("gyro", Seeds.gyro(seed));
    streams.put("gyro's accelerometer", Seeds.gyroAccelerometer(seed));
    streams.put("gyro's accelerometer's up axis", Seeds.upAxis(Seeds.gyroAccelerometer(seed)));
    streams.put("backup IMU", Seeds.backupGyro(seed));
    streams.put("backup IMU's accelerometer", Seeds.backupAccelerometer(seed));
    streams.put(
        "backup IMU's accelerometer's up axis", Seeds.upAxis(Seeds.backupAccelerometer(seed)));
    for (int camera = 0; camera < CAMERAS; camera++) {
      String[] order = {"first", "second", "third", "fourth", "fifth", "sixth"};
      streams.put(
          (camera < order.length ? order[camera] : "#" + (camera + 1)) + " camera",
          VisionSim.cameraSeed(seed, camera));
    }
    return streams;
  }

  /**
   * A season-worn robot driven through launches, turns, and a hit for 3 s, its truth and every
   * sensor reading taken down to the bit.
   */
  private static List<Long> fingerprint(long seed) {
    SensorErrors errors = SensorErrorsBuilder.builder(SensorErrors.TYPICAL).seed(seed).build();
    TruthRig rig =
        new TruthRig(
            Robots.SIM.definition().body(),
            Robots.SIM.definition().drivetrain().modules().wheelCoefficientOfFriction(),
            errors);
    List<Long> bits = new ArrayList<>();
    for (int loop = 0; loop < 150; loop++) {
      if (loop % 50 == 0) {
        rig.steer(Rotation2d.fromDegrees(40 * loop / 50.0));
        rig.driveVolts(loop == 100 ? -8 : 10);
      }
      if (loop == 75) {
        rig.simulation.hit(new org.wpilib.math.geometry.Transform2d(0.1, 0.2, Rotation2d.ZERO));
      }
      rig.loops(1);
      Pose3d truth = rig.simulation.truePose3d();
      add(bits, truth.getX(), truth.getY(), truth.getZ(), truth.getRotation().getZ());
      GyroIO.GyroIOInputs gyro = new GyroIO.GyroIOInputs();
      rig.simulation.gyro().updateInputs(gyro);
      add(
          bits,
          gyro.yaw.getRadians(),
          gyro.accelerationXMetersPerSecSq,
          gyro.accelerationYMetersPerSecSq);
      add(bits, gyro.odometryYawsRad);
      for (ModuleIO module : rig.modules) {
        ModuleIO.ModuleIOInputs inputs = new ModuleIO.ModuleIOInputs();
        module.updateInputs(inputs);
        add(
            bits,
            inputs.drivePositionRad,
            inputs.driveStatorCurrentAmps,
            inputs.steerAngle.getRadians());
        add(bits, inputs.odometryDrivePositionsRad);
      }
    }
    return bits;
  }

  private static void add(List<Long> bits, double... values) {
    for (double value : values) {
      bits.add(Double.doubleToRawLongBits(value));
    }
  }

  private static double correlation(double[] a, double[] b) {
    double meanA = 0;
    double meanB = 0;
    for (int i = 0; i < a.length; i++) {
      meanA += a[i] / a.length;
      meanB += b[i] / b.length;
    }
    double ab = 0;
    double aa = 0;
    double bb = 0;
    for (int i = 0; i < a.length; i++) {
      ab += (a[i] - meanA) * (b[i] - meanB);
      aa += (a[i] - meanA) * (a[i] - meanA);
      bb += (b[i] - meanB) * (b[i] - meanB);
    }
    return ab / Math.sqrt(aa * bb);
  }
}
