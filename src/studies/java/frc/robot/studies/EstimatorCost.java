package frc.robot.studies;

import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.DrivetrainReadingsBuilder;
import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.localization.ImuReadingsBuilder;
import com.michaelgrundvig.frc.localization.Localizer;
import com.michaelgrundvig.frc.localization.LoopContext;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.PoseReset;
import com.michaelgrundvig.frc.localization.VisionMeasurement;
import com.michaelgrundvig.frc.localization.YawSamples;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.wpilib.math.geometry.Pose2d;

/**
 * What each estimator costs a robot loop: one run's inputs, kept loop by loop, played again into a
 * fresh estimator alone on the thread, with the time each loop took and the bytes it allocated.
 *
 * <p>The robot's own estimator is timed from its drivetrain readings to its update, given the
 * measurements its vision pipeline made (solving the images isn't in it). WPILib's is timed the
 * same way given the same measurements (B), and given the images, with solving and filtering them
 * (C).
 */
final class EstimatorCost {
  /** How many times the run is played into each estimator: the last is the one measured. */
  static final int PASSES = 5;

  private EstimatorCost() {}

  /**
   * One loop's inputs.
   *
   * @param nowNanos the robot's clock
   * @param enabled whether the robot was enabled
   * @param readings what the drivetrain's sensors read
   * @param measurements the measurements the vision pipeline made, as the localizer was given them
   * @param images the images the cameras sent
   */
  record Loop(
      long nowNanos,
      boolean enabled,
      DrivetrainReadings readings,
      List<VisionMeasurement> measurements,
      List<WpilibContender.Image> images) {
    Loop {
      // The drivetrain hands its IMUs' readings over in arrays it fills again next loop.
      readings =
          DrivetrainReadingsBuilder.builder(readings)
              .gyro(kept(readings.gyro()))
              .backupImu(kept(readings.backupImu()))
              .build();
      measurements = List.copyOf(measurements);
      images = List.copyOf(images);
    }

    private static ImuReadings kept(ImuReadings imu) {
      YawSamples yaws = imu.yaws();
      long[] times = new long[yaws.count()];
      double[] values = new double[yaws.count()];
      for (int i = 0; i < times.length; i++) {
        times[i] = yaws.timeNanos(i);
        values[i] = yaws.yawRadians(i);
      }
      return ImuReadingsBuilder.builder(imu).yaws(YawSamples.of(times, values)).build();
    }
  }

  /**
   * What one estimator cost.
   *
   * @param name the estimator
   * @param medianMicros the middle loop's time, in microseconds
   * @param p99Micros the time 99% of loops were within
   * @param worstMicros the slowest loop's
   * @param bytesPerLoop the bytes allocated, a loop, on average
   */
  record Cost(
      String name, double medianMicros, double p99Micros, double worstMicros, double bytesPerLoop) {
    String csv() {
      return String.format(
          Locale.ROOT,
          "%s,%.2f,%.2f,%.2f,%.0f",
          name,
          medianMicros,
          p99Micros,
          worstMicros,
          bytesPerLoop);
    }
  }

  /** What an estimator does with one loop's inputs. */
  private interface Player {
    void play(Loop loop, int index);
  }

  /** Each estimator's cost over a run's loops, the robot's own first. */
  static List<Cost> measure(List<Loop> loops, Pose2d start) {
    List<Cost> costs = new ArrayList<>();
    Cost ours = null;
    for (int pass = 0; pass < PASSES; pass++) {
      ours = time(EstimatorRuns.OURS, loops, ours(start));
    }
    costs.add(ours);
    for (String name : List.of("B", "C", "D")) {
      Cost cost = null;
      for (int pass = 0; pass < PASSES; pass++) {
        cost = time(name, loops, wpilib(name, start));
      }
      costs.add(cost);
    }
    return costs;
  }

  /** How sure a placement on the robot's start is, as the robot's own localizer is told. */
  private static final PoseCovariance PLACED =
      PoseCovariance.fromStdDevs(0.05, 0.05, Math.toRadians(2));

  private static Player ours(Pose2d start) {
    Localizer localizer = ExampleRobot.SIM.localizerOn(EstimatorRuns.FIELD);
    return (loop, index) -> {
      localizer.odometry(loop.readings(), new LoopContext(loop.nowNanos(), loop.enabled()));
      if (index == 0) {
        localizer.place(new PoseReset(loop.nowNanos(), start, PLACED, "placed on its start"));
      }
      localizer.measurements(loop.measurements());
      localizer.pose();
    };
  }

  private static Player wpilib(String name, Pose2d start) {
    WpilibContender contender =
        EstimatorRuns.contenders().stream()
            .filter(each -> each.name().equals(name))
            .findFirst()
            .orElseThrow();
    return (loop, index) -> {
      contender.odometry(loop.readings(), new LoopContext(loop.nowNanos(), loop.enabled()));
      if (index == 0) {
        contender.place(start, loop.nowNanos());
      }
      ImuReadings imu = EstimatorRig.imu(loop.readings());
      contender.imu(imu.yawRateRadiansPerSecond(), imu.pitchRadians(), imu.rollRadians());
      if (contender.feed() == WpilibContender.Feed.SAME_INPUTS) {
        contender.measurements(loop.measurements());
      } else {
        contender.images(loop.images());
      }
      contender.pose();
    };
  }

  private static Cost time(String name, List<Loop> loops, Player player) {
    com.sun.management.ThreadMXBean threads =
        (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
    double[] micros = new double[loops.size()];
    long bytes = 0;
    for (int i = 0; i < loops.size(); i++) {
      Loop loop = loops.get(i);
      long bytesBefore = threads.getCurrentThreadAllocatedBytes();
      long before = System.nanoTime();
      player.play(loop, i);
      long after = System.nanoTime();
      bytes += threads.getCurrentThreadAllocatedBytes() - bytesBefore;
      micros[i] = (after - before) / 1e3;
    }
    return new Cost(
        name,
        EstimatorScores.percentile(micros, 0.5),
        EstimatorScores.percentile(micros, 0.99),
        EstimatorScores.worst(micros),
        (double) bytes / Math.max(1, loops.size()));
  }
}
