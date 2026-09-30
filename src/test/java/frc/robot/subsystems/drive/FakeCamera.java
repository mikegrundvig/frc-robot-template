package frc.robot.subsystems.drive;

import frc.robot.lib.localization.PoseCovariance;
import frc.robot.lib.localization.VisionMeasurement;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A stand-in camera for the estimator's simulation tests, until PhotonLib's simulated cameras are
 * wired in: it sees the robot's true pose (never the estimate), with noise, a frame rate, and a
 * delay before each result arrives, and reports a heading-constrained solve as a real pipeline
 * would. It can be told to lie once, as a misread tag does.
 */
final class FakeCamera {
  private final DriveSimulation robot;
  private final double stdDevMeters;
  private final long latencyNanos;
  private final long frameNanos;
  private final Random noise;
  private final ArrayDeque<VisionMeasurement> inFlight = new ArrayDeque<>();
  private long lastFrameNanos = Long.MIN_VALUE;
  private Transform2d nextLie = Transform2d.ZERO;

  FakeCamera(
      DriveSimulation robot,
      double stdDevMeters,
      double latencySeconds,
      double framesPerSecond,
      long seed) {
    this.robot = robot;
    this.stdDevMeters = stdDevMeters;
    latencyNanos = Math.round(latencySeconds * 1e9);
    frameNanos = Math.round(1e9 / framesPerSecond);
    noise = new Random(seed);
  }

  /** The next picture's result will be off by this much, however sure it claims to be. */
  void lieOnce(Transform2d lie) {
    nextLie = lie;
  }

  /** Takes a picture if one is due, and returns the results that have arrived by now. */
  List<VisionMeasurement> poll(long nowNanos) {
    if (lastFrameNanos == Long.MIN_VALUE || nowNanos - lastFrameNanos >= frameNanos) {
      lastFrameNanos = nowNanos;
      Pose2d truth = robot.truePose();
      Pose2d seen =
          new Pose2d(
                  truth
                      .getTranslation()
                      .plus(
                          new Translation2d(
                              stdDevMeters * noise.nextGaussian(),
                              stdDevMeters * noise.nextGaussian())),
                  truth.getRotation())
              .plus(nextLie);
      nextLie = Transform2d.ZERO;
      inFlight.addLast(
          new VisionMeasurement(
              nowNanos,
              seen,
              PoseCovariance.fromStdDevs(stdDevMeters, stdDevMeters, Double.POSITIVE_INFINITY),
              "fake",
              List.of(26),
              VisionMeasurement.Solver.HEADING_CONSTRAINED));
    }
    List<VisionMeasurement> arrived = new ArrayList<>();
    while (!inFlight.isEmpty() && inFlight.getFirst().timestampNanos() + latencyNanos <= nowNanos) {
      arrived.add(inFlight.removeFirst());
    }
    return arrived;
  }
}
