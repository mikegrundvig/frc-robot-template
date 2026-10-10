package frc.robot.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.HolonomicSample;
import org.wpilib.math.trajectory.HolonomicTrajectory;

/** The paths the drivetrain's moves plan, held to what a move may ask of the robot. */
class TrajectoriesTest {
  /** About the template's robot's limits, at half its model's accelerations. */
  private static final Trajectories.Limits LIMITS =
      new Trajectories.Limits(4.4, 2 * Math.PI, 2.3, 9.2);

  private static final Pose2d ORIGIN = Pose2d.ZERO;
  private static final ChassisVelocities STILL = new ChassisVelocities();

  private static Keyframe at(double x, double y, double degrees) {
    return Keyframe.at(new Pose2d(x, y, Rotation2d.fromDegrees(degrees)));
  }

  private static double degrees(HolonomicTrajectory path, double t) {
    return path.sampleAt(t).pose.getRotation().getDegrees();
  }

  /** A line's place and heading move on their own profiles, and both end at the goal, still. */
  @Test
  void aLineEndsAtItsGoalStill() {
    Trajectories.Plan plan =
        Trajectories.line(
            ORIGIN, STILL, new Translation2d(2, 1), Rotation2d.fromDegrees(90), LIMITS);
    HolonomicSample end = plan.trajectory().end();

    assertThat(end.pose.getTranslation().getDistance(new Translation2d(2, 1))).isLessThan(1e-9);
    assertThat(end.pose.getRotation().getDegrees()).isCloseTo(90, within(1e-6));
    assertThat(Math.hypot(end.velocity.vx, end.velocity.vy)).isZero();
    // 2.24 m at 2.3 m/s², never reaching top speed: 2√(2.24/2.3) s.
    assertThat(plan.trajectory().duration).isCloseTo(1.97, within(0.03));
  }

  /**
   * Through three keyframes: it passes each where it is, faces each keyframe's heading by the time
   * it passes it, and never asks more than the limits: top speed, and acceleration along the path
   * and across it, each the move's share.
   */
  @Test
  void aPathPassesEachKeyframeFacingIt() {
    List<Keyframe> keyframes = List.of(at(2, 1, 0), at(4, 0, 90), at(6, 1.5, 180));
    Trajectories.Plan plan = Trajectories.through(ORIGIN, STILL, keyframes, LIMITS);
    HolonomicTrajectory path = plan.trajectory();

    for (int k = 0; k < keyframes.size(); k++) {
      Pose2d passes = path.sampleAt(plan.passing()[k]).pose;
      Pose2d keyframe = Objects.requireNonNull(keyframes.get(k).pose());
      assertThat(passes.getTranslation().getDistance(keyframe.getTranslation())).isLessThan(0.001);
      assertThat(passes.getRotation().minus(keyframe.getRotation()).getDegrees())
          .as("facing keyframe %d as it passes", k)
          .isCloseTo(0, within(0.01));
    }
    assertThat(plan.passing()).isSorted();

    for (HolonomicSample sample : path.getSamples()) {
      assertThat(Math.hypot(sample.velocity.vx, sample.velocity.vy)).isLessThan(4.4 + 1e-6);
      assertThat(Math.abs(sample.velocity.omega)).isLessThan(2 * Math.PI + 1e-6);
    }
    // Along and across together, sampled every loop, each held to 2.3 m/s².
    for (double t = 0; t + 0.02 <= path.duration; t += 0.02) {
      ChassisVelocities was = path.sampleAt(t).velocity;
      ChassisVelocities next = path.sampleAt(t + 0.02).velocity;
      double accelerating = Math.hypot(next.vx - was.vx, next.vy - was.vy) / 0.02;
      assertThat(accelerating).as("at %.2f s", t).isLessThan(2.3 * Math.sqrt(2) + 0.1);
    }
  }

  /**
   * A keyframe's heading is turned to once the one before has been passed and faced: on a short hop
   * with a half turn, the path waits at its end, turning, until the robot faces it.
   */
  @Test
  void aShortHopWithAHalfTurnWaitsToFaceIt() {
    Trajectories.Plan plan = Trajectories.through(ORIGIN, STILL, List.of(at(0.3, 0, 180)), LIMITS);
    HolonomicTrajectory path = plan.trajectory();

    // 0.3 m takes 0.72 s; a half turn at 9.2 rad/s², 1.17 s.
    assertThat(plan.passing()[0]).isCloseTo(0.72, within(0.03));
    assertThat(path.duration).isCloseTo(1.17, within(0.03));
    assertThat(path.end().pose.getTranslation().getDistance(new Translation2d(0.3, 0)))
        .isLessThan(1e-9);
    assertThat(degrees(path, path.duration)).isCloseTo(180, within(0.01));
    assertThat(degrees(path, 0.9)).as("still turning, there").isBetween(90.0, 179.0);
  }

  /** Each heading is reached the nearer way round from the keyframe's before it. */
  @Test
  void headingsTurnTheNearerWay() {
    Trajectories.Plan plan =
        Trajectories.through(ORIGIN, STILL, List.of(at(1, 0, -170), at(2, 0, 170)), LIMITS);
    HolonomicTrajectory path = plan.trajectory();

    double most = 0;
    for (HolonomicSample sample : path.getSamples()) {
      most = Math.max(most, sample.pose.getRotation().getDegrees());
      if (sample.time > plan.passing()[0]) {
        break;
      }
    }
    assertThat(most).as("never turned left on the way to -170°").isLessThan(1);
    assertThat(path.end().velocity.omega).isZero();
    assertThat(Math.abs(degrees(path, path.duration))).isCloseTo(170, within(0.01));
  }

  /** Keyframes too near where the robot is, or each other, have no path through them. */
  @Test
  void keyframesTooCloseHaveNoPath() {
    assertThatThrownBy(() -> Trajectories.through(ORIGIN, STILL, List.of(at(0.01, 0, 90)), LIMITS))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("(0.01, 0.00) is within 5 cm of where the robot starts");
    assertThatThrownBy(
            () ->
                Trajectories.through(
                    ORIGIN, STILL, List.of(at(1, 0, 0), at(1, 0.02, 0).named("there")), LIMITS))
        .hasMessage("there is within 5 cm of the keyframe before");
  }

  /** Already moving along it, a path starts at the robot's speed rather than from rest. */
  @Test
  void aMovingRobotStartsAtItsSpeed() {
    Trajectories.Plan plan =
        Trajectories.through(
            ORIGIN, new ChassisVelocities(1.5, 0, 0), List.of(at(3, 0, 0)), LIMITS);

    assertThat(plan.trajectory().start().velocity.vx).isCloseTo(1.5, within(1e-6));
  }
}
