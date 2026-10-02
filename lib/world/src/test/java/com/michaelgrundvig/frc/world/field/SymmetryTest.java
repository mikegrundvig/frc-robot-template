package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.TestFields.LENGTH;
import static com.michaelgrundvig.frc.world.field.TestFields.WIDTH;
import static com.michaelgrundvig.frc.world.field.TestFields.assertPose;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

class SymmetryTest {
  @Test
  void rotatedTurnsAPoseHalfATurnAboutTheCenter() {
    Pose2d image = Symmetry.ROTATED.apply(pose(1, 2, 30), LENGTH, WIDTH);

    assertPose(image, 15, 6, 210);
  }

  @Test
  void mirroredReflectsAPoseAcrossTheCenterLine() {
    Pose2d image = Symmetry.MIRRORED.apply(pose(1, 2, 30), LENGTH, WIDTH);

    assertPose(image, 15, 2, 150);
  }

  @ParameterizedTest
  @EnumSource(Symmetry.class)
  void applyingItTwiceGivesBackThePose(Symmetry symmetry) {
    for (Pose2d pose :
        new Pose2d[] {pose(1, 2, 30), pose(8, 4, 0), pose(15.5, 0.25, -135), pose(3, 7, 90)}) {
      Pose2d twice = symmetry.apply(symmetry.apply(pose, LENGTH, WIDTH), LENGTH, WIDTH);

      assertPose(twice, pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
  }

  /** The 2026 code rotated positions but mirrored headings; a robot aimed at a target missed. */
  @ParameterizedTest
  @EnumSource(Symmetry.class)
  void positionAndHeadingTurnTogether(Symmetry symmetry) {
    Translation2d robot = new Translation2d(2, 1);
    Translation2d target = new Translation2d(5, 6);
    Rotation2d aim = target.minus(robot).getAngle().orElseThrow();

    Translation2d robotImage = symmetry.apply(robot, LENGTH, WIDTH);
    Translation2d targetImage = symmetry.apply(target, LENGTH, WIDTH);
    Rotation2d aimImage = symmetry.apply(aim);

    Rotation2d toTargetImage = targetImage.minus(robotImage).getAngle().orElseThrow();
    assertThat(aimImage.minus(toTargetImage).getDegrees()).isCloseTo(0.0, within(1e-9));
  }

  private static Pose2d pose(double x, double y, double degrees) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
  }
}
