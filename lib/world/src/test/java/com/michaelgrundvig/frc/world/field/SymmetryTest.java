package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.TestFields.LENGTH;
import static com.michaelgrundvig.frc.world.field.TestFields.WIDTH;
import static com.michaelgrundvig.frc.world.field.TestFields.assertPose;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

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

  /**
   * A pose in 3D turns as a heading does: its x axis (a tag's face) goes where the mirror image of
   * the x axis goes, its z axis too, and it's still a pose (its y axis turned back).
   */
  @ParameterizedTest
  @EnumSource(Symmetry.class)
  void aPoseIn3dTurnsItsAxesWithTheField(Symmetry symmetry) {
    Pose3d pose = new Pose3d(3, 2, 1.2, new Rotation3d(0.3, -0.4, 2.1));

    Pose3d image = symmetry.apply(pose, LENGTH, WIDTH);
    Pose3d twice = symmetry.apply(image, LENGTH, WIDTH);

    Translation2d expected = symmetry.apply(pose.getTranslation().toTranslation2d(), LENGTH, WIDTH);
    assertThat(image.getX()).isCloseTo(expected.getX(), within(1e-12));
    assertThat(image.getY()).isCloseTo(expected.getY(), within(1e-12));
    assertThat(image.getZ()).isEqualTo(1.2);
    assertThat(image.getRotation().toRotation2d().getDegrees())
        .as("its heading, as a 2D pose's")
        .isCloseTo(symmetry.apply(pose.toPose2d().getRotation()).getDegrees(), within(1e-9));
    for (Translation3d axis :
        List.of(
            new Translation3d(1, 0, 0), new Translation3d(0, 1, 0), new Translation3d(0, 0, 1))) {
      Translation3d before = axis.rotateBy(pose.getRotation());
      Translation3d after = axis.rotateBy(image.getRotation());
      Translation2d flat =
          symmetry.apply(before.toTranslation2d(), 0, 0); // a direction: the field's size drops out
      double sign = symmetry == Symmetry.MIRRORED && axis.getY() == 1 ? -1 : 1;
      assertThat(after.getX()).as("axis %s", axis).isCloseTo(sign * flat.getX(), within(1e-9));
      assertThat(after.getY()).as("axis %s", axis).isCloseTo(sign * flat.getY(), within(1e-9));
      assertThat(after.getZ()).as("axis %s", axis).isCloseTo(sign * before.getZ(), within(1e-9));
    }
    assertThat(twice.getTranslation().getDistance(pose.getTranslation())).isLessThan(1e-12);
    // A quaternion and its negative are one rotation: a whole turn is none.
    double turn = twice.getRotation().relativeTo(pose.getRotation()).getAngle();
    assertThat(Math.min(turn, 2 * Math.PI - turn)).isLessThan(1e-9);
  }

  /**
   * Something placed from a pose by an offset lands, placed from the pose's image by the offset's
   * image, on its own image: how a target placed from a blue tag is placed from the tag's partner.
   */
  @ParameterizedTest
  @EnumSource(Symmetry.class)
  void anOffsetFromAPoseLandsOnTheImageFromThePosesImage(Symmetry symmetry) {
    Pose3d pose = new Pose3d(3, 2, 1.2, new Rotation3d(0.3, -0.4, 2.1));
    Transform3d offset = new Transform3d(0.4, -0.7, 0.25, new Rotation3d(-0.6, 1.1, 0.2));

    Pose3d image = symmetry.apply(pose.transformBy(offset), LENGTH, WIDTH);
    Pose3d placed = symmetry.apply(pose, LENGTH, WIDTH).transformBy(symmetry.inImage(offset));

    assertThat(placed.getTranslation().getDistance(image.getTranslation())).isLessThan(1e-12);
    double turn = placed.getRotation().relativeTo(image.getRotation()).getAngle();
    assertThat(Math.min(turn, 2 * Math.PI - turn)).isLessThan(1e-9);
    assertThat(symmetry.apply(pose.getRotation()))
        .isEqualTo(symmetry.apply(pose, LENGTH, WIDTH).getRotation());
  }

  private static Pose2d pose(double x, double y, double degrees) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
  }
}
