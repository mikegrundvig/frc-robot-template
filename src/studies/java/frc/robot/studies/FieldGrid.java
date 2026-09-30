package frc.robot.studies;

import static org.wpilib.units.Units.Meters;

import frc.robot.lib.config.Body;
import frc.robot.lib.field.FieldModel;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Where the robot might be: positions on a grid over the whole field, each at several headings.
 * Nothing about any game goes in, so every spot counts the same unless the {@link Weights} say
 * otherwise.
 */
public final class FieldGrid {
  private final int columns;
  private final int rows;
  private final double spacing;
  private final List<Sample> samples = new ArrayList<>();

  /**
   * @param body the robot, kept half its size (bumpers included) off the walls
   * @param spacingMeters the most the positions are apart, both ways: each cell of the grid is this
   *     size or a little smaller, so the grid fits the field evenly
   * @param headings how many headings at each position, starting straight downfield
   */
  public FieldGrid(
      FieldModel field, Body body, double spacingMeters, int headings, Weights weights) {
    double margin = Math.max(body.bumperLength().in(Meters), body.bumperWidth().in(Meters)) / 2;
    double spanX = field.length() - 2 * margin;
    double spanY = field.width() - 2 * margin;
    if (!(spanX > 0 && spanY > 0)) {
      throw new IllegalArgumentException("the robot doesn't fit on the field " + field.name());
    }
    // Each position is the center of its cell, so the grid sits evenly between the walls.
    columns = (int) Math.ceil(spanX / spacingMeters - 1e-9);
    rows = (int) Math.ceil(spanY / spacingMeters - 1e-9);
    double stepX = spanX / columns;
    double stepY = spanY / rows;
    spacing = Math.max(stepX, stepY);
    for (int row = 0; row < rows; row++) {
      for (int column = 0; column < columns; column++) {
        Translation2d position =
            new Translation2d(margin + (column + 0.5) * stepX, margin + (row + 0.5) * stepY);
        double weight = weights.at(field, position);
        for (int heading = 0; heading < headings; heading++) {
          Rotation2d facing = Rotation2d.fromRadians(2 * Math.PI * heading / headings);
          samples.add(new Sample(new Pose2d(position, facing), weight, column, row));
        }
      }
    }
  }

  /** Every pose, row by row from the field's y = 0 side, each position's headings together. */
  public List<Sample> samples() {
    return List.copyOf(samples);
  }

  /** Positions along the field's length. */
  public int columns() {
    return columns;
  }

  /** Positions across its width. */
  public int rows() {
    return rows;
  }

  /** How far apart the positions really are, the larger of the two ways. */
  public double spacingMeters() {
    return spacing;
  }
}
