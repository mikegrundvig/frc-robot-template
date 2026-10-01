package frc.robot.studies;

import org.wpilib.math.geometry.Pose2d;

/**
 * A pose the study puts the robot at: how much it counts, and where it goes on the report's map.
 *
 * @param pose where the robot stands on the field, and which way it faces
 * @param weight how much it counts: 1 unless the weights say otherwise, 0 to leave it out
 * @param column its column on the map, counted from the blue alliance's wall
 * @param row its row on the map, counted from the field's y = 0 side
 */
public record Sample(Pose2d pose, double weight, int column, int row) {}
