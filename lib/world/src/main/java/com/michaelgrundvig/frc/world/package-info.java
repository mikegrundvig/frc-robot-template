/**
 * The world model: what the field and the robot are, described once, and read by everything that
 * needs to know them: the robot program, localization, the simulator, and the views.
 *
 * <ul>
 *   <li>{@code config}: the validation kit every configuration record is built with, by name, and
 *       checked with.
 *   <li>{@code physical}: what the field and the robot are both made of: mass and inertia, shapes
 *       and materials, and the plain geometry they're written in, where WPILib's math has none.
 *   <li>{@code field}: the field, its tags, the places and zones code refers to by name, its
 *       terrain, and the elements that stand on it.
 *   <li>{@code robot}: the robot's definition, its body, drivetrain, cameras, and IMUs.
 * </ul>
 *
 * <p>The field and the robot don't know about each other, and the kit and the physical values know
 * about neither (the model's {@code ArchitectureTest}); none of it reads a clock or holds state, so
 * the robot, the simulator's parallel runs, and a replay all read the same model (the robot
 * program's).
 */
@NullMarked
package com.michaelgrundvig.frc.world;

import org.jspecify.annotations.NullMarked;
