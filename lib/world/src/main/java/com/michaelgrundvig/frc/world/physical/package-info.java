/**
 * What the field and the robot are both made of: how much a part weighs and how hard it is to turn
 * ({@link com.michaelgrundvig.frc.world.physical.MassProperties}, {@link
 * com.michaelgrundvig.frc.world.physical.Inertia}), its {@link
 * com.michaelgrundvig.frc.world.physical.Shape} and {@link
 * com.michaelgrundvig.frc.world.physical.Material}, and the plain geometry they're written in where
 * WPILib's math has none: a point or a direction in 3D ({@link
 * com.michaelgrundvig.frc.world.physical.Vector3}), a polygon ({@link
 * com.michaelgrundvig.frc.world.physical.Polygon2d}), and triangles in 3D ({@link
 * com.michaelgrundvig.frc.world.physical.TriangleMesh}).
 *
 * <p>The field and the robot's definition can't see each other, and both build on this, so it knows
 * nothing of either: only the validation kit, which its configuration records check their values
 * with ({@code ArchitectureTest}). Everything here is immutable and computed the same on every
 * computer.
 */
@NullMarked
package com.michaelgrundvig.frc.world.physical;

import org.jspecify.annotations.NullMarked;
