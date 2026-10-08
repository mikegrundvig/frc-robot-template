/**
 * The bridges out: writers that give other tools what the robot's configuration says, so a value is
 * set once, in {@code Robots.java}, and every tool that shows it agrees. Each writer is lossy: its
 * Javadoc lists what it drops, and a test pins the list.
 *
 * <ul>
 *   <li>{@link com.michaelgrundvig.frc.world.bridges.AdvantageScopeAssets}: a schematic robot for
 *       AdvantageScope's 3D field, and its config.
 * </ul>
 *
 * <p>The build runs them, never the robot: each tool's Gradle plugin ({@code build-logic/}) calls a
 * writer's {@code main} on the compiled program, which finds the robots through the {@link
 * com.michaelgrundvig.frc.world.bridges.RobotCatalog} the program names, builds their configuration
 * records, and writes files. Nothing here starts the robot program, reads a sensor, or reads
 * another tool's files.
 *
 * <p>The world model's rules hold here too: no clock, unseeded draw, thread, or static state (the
 * robot program's {@code ArchitectureTest}), and no robot code or hardware, which Gradle keeps out.
 * The writers read the model; the field and the robot's definition never see them (the model's
 * {@code ArchitectureTest}).
 */
@NullMarked
package com.michaelgrundvig.frc.world.bridges;

import org.jspecify.annotations.NullMarked;
