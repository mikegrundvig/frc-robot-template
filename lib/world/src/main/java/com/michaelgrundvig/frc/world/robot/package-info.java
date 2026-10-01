/**
 * The robot's definition: what a robot is. Its body (frame, bumpers, mass), its drivetrain as built
 * (modules, motors, gearing, where the wheels sit), and where its cameras and IMUs are mounted, in
 * one {@link com.michaelgrundvig.frc.world.robot.RobotDefinition}. Mechanisms' joints join it with
 * the first mechanism that describes one.
 *
 * <p>It's a description, read by everything that needs to know the robot: localization (where the
 * wheels and sensors sit), the simulator (all of it), and the views. It holds no state (where an
 * arm is now comes from its sensors) and no behavior (what an arm does is its subsystem's), and
 * nothing here reads anything back: not the field, not localization, not the robot program.
 *
 * <p>It's set where the rest of a robot's configuration is, in {@code Robots}, and checked the same
 * way: each record checks its own values with the validation kit ({@code
 * com.michaelgrundvig.frc.world.config}), and {@code RobotDefinition} checks across them (wheels
 * inside the frame, cameras and IMUs inside the robot). How the robot is driven (gains, limits) and
 * how it's wired (CAN IDs) are the robot program's own configuration around it, since only its
 * mechanisms and IO read them.
 *
 * <p>What drives a mechanism is an open set ({@link com.michaelgrundvig.frc.world.robot.Actuator}):
 * any {@link com.michaelgrundvig.frc.world.robot.Motor}, from WPILib's named motors or a datasheet,
 * through any {@link com.michaelgrundvig.frc.world.robot.Gearing}.
 */
package com.michaelgrundvig.frc.world.robot;
