/**
 * The robot's definition: what a robot is. Its body (frame, bumpers, mass), its drivetrain as built
 * (modules, motors, gearing, where the wheels sit), where its cameras and IMUs are mounted, and its
 * mechanisms, in one {@link com.michaelgrundvig.frc.world.robot.RobotDefinition}.
 *
 * <p>A mechanism is joints ({@link com.michaelgrundvig.frc.world.robot.Joint}), each moving a link
 * on the chassis or on another joint's link; couplings, for joints that move as functions of others
 * ({@link com.michaelgrundvig.frc.world.robot.Coupling}); drives, each an actuator's force ({@link
 * com.michaelgrundvig.frc.world.robot.Actuator}: motors, a cylinder, a spring, a team's own law)
 * through a transmission ({@link com.michaelgrundvig.frc.world.robot.Transmission}: a gearing, a
 * drum, several joints at once, or a line between two parts); and sensors ({@link
 * com.michaelgrundvig.frc.world.robot.Sensor}). Every reference between them is the object itself,
 * never a name. What robot code needs from them, Phoenix's ratios, the ideal gains, gravity at each
 * joint, where each link is, is derived ({@link com.michaelgrundvig.frc.world.robot.RobotMath}).
 *
 * <p>It also names the robot's places and the interlocks between its joints: each joint's named
 * positions and outputs ({@link com.michaelgrundvig.frc.world.robot.Position}, {@link
 * com.michaelgrundvig.frc.world.robot.Output}), the configurations the robot takes ({@link
 * com.michaelgrundvig.frc.world.robot.Configuration}: several joints, each at a named place), and
 * joint-to-joint interlocks ({@link com.michaelgrundvig.frc.world.robot.Interlock}, on {@link
 * com.michaelgrundvig.frc.world.robot.Condition}s: a joint at a named place, at least or at most a
 * position, or between two). These are facts about the robot, where its intake is out and that its
 * roller mustn't run stowed, which every reader reads alike.
 *
 * <p>It's a description, read by everything that needs to know the robot: the mechanisms' code,
 * localization (where the wheels and sensors sit), the simulator (all of it), Pitwall, and the
 * views. It holds no state (where an arm is now comes from its sensors) and no behavior: what to do
 * with its places, when to go where and what to wait for, is the robot program's. Nothing here
 * reads anything back: not the field, not localization, not the robot program. Interlocks speak
 * only of the robot's own joints; what the field or the game allows is the robot program's too.
 *
 * <p>A robot's joints, their places and its configurations are constants in {@code Robots}, each
 * joint written in one block: {@code Joint.pivot(...).range(...).positions(...).motor(...)
 * .control(...)}. Behavior code names the constants themselves ({@code joints.of(PIVOT)}, {@code
 * pivot.goTo(PIVOT_OUT)}), so a part missing or misnamed fails to compile, and nothing is ever
 * found by its label. Each block says each thing once. A practice robot of the same design is the
 * same definition with its differences ({@code COMPETITION.with(PIVOT.motorsOn(...))}), each joint
 * changed one thing at a time and the rest kept: a joint re-wired or re-tuned is still the same
 * joint.
 *
 * <p>It's set where the rest of a robot's configuration is, in {@code Robots}, and checked the same
 * way: each record checks its own values with the validation kit ({@code
 * com.michaelgrundvig.frc.world.config}), and {@code RobotDefinition} checks across them (wheels
 * inside the frame, cameras and IMUs inside the robot, a drive's joints among the robot's, named
 * outputs within what its motors can do, each joint's control fitting it, one sensor on each input,
 * configurations obeying the interlocks), each saying what it finds ({@link
 * com.michaelgrundvig.frc.world.config.Finding}). How each joint is driven (gains, limits: {@link
 * com.michaelgrundvig.frc.world.robot.MechanismControl}) and wired (CAN IDs, encoder offsets:
 * {@link com.michaelgrundvig.frc.world.robot.MechanismWiring}, {@link
 * com.michaelgrundvig.frc.world.robot.SensorWiring}) is in its block too ({@link
 * com.michaelgrundvig.frc.world.robot.Driven}), and the drivetrain's in its geometry; only the
 * robot program's mechanisms and IO read them, and they're no part of what a joint is.
 */
@NullMarked
package com.michaelgrundvig.frc.world.robot;

import org.jspecify.annotations.NullMarked;
