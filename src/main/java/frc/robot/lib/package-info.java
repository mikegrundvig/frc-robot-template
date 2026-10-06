/**
 * The robot program's own components: its loop timing ({@code timing}), its wiring ({@code
 * wiring}), which only the robot's loop and IO read, and the Phoenix helper ({@code phoenix}),
 * which turns the robot's definition, wiring and control into Phoenix's settings for the IO to
 * apply. The libraries every part of the robot reads are projects of their own, in {@code lib/}:
 * the world model ({@code com.michaelgrundvig.frc.world}: the validation kit, the field, and the
 * robot's definition) and localization ({@code com.michaelgrundvig.frc.localization}). Gradle keeps
 * them from reaching the robot program.
 *
 * <p>Rules, enforced by the build ({@code ArchitectureTest}):
 *
 * <ul>
 *   <li>Components never depend on robot code, so they can be tested without a robot. Beyond the
 *       components themselves and the JDK they use WPILib's math and units, the validation kit,
 *       and, for the wiring alone, a few plain values of WPILib's (a CAN port, how Systemcore is
 *       mounted).
 *   <li>No hardware access, robot state, clock reads, or static state inside a component: inputs
 *       come in through parameters, which is what makes them testable and replayable. The Phoenix
 *       helper alone reads the robot's model and Phoenix's configuration classes, and never a
 *       device.
 *   <li>No cycles between component packages.
 * </ul>
 *
 * <p>Every component ships with its unit tests.
 */
package frc.robot.lib;
