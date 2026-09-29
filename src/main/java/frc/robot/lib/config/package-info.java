/**
 * The vocabulary for describing a robot: its body, drivetrain, cameras, and motors. Robot code
 * assembles these into its own configuration ({@code frc.robot.RobotConfig}); components read them.
 *
 * <p>Conventions:
 *
 * <ul>
 *   <li>Physical quantities are WPILib units types ({@code Inches.of(27.5)}, {@code
 *       Pounds.of(135)}), so a value written in inches can never be read as meters. Code that runs
 *       every loop converts once, when it is built, and keeps plain SI values.
 *   <li>Every value is set by name, through the builder generated for each {@link
 *       frc.robot.lib.config.ConfigRecord}: {@code Body.builder().frameLength(...)...build()}. The
 *       builder asks for each value in turn, so none can be left out or put in the wrong place, and
 *       the architecture tests reject building one any other way.
 *   <li>Every record checks its values when it is built. A bad value fails at startup, and first in
 *       the tests, which build every robot's configuration.
 *   <li>Physical measurements have no defaults: every robot states its own.
 *   <li>Derived values (the bumper outline, wheel positions, free speed) are computed, never
 *       written down twice.
 * </ul>
 */
package frc.robot.lib.config;
