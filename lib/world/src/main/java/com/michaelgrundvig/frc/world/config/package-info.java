/**
 * The validation kit: how a configuration record is built by name and checks its values. Every
 * configuration record uses it: the robot's definition ({@code
 * com.michaelgrundvig.frc.world.robot}), the components' own settings (vision's, the estimator's),
 * and each mechanism's gains and limits, which robot code assembles into its own configuration
 * ({@code frc.robot.RobotConfig}).
 *
 * <p>Every value that describes the robot (a dimension, a gain, a limit, a threshold) belongs in
 * that configuration, committed in {@code Robots}. To tune some of it on a running robot, the
 * mechanism that owns it declares them in a {@code LiveConfig} (Pitwall's), each with the range
 * that's safe to dial in: they become WPILib tunables, checked like committed values before a
 * change applies, returned to committed when the field connects, and reported as the builder calls
 * that commit them. A plain WPILib tunable is for a switch the drive team flips.
 *
 * <p>A value is checked in two places, each where what it knows lives:
 *
 * <ol>
 *   <li><b>Its record</b>, when built: what the value may be at all, alone or with the record's
 *       other values.
 *   <li><b>Its tuning limits</b>, declared with it in the mechanism's {@code LiveConfig}: how far
 *       the pit may take it while the robot runs. A limit may come from what the robot is: the
 *       drivetrain's top speed is tuned up to its free speed, which its definition says.
 * </ol>
 *
 * <p>Both check the committed configuration in the tests and at startup, and every change the pit
 * makes before it applies.
 *
 * <p>Conventions:
 *
 * <ul>
 *   <li>Physical quantities are WPILib units types ({@code Inches.of(27.5)}, {@code
 *       Pounds.of(135)}), so a value written in inches can never be read as meters. Code that runs
 *       every loop converts once, when it is built, and keeps plain SI values.
 *   <li>Every value is set by name, through the builder generated for each {@link
 *       com.michaelgrundvig.frc.world.config.ConfigRecord}: {@code
 *       Body.builder().frameLength(...)...build()}. The builder asks in turn for each value the
 *       record can't do without, so none can be left out or put in the wrong place; one a robot may
 *       not have (an {@code Optional}, a list its record lets be empty) is set by name after them,
 *       or left out. The architecture tests reject building one any other way.
 *   <li>Every record checks its values when it is built, with {@link
 *       com.michaelgrundvig.frc.world.config.Checks}: each value with its {@link
 *       com.michaelgrundvig.frc.world.config.Rule} ({@code positive()}, {@code
 *       between(Degrees.of(-90), Degrees.of(90))}, or any lambda), then checks across its values.
 *       Every problem is reported at once, each naming its value ({@code Gains.steerKp: -3 must be
 *       zero or more}). A bad value fails at startup, and first in the tests, which build every
 *       robot's configuration.
 *   <li>Physical measurements have no defaults: every robot states its own.
 *   <li>Derived values (the bumper outline, wheel positions, free speed) are computed, never
 *       written down twice.
 * </ul>
 */
@NullMarked
package com.michaelgrundvig.frc.world.config;

import org.jspecify.annotations.NullMarked;
