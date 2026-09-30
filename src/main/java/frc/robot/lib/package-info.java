/**
 * The component library: the typed configuration, the field model, localization, calibration math,
 * loop timing, and other reusable parts.
 *
 * <p>Rules, enforced by the build ({@code ArchitectureTest}):
 *
 * <ul>
 *   <li>Components never depend on robot code, so any robot can use them and they can be tested
 *       without one. Beyond the library itself and the JDK they use WPILib's math, units, and field
 *       layouts, and a few plain values (the alliance, a CAN port, an IMU's mount).
 *   <li>No hardware access, robot state, clock reads, or static state inside a component: inputs
 *       come in through parameters, which is what makes them testable and replayable.
 *   <li>No cycles between component packages.
 * </ul>
 *
 * <p>Every component ships with its unit tests.
 */
package frc.robot.lib;
