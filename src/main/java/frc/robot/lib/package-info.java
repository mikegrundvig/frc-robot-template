/**
 * The component library: the typed configuration, the field model, localization, calibration math,
 * loop timing, the robot's wiring, and other reusable parts.
 *
 * <p>Rules, enforced by the build ({@code ArchitectureTest}):
 *
 * <ul>
 *   <li>Components never depend on robot code, so any robot can use them and they can be tested
 *       without one. Beyond the library itself and the JDK they use WPILib's math, units, and field
 *       layouts, and a few plain values (the alliance; for the wiring alone, a CAN port and how
 *       Systemcore is mounted).
 *   <li>The world's description keeps to itself: the validation kit ({@code config}) needs nothing
 *       else; the field and the robot's definition need only the kit, and not each other;
 *       localization and the calibration math read those three. None of them names hardware.
 *   <li>No hardware access, robot state, clock reads, or static state inside a component: inputs
 *       come in through parameters, which is what makes them testable and replayable.
 *   <li>No cycles between component packages.
 * </ul>
 *
 * <p>Every component ships with its unit tests.
 */
package frc.robot.lib;
