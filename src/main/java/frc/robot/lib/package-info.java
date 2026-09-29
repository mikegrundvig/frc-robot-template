/**
 * The component library: planners, estimators, geometry, the field model, and other reusable parts.
 *
 * <p>Rules, enforced by the build ({@code ArchitectureTest} and the coverage floor):
 *
 * <ul>
 *   <li>Components depend only on the library itself, the JDK, and WPILib: never on robot code, so
 *       any robot can use them and they can be tested without one.
 *   <li>No hardware access, clock reads, or static state inside a component: inputs come in through
 *       parameters, which is what makes them testable and replayable.
 *   <li>Every component ships with a full unit-test suite (95% line and 90% branch coverage floor).
 * </ul>
 */
package frc.robot.lib;
