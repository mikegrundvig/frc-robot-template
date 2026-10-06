/**
 * The REV helper: a mechanism's SPARK settings, from the same three inputs as the Phoenix helper's
 * (the robot's definition, the mechanism's wiring, its control), carried from what the Phoenix
 * helper works out, so a mechanism on REV hardware is described once, as on CTRE's ({@link
 * frc.robot.lib.rev.RevMechanism}). Pure functions: it builds configurations and never touches a
 * device; the IO layer applies them ({@code ArchitectureTest}).
 */
package frc.robot.lib.rev;
