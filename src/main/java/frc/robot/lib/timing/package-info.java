/**
 * Where the robot loop's time goes. The loop never blocks (ADR 0007), and its time is budgeted, not
 * hoped for: every loop is timed section by section, and overruns are counted and named, so a new
 * source of delay shows up in the log before it shows up in a match.
 *
 * <p>Components here take timestamps as arguments, like the rest of the library; the robot code
 * reads the clock. Timings are for logs and alerts only: nothing that controls the robot may depend
 * on them, or a replay would no longer reproduce the match.
 */
package frc.robot.lib.timing;
