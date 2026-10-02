/**
 * Where the robot's devices are wired: which CAN bus, which IDs, which way each motor turns, and
 * how each encoder reads. Only the robot's IO reads it, to reach real hardware, so it names the
 * hardware's own values (a Systemcore CAN port, how Systemcore is mounted) that the robot's
 * description never needs.
 *
 * <p>It's kept apart from the robot's definition (what the robot is: its frame, its wheels, where
 * its sensors sit), which localization and the simulator read, and which names no hardware. A robot
 * that only runs in the simulator has a definition and no wiring.
 */
package frc.robot.lib.wiring;
