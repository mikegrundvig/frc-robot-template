package com.michaelgrundvig.frc.world.robot;

/**
 * A part of the robot that things are fixed to: the chassis, or the link a {@link Joint} moves. A
 * joint hangs from one, and a spring's or a cylinder's end, a beam break, or a camera is fixed to
 * one. Each is referred to as itself, never by name: a reference to a part that doesn't exist can't
 * be written, and a joint built after its parent can't be its own ancestor.
 */
public sealed interface Part permits Part.Chassis, Joint {
  /** The robot's chassis, its frame: the robot's own frame, x forward, y left, z up. */
  Part CHASSIS = Chassis.CHASSIS;

  /** What logs, writers and messages call it. */
  String label();

  /** The chassis: whatever doesn't move on a joint. */
  enum Chassis implements Part {
    CHASSIS;

    @Override
    public String label() {
      return "chassis";
    }
  }
}
