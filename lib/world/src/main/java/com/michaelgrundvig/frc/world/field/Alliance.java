package com.michaelgrundvig.frc.world.field;

/**
 * An alliance: which end of the field a place is for. Places are written once, for the blue
 * alliance, and {@link FieldModel#forAlliance} gives red's.
 *
 * <p>The model's own, rather than the Driver Station's enum in WPILib's main library, so that
 * nothing reading the model is handed that library: its clock, hardware, and NetworkTables stay out
 * of reach. The robot program turns the Driver Station's alliance into this one where it reads it.
 */
public enum Alliance {
  BLUE,
  RED
}
