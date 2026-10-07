package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.measure.Distance;

/**
 * What moves a mechanism: an actuator's force, and where it goes. An arm's two motors through its
 * gearbox are one drive, and the gas spring holding it up another. The common case is a joint's own
 * shorthand: {@code arm.motors(Motor.KRAKEN_X60, 2, gearing)}.
 *
 * <p>Motors turn a shaft, so they go through a gearing, a drum, or a combination of joints; a
 * cylinder and a gas spring push along a line, so they go between two parts. A spring or a team's
 * law may go either way.
 *
 * @param label what logs call it: unique among the robot's drives
 * @param actuator what makes the force
 * @param transmission where it goes
 */
@ConfigRecord
public record Drive(String label, Actuator actuator, Transmission transmission) {
  public Drive {
    Checks.on(Drive.class, label)
        .field("label", label, notBlank())
        .check("transmission", () -> mismatch(actuator, transmission))
        .orThrow();
  }

  /** Starts describing a drive, one named value at a time. */
  public static DriveBuilder.LabelStage builder() {
    return DriveBuilder.builder();
  }

  /**
   * The encoder on its motors' rotor, which their controller reads whether or not anyone writes it
   * down: it turns as the motors' shaft does, by the transmission's ratios, so it reads the joints
   * it moves as they do, a differential's both. Never absolute: a rotor counts from wherever it
   * started. Labeled after the drive ({@code "arm rotor"}), and not among the robot's sensors,
   * since the drive already says all of it; derived values take it as they take any encoder ({@code
   * mechanisms.sensorToMechanismRatio(arm.rotor())}).
   *
   * @throws IllegalStateException if it isn't motors, which alone have a rotor
   */
  public Sensor.Encoder rotor() {
    if (!(actuator instanceof Actuator.Motors)) {
      throw new IllegalStateException(label + " has no rotor: it isn't motors");
    }
    List<Transmission.Term> terms =
        switch (transmission) {
          case Transmission.Geared geared ->
              List.of(term(geared.joint(), geared.gearing().ratio(), Optional.empty()));
          case Transmission.Drum drum ->
              List.of(term(drum.joint(), drum.gearing().ratio(), Optional.of(drum.pitchRadius())));
          case Transmission.Combined combined -> combined.terms();
          case Transmission.Between line ->
              throw new IllegalStateException(label + " pushes along a line: it has no rotor");
        };
    return Sensor.Encoder.builder().label(label + " rotor").terms(terms).absolute(false).build();
  }

  private static Transmission.Term term(Joint joint, double ratio, Optional<Distance> pitchRadius) {
    return Transmission.Term.builder().joint(joint).ratio(ratio).pitchRadius(pitchRadius).build();
  }

  /** Why an actuator can't go through a transmission, or null when it can. */
  private static @Nullable String mismatch(Actuator actuator, Transmission transmission) {
    boolean line = transmission instanceof Transmission.Between;
    return switch (actuator) {
      case Actuator.Motors motors when line ->
          "motors turn a shaft: give them a gearing, a drum, or a combination of joints, not a"
              + " line between parts";
      default -> null;
    };
  }
}
