package frc.robot.lib.phoenix;

import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT_MOTOR;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.michaelgrundvig.frc.world.robot.Motor;
import frc.robot.lib.wiring.CanBus;
import frc.robot.lib.wiring.MechanismWiring;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;

/** A TalonFXS takes the settings worked out for a TalonFX, in its own terms. */
class TalonFxsTest {
  private static TalonFXConfiguration pivot() {
    MechanismWiring wiring =
        MechanismWiring.builder()
            .canBus(CanBus.port(CANPort.CAN_S0))
            .controller(MechanismWiring.Controller.TALON_FXS)
            .motors(List.of(MechanismWiring.Motor.builder().id(21).clockwise(true).build()))
            .cancoder(Optional.empty())
            .dataPort(Optional.empty())
            .build();
    return PhoenixMechanism.configure(
            ROBOT.math(), PIVOT, PIVOT_MOTOR, wiring, SETUPS.get(0).control())
        .leader();
  }

  /**
   * The same slot, limits, direction and ratio; its rotor is its motor's hall sensor, and it's told
   * which motor is plugged in.
   */
  @Test
  void itTakesATalonFxsSettingsWithItsMotorsArrangement() {
    TalonFXConfiguration fx = pivot();

    TalonFXSConfiguration fxs = PhoenixMechanism.talonFxs(fx, Motor.NEO);

    assertThat(fxs.Commutation.MotorArrangement).isEqualTo(MotorArrangementValue.NEO_JST);
    assertThat(fxs.ExternalFeedback.ExternalFeedbackSensorSource)
        .isEqualTo(ExternalFeedbackSensorSourceValue.Commutation);
    assertThat(fxs.ExternalFeedback.SensorToMechanismRatio)
        .isEqualTo(fx.Feedback.SensorToMechanismRatio);
    assertThat(fxs.Slot0.kP).isEqualTo(fx.Slot0.kP);
    assertThat(fxs.Slot0.kG).isEqualTo(fx.Slot0.kG);
    assertThat(fxs.MotorOutput.Inverted).isEqualTo(fx.MotorOutput.Inverted);
    assertThat(fxs.CurrentLimits.StatorCurrentLimit).isEqualTo(fx.CurrentLimits.StatorCurrentLimit);
    assertThat(PhoenixMechanism.talonFxs(fx, Motor.MINION).Commutation.MotorArrangement)
        .isEqualTo(MotorArrangementValue.Minion_JST);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> PhoenixMechanism.talonFxs(fx, Motor.KRAKEN_X60))
        .withMessage("a TalonFXS doesn't drive a Kraken X60");
  }
}
