package frc.robot.lib.phoenix;

import static org.assertj.core.api.Assertions.assertThat;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import frc.robot.lib.wiring.ControllerPins;
import org.junit.jupiter.api.Test;

/**
 * A Talon's limits from what's on them: it stops only on a limit switch, on its own pin with its
 * polarity or on a CANdi it reads over CAN, and never resets its position as one trips.
 */
class PhoenixLimitsTest {
  @Test
  void aLimitSwitchOnAPinStopsItAndABeamDoesnt() {
    HardwareLimitSwitchConfigs limits = new HardwareLimitSwitchConfigs();

    PhoenixMechanism.limits(
        limits,
        ControllerPins.NONE
            .with(
                SensorWiring.Direction.FORWARD,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, false))
            .with(
                SensorWiring.Direction.REVERSE,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_CLOSED, true)));

    assertThat(limits.ForwardLimitEnable).as("a beam: read, not stopped on").isFalse();
    assertThat(limits.ReverseLimitEnable).isTrue();
    assertThat(limits.ReverseLimitSource).isEqualTo(ReverseLimitSourceValue.LimitSwitchPin);
    assertThat(limits.ForwardLimitType).isEqualTo(ForwardLimitTypeValue.NormallyOpen);
    assertThat(limits.ReverseLimitType).isEqualTo(ReverseLimitTypeValue.NormallyClosed);
    assertThat(limits.ReverseLimitAutosetPositionEnable).isFalse();
  }

  @Test
  void aCandisLimitSwitchIsReadOverCanAndNothingStopsNothing() {
    HardwareLimitSwitchConfigs limits = new HardwareLimitSwitchConfigs();

    PhoenixMechanism.limits(
        limits,
        ControllerPins.NONE.with(
            SensorWiring.Direction.FORWARD, new ControllerPins.OnCandi(41, 2)));

    assertThat(limits.ForwardLimitEnable).isTrue();
    assertThat(limits.ForwardLimitSource).isEqualTo(ForwardLimitSourceValue.RemoteCANdiS2);
    assertThat(limits.ForwardLimitRemoteSensorID).isEqualTo(41);
    assertThat(limits.ForwardLimitType)
        .as("the CANdi's closed already means tripped")
        .isEqualTo(ForwardLimitTypeValue.NormallyOpen);
    assertThat(limits.ReverseLimitEnable).as("nothing on it").isFalse();
  }
}
