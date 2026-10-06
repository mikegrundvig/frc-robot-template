package frc.robot.lib.rev;

import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.MechanismWiringBuilder;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.lib.phoenix.PhoenixMechanism;
import frc.robot.subsystems.mechanisms.JointFixtures;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * The REV helper: a SPARK's settings carried from the Phoenix helper's for the same mechanism, in a
 * SPARK's units. The fixtures' pivot on a NEO through 25:1.
 */
class RevMechanismTest {
  private static final Drive NEO =
      JointFixtures.motors(PIVOT, Motor.NEO, 2, JointFixtures.gearing(25));

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
          .drives(
              List.of(
                  NEO,
                  JointFixtures.ROLLER_MOTOR,
                  JointFixtures.ELEVATOR_MOTORS,
                  JointFixtures.TURRET_MOTOR))
          .build();

  private static final RobotMath MATH = ROBOT.math();

  private static final MechanismControl CONTROL =
      MechanismControlBuilder.builder(SETUPS.get(0).control())
          .softLimits(
              Optional.of(MechanismControl.SoftLimits.builder().lower(0.1).upper(2.2).build()))
          .build();

  private static MechanismWiring wiring(
      MechanismWiring.Controller controller, Optional<MechanismWiring.Cancoder> cancoder) {
    return MechanismWiring.builder()
        .canBus(CanBus.port(CanBus.Systemcore.CAN_S0))
        .controller(controller)
        .motors(
            List.of(
                MechanismWiring.Motor.builder().id(41).clockwise(true).build(),
                MechanismWiring.Motor.builder().id(42).clockwise(false).build()))
        .cancoder(cancoder)
        .dataPort(Optional.empty())
        .build();
  }

  private static final MechanismWiring SPARKS =
      wiring(MechanismWiring.Controller.SPARK_MAX, Optional.empty());

  /**
   * Its loop's gain is the Phoenix slot's kP per rotor turn as a share of 12 V, for positions, and
   * per rotor RPM for speeds; its soft limits are in rotor turns; and what the robot adds is the
   * slot's feedforward and kD, as Phoenix holds them.
   */
  @Test
  void itCarriesThePhoenixSlotInASparksUnits() {
    RevMechanism.Settings rev = RevMechanism.configure(MATH, PIVOT, NEO, SPARKS, CONTROL);
    SlotConfigs slot =
        SlotConfigs.from(
            PhoenixMechanism.configure(MATH, PIVOT, NEO, SPARKS, CONTROL).leader().Slot0);

    assertThat(rev.rotorRatio()).isCloseTo(25, within(1e-12));
    assertThat(rev.positionKp()).isCloseTo(slot.kP / (12 * 25), within(1e-12));
    assertThat(rev.velocityKp()).isCloseTo(slot.kP / (12 * 25 * 60), within(1e-15));
    assertThat(rev.softLimits().orElseThrow())
        .containsExactly(0.1 / (2 * Math.PI) * 25, 2.2 / (2 * Math.PI) * 25);
    RevMechanism.Feedforward feedforward = rev.feedforward();
    assertThat(feedforward.kV()).isEqualTo(slot.kV);
    assertThat(feedforward.kG()).isEqualTo(slot.kG);
    assertThat(feedforward.arm()).as("a pivot's gravity turns with it").isTrue();
    assertThat(feedforward.armOffset()).isEqualTo(slot.GravityArmPositionOffset);
    assertThat(feedforward.kD()).isEqualTo(slot.kD);
    assertThat(rev.followers()).extracting(RevMechanism.Follower::id).containsExactly(42);
  }

  /** At rest at the angle where gravity pulls hardest, it holds with kG; moving, kS and kV too. */
  @Test
  void itsFeedforwardIsTheSlotsInVolts() {
    RevMechanism.Feedforward feedforward = new RevMechanism.Feedforward(0.1, 2, 0, 0.5, true, 0, 1);

    assertThat(feedforward.atPosition(0, 0, 0)).isCloseTo(0.5, within(1e-12));
    assertThat(feedforward.atPosition(0.25, 0, 0)).as("straight up").isCloseTo(0, within(1e-12));
    assertThat(feedforward.atPosition(0, 1, 0.5)).isCloseTo(0.1 + 2 + 0.5 + 0.5, within(1e-12));
    assertThat(feedforward.atSpeed(0, -1)).isCloseTo(-0.1 - 2 + 0.5, within(1e-12));
  }

  /** An absolute encoder on the data port is carried in the settings, for reading at startup. */
  @Test
  void anEncoderOnTheDataPortIsReadAtStartup() {
    Sensor.Encoder absolute = Sensor.Encoder.absolute("pivot absolute", PIVOT, 8192);
    RobotMath math =
        RobotDefinitionBuilder.builder(ROBOT).sensors(List.of(absolute)).build().math();
    MechanismWiring withPort =
        MechanismWiringBuilder.builder(SPARKS)
            .dataPort(
                Optional.of(
                    MechanismWiring.DataPortEncoder.builder()
                        .offset(Rotations.of(0))
                        .reversed(false)
                        .build()))
            .build();

    assertThat(RevMechanism.configure(math, PIVOT, NEO, absolute, withPort, CONTROL).absolute())
        .contains(absolute);
    assertThat(RevMechanism.configure(math, PIVOT, NEO, SPARKS, CONTROL).absolute()).isEmpty();
    assertThatIllegalArgumentException()
        .isThrownBy(() -> RevMechanism.configure(math, PIVOT, NEO, withPort, CONTROL))
        .withMessageStartingWith("pivot's wiring names a data-port encoder");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> RevMechanism.configure(math, PIVOT, NEO, absolute, SPARKS, CONTROL))
        .withMessage("pivot's wiring names no data-port encoder for pivot absolute: wire it");
  }

  @Test
  void itRefusesWhatASparkCantTakeYet() {
    MechanismControl integral =
        MechanismControlBuilder.builder(CONTROL)
            .gains(MechanismControlGainsBuilder.builder(CONTROL.gains()).kI(1).build())
            .build();
    MechanismWiring.Cancoder cancoder =
        MechanismWiring.Cancoder.builder()
            .id(43)
            .offset(Rotations.of(0))
            .clockwise(false)
            .feedback(MechanismWiring.Feedback.ROTOR)
            .build();

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RevMechanism.configure(
                    MATH,
                    PIVOT,
                    NEO,
                    wiring(MechanismWiring.Controller.TALON_FXS, Optional.empty()),
                    CONTROL))
        .withMessage("pivot's wiring is a TalonFXS's, not a SPARK's");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RevMechanism.configure(
                    MATH,
                    PIVOT,
                    NEO,
                    wiring(MechanismWiring.Controller.SPARK_MAX, Optional.of(cancoder)),
                    CONTROL))
        .withMessageStartingWith("pivot's wiring names a CANcoder, which a SPARK doesn't read");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> RevMechanism.configure(MATH, PIVOT, NEO, SPARKS, integral))
        .withMessageStartingWith("pivot's control has a kI");
  }
}
