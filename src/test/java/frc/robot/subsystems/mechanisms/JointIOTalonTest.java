package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT_MOTOR;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
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
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import frc.robot.RobotSim;
import frc.robot.lib.phoenix.PhoenixMechanism;
import frc.robot.lib.wiring.ControllerPins;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * The CTRE joint against Phoenix's own simulated devices: what the robot reads from them, in the
 * joint's units. The settings it applies are the Phoenix helper's, tested there.
 */
@ExtendWith(RobotSim.class)
class JointIOTalonTest {
  private static final String BUS = "joints";

  @BeforeAll
  static void quietCtreLog() {
    SignalLogger.enableAutoLogging(false);
  }

  private static JointSetup wired(
      JointSetup setup,
      Optional<MechanismWiring.Cancoder> cancoder,
      MechanismWiring.Motor... motors) {
    return JointSetupBuilder.builder(setup)
        .wiring(
            Optional.of(
                MechanismWiring.builder()
                    .canBus(CanBus.canivore(BUS))
                    .controller(MechanismWiring.Controller.TALON_FX)
                    .motors(List.of(motors))
                    .cancoder(cancoder)
                    .dataPort(Optional.empty())
                    .build()))
        .build();
  }

  private static MechanismWiring.Motor motor(int id) {
    return MechanismWiring.Motor.builder().id(id).clockwise(false).build();
  }

  /** Waits up to two seconds, reading again every few milliseconds, for something to be true. */
  private static boolean eventually(Runnable read, BooleanSupplier done)
      throws InterruptedException {
    for (int i = 0; i < 200; i++) {
      read.run();
      if (done.getAsBoolean()) {
        return true;
      }
      Thread.sleep(10);
    }
    return false;
  }

  /**
   * Its leader's limit pins read tripped by their switches' polarity: a normally open switch as it
   * closes to ground, a normally closed one as it opens. Phoenix reports the pin as it is, closed
   * or open, whatever polarity it's told for its own stop, so the polarity is applied as it's read.
   */
  @Test
  void itsLimitPinsReadTrippedByTheirPolarity() throws InterruptedException {
    ControllerPins pins =
        ControllerPins.NONE
            .with(
                SensorWiring.Direction.FORWARD,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, false))
            .with(
                SensorWiring.Direction.REVERSE,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_CLOSED, true));
    JointIOTalon io =
        new JointIOTalon(ROBOT.math(), wired(SETUPS.get(0), Optional.empty(), motor(55)), pins);
    TalonFX device = new TalonFX(55, new CANBus(BUS));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    device.getSimState().setForwardLimit(true);
    device.getSimState().setReverseLimit(true);
    boolean closed =
        eventually(
            () -> io.updateInputs(inputs), () -> inputs.forwardLimit && !inputs.reverseLimit);
    device.getSimState().setForwardLimit(false);
    device.getSimState().setReverseLimit(false);
    boolean open =
        eventually(
            () -> io.updateInputs(inputs), () -> !inputs.forwardLimit && inputs.reverseLimit);

    assertThat(closed).as("both closed: forward, normally open, tripped; reverse clear").isTrue();
    assertThat(open).as("both open: forward clear; reverse, normally closed, tripped").isTrue();
  }

  /**
   * A pivot read by its rotor alone starts where its definition rests it, and reads its rotor's
   * turns through its 25:1: twenty-five more are one more turn of the pivot. Within 0.01 rad, since
   * Phoenix's simulated position reads a turn 0.0016 rad short; a wrong ratio or a missing
   * reference (1.5°, 0.026 rad) is far outside it.
   */
  @Test
  void itReadsItsRotorThroughItsGearingFromWhereItRests() throws InterruptedException {
    JointIOTalon io =
        new JointIOTalon(ROBOT.math(), wired(SETUPS.get(0), Optional.empty(), motor(51)));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    boolean resting =
        eventually(
            () -> io.updateInputs(inputs),
            () -> inputs.connected && Math.abs(inputs.position - PIVOT.reference()) < 0.01);
    // Its raw rotor started at 0, where the startup told it the pivot rests.
    new TalonFX(51, new CANBus(BUS)).getSimState().setRawRotorPosition(25);
    boolean turned =
        eventually(
            () -> io.updateInputs(inputs),
            () -> Math.abs(inputs.position - (PIVOT.reference() + 2 * Math.PI)) < 0.01);

    assertThat(resting).as("it starts at its reference, 1.5°").isTrue();
    assertThat(turned).as("25 rotor turns are one of the pivot").isTrue();
    assertThat(inputs.unconfigured).as("every device took its configuration").isEmpty();
  }

  /** The elevator's two motors: the first leads, and it reads meters through its drum. */
  @Test
  void aSlideReadsMetersThroughItsDrum() throws InterruptedException {
    JointIOTalon io =
        new JointIOTalon(
            ROBOT.math(), wired(SETUPS.get(2), Optional.empty(), motor(53), motor(54)));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    eventually(() -> io.updateInputs(inputs), () -> inputs.connected);

    // Ten rotor turns through 10:1 are one turn of the 1 in drum: 2π inches of travel.
    new TalonFX(53, new CANBus(BUS)).getSimState().setRawRotorPosition(10);
    double travel = 2 * Math.PI * 0.0254;
    boolean moved =
        eventually(
            () -> io.updateInputs(inputs),
            () -> Math.abs(inputs.position - (ELEVATOR.reference() + travel)) < 1e-4);

    assertThat(moved).isTrue();
    assertThat(inputs.position).isCloseTo(travel, within(1e-4));
  }

  /** A tuned kP reaches the motor controller's slot, in Phoenix's units, without a restart. */
  @Test
  void aTunedGainReachesItsSlot() throws InterruptedException {
    JointSetup setup = wired(SETUPS.get(0), Optional.empty(), motor(52));
    JointIOTalon io = new JointIOTalon(ROBOT.math(), setup);
    MechanismControl tuned =
        MechanismControlBuilder.builder(setup.control())
            .gains(MechanismControlGainsBuilder.builder(setup.control().gains()).kP(55).build())
            .build();
    double expected =
        PhoenixMechanism.configure(
                ROBOT.math(),
                PIVOT,
                ROBOT.math().drivesOf(PIVOT).get(0),
                setup.wiring().orElseThrow(),
                tuned)
            .leader()
            .Slot0
            .kP;

    io.retune(tuned);
    Slot0Configs slot = new Slot0Configs();
    TalonFX motor = new TalonFX(52, new CANBus(BUS));
    boolean took =
        eventually(
            () -> motor.getConfigurator().refresh(slot),
            // Phoenix keeps a gain as a float.
            () -> Math.abs(slot.kP - expected) < 1e-6 * expected);

    assertThat(took).as("kP %s in Phoenix's units, read %s", expected, slot.kP).isTrue();
  }

  @Test
  void onlyAnFocMotorRunsWithFoc() {
    Drive foc = JointFixtures.motors(PIVOT, Motor.KRAKEN_X60_FOC, 1, JointFixtures.gearing(25));

    assertThat(JointIOTalon.foc(PIVOT_MOTOR)).isFalse();
    assertThat(JointIOTalon.foc(foc)).isTrue();
  }

  @Test
  void aSetupWithoutWiringIsRefused() {
    assertThatThrownBy(() -> new JointIOTalon(ROBOT.math(), SETUPS.get(0)))
        .hasMessage("pivot has no wiring: wire it");
  }

  @Test
  void aCancoderTheDefinitionDoesntHaveIsRefused() {
    MechanismWiring.Cancoder cancoder =
        MechanismWiring.Cancoder.builder()
            .id(52)
            .offset(Rotations.of(0))
            .clockwise(false)
            .feedback(MechanismWiring.Feedback.FUSED)
            .build();

    assertThatThrownBy(
            () ->
                new JointIOTalon(
                    ROBOT.math(), wired(SETUPS.get(0), Optional.of(cancoder), motor(55))))
        .hasMessage(
            "pivot's wiring names a CANcoder, and its definition has no absolute encoder on it");
  }

  /**
   * A pivot whose TalonFX reads its rotor, its CANcoder read by the robot once: powered on at 60°,
   * not where it rests, it's told so by the CANcoder's first good reading, and holds it. Until
   * 2026-10-05 it took its reference, 1.5°, whatever the CANcoder said.
   */
  @Test
  void aPivotReadByItsRotorIsToldWhereItIsByItsCancoder() throws InterruptedException {
    RobotDefinition withEncoder =
        RobotDefinitionBuilder.builder(ROBOT)
            .sensors(List.of(Sensor.Encoder.absolute("pivot absolute", PIVOT, 4096)))
            .build();
    MechanismWiring.Cancoder cancoder =
        MechanismWiring.Cancoder.builder()
            .id(57)
            .offset(Rotations.of(0))
            .clockwise(false)
            .feedback(MechanismWiring.Feedback.ROTOR)
            .build();
    new CANcoder(57, new CANBus(BUS)).getSimState().setRawPosition(60.0 / 360);

    JointIOTalon io =
        new JointIOTalon(
            withEncoder.math(), wired(SETUPS.get(0), Optional.of(cancoder), motor(56)));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    boolean told =
        eventually(
            () -> io.updateInputs(inputs),
            () -> Math.abs(inputs.position - Math.toRadians(60)) < 0.01);
    boolean held =
        eventually(
            () -> io.updateInputs(inputs),
            () -> inputs.connected && Math.abs(inputs.position - Math.toRadians(60)) < 0.01);

    assertThat(told).as("its CANcoder's 60°, not its reference").isTrue();
    assertThat(held).as("its rotor reads it from then on").isTrue();
  }

  /**
   * A NEO pivot on a TalonFXS, through 25:1: it takes the same settings a TalonFX would, and reads
   * its rotor, the NEO's hall sensor, through its gearing from where it rests.
   */
  @Test
  void aNeoOnATalonFxsReadsItsRotorThroughItsGearing() throws InterruptedException {
    RobotDefinition onNeo =
        RobotDefinitionBuilder.builder(ROBOT)
            .drives(
                List.of(
                    JointFixtures.motors(PIVOT, Motor.NEO, 1, JointFixtures.gearing(25)),
                    JointFixtures.ROLLER_MOTOR,
                    JointFixtures.ELEVATOR_MOTORS,
                    JointFixtures.TURRET_MOTOR))
            .build();
    JointSetup setup = wired(SETUPS.get(0), Optional.empty(), motor(58));
    JointSetup onFxs =
        JointSetupBuilder.builder(setup)
            .wiring(
                Optional.of(
                    MechanismWiringBuilder.builder(setup.wiring().orElseThrow())
                        .controller(MechanismWiring.Controller.TALON_FXS)
                        .build()))
            .build();
    JointIOTalon io = new JointIOTalon(onNeo.math(), onFxs);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    boolean resting =
        eventually(
            () -> io.updateInputs(inputs),
            () -> inputs.connected && Math.abs(inputs.position - PIVOT.reference()) < 0.01);
    new TalonFXS(58, new CANBus(BUS)).getSimState().setRawRotorPosition(25);
    boolean turned =
        eventually(
            () -> io.updateInputs(inputs),
            () -> Math.abs(inputs.position - (PIVOT.reference() + 2 * Math.PI)) < 0.01);

    assertThat(resting).isTrue();
    assertThat(turned).as("25 rotor turns are one of the pivot").isTrue();
    assertThat(inputs.unconfigured).as("the TalonFXS took its configuration").isEmpty();
  }
}
