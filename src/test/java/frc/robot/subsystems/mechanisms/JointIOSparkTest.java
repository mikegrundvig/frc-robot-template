package frc.robot.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_FLEX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROLLER;
import static frc.robot.subsystems.mechanisms.JointFixtures.gearing;
import static frc.robot.subsystems.mechanisms.JointFixtures.unmotored;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfigAccessor;
import frc.robot.RobotSim;
import frc.robot.lib.rev.RevMechanism;
import frc.robot.lib.wiring.ControllerPins;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.system.DCMotor;

/**
 * The REV joint against REV's own simulated SPARK: what it tells the SPARK, in the SPARK's units,
 * and what it reads back, in the joint's. The settings are the REV helper's, from the program's
 * plan, tested there.
 */
@ExtendWith(RobotSim.class)
class JointIOSparkTest {
  /** The pivot on a NEO through 25:1, its SPARK MAX at an ID. */
  private static Joint.Revolute pivot(int id) {
    return unmotored(PIVOT).motor(Motor.NEO, gearing(25), SPARK_MAX.can(id));
  }

  /** A joint's IO, on the fixtures' robot with the joint as given. */
  private static JointIOSpark io(Joint joint, ControllerPins pins) {
    return new JointIOSpark(ROBOT.with(joint).math(), joint, pins);
  }

  private static JointIOSpark io(Joint joint) {
    return io(joint, ControllerPins.NONE);
  }

  /**
   * A pivot read by its rotor starts where its definition rests it, and reads its rotor's turns
   * through its 25:1: twenty-five more are one more turn of the pivot.
   */
  @Test
  void itReadsItsRotorThroughItsGearingFromWhereItRests() {
    JointIOSpark io = io(pivot(31));
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNEO(1));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    io.updateInputs(inputs);
    double resting = inputs.position;
    // The simulation's own position starts at 0, not where the startup set the encoder.
    sim.setPosition(PIVOT.reference() / (2 * Math.PI) * 25 + 25);
    sim.iterate(0, 12, 0.02);
    io.updateInputs(inputs);

    assertThat(resting)
        .as("it starts at its reference, 1.5°")
        .isCloseTo(PIVOT.reference(), within(1e-9));
    // A SPARK reports its position as a float.
    assertThat(inputs.position).isCloseTo(PIVOT.reference() + 2 * Math.PI, within(1e-6));
    assertThat(inputs.unconfigured).as("every SPARK took its settings").isEmpty();
  }

  /**
   * The SPARK is told each limit input's polarity, and stops on a limit switch only: a beam read
   * through its forward input lets the motor run on. REV applies the polarity on the device (its
   * docs: {@code isPressed} is "based on the selected polarity"), and its simulator's "pressed" is
   * after it, so what the SPARK took is read back from it, and a press reads tripped.
   *
   * <p>REVLib 2027.0.0-alpha-8 reads a limit input's type back the other way round: set normally
   * open, it reads normally closed, and a fresh SPARK reads normally closed where REV's docs say
   * its default is normally open, which points at the reading, not the setting. Pinned here, so a
   * REVLib that changes it fails this test; which side is right waits on a SPARK on the bench (open
   * question M22).
   */
  @Test
  void itsLimitInputsTakeTheirPolarityAndStopOnlyOnALimitSwitch() {
    ControllerPins pins =
        ControllerPins.NONE
            .with(
                SensorWiring.Direction.FORWARD,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, false))
            .with(
                SensorWiring.Direction.REVERSE,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_CLOSED, true));
    JointIOSpark io = io(pivot(47), pins);
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNEO(1));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    LimitSwitchConfigAccessor took = ((SparkMax) io.leader()).configAccessor.limitSwitch;

    sim.getForwardLimitSwitchSim().setPressed(true);
    io.updateInputs(inputs);

    assertThat(took.getForwardSwitchType())
        .as("set normally open; alpha-8 reads it back the other way")
        .isEqualTo(LimitSwitchConfig.Type.kNormallyClosed);
    assertThat(took.getReverseSwitchType())
        .as("set normally closed; alpha-8 reads it back the other way")
        .isEqualTo(LimitSwitchConfig.Type.kNormallyOpen);
    assertThat(took.getForwardLimitSwitchTriggerBehavior())
        .as("a beam: read, never stopped on")
        .isEqualTo(LimitSwitchConfig.Behavior.kKeepMovingMotor);
    assertThat(took.getReverseLimitSwitchTriggerBehavior())
        .as("a limit switch: the SPARK stops on it")
        .isEqualTo(LimitSwitchConfig.Behavior.kStopMovingMotor);
    assertThat(inputs.forwardLimit).isTrue();
    assertThat(inputs.reverseLimit).isFalse();
  }

  /**
   * A position goes to the SPARK's loop in rotor turns, with the feedforward the robot works out: a
   * pivot 45° up is 25 × 1/8 rotor turns. A little past where it rests, its loop's output is its kP
   * on the error plus the feedforward's share of 12 V (far off, the current limit caps it).
   */
  @Test
  void aPositionGoesInRotorTurnsWithItsFeedforwardInVolts() {
    Joint.Revolute pivot = pivot(32);
    RobotMath math = ROBOT.with(pivot).math();
    JointIOSpark io = io(pivot);
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNEO(1));
    sim.enable();
    RevMechanism.Settings settings =
        RevMechanism.configure(
            ControllerPlan.of(
                math,
                PIVOT,
                math.drivesOf(PIVOT).get(0),
                pivot.driven().motors().orElseThrow().controllers(),
                ControllerSettings.control(pivot)));

    // REV's simulation steps its own position into the encoder: start it where the pivot rests.
    sim.setPosition(PIVOT.reference() / (2 * Math.PI) * 25);
    io.setPosition(Math.toRadians(45), 0, 0.6);
    sim.iterate(0, 12, 0.02);
    double far = sim.getSetpoint();
    double near = PIVOT.reference() + Math.toRadians(0.5);
    io.setPosition(near, 0, 0.6);
    for (int i = 0; i < 25; i++) {
      sim.iterate(0, 12, 0.02);
    }

    double rest = PIVOT.reference() / (2 * Math.PI) * 25;
    double target = near / (2 * Math.PI) * 25;
    double feedforward = 0.6 + settings.plan().feedforward(near, 0, 0);
    // A SPARK holds its setpoint as a float.
    assertThat(far).isCloseTo(25.0 / 8, within(1e-6));
    assertThat(sim.getSetpoint()).isCloseTo(target, within(1e-6));
    assertThat(sim.getAppliedOutput())
        .isCloseTo(settings.positionKp() * (target - rest) + feedforward / 12, within(1e-3));
  }

  /** The pit's tuning reaches the SPARK's loop, as a TalonFX's slot: its kP, in rotor turns. */
  @Test
  void aTunedGainReachesItsLoop() {
    Joint.Revolute pivot = pivot(33);
    JointIOSpark io = io(pivot);
    MechanismControl control = ControllerSettings.control(pivot);
    MechanismControl tuned =
        MechanismControlBuilder.builder(control)
            .gains(MechanismControlGainsBuilder.builder(control.gains()).kP(60).build())
            .build();

    io.retune(tuned);

    // 60 V a radian is 60 × 2π a turn of the pivot, 25 rotor turns, as a share of 12 V.
    double expected = 60 * 2 * Math.PI / (12 * 25);
    assertThat(((SparkMax) io.leader()).configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0))
        .isCloseTo(expected, within(1e-6 * expected));
  }

  /**
   * The slide's two Vortexes on SPARK Flexes: the first leads, and reads meters through its drum.
   */
  @Test
  void aSlideOnTwoSparkFlexesReadsMetersThroughItsDrum() {
    JointIOSpark io =
        io(
            unmotored(ELEVATOR)
                .motors(
                    Motor.NEO_VORTEX,
                    gearing(10),
                    Inches.of(1),
                    SPARK_FLEX.can(34),
                    SPARK_FLEX.can(35)));
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNeoVortex(1));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    io.updateInputs(inputs);
    double start = inputs.position;
    // Ten rotor turns through 10:1 are one turn of the 1 in drum: 2π inches of travel.
    sim.setPosition(sim.getPosition() + 10);
    sim.iterate(0, 12, 0.02);
    io.updateInputs(inputs);

    assertThat(inputs.position - start).isCloseTo(2 * Math.PI * 0.0254, within(1e-9));
    assertThat(inputs.unconfigured).isEmpty();
  }

  /** A speed goes to the SPARK's speed slot in rotor RPM: the roller's 1 rad/s through 3:1. */
  @Test
  void aSpeedGoesInRotorRpm() {
    JointIOSpark io = io(unmotored(ROLLER).motor(Motor.NEO_550, gearing(3), SPARK_MAX.can(36)));
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNeo550(1));
    sim.enable();

    io.setVelocity(1);
    sim.iterate(0, 12, 0.02);

    assertThat(sim.getSetpoint()).isCloseTo(1 / (2 * Math.PI) * 3 * 60, within(1e-4));
    assertThat(sim.getClosedLoopSlot()).isEqualTo(ClosedLoopSlot.kSlot1);
  }

  /**
   * Homing pushes past the soft limits, which a SPARK keeps itself: they're switched off while it
   * homes, and back on once it's found where it is.
   */
  @Test
  void homingSwitchesItsSoftLimitsOffAndBackOn() {
    JointIOSpark io = io(pivot(37).retuned(control -> control.withSoftLimits(0.01, 2.3)));
    SparkMax spark = (SparkMax) io.leader();
    boolean before = spark.configAccessor.softLimit.getForwardSoftLimitEnabled();

    io.home(-2);
    boolean homing = spark.configAccessor.softLimit.getForwardSoftLimitEnabled();
    io.resetPosition(0);
    boolean after = spark.configAccessor.softLimit.getForwardSoftLimitEnabled();

    assertThat(before).isTrue();
    assertThat(homing).isFalse();
    assertThat(after).isTrue();
  }

  /**
   * A pivot with REV's Through Bore on its SPARK's data port, mounted reversed: the SPARK takes the
   * encoder's direction and its offset, and reads zero at the low edge of the window one turn of it
   * tells apart, centered on the pivot's travel (0° to 135°): -112.5°, so its reading never wraps
   * between the stops. Powered on at 60°, not where it rests, the pivot is told so by the encoder's
   * first good reading, and its motor's encoder carries it on. REV's simulation reports the reading
   * it's given, applying neither the offset nor the direction, so the test gives it the SPARK's
   * reading.
   */
  @Test
  void aPivotWithAnEncoderOnItsDataPortIsToldWhereItIs() {
    JointIOSpark io = io(pivot(38).absoluteEncoder(8192, REVERSED));
    SparkMax spark = (SparkMax) io.leader();
    // At 60°, a sixth of a turn past the pivot's zero and so 0.3125 + 1/6 of one past the edge.
    new SparkAbsoluteEncoderSim(spark).setPosition(0.3125 + 60.0 / 360);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    io.updateInputs(inputs);
    double told = inputs.position;
    io.updateInputs(inputs);
    double carried = inputs.position;
    // The joint moves on without its motor, as across backlash or a slipping belt.
    new SparkAbsoluteEncoderSim(spark).setPosition(0.3125 + 70.0 / 360);
    io.updateInputs(inputs);

    assertThat(spark.configAccessor.absoluteEncoder.getInverted()).isTrue();
    // The raw reading at the edge, -0.3125 turns, reversed and offset 0.1: 0.4125.
    assertThat(spark.configAccessor.absoluteEncoder.getZeroOffset())
        .isCloseTo(0.4125, within(1e-6));
    assertThat(Math.toDegrees(told)).isCloseTo(60, within(1e-3));
    assertThat(Math.toDegrees(carried)).as("then from its rotor").isCloseTo(60, within(1e-3));
    assertThat(Math.toDegrees(inputs.position)).as("its rotor's").isCloseTo(60, within(1e-3));
    assertThat(Math.toDegrees(inputs.absolutePosition))
        .as("its encoder's, beside it")
        .isCloseTo(70, within(1e-3));
  }

  /**
   * A pivot whose SPARK's loop closes on its data-port encoder is read by the encoder each loop,
   * wherever its motor's count is, and is moved in the encoder's turns from the window's edge: 45°
   * is 0.3125 + 1/8 of a turn, and a turn a second 60 of the encoder's RPM. Its loop runs on the
   * encoder, without position wrapping, its kP per turn of the encoder. REV's simulation runs its
   * loop on the motor's own encoder whatever it's told, so what the SPARK does with these is read
   * from what it took, not seen.
   */
  @Test
  void aPivotClosedOnItsDataPortEncoderIsReadByItAndMovedInItsTurns() {
    // Its loop on the encoder, which tells it where it is: no homing recipe.
    MechanismControl unhomed =
        MechanismControlBuilder.builder(ControllerSettings.control(PIVOT))
            .homing(Optional.empty())
            .build();
    JointIOSpark io =
        io(pivot(39).absoluteEncoder(8192, REVERSED.closesLoop()).retuned(old -> unhomed));
    SparkMax spark = (SparkMax) io.leader();
    SparkSim sim = new SparkSim(spark, DCMotor.getNEO(1));
    sim.enable();
    SparkAbsoluteEncoderSim encoder = new SparkAbsoluteEncoderSim(spark);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    encoder.setPosition(0.3125 + 60.0 / 360);
    io.updateInputs(inputs);
    io.updateInputs(inputs);
    double seeded = inputs.position;
    // The motor's count stays where the encoder first said; the joint moves on, or slipped.
    encoder.setPosition(0.3125 + 70.0 / 360);
    io.updateInputs(inputs);
    double moved = inputs.position;
    io.setPosition(Math.toRadians(45), 0, 0);
    sim.iterate(0, 12, 0.02);
    double toward = sim.getSetpoint();
    io.setVelocity(2 * Math.PI);
    sim.iterate(0, 12, 0.02);

    assertThat(spark.configAccessor.closedLoop.getFeedbackSensor())
        .isEqualTo(FeedbackSensor.kAbsoluteEncoder);
    assertThat(spark.configAccessor.closedLoop.getPositionWrappingEnabled()).isFalse();
    // 30 V a radian is 30 × 2π a turn of the pivot, one of the encoder's, as a share of 12 V.
    double kP = 30 * 2 * Math.PI / 12;
    assertThat(spark.configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0))
        .isCloseTo(kP, within(1e-6 * kP));
    assertThat(Math.toDegrees(seeded)).isCloseTo(60, within(1e-3));
    assertThat(Math.toDegrees(moved)).as("from its encoder").isCloseTo(70, within(1e-3));
    assertThat(toward).isCloseTo(0.3125 + 45.0 / 360, within(1e-6));
    assertThat(sim.getSetpoint()).isCloseTo(60, within(1e-4));
  }

  /** The pivot's through-bore on its SPARK's data port, reversed and offset 0.1, seeded once. */
  private static final MechanismWiring.DataPortEncoder REVERSED =
      DATA_PORT.offset(Rotations.of(0.1)).inverted();
}
