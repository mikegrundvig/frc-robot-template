/**
 * The Phoenix helper: a mechanism's Phoenix 6 settings from what it is, how it's wired, and how
 * it's driven, so a mechanism is described once, in the robot's definition, and its TalonFX's and
 * CANcoder's ratios, gains, gravity, limits and followers come out of it ({@link
 * frc.robot.lib.phoenix.PhoenixMechanism}). Nothing is retyped by hand, so no ratio is written as
 * an integer division, no gain lands per rotor turn in a slot that reads per mechanism turn, and no
 * cruise asks more than the motors turn.
 *
 * <p>Three inputs, each with one owner:
 *
 * <ul>
 *   <li>the robot's definition, through its {@code RobotMath}: the joint, the drive that moves it,
 *       its encoder, and what's derived from them (ratios, the encoder's wrap, gravity, the ideal
 *       gains and the free speed);
 *   <li>the mechanism's wiring ({@link com.michaelgrundvig.frc.world.robot.MechanismWiring}): CAN
 *       IDs, which way each motor turns, the CANcoder's offset and direction;
 *   <li>its control ({@link com.michaelgrundvig.frc.world.robot.MechanismControl}): gains, soft
 *       limits, Motion Magic's limits, current limits and neutral mode, in the joint's units.
 * </ul>
 *
 * <p>The units are fixed in {@link frc.robot.lib.phoenix.PhoenixUnits}: a revolute joint's Phoenix
 * position is the joint's rotations, a prismatic joint's its drum's rotations (the ratio is the
 * gearing alone, the drum a {@code Transmission.Drum}'s or a combined transmission's term's), and
 * gains, cruise and acceleration follow. The meters are converted there alone, so a mechanism's
 * control stays in meters.
 *
 * <p>Pure functions: the helper builds configurations and never touches a device, so it's tested
 * without hardware; the IO layer applies what it builds ({@code ArchitectureTest}).
 */
package frc.robot.lib.phoenix;
