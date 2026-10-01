/**
 * The simulator's controls: what the WPILib simulator GUI does for a robot program, from Pitwall's
 * page, and the simulated robot's parts failing live, which the GUI can't do. Simulator only: on a
 * robot there are none.
 *
 * <ul>
 *   <li>{@link com.michaelgrundvig.frc.pitwall.simulator.Simulator} drives the simulated Driver
 *       Station (enabled, the mode, the alliance station, the field system, the match time) and
 *       joysticks from the page, pauses and steps the simulator's clock, and stops the program, all
 *       through WPILib's own simulation ({@link
 *       com.michaelgrundvig.frc.pitwall.simulator.SimulatorHal}).
 *   <li>{@link com.michaelgrundvig.frc.pitwall.simulator.Components} is the registry of what can
 *       fail: each {@link com.michaelgrundvig.frc.pitwall.simulator.Component} (a camera, the gyro,
 *       a module, the robot itself) and its {@link
 *       com.michaelgrundvig.frc.pitwall.simulator.Fault}s, each with its {@link
 *       com.michaelgrundvig.frc.pitwall.simulator.Parameter}s, and the scenarios made of them. The
 *       robot's simulation declares them; Pitwall knows nothing of any robot's parts.
 * </ul>
 */
package com.michaelgrundvig.frc.pitwall.simulator;
