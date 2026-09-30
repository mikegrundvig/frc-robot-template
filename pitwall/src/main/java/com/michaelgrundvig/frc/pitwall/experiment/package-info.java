/**
 * Experiments: test motions a mechanism runs the same way every time, recorded and scored, so that
 * one set of gains can be compared with another.
 *
 * <ul>
 *   <li>{@link com.michaelgrundvig.frc.pitwall.experiment.Profiles} builds the motion: a step, or a
 *       step and back.
 *   <li>{@link com.michaelgrundvig.frc.pitwall.experiment.Trace} records what happened, one {@link
 *       com.michaelgrundvig.frc.pitwall.experiment.Series} per signal.
 *   <li>{@link com.michaelgrundvig.frc.pitwall.experiment.StepResponse} scores each jump: rise,
 *       overshoot, settling, and steady-state error.
 * </ul>
 */
package com.michaelgrundvig.frc.pitwall.experiment;
