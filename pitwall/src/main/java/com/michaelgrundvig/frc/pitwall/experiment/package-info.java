/**
 * Experiments: test motions a mechanism runs the same way every time, recorded and scored, so that
 * one set of gains can be compared with another.
 *
 * <ul>
 *   <li>{@link com.michaelgrundvig.frc.pitwall.experiment.Rig} is everything tuning needs to know
 *       about one mechanism: its gains (values of its live configuration), how to command it, how
 *       far it may go, and what to record.
 *   <li>{@link com.michaelgrundvig.frc.pitwall.experiment.Profiles} builds the motion: a step, or a
 *       step and back.
 *   <li>{@link com.michaelgrundvig.frc.pitwall.experiment.Experiment} names a motion on a rig, and
 *       {@link com.michaelgrundvig.frc.pitwall.experiment.Experiments} runs it from the page as a
 *       command, only in utility mode, and only while the page keeps in touch.
 *   <li>{@link com.michaelgrundvig.frc.pitwall.experiment.Run} is one run: the gains it ran with, a
 *       {@link com.michaelgrundvig.frc.pitwall.experiment.Trace} of what happened (one {@link
 *       com.michaelgrundvig.frc.pitwall.experiment.Series} per signal), and its scores.
 *   <li>{@link com.michaelgrundvig.frc.pitwall.experiment.StepResponse} scores each jump: rise,
 *       overshoot, settling, and steady-state error.
 * </ul>
 *
 * <p>Nothing here changes a gain: the page does, through Pitwall's checks, like any other value.
 */
package com.michaelgrundvig.frc.pitwall.experiment;
