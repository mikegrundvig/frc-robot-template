/**
 * Scenes: the world drawn in 3D on the page, from data the robot program gives it.
 *
 * <p>A {@link com.michaelgrundvig.frc.pitwall.scene.Scene} is named frames and shapes in them,
 * grouped into {@link com.michaelgrundvig.frc.pitwall.scene.Layer}s. A layer describes what stays
 * once ({@link com.michaelgrundvig.frc.pitwall.scene.SceneBuilder}), says what moves each loop
 * while a page shows the scene ({@link com.michaelgrundvig.frc.pitwall.scene.SceneState}), and
 * answers a click ({@link com.michaelgrundvig.frc.pitwall.scene.Inspection}). The page draws every
 * shape itself, in its own colors, light or dark; nothing here knows of fields, robots, or drawing.
 * {@link com.michaelgrundvig.frc.pitwall.scene.Scenes} holds them, and sends each to the pages that
 * show it.
 */
package com.michaelgrundvig.frc.pitwall.scene;
