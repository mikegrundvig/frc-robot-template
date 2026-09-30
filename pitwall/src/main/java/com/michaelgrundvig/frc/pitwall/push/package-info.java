/**
 * Pushing to the page: an event goes out only when something changed.
 *
 * <p>{@link com.michaelgrundvig.frc.pitwall.push.Delta} works out what changed since the last
 * event, as a JSON merge patch, and {@link com.michaelgrundvig.frc.pitwall.push.Reading} keeps a
 * noisy number from counting as a change until it has really moved.
 */
package com.michaelgrundvig.frc.pitwall.push;
