/**
 * Pushing to the page: an event goes out only when something changed.
 *
 * <p>{@link com.michaelgrundvig.frc.pitwall.push.Delta} works out what changed since the last
 * event, as a JSON merge patch, and {@link com.michaelgrundvig.frc.pitwall.push.Reading} keeps a
 * noisy number from counting as a change until it has really moved. {@link
 * com.michaelgrundvig.frc.pitwall.push.Broadcaster} runs it once per loop while a page is watching,
 * and hands each page's {@link com.michaelgrundvig.frc.pitwall.push.Stream} its events, which the
 * web server writes. Nothing here knows about the web server.
 */
package com.michaelgrundvig.frc.pitwall.push;
