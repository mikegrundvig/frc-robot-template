// What became of a request to the simulator, said where it was made, the same way by every control
// in Sim: nothing when it's done (or a word for a while, for an event, which changes nothing else
// on the page); that it waits for the robot's next loop (202: the simulator is paused, or its loop
// was slow); or the refusal, in the simulator's own words, read out when the reader is free.

import { announce } from "./core.js";

/** The notes saying a request waits for the next loop: the loop that runs ends them. */
const waiting = new Set();

/** How many times each note was written, so a timer hides only the note it was set for. */
const written = new WeakMap();

/** Writes a note under a control: empty text hides it. */
export function note(message, text, bad = false) {
  waiting.delete(message);
  written.set(message, (written.get(message) || 0) + 1);
  message.hidden = !text;
  message.textContent = text;
  message.className = "message" + (bad ? " bad" : "");
}

/**
 * Says what became of a request, under what was pressed.
 *
 * @param message the note to say it in
 * @param result what core.js's send() answered
 * @param options {done, shown, paused}: what to read out once it's done; whether to show that too,
 *     for a while; and whether the simulator is paused, so a request left waiting says what to
 *     press
 * @return whether it was done now
 */
export function outcome(message, result, { done = "", shown = false, paused = false } = {}) {
  if (result.ok && result.status === 202) {
    note(message, paused
      ? "Asked: it happens with the next loop. The simulator is paused: Step or Resume."
      : "Asked: it waits for the robot's next loop.");
    waiting.add(message);
    return false;
  }
  if (result.ok) {
    note(message, shown && done ? "✓ " + done : "");
    if (done) announce(done);
    if (shown && done) {
      const at = written.get(message);
      setTimeout(() => {
        if (written.get(message) === at) note(message, ""); // never a later note, a refusal say
      }, 5000);
    }
    return true;
  }
  const why = result.problem.detail || result.problem.title;
  note(message, "✗ " + why, true);
  announce("Refused: " + why);
  return false;
}

/** A loop ran: what waited for one is done, and its note goes. */
export function loopRan() {
  for (const message of [...waiting]) note(message, "");
}
