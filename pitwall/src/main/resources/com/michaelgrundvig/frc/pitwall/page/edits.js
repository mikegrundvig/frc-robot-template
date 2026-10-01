// Edits to configuration values: typed (or staged by a button) here, then applied together, checked
// whole on the robot, all or none. Tune and Experiment share them, so a value staged in one shows
// staged in the other, and the apply bar counts them all.

import { robot, onChange, send, shown, say } from "./core.js";

/** What's staged, by the value's tunable path: the text as typed, so a typo stays to be fixed. */
const staged = new Map();
const listeners = [];

/**
 * What became of the last apply: {@code applying} while it waits for the robot; {@code refusal},
 * the robot's problem details when it refused or was locked, until the next apply or discard.
 */
export const outcome = { applying: false, refusal: null };

/** Calls {@code listener()} whenever what's staged, or what became of applying it, changes. */
export function onEdits(listener) {
  listeners.push(listener);
}

function changed() {
  for (const listener of listeners) {
    try {
      listener();
    } catch (error) {
      console.error(error);
    }
  }
}

/** A configuration value as the robot describes it ({config, field, unit, limits, committed}). */
export function described(path) {
  return (robot.doc && robot.doc.tunables[path]) || null;
}

/** Whether two numbers are the same value, as the robot's report judges it. */
export function near(a, b) {
  return Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(b));
}

/**
 * Stages what was typed for a value. Typing the value it has already ("16" for 16.0) stages
 * nothing: numbers are compared, not text.
 */
export function stage(path, text) {
  const number = Number(String(text).trim());
  const now = robot.doc.values[path];
  if (String(text).trim() !== "" && Number.isFinite(number) && near(number, now)) {
    staged.delete(path);
  } else {
    staged.set(path, String(text));
  }
  changed();
}

/** Stages a number for a value, as people write it. */
export function stageNumber(path, number) {
  stage(path, shown(Number(number.toPrecision(6))));
}

/** Drops a value's staged edit. */
export function unstage(path) {
  if (staged.delete(path)) changed();
}

/** Drops every staged edit, and what the last apply said. */
export function discard() {
  staged.clear();
  outcome.refusal = null;
  changed();
}

/**
 * A value's edit: {text} as typed, or null when nothing is staged; {number} and {valid} when it
 * reads as a number; {isEdit} when that differs from the value now.
 */
export function edit(path) {
  const text = staged.has(path) ? staged.get(path) : null;
  if (text === null) return { text: null, valid: true, isEdit: false };
  const trimmed = text.trim();
  const number = Number(trimmed);
  const valid = trimmed !== "" && Number.isFinite(number);
  if (!valid) return { text, valid: false, isEdit: true };
  const now = robot.doc.values[path];
  return { text, number, valid: true, isEdit: now === undefined || !near(number, now) };
}

/**
 * Everything staged: {changes} to send, {path: number}; {invalid}, the paths whose text isn't a
 * number; and {count}, how many edits there are.
 */
export function pending() {
  const changes = {};
  const invalid = [];
  for (const path of staged.keys()) {
    if (!(path in robot.doc.values)) continue; // no longer tunable: a redeploy took it away
    const state = edit(path);
    if (!state.isEdit) continue;
    if (!state.valid) invalid.push(path);
    else changes[path] = state.number;
  }
  return { changes, invalid, count: Object.keys(changes).length + invalid.length };
}

/**
 * Applies every staged edit, all or none. The robot checks them whole (each value's limits, its
 * configuration's records, the robot's rules) and either applies all of them or none, saying why.
 *
 * @return whether they were applied
 */
export async function apply() {
  const { changes, invalid } = pending();
  if (invalid.length > 0 || Object.keys(changes).length === 0 || outcome.applying) return false;
  outcome.applying = true;
  outcome.refusal = null;
  changed();
  const result = await send("PATCH", "/api/values", changes);
  outcome.applying = false;
  const count = Object.keys(changes).length;
  if (result.status === 202) {
    // The robot loop didn't answer in time: the edits stay staged, and each stops counting as one
    // once the robot has its value.
    say("Queued: the robot will apply it on its next loop, or refuse it.");
    changed();
    return false;
  }
  if (result.ok) {
    for (const path of Object.keys(changes)) staged.delete(path);
    say("Applied " + count + (count === 1 ? " change." : " changes."));
    changed();
    return true;
  }
  outcome.refusal = { status: result.status, ...result.problem };
  say(result.status === 422 ? "Refused: nothing changed. " + (result.problem.detail || "")
      : result.problem.detail || result.problem.title, true);
  changed();
  return false;
}

// A staged value that changes on the robot may now match its edit (another page applied the same),
// and is then no edit at all: the rows and the apply bar recount. Other changes don't concern them.
onChange((patch, snapshot) => {
  if (staged.size === 0) return;
  if (snapshot || "tunables" in patch
      || ("values" in patch && Object.keys(patch.values).some((path) => staged.has(path)))) {
    changed();
  }
});
