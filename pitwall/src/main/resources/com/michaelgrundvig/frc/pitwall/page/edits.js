// Edits to configuration values: typed (or staged by a button) here, then applied together, checked
// whole on the robot, all or none. Tune and Experiment share them, so a value staged in one shows
// staged in the other, and the apply bar counts them all.

import { robot, onChange, send, shown, say, announce, near } from "./core.js";
import { expect, record } from "./history.js";

/** What's staged, by the value's tunable path: the text as typed, so a typo stays to be fixed. */
const staged = new Map();
const listeners = [];

/**
 * What became of the last apply: {@code applying} while it waits for the robot; {@code refusal},
 * the robot's problem details when it refused or was locked ({status, title, detail, errors}),
 * until the next apply or discard. Its errors are each a reason tied to the values it concerns
 * ({layer, by, paths, config, group, text}).
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
  forget(path);
  changed();
}

/**
 * Forgets the last refusal's reasons that concern a value being edited again: each was about the
 * value as it was. The robot checks it afresh at the next apply.
 */
function forget(path) {
  const refusal = outcome.refusal;
  if (!refusal || !refusal.errors) return;
  refusal.errors = refusal.errors.filter((error) => !(error.paths || []).includes(path));
  if (refusal.errors.length === 0) outcome.refusal = null;
}

/** The last refusal's reasons that concern a value, as the robot gave them. */
export function reasonsFor(path) {
  const refusal = outcome.refusal;
  if (!refusal || !refusal.errors) return [];
  return refusal.errors.filter((error) => (error.paths || []).includes(path));
}

/** Closes the last refusal, leaving the edits staged. */
export function closeRefusal() {
  outcome.refusal = null;
  changed();
}

/** Stages a number for a value, as people write it. */
export function stageNumber(path, number) {
  stage(path, shown(Number(number.toPrecision(6))));
}

/**
 * Stages several values at once, {path: number}, such as a run's gains or what a history entry
 * changed: those still tunable, each as people write it.
 *
 * @return how many were staged
 */
export function stageAll(values) {
  let count = 0;
  for (const [path, number] of Object.entries(values)) {
    if (!(path in robot.doc.values) || typeof number !== "number") continue;
    const now = robot.doc.values[path];
    if (near(number, now)) staged.delete(path);
    else staged.set(path, shown(Number(number.toPrecision(6))));
    forget(path);
    count++;
  }
  changed();
  say("Staged " + count + (count === 1 ? " value" : " values") + ": review, then apply.");
  return count;
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
 * reads as a number; {isEdit} when that differs from the value now; {outside}, why it's outside
 * the value's tuning limits, checked here before sending (the robot checks again); and {share}
 * and {big}, how far it moves the value as a share of its limits' span, and whether that's a big
 * step, more than a quarter of it.
 */
export function edit(path) {
  const text = staged.has(path) ? staged.get(path) : null;
  if (text === null) return { text: null, valid: true, isEdit: false };
  const trimmed = text.trim();
  const number = Number(trimmed);
  const valid = trimmed !== "" && Number.isFinite(number);
  if (!valid) return { text, valid: false, isEdit: true };
  const now = robot.doc.values[path];
  const isEdit = now === undefined || !near(number, now);
  const value = described(path);
  const outside = value ? whyOutside(number, value) : null;
  const span = value && typeof value.min === "number" && typeof value.max === "number"
    ? value.max - value.min : 0;
  const share = span > 0 && typeof now === "number" ? Math.abs(number - now) / span : 0;
  return { text, number, valid: true, isEdit, outside, share, big: isEdit && !outside && share > BIG };
}

/**
 * A big step: a quarter of a value's tuning range (decided). Measured against the range, not the
 * value now, so zeroing a gain, or editing one committed as zero, never misfires; a value with no
 * range has no big steps.
 */
const BIG = 0.25;

/** Why a number is outside a value's tuning limits, in the robot's words; null when it's within. */
function whyOutside(number, value) {
  if (typeof value.min === "number" && number < value.min) {
    return shown(number) + " is below its limit of " + shown(value.min);
  }
  if (typeof value.max === "number" && number > value.max) {
    return shown(number) + " is above its limit of " + shown(value.max);
  }
  return null;
}

/**
 * Everything staged: {changes} to send, {path: number}; {invalid}, the paths whose text isn't a
 * number; {outside}, those outside their limits; {big}, the big steps; and {count}, how many
 * edits there are.
 */
export function pending() {
  const changes = {};
  const invalid = [];
  const outside = [];
  const big = [];
  for (const path of staged.keys()) {
    if (!(path in robot.doc.values)) continue; // no longer tunable: a redeploy took it away
    const state = edit(path);
    if (!state.isEdit) continue;
    if (!state.valid) invalid.push(path);
    else if (state.outside) outside.push(path);
    else changes[path] = state.number;
    if (state.big) big.push({ path, from: robot.doc.values[path], to: state.number, share: state.share });
  }
  return { changes, invalid, outside, big,
           count: Object.keys(changes).length + invalid.length + outside.length };
}

/**
 * Nudges a value a step: a tenth of itself (or of its edit), up or down, which is how gains get
 * tuned, and quicker than typing on a phone. At zero, a hundredth of its limits' span. Kept within
 * its limits.
 *
 * @param direction 1 for up, -1 for down
 */
export function nudge(path, direction) {
  const value = described(path);
  const state = edit(path);
  const base = state.valid && state.text !== null ? state.number : robot.doc.values[path];
  const span = typeof value.min === "number" && typeof value.max === "number"
    ? value.max - value.min : 0;
  let next = base === 0 ? direction * (span > 0 ? span / 100 : 0.1) : base * (1 + direction * 0.1);
  if (typeof value.min === "number") next = Math.max(value.min, next);
  if (typeof value.max === "number") next = Math.min(value.max, next);
  stage(path, shown(Number(next.toPrecision(4))));
}

/**
 * Applies every staged edit, all or none. The robot checks them whole (each value's limits, its
 * configuration's records, the robot's rules) and either applies all of them or none, saying why.
 * The session's history records what it applied.
 *
 * @param source where the history says it came from
 * @return whether they were applied
 */
export async function apply(source = "this page") {
  const { changes, invalid, outside } = pending();
  if (invalid.length > 0 || outside.length > 0 || Object.keys(changes).length === 0
      || outcome.applying) {
    return false;
  }
  outcome.applying = true;
  outcome.refusal = null;
  changed();
  const items = expect(changes);
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
    record(source, "apply", items);
    say("Applied " + count + (count === 1 ? " change." : " changes."));
    changed();
    return true;
  }
  if (result.status === 0) {
    say(result.problem.detail, true); // not answering: nothing to show beside the values
    changed();
    return false;
  }
  // Shown above the apply bar, and each reason beside its value: the edits stay staged, so fixing
  // one and applying again is the whole loop.
  outcome.refusal = { status: result.status, ...result.problem };
  const reasons = (result.problem.errors || []).length;
  announce(result.status === 422
    ? "Refused: nothing changed. " + reasons + (reasons === 1 ? " reason." : " reasons.")
    : "Nothing changed. " + (result.problem.detail || result.problem.title), true);
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
