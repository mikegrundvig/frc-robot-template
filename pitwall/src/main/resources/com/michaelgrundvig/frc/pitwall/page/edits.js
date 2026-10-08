// Edits to configuration values: typed (or staged by a button) here, then applied together, checked
// whole on the robot, all or none. Tune and Experiment share them, so a value staged in one shows
// staged in the other, and the apply bar counts them all.
//
// An edit is staged against the robot program it was typed for. When that program is gone (a
// redeploy) or the field connects (a match runs the committed configuration), the edits no longer
// mean what they did: they move into the session's history, where Restore stages them again.

import { robot, onChange, send, shown, say, announce, near } from "./core.js";
import { expect, record, recordWhenSeen } from "./history.js";

/**
 * What's staged, by the value's tunable path: {text} as typed, so a typo stays to be fixed; the
 * {program} it was staged against; and, for the history, the value's {config}, {field}, and what
 * it was ({from}).
 */
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
 * Whether text is the value a configuration value has now: the same number ("16" for 16.0), or
 * what the page shows for it ("2.53771" for 2.537712345), which retyped is no edit.
 */
function same(text, now) {
  const trimmed = String(text).trim();
  if (typeof now !== "number" || trimmed === "") return false;
  const number = Number(trimmed);
  return trimmed === shown(now) || (Number.isFinite(number) && near(number, now));
}

/** Stages what was typed for a value; what it has already stages nothing. */
export function stage(path, text) {
  put(path, String(text));
  changed();
}

/**
 * Stages a number for a value, exactly: a value's committed number, a run's gain, or one from the
 * history, staged as itself (String round-trips a double), so applying it gives that very number.
 */
export function stageNumber(path, number) {
  stage(path, String(number));
}

function put(path, text) {
  const doc = robot.doc;
  const value = described(path);
  if (!value || same(text, doc.values[path])) {
    staged.delete(path);
  } else {
    const kept = staged.get(path);
    staged.set(path, { text, program: doc.robot.program, config: value.config, field: value.field,
                       from: kept ? kept.from : doc.values[path] });
  }
  forget(path);
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

/**
 * Stages several values at once, {path: number}, exactly, such as a run's gains or what a history
 * entry changed: those still tunable.
 *
 * @return how many were staged
 */
export function stageAll(values) {
  let count = 0;
  for (const [path, number] of Object.entries(values)) {
    if (!described(path) || typeof number !== "number") continue;
    put(path, String(number));
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
 * step.
 */
export function edit(path) {
  const entry = staged.get(path);
  if (!entry) return { text: null, valid: true, isEdit: false };
  const text = entry.text;
  const trimmed = text.trim();
  const number = Number(trimmed);
  const valid = trimmed !== "" && Number.isFinite(number);
  if (!valid) return { text, valid: false, isEdit: true };
  const now = robot.doc.values[path];
  const isEdit = !same(text, now);
  const value = described(path);
  const outside = value ? whyOutside(number, value) : null;
  const span = value && typeof value.min === "number" && typeof value.max === "number"
    ? value.max - value.min : 0;
  const share = span > 0 && typeof now === "number" ? Math.abs(number - now) / span : 0;
  return { text, number, valid: true, isEdit, outside, share, big: isEdit && !outside && share > BIG };
}

/**
 * A big step moves a value by more than a quarter of its tuning range (its limits' span). Measured
 * against the range, not the value now, so zeroing a gain, or editing one committed as zero, never
 * misfires; a value without limits at both ends has no range, so no big steps.
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
 * What's staged, or just the values named: {changes} to send, {path: number}; {invalid}, the paths
 * whose text isn't a number; {outside}, those outside their limits; {big}, the big steps; and
 * {count}, how many edits there are.
 *
 * @param only the paths to count, or null for every staged edit
 */
export function pending(only = null) {
  const changes = {};
  const invalid = [];
  const outside = [];
  const big = [];
  for (const path of staged.keys()) {
    if (only && !only.includes(path)) continue;
    if (!(path in robot.doc.values)) continue;
    const state = edit(path);
    if (!state.isEdit) continue;
    if (!state.valid) invalid.push(path);
    else if (state.outside) outside.push(path);
    else changes[path] = state.number;
    if (state.big) {
      big.push({ path, from: robot.doc.values[path], to: state.number, share: state.share });
    }
  }
  return { changes, invalid, outside, big,
           count: Object.keys(changes).length + invalid.length + outside.length };
}

/** The values with an edit staged, in the order they were staged. */
export function edited() {
  return [...staged.keys()].filter((path) => path in robot.doc.values && edit(path).isEdit);
}

/**
 * Nudges a value a step: a tenth of its size (or of its edit's), up or down, which is how gains get
 * tuned, and quicker than typing on a phone. At zero, a hundredth of its limits' span. Rounded to
 * four figures, then kept within its limits.
 *
 * @param direction 1 for up, -1 for down
 */
export function nudge(path, direction) {
  const value = described(path);
  if (!value) return;
  const state = edit(path);
  const base = state.valid && state.text !== null ? state.number : robot.doc.values[path];
  const span = typeof value.min === "number" && typeof value.max === "number"
    ? value.max - value.min : 0;
  let next = base === 0 ? direction * (span > 0 ? span / 100 : 0.1)
    : base + direction * Math.abs(base) * 0.1;
  next = Number(next.toPrecision(4));
  if (typeof value.min === "number") next = Math.max(value.min, next);
  if (typeof value.max === "number") next = Math.min(value.max, next);
  stage(path, shown(next));
}

/**
 * Why nothing can be applied now, in words for beside the edits; null while it can. The robot
 * would refuse it anyway: saying so here keeps a press from looking like it did something.
 */
export function whyLocked() {
  if (!robot.connected) return "Not connected to the robot: the edits wait here.";
  if (!robot.doc) return "The robot hasn't sent its values yet.";
  if (robot.doc.tuning.fieldConnected) return "The field system is connected: nothing can change.";
  if (!robot.doc.tuning.enabled) return "Tuning is off: switch it on to apply.";
  return null;
}

/**
 * Applies the staged edits, all or none: every one, or with {@code only}, just those values (a
 * rig's gains, for its run). The robot checks them whole (each value's limits, then its
 * configuration's records) and either applies all of them or none, saying why. The session's
 * history records what it applied.
 *
 * @param options {only}: the paths to apply, or every staged edit; {source}: where the history
 *     says it came from
 * @return whether they were applied
 */
export async function apply({ only = null, source = "this page" } = {}) {
  if (outcome.applying) return false;
  const lock = whyLocked();
  if (lock) {
    say(lock, true, true);
    return false;
  }
  const { changes, invalid, outside } = pending(only);
  if (invalid.length > 0 || outside.length > 0 || Object.keys(changes).length === 0) return false;
  // What was sent, as typed: an edit retyped while the robot answers stays staged.
  const sent = new Map(Object.keys(changes).map((path) => [path, staged.get(path).text]));
  outcome.applying = true;
  outcome.refusal = null;
  changed();
  const items = expect(changes);
  const result = await send("PATCH", "/api/values", changes);
  outcome.applying = false;
  const count = Object.keys(changes).length;
  if (result.status === 202) {
    // The robot loop didn't answer in time: the edits stay staged, each dropped as the robot is
    // seen to have its value, and recorded then.
    recordWhenSeen(source, items);
    say("Queued: the robot will apply it on its next loop, or refuse it.");
    changed();
    return false;
  }
  if (result.ok) {
    for (const [path, text] of sent) {
      if (staged.has(path) && staged.get(path).text === text) staged.delete(path);
    }
    record(source, "apply", items);
    say("Applied " + count + (count === 1 ? " change." : " changes."));
    changed();
    return true;
  }
  if (result.status === 0) {
    say(result.problem.detail, true, true); // not answering: nothing to show beside the values
    changed();
    return false;
  }
  // Shown above the apply bar, and each reason beside its value: the edits stay staged, so fixing
  // one and applying again is the whole loop. A refusal is a result, read out politely; a lock is
  // read out at once.
  outcome.refusal = { status: result.status, ...result.problem };
  const reasons = (result.problem.errors || []).length;
  announce(result.status === 422
    ? "Refused: nothing changed. " + reasons + (reasons === 1 ? " reason." : " reasons.")
    : "Nothing changed. " + (result.problem.detail || result.problem.title), result.status === 409);
  changed();
  return false;
}

/**
 * Moves staged edits that no longer mean what they did into the history, where Restore stages them
 * again, and drops them.
 */
function lose(paths, why) {
  const items = [];
  for (const path of paths) {
    const entry = staged.get(path);
    const number = Number(entry.text.trim());
    if (entry.text.trim() !== "" && Number.isFinite(number)) {
      items.push({ path, config: entry.config, field: entry.field, from: entry.from, to: number });
    }
    staged.delete(path);
  }
  record("staged, not applied, " + why, "staged", items);
  say(paths.length === 1 ? "A staged edit was set aside " + why + ": stage it again from Changes."
    : paths.length + " staged edits were set aside " + why + ": stage them again from Changes.",
  true, true);
}

onChange((patch, snapshot) => {
  const doc = robot.doc;
  // Edits staged against a program that's gone, or before the field connected, don't carry over.
  if (snapshot) {
    const stale = [...staged].filter(([, entry]) => entry.program !== doc.robot.program)
      .map(([path]) => path);
    if (stale.length > 0) {
      outcome.refusal = null; // about the old program's values
      lose(stale, "before the robot restarted");
    }
  }
  if (doc.tuning.fieldConnected && staged.size > 0) {
    lose([...staged.keys()], "as the field connected");
  }
  // A staged value the robot now has (this page's queued apply, or another page's) is no edit.
  let touched = snapshot || "tunables" in patch || "tuning" in patch;
  for (const [path, value] of Object.entries(patch.values || {})) {
    const entry = staged.get(path);
    if (!entry) continue;
    touched = true;
    if (same(entry.text, value)) staged.delete(path);
  }
  if (touched) changed();
});
