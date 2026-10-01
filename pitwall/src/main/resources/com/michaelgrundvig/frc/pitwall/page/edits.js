// Edits to configuration values, shared by Tune and Experiment: what someone typed waits in its
// input until applied, then every edit goes to the robot at once, checked whole, all or none.

import { robot, send, shown } from "./core.js";

/**
 * Edits waiting in some inputs, [path, input]: {path: number}, and whether all are numbers. Only
 * what someone typed counts; a value changed elsewhere (another page, a dashboard) just shows.
 */
export function pending(inputs) {
  const changes = {};
  let valid = true;
  for (const [path, input] of inputs) {
    if (!input.dataset.edited) continue;
    if (input.value.trim() === "" || input.value === shown(robot.doc.values[path])) continue;
    const number = Number(input.value);
    const ok = Number.isFinite(number);
    input.classList.toggle("invalid", !ok);
    if (ok) changes[path] = number; else valid = false;
  }
  return { changes, valid };
}

/** Applies edits to configuration values: checked whole on the robot, all or none. */
export async function applyEdits(inputs, say) {
  const { changes, valid } = pending(inputs);
  if (!valid) {
    say("Only numbers can be applied.", true);
    return false;
  }
  if (Object.keys(changes).length === 0) return false;
  const result = await send("PATCH", "/api/values", changes);
  sayResult(result, say);
  if (!result.ok) return false;
  // Applied: the robot's next patch says so, and shows the same.
  for (const [path, input] of inputs) {
    if (!(path in changes)) continue;
    delete input.dataset.edited;
    input.value = shown(changes[path]);
  }
  return true;
}

/** Says what became of a request: queued, refused and why, or nothing when it applied. */
export function sayResult(result, say) {
  if (result.status === 202) say("Queued: the robot will apply it on its next loop.");
  else if (!result.ok) say(result.problem.detail || result.problem.title, true);
  else say("");
}

/** A function that says things in a message box: a refusal gives each reason on its own line. */
export function sayIn(box) {
  return (text, bad) => {
    box.hidden = !text;
    box.textContent = text || "";
    box.classList.toggle("bad", Boolean(bad));
  };
}

/** The builder calls that commit the tuned values at these paths, in the tuning table's order. */
export function builderLines(paths) {
  const doc = robot.doc;
  return Object.keys(doc.tunables)
    .filter((path) => paths.includes(path) && doc.changes[path])
    .map((path) => {
      const change = doc.changes[path];
      return `${change.config} ${change.field}: ${change.committed} -> ${change.tuned}   `
          + change.builderCall;
    });
}
