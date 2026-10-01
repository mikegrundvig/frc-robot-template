// A configuration value's row, the same in Tune and in Experiment: its name; the value and its
// unit; its limits; its committed value; and a note on its state. Built once and updated in place,
// so a row someone is typing in is never rebuilt under them.

import { robot, send, shown, h, say } from "./core.js";
import { described, edit, stage, apply, reasonsFor } from "./edits.js";

/**
 * Makes a value's row.
 *
 * @param path the value's tunable path, such as /Drive/config/gains/steerKp
 * @param context which view it's in ("tune", "exp"), so its elements' ids stay apart
 */
export function makeRow(path, context) {
  const value = described(path);
  const name = value.field.slice(value.field.lastIndexOf(".") + 1);
  const noteId = "note-" + context + "-" + path.replace(/[^a-zA-Z0-9]/g, "-");
  const row = { path, name };
  row.tags = h("span", { class: "tags nameline" });
  row.input = h("input", { class: "val", type: "text", inputmode: "decimal", autocomplete: "off",
    spellcheck: "false", "aria-label": name + (value.unit ? ", " + value.unit : ""),
    "aria-describedby": noteId });
  row.input.addEventListener("input", () => stage(path, row.input.value));
  row.input.addEventListener("keydown", (event) => {
    if (event.key === "Enter") {
      event.preventDefault();
      apply();
    }
  });
  row.unit = h("span", { class: "unit" }, value.unit);
  row.limits = h("span", { class: "limits-text" });
  row.committed = h("span");
  row.revert = h("button", { type: "button", class: "revert",
    title: "Back to the committed value", "aria-label": name + ": back to the committed value",
    onclick: async () => {
      const result = await send("PATCH", "/api/values", { [path]: null });
      if (!result.ok) say(result.problem.detail, true);
    } }, "↺");
  row.note = h("div", { class: "note", id: noteId });
  row.el = h("div", { class: "row", role: "group", "aria-label": value.config + " " + value.field },
    h("div", { class: "c-name" },
      h("div", { class: "nameline" }, h("span", { class: "name" }, name), row.tags)),
    h("div", { class: "c-value" }, row.input, row.unit),
    h("div", { class: "c-range" }, row.limits),
    h("div", { class: "c-committed" }, row.committed, row.revert),
    row.note);
  return row;
}

/** Shows a row's value as it is now: on the robot, staged here, and committed. */
export function updateRow(row) {
  const doc = robot.doc;
  const value = described(row.path);
  if (!value) return;
  const now = doc.values[row.path];
  const state = edit(row.path);
  const tuned = row.path in doc.changes;
  const reasons = reasonsFor(row.path);
  const notNumber = state.text !== null && !state.valid;
  const bad = notNumber || reasons.length > 0;
  const edited = state.isEdit && state.valid && !bad;
  row.el.classList.toggle("tuned", tuned);
  row.el.classList.toggle("edited", edited);
  row.el.classList.toggle("refused", bad);
  replace(row.tags,
    tuned && h("span", { class: "tag tuned" }, "● tuned"),
    edited && h("span", { class: "tag edit" }, "✎ edit"),
    notNumber && h("span", { class: "tag refused" }, "✗ not a number"),
    reasons.length > 0 && h("span", { class: "tag refused" }, "✗ refused"));
  // Never overwrite what someone is typing.
  if (document.activeElement !== row.input) {
    const text = state.text !== null ? state.text : shown(now);
    if (row.input.value !== text) row.input.value = text;
  }
  const locked = !doc.tuning.enabled;
  row.input.disabled = locked;
  row.input.setAttribute("aria-invalid", bad ? "true" : "false");
  row.limits.textContent = value.limits ? "limits " + value.limits : "no limits";
  replace(row.committed, "committed ", h("b", null, shown(value.committed)));
  row.revert.hidden = !tuned;
  row.revert.disabled = locked;
  replace(row.note,
    state.isEdit && state.valid && h("div", { class: "edit" }, "✎ " + withUnit(now, value) + " → "
        + withUnit(state.number, value) + ", staged: not applied yet"),
    notNumber && h("div", { class: "why" }, "✗ Only numbers can be applied."),
    reasons.map((reason) => h("div", { class: "why" }, "✗ Refused by " + refuser(reason) + ": ",
      h("span", null, reason.text))));
}

/** Who refused a change, as the row says it: "its tuning limits", "SwerveConfig.Gains's check". */
export function refuser(reason) {
  switch (reason.layer) {
    case "limit": return "its tuning limits";
    case "record": return (reason.by || "its record") + "'s own check";
    case "rule": return "the robot rule “" + reason.by + "”";
    case "tunable": return "its tunable";
    default: return "the robot";
  }
}

/** A number with its unit, as the row shows it. */
export function withUnit(number, value) {
  return shown(number) + (value.unit ? " " + value.unit : "");
}

/**
 * Replaces an element's children, but only when they differ: a row updated with nothing new keeps
 * its elements, so a button isn't rebuilt between a press and its release.
 */
export function replace(element, ...children) {
  const next = children.flat().filter((child) => child !== null && child !== undefined
      && child !== false);
  const scratch = document.createElement("div");
  for (const child of next) scratch.append(child instanceof Node ? child : String(child));
  if (scratch.innerHTML === element.innerHTML) return false;
  element.replaceChildren(...scratch.childNodes);
  return true;
}
