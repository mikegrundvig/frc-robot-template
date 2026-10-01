// A configuration value's row, the same in Tune and in Experiment: its name and what it means; the
// value and its unit; its tuning limits, with the committed value, the value now, and any edit
// marked on them; the committed value; and a note on its state. Every part shows at every width.
// Built once and updated in place, so a row someone is typing in is never rebuilt under them.

import { robot, send, shown, h, say, replace, withUnit } from "./core.js";
import { described, edit, stage, apply, reasonsFor } from "./edits.js";
import { firstSentence, nameOf } from "./docs.js";

/**
 * Makes a value's row.
 *
 * @param path the value's tunable path, such as /Drive/config/gains/steerKp
 * @param context which view it's in ("tune", "exp"), so its elements' ids stay apart
 * @param options {select(row, how)}: called when the row is asked what it means, by its name
 *     ("name"), by focusing it ("focus"), or with "?" in its value ("key")
 */
export function makeRow(path, context, options = {}) {
  const value = described(path);
  const name = nameOf(value);
  const id = context + "-" + path.replace(/[^a-zA-Z0-9]/g, "-");
  const row = { path, name };
  const select = (how) => options.select && options.select(row, how);
  row.nameButton = h("button", { type: "button", class: "link name",
    "aria-label": name + ": what it means", onclick: () => select("name") }, name);
  row.tags = h("span", { class: "tags nameline" });
  row.meaning = h("div", { class: "meaning" });
  row.input = h("input", { class: "val", type: "text", inputmode: "decimal", autocomplete: "off",
    spellcheck: "false", "aria-label": name + (value.unit ? ", " + value.unit : ""),
    "aria-describedby": "note-" + id + " limits-" + id });
  row.input.dataset.path = path;
  row.input.addEventListener("input", () => stage(path, row.input.value));
  row.input.addEventListener("focus", () => select("focus"));
  row.input.addEventListener("keydown", (event) => {
    if (event.key === "Enter") {
      event.preventDefault();
      apply();
    } else if (event.key === "?") {
      event.preventDefault(); // a number has no "?": it asks what the value means
      select("key");
    }
  });
  row.unit = h("span", { class: "unit" }, value.unit);
  row.range = makeRange();
  row.limits = h("span", { class: "limits-text", id: "limits-" + id });
  row.committed = h("span");
  row.revert = h("button", { type: "button", class: "revert",
    title: "Back to the committed value", "aria-label": name + ": back to the committed value",
    onclick: async () => {
      const result = await send("PATCH", "/api/values", { [path]: null });
      if (!result.ok) say(result.problem.detail, true);
    } }, "↺");
  row.note = h("div", { class: "note", id: "note-" + id });
  row.el = h("div", { class: "row", role: "group", "aria-label": value.config + " " + value.field },
    h("div", { class: "c-name" }, h("div", { class: "nameline" }, row.nameButton, row.tags),
      row.meaning),
    h("div", { class: "c-value" }, row.input, row.unit),
    h("div", { class: "c-range" }, row.range.el, row.limits),
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
  // What it means, its first sentence: reading the list teaches. A robot whose Pitwall sends
  // documentation says so when there's none, so the gap gets filled.
  if (typeof value.meaning === "string") {
    row.meaning.hidden = false;
    row.meaning.classList.toggle("missing", !value.meaning);
    row.meaning.textContent = value.meaning ? firstSentence(value.meaning)
        : "No description yet: add @param " + row.name + " to its record's Javadoc";
  } else {
    row.meaning.hidden = true;
  }
  // Never overwrite what someone is typing.
  if (document.activeElement !== row.input) {
    const text = state.text !== null ? state.text : shown(now);
    if (row.input.value !== text) row.input.value = text;
  }
  const locked = !doc.tuning.enabled;
  row.input.disabled = locked;
  row.input.setAttribute("aria-invalid", bad ? "true" : "false");
  placeRange(row.range, value, now, state.valid && state.text !== null ? state.number : null);
  row.limits.textContent = value.limits
    ? "limits " + value.limits + (value.unit ? " " + value.unit : "") : "no limits";
  row.limits.classList.toggle("sr", !row.range.el.hidden); // the bar shows them; still read aloud
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

/**
 * The tuning limits as a bar, for a value limited at both ends: the committed value a hollow
 * diamond, the value now a dot, an edit a ring (square, outside the limits). The limits' text is
 * what a screen reader says, so the bar is left out of it.
 */
export function makeRange() {
  const range = {
    committed: h("div", { class: "mk c", title: "committed" }),
    now: h("div", { class: "mk cur", title: "now" }),
    edit: h("div", { class: "mk ed", title: "staged", hidden: true }),
    low: h("span"),
    high: h("span"),
  };
  range.el = h("div", { class: "range", "aria-hidden": "true" },
    h("div", { class: "track" }), range.committed, range.now, range.edit,
    h("div", { class: "ends" }, range.low, range.high));
  return range;
}

/** Places the marks on a range bar; hides it for a value not limited at both ends. */
export function placeRange(range, value, now, edited) {
  const limited = typeof value.min === "number" && typeof value.max === "number"
      && value.max > value.min;
  range.el.hidden = !limited;
  if (!limited) return;
  const at = (x) => (Math.min(1, Math.max(0, (x - value.min) / (value.max - value.min))) * 100)
      .toFixed(2) + "%";
  range.committed.style.left = at(value.committed);
  range.now.style.left = at(now);
  range.edit.hidden = edited === null;
  if (edited !== null) {
    range.edit.style.left = at(edited);
    range.edit.classList.toggle("out", edited < value.min || edited > value.max);
  }
  range.low.textContent = shown(value.min);
  range.high.textContent = shown(value.max) + (value.unit ? " " + value.unit : "");
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
