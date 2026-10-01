// Tune: every live configuration's tunable values, by mechanism, then by record. Edits wait here
// until applied, then all change at once, or none do.

import { robot, onChange, h } from "./core.js";
import { onEdits, outcome } from "./edits.js";
import { makeRow, updateRow, replace, refuser } from "./rows.js";

const rows = new Map(); // path -> its row
const counts = new Map(); // configuration -> its heading's count
const notes = new Map(); // "configuration/record path" -> the note under that record's heading
let list;
let filter;
let builtFrom = null;
let visible = false;

function mount(view) {
  filter = h("input", { type: "search", autocomplete: "off", "aria-label": "Filter values",
    placeholder: "Filter by name or meaning  ( / )", oninput: build });
  list = h("div");
  view.append(
    h("h1", { class: "view-title" }, "Tune"),
    h("p", { class: "lede" }, "Every value the mechanisms let you tune while the robot runs. "
        + "Edits wait here until you apply them; then they all change at once, or none do."),
    h("div", { class: "toolbar" }, filter,
      h("div", { class: "legend", "aria-label": "What the marks mean" },
        h("span", { class: "tag tuned" }, "● tuned"), "not committed yet",
        h("span", { class: "tag edit" }, "✎ edit"), "staged here, not applied")),
    list);
  onChange(render);
  onEdits(() => {
    rows.forEach(updateRow);
    updateNotes();
  });
  // "/" finds a value, as in many tools, unless someone is typing.
  document.addEventListener("keydown", (event) => {
    if (!visible || event.key !== "/" || typing(event.target)) return;
    event.preventDefault();
    filter.focus();
  });
}

function show() {
  visible = true;
}

/**
 * The rest of the address names a value, by its configuration and its path there
 * (#tune/drivetrain/gains.steerKp): it's shown and focused.
 */
function route(rest) {
  if (!rest || !robot.doc) return;
  const cut = rest.indexOf("/");
  const config = rest.slice(0, cut);
  const field = rest.slice(cut + 1);
  const path = Object.keys(robot.doc.tunables).find((candidate) =>
    robot.doc.tunables[candidate].config === config && robot.doc.tunables[candidate].field === field);
  if (!path) return;
  if (!rows.has(path)) {
    filter.value = ""; // filtered out: show everything again
    build();
  }
  const row = rows.get(path);
  if (!row) return;
  row.el.scrollIntoView({ block: "center" });
  row.input.focus({ preventScroll: true });
}

function hide() {
  visible = false;
}

function typing(target) {
  return target instanceof HTMLInputElement || target instanceof HTMLTextAreaElement
      || target instanceof HTMLSelectElement;
}

function render(patch, snapshot) {
  const doc = robot.doc;
  // A snapshot rebuilds every row: after a redeploy, the values and their limits may differ.
  if (snapshot || "tunables" in patch || builtFrom !== doc.tunables) {
    build();
    return;
  }
  if ("tuning" in patch || "changes" in patch) {
    rows.forEach(updateRow);
    updateCounts();
  } else if ("values" in patch) {
    for (const path of Object.keys(patch.values)) {
      const row = rows.get(path);
      if (row) updateRow(row);
    }
  }
}

/** The values, by configuration, then by the record they're in; only those the filter matches. */
function build() {
  const doc = robot.doc;
  if (!doc) return;
  builtFrom = doc.tunables;
  const focused = document.activeElement;
  const refocus = focused && focused.dataset ? focused.dataset.path : null;
  rows.clear();
  counts.clear();
  notes.clear();
  const query = filter.value.trim().toLowerCase();
  const configs = new Map(); // name -> group -> [paths]
  for (const [path, value] of Object.entries(doc.tunables)) {
    if (!matches(value, query)) continue;
    if (!configs.has(value.config)) configs.set(value.config, new Map());
    const groups = configs.get(value.config);
    const group = groupOf(value);
    if (!groups.has(group)) groups.set(group, []);
    groups.get(group).push(path);
  }
  const sections = [];
  for (const [config, groups] of configs) {
    const count = h("span", { class: "meta" });
    counts.set(config, count);
    const configNote = h("div", { class: "group-note" });
    notes.set(config + "/", configNote);
    const parts = [];
    for (const [group, paths] of groups) {
      const rowList = h("div", { class: "rowlist" });
      for (const path of paths) {
        const row = makeRow(path, "tune");
        row.input.dataset.path = path;
        rows.set(path, row);
        updateRow(row);
        rowList.append(row.el);
      }
      const note = h("div", { class: "group-note" });
      if (group) notes.set(config + "/" + group, note);
      parts.push(h("section", { class: "group", "aria-label": config + (group ? " " + group : "") },
        group && h("h3", { class: "group-h" }, h("span", { class: "mono" }, group)),
        group && note,
        rowList));
    }
    sections.push(h("section", { class: "mech", "aria-label": config },
      h("h2", { class: "mech-h" }, config, count), configNote, parts));
  }
  if (sections.length === 0) {
    sections.push(h("p", { class: "muted" }, query
      ? "Nothing matches “" + filter.value + "”. The filter searches names and what they mean."
      : "This robot has no tunable configuration values: a mechanism offers them by publishing "
        + "a LiveConfig."));
  }
  list.replaceChildren(...sections);
  updateCounts();
  updateNotes();
  if (refocus && rows.has(refocus)) rows.get(refocus).input.focus();
}

/** The record a value is in, by its path in the configuration: "gains" for gains.steerKp. */
function groupOf(value) {
  if (typeof value.group === "string") return value.group;
  const cut = value.field.lastIndexOf(".");
  return cut < 0 ? "" : value.field.slice(0, cut);
}

function matches(value, query) {
  if (!query) return true;
  return [value.config, value.field, value.unit, value.record, value.meaning]
    .filter(Boolean).join(" ").toLowerCase().includes(query);
}

/**
 * A refusal's reasons that name no value (a record's check of itself) go under that record's
 * heading, or its configuration's when the record has no values of its own here.
 */
function updateNotes() {
  const placed = new Map();
  const errors = (outcome.refusal && outcome.refusal.errors) || [];
  for (const error of errors) {
    if ((error.paths || []).length > 0 || !error.config) continue;
    const key = notes.has(error.config + "/" + error.group) ? error.config + "/" + error.group
        : error.config + "/";
    if (!placed.has(key)) placed.set(key, []);
    placed.get(key).push(h("div", null, "✗ Refused by " + refuser(error)
        + (error.group && key.endsWith("/") ? " (" + error.group + ")" : "") + ": ",
      h("span", null, error.text)));
  }
  for (const [key, note] of notes) replace(note, placed.get(key) || []);
}

/** Each configuration's heading says how many values it has, and how many are tuned. */
function updateCounts() {
  const doc = robot.doc;
  for (const [config, count] of counts) {
    const paths = Object.keys(doc.tunables).filter((path) => doc.tunables[path].config === config);
    const tuned = paths.filter((path) => path in doc.changes).length;
    replace(count, paths.length + (paths.length === 1 ? " value" : " values")
        + (tuned ? " · " + tuned + " tuned" : ""));
  }
}

export default { id: "tune", title: "Tune", mount, show, hide, route };
