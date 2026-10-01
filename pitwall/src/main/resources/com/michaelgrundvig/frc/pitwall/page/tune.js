// Tune: every live configuration's tunable values, by mechanism, then by record, each beside what
// it means. Edits wait here until applied, then all change at once, or none do.
//
// On a laptop, what the focused value means (or a record's, or a mechanism's, when its heading is
// picked) shows in a panel beside the values; on a phone, it opens under the row when its name is
// tapped. The address names the value (#tune/drivetrain/gains.steerKp), so a reload comes back to
// it and a link can be sent.

import { robot, onChange, onConnection, h, replace } from "./core.js";
import { tuningOffBanner } from "./bar.js";
import { onEdits, outcome } from "./edits.js";
import { makeRow, updateRow, refuser, changedElsewhere } from "./rows.js";
import { onHistory } from "./history.js";
import { valueDocs, groupDocs, configDocs, groupOf } from "./docs.js";

const rows = new Map(); // path -> its row
const counts = new Map(); // configuration -> its heading's count
const notes = new Map(); // "configuration/record path" -> the note under that record's heading
const headings = new Map(); // "configuration/record path" -> that heading
const narrow = matchMedia("(max-width: 719px)");
let list;
let filter;
let panel;
let inline = null; // the documentation open under a row or heading, on a phone
let selected = null; // {path}, {config, group}, or {config}: what the documentation is about
let builtFrom = null;
let visible = false;
let notice;
let noticeTimer = null;
let refusalsSeen = null; // the dashboard refusals already shown, to tell a new one

function mount(view) {
  filter = h("input", { type: "search", autocomplete: "off", "aria-label": "Filter values",
    placeholder: "Filter by name or meaning  ( / )", oninput: build });
  list = h("div");
  panel = h("aside", { class: "docs", "aria-label": "What it means" });
  notice = h("div", { class: "banner warn", role: "status", hidden: true });
  view.append(
    h("h1", { class: "view-title" }, "Tune"),
    h("p", { class: "lede" }, "Every value the mechanisms let you tune while the robot runs. "
        + "Edits wait here until you apply them; then they all change at once, or none do."),
    tuningOffBanner("Values"),
    notice,
    h("div", { class: "tune" },
      h("div", null,
        h("div", { class: "toolbar" }, filter,
          h("div", { class: "legend", "aria-label": "What the marks mean" },
            h("span", { class: "tag tuned" }, "● tuned"), "not committed yet",
            h("span", { class: "tag edit" }, "✎ edit"), "staged here, not applied",
            h("span", { "aria-hidden": "true" }, "◇ committed · ● now · ○ edit, on the limits"))),
        list),
      panel));
  onChange(render);
  onEdits(() => {
    rows.forEach(updateRow);
    updateNotes();
    showDocs();
  });
  narrow.addEventListener("change", showDocs);
  onConnection(() => rows.forEach(updateRow));
  // A value someone else changed says so on its row.
  onHistory((entry) => {
    if (entry.source === "this page" || entry.kind === "reset") return;
    for (const item of entry.items) {
      const row = rows.get(item.path);
      if (row) changedElsewhere(row, entry.source);
    }
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

function hide() {
  visible = false;
}

/**
 * The rest of the address names a value, by its configuration and its path there
 * (#tune/drivetrain/gains.steerKp): it's shown, focused, and its meaning opened.
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
  selected = { path };
  showDocs();
  row.el.scrollIntoView({ block: "center" });
  row.input.focus({ preventScroll: true });
}

function typing(target) {
  return target instanceof HTMLInputElement || target instanceof HTMLTextAreaElement
      || target instanceof HTMLSelectElement;
}

function render(patch, snapshot) {
  const doc = robot.doc;
  if (snapshot || "tuning" in patch) noteRefusal(snapshot);
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
  if ("values" in patch || "changes" in patch || "configs" in patch || "rigs" in patch) showDocs();
}

/**
 * A dashboard's change the robot refused, shown here for a while as it happens: the dashboard just
 * snaps back, unable to say why. Changes keeps the list.
 */
function noteRefusal(snapshot) {
  const refused = robot.doc.tuning.refused || [];
  const latest = refused[refused.length - 1];
  const known = refusalsSeen;
  refusalsSeen = refused.join("\n");
  if (snapshot || known === null || !latest || refusalsSeen === known) return;
  notice.replaceChildren(h("span", { class: "grow" }, "✗ ", h("b", null, "A dashboard's change "
      + "was refused: "), h("span", { class: "mono" }, latest)),
    h("a", { href: "#changes" }, "See them all in Changes"));
  notice.hidden = false;
  clearTimeout(noticeTimer);
  noticeTimer = setTimeout(() => (notice.hidden = true), 15000);
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
  headings.clear();
  inline = null;
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
    const docs = doc.configs && doc.configs[config];
    const configHeading = h("h2", { class: "mech-h" },
      h("button", { type: "button", class: "link", "aria-label": config + ": what it is",
                    onclick: () => select({ config }) }, config),
      docs && docs.record && h("span", { class: "meta mono" }, docs.record), count);
    headings.set(config + "/", configHeading);
    const parts = [];
    for (const [group, paths] of groups) {
      const rowList = h("div", { class: "rowlist" });
      for (const path of paths) {
        const row = makeRow(path, "tune", { select: selectRow });
        rows.set(path, row);
        updateRow(row);
        rowList.append(row.el);
      }
      const note = h("div", { class: "group-note" });
      let heading = null;
      if (group) {
        notes.set(config + "/" + group, note);
        const about = docs && (docs.groups || []).find((candidate) => candidate.path === group);
        heading = h("h3", { class: "group-h" },
          h("button", { type: "button", class: "link mono", "aria-label": group + ": what it is",
                        onclick: () => select({ config, group }) }, group),
          about && about.record && h("span", { class: "meta mono" }, about.record),
          about && about.doc && about.doc.summary && h("span", { class: "gsum" }, about.doc.summary));
        headings.set(config + "/" + group, heading);
      }
      parts.push(h("section", { class: "group", "aria-label": config + (group ? " " + group : "") },
        heading, group && note, rowList));
    }
    sections.push(h("section", { class: "mech", "aria-label": config },
      configHeading, configNote, parts));
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
  showDocs();
  if (refocus && rows.has(refocus)) rows.get(refocus).input.focus();
}

function matches(value, query) {
  if (!query) return true;
  return [value.config, value.field, value.unit, value.record, value.meaning]
    .filter(Boolean).join(" ").toLowerCase().includes(query);
}

/**
 * A row asked what it means. On a laptop, focusing it is enough: the panel follows. On a phone,
 * its name (or "?") opens it under the row, and again closes it; focusing it to type doesn't.
 */
function selectRow(row, how) {
  const value = robot.doc.tunables[row.path];
  if (narrow.matches && how === "focus") return;
  if (narrow.matches && selected && selected.path === row.path && inline) {
    selected = null;
  } else {
    selected = { path: row.path };
  }
  // The address names it, so a reload comes back here; no new history entry for each focus.
  if (selected && value) history.replaceState(null, "", "#tune/" + value.config + "/" + value.field);
  showDocs();
}

/** A record's or a mechanism's heading asked what it is. */
function select(what) {
  const same = selected && !selected.path && selected.config === what.config
      && selected.group === what.group;
  selected = narrow.matches && same && inline ? null : what;
  showDocs();
}

/** What the selection means: in the panel on a laptop, under its row or heading on a phone. */
function showDocs() {
  if (!panel) return;
  const parts = !selected ? null
    : selected.path ? (rows.has(selected.path) ? valueDocs(selected.path) : null)
    : selected.group !== undefined ? groupDocs(selected.config, selected.group)
    : configDocs(selected.config);
  for (const row of rows.values()) {
    row.el.classList.toggle("selected", Boolean(selected && selected.path === row.path));
  }
  if (!narrow.matches) {
    if (inline) inline.remove();
    inline = null;
    panel.hidden = false;
    replace(panel, parts || h("p", { class: "muted" }, "Pick a value, a record, or a mechanism "
        + "(its name, or focus its value) to see what it means. Press ? in a value for it too."));
    return;
  }
  panel.hidden = true;
  const anchor = !selected ? null
    : selected.path ? rows.get(selected.path)?.el
    : headings.get(selected.config + "/" + (selected.group || ""));
  if (!parts || !anchor) {
    if (inline) inline.remove();
    inline = null;
    return;
  }
  if (!inline) {
    inline = h("div", { class: "docs docs-inline", "aria-label": "What it means" });
  }
  replace(inline, h("button", { type: "button", class: "link meta", onclick: () => {
    selected = null;
    showDocs();
  } }, "Close ✕"), parts);
  // Under the row, inside it, so it stays with the row; after a heading.
  if (selected.path) {
    if (inline.parentElement !== anchor) anchor.append(inline);
  } else if (inline.previousElementSibling !== anchor) {
    anchor.after(inline);
  }
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
