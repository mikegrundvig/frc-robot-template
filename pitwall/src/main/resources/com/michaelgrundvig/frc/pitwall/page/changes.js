// Changes: what to commit and where it goes; everything this session changed, each restorable;
// and what dashboards tried that the robot refused. Nothing tuned is saved on the robot: a restart,
// or the field connecting, returns every value to committed. Committing is writing the builder
// calls into the robot's configuration.

import { robot, onChange, send, h, say, replace, confirm, go, shown } from "./core.js";
import { stageAll } from "./edits.js";
import { entries, onHistory, expect, record, undid, itemsText, clock } from "./history.js";

let reports;
let none;
let copyAll;
let resetAll;
let refusalList;
let historyList;
const seen = new Map(); // a dashboard refusal's text -> when this page first saw it

function mount(view) {
  reports = h("div");
  none = h("p", { class: "muted" }, "✓ Everything matches the committed configuration.");
  copyAll = h("button", { type: "button", class: "primary", onclick: () => copyReport(copyAll) },
    "Copy the report");
  resetAll = h("button", { type: "button", class: "danger", onclick: returnAll },
    "Return all to committed…");
  refusalList = h("ul", { class: "hist" });
  historyList = h("ul", { class: "hist" });
  view.append(
    h("h1", { class: "view-title" }, "Changes"),
    h("p", { class: "lede" }, "What to commit, and where it goes."),
    h("section", { class: "card" },
      h("h2", null, "To commit"),
      h("p", { class: "meta" }, "Nothing is saved on the robot: a restart, or the field "
          + "connecting, returns every value to committed. To keep a value, write its builder call "
          + "into the robot's configuration and commit it."),
      none, reports,
      h("div", { class: "acts" }, copyAll, resetAll),
      h("p", { class: "meta phone-only" }, "Committing is a laptop's job: copying works here, "
          + "but the code is there.")),
    h("section", { class: "card" },
      h("h2", null, "This session", h("span", { class: "meta" }, "kept in this tab, through "
          + "reloads, until it closes")),
      historyList),
    h("section", { class: "card" },
      h("h2", null, "Refused from dashboards"),
      refusalList,
      h("p", { class: "meta" }, "A dashboard's refused change snaps back on the dashboard, which "
          + "can't say why; the reason is here.")));
  onChange(render);
  onHistory(renderHistory);
  renderHistory();
}

function render(patch, snapshot) {
  if (snapshot || "changes" in patch || "tunables" in patch || "tuning" in patch) renderReports();
  if (snapshot || "tuning" in patch) {
    renderRefusals();
    renderHistory();
  }
}

/** The tuned values by configuration and record, each with the builder calls that commit them. */
function renderReports() {
  const doc = robot.doc;
  const groups = new Map(); // "config\nrecord path" -> {config, group, record, paths}
  for (const path of Object.keys(doc.tunables)) {
    if (!(path in doc.changes)) continue;
    const value = doc.tunables[path];
    const group = typeof value.group === "string" ? value.group
        : value.field.includes(".") ? value.field.slice(0, value.field.lastIndexOf(".")) : "";
    const key = value.config + "\n" + group;
    if (!groups.has(key)) groups.set(key, { config: value.config, group, record: value.record, paths: [] });
    groups.get(key).paths.push(path);
  }
  const count = Object.keys(doc.changes).length;
  none.hidden = count > 0;
  copyAll.disabled = count === 0;
  resetAll.disabled = count === 0 || !doc.tuning.enabled;
  replace(reports, [...groups.values()].map((group) => report(group)));
  // Rebuilt above only if the text changed: give the buttons their handlers again.
  for (const button of reports.querySelectorAll("button[data-copy]")) {
    button.onclick = () => copy(button.dataset.copy, button);
  }
  for (const button of reports.querySelectorAll("button[data-revert]")) {
    button.disabled = !doc.tuning.enabled;
    button.onclick = () => revert(button.dataset.revert.split("\n"));
  }
}

/** Stages the committed values of these tuned values, to see and apply in Tune. */
function revert(paths) {
  const values = {};
  for (const path of paths) {
    if (robot.doc.tunables[path]) values[path] = robot.doc.tunables[path].committed;
  }
  stageAll(values);
  go("#tune");
}

function report({ config, group, record, paths }) {
  const doc = robot.doc;
  const calls = paths.map((path) => doc.changes[path].builderCall);
  const lines = paths.map((path) => {
    const change = doc.changes[path];
    return change.builderCall + "   // was " + change.committed;
  });
  const leaf = group.slice(group.lastIndexOf(".") + 1);
  return h("div", { class: "report" },
    h("h3", null, config + (group ? " · " + group : ""),
      record && h("span", { class: "mono meta" }, record),
      h("span", { class: "tag tuned" }, "● " + paths.length + " tuned")),
    h("p", { class: "where" }, group
      ? "In " + robot.doc.robot.name + "'s configuration: the " + config + "'s ." + leaf
        + "(…) builder."
      : "In " + robot.doc.robot.name + "'s configuration: the " + config + "'s builder."),
    h("pre", null, lines.join("\n")),
    h("div", { class: "acts" },
      h("button", { type: "button", "data-copy": calls.join("\n") }, "Copy"),
      h("button", { type: "button", "data-revert": paths.join("\n"),
                    title: "Stage their committed values, to apply in Tune" }, "Revert these")));
}

/** Copies text, saying so on the button that did it. */
async function copy(text, button) {
  try {
    await navigator.clipboard.writeText(text); // needs a secure page, such as localhost
  } catch (error) {
    const area = h("textarea"); // the fallback for http://robot.local
    area.value = text;
    document.body.append(area);
    area.select();
    document.execCommand("copy");
    area.remove();
  }
  const was = button.textContent;
  button.textContent = "Copied ✓";
  setTimeout(() => (button.textContent = was), 1500);
  say("Copied.");
}

/** Copies the robot's own report: every change with the builder call that commits it. */
async function copyReport(button) {
  const text = await fetch("/api/changes", { headers: { Accept: "text/plain" } })
      .then((response) => response.text());
  copy(text, button);
}

/**
 * Returns every tuned value to committed, once asked, listing each. The history keeps what it
 * undid, so Restore brings it back.
 */
async function returnAll() {
  const doc = robot.doc;
  const tuned = Object.keys(doc.tunables).filter((path) => path in doc.changes);
  const ok = await confirm("Return all to committed?", [
    h("p", null, "These " + tuned.length + (tuned.length === 1 ? " value goes" : " values go")
        + " back to committed on the robot:"),
    h("ul", null, tuned.map((path) => {
      const value = doc.tunables[path];
      return h("li", null, value.config + " " + value.field + ": " + shown(doc.values[path])
          + " → " + shown(value.committed));
    })),
    h("p", { class: "meta" }, "This session's history keeps them: Restore brings them back."),
  ], "Return " + tuned.length + " to committed");
  if (!ok) return;
  const committed = {};
  for (const path of tuned) committed[path] = doc.tunables[path].committed;
  const items = expect(committed);
  const result = await send("DELETE", "/api/changes");
  if (result.ok) {
    record("returned to committed, from this page", "reset", items);
    say("Every value is back to committed.");
  } else {
    say(result.problem.detail, true);
  }
}

/**
 * Everything this session changed, newest first, each restorable: what a change set (staged again
 * as it was), or what a return to committed undid (the field connecting, a restart, Return all).
 */
function renderHistory() {
  if (!historyList) return;
  const locked = !robot.doc || !robot.doc.tuning.enabled;
  replace(historyList, entries.length === 0
    ? h("li", null, h("span", { class: "muted" }, "Nothing changed yet."))
    : entries.slice().reverse().map((entry, index) => h("li", null,
      h("span", { class: "t" }, clock(entry.time)),
      h("div", null, h("div", { class: "src" }, entry.source),
        h("div", { class: "items" }, itemsText(entry))),
      h("button", { type: "button", "data-entry": String(entries.length - 1 - index),
                    disabled: locked },
        undid(entry) ? "Restore what it undid" : "Restore these"))));
  for (const button of historyList.querySelectorAll("button[data-entry]")) {
    button.disabled = locked;
    button.onclick = () => restore(entries[Number(button.dataset.entry)]);
  }
}

/** Stages an entry's values again, to see and apply in Tune. */
function restore(entry) {
  const values = {};
  for (const item of entry.items) values[item.path] = undid(entry) ? item.from : item.to;
  stageAll(values);
  go("#tune");
}

/** The latest dashboard changes the robot refused, newest first, with when this page saw them. */
function renderRefusals() {
  const refused = robot.doc.tuning.refused || [];
  const now = Date.now();
  for (const text of refused) if (!seen.has(text)) seen.set(text, now);
  replace(refusalList, refused.length === 0 ? h("li", null, h("span", { class: "muted" }, "None."))
    : refused.slice().reverse().map((text) => h("li", null,
      h("span", { class: "t" }, clock(seen.get(text))), h("span", { class: "items" }, text))));
}

export default { id: "changes", title: "Changes", mount };
