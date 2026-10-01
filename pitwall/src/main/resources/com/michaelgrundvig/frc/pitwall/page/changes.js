// Changes: what to commit and where it goes; everything this session changed, each restorable;
// and what dashboards tried that the robot refused. Nothing tuned is saved on the robot: a restart,
// or the field connecting, returns every value to committed. Committing is writing the builder
// calls into the robot's configuration.

import {
  robot, onChange, onConnection, send, h, say, replace, confirm, go, shown,
} from "./core.js";
import { stageAll, whyLocked } from "./edits.js";
import { entries, onHistory, expect, record, undid, itemsText, clock } from "./history.js";

let reports;
let none;
let copyAll;
let resetAll;
let refusalList;
let historyList;
let copyBox;
let copyArea;
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
  // Where text goes when the browser won't copy it (an older one, or one that won't on http):
  // selected, for Ctrl+C.
  copyArea = h("textarea", { class: "copyarea", readonly: true, rows: 6,
                             "aria-label": "The text to copy" });
  copyBox = h("div", { class: "copybox", hidden: true },
    h("p", { class: "meta" }, "This browser won't copy it for the page: the text below is "
        + "selected, so press Ctrl+C (⌘C on a Mac)."),
    copyArea,
    h("button", { type: "button", class: "small", onclick: () => (copyBox.hidden = true) },
      "Close"));
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
      copyBox,
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
  onConnection(() => {
    if (!robot.doc) return;
    renderReports();
    renderHistory();
  });
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
  resetAll.disabled = count === 0 || Boolean(whyLocked());
  replace(reports, [...groups.values()].map((group) => report(group)));
  // Rebuilt above only if the text changed: give the buttons their handlers again.
  for (const button of reports.querySelectorAll("button[data-copy]")) {
    button.onclick = () => copy(button.dataset.copy, button);
  }
  for (const button of reports.querySelectorAll("button[data-revert]")) {
    button.disabled = Boolean(whyLocked());
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

/**
 * Copies text, saying so on the button that did it. Where the browser won't, the text is shown,
 * selected, for Ctrl+C: never a "Copied" that didn't happen.
 */
async function copy(text, button) {
  if (!(await copied(text))) {
    copyArea.value = text;
    copyBox.hidden = false;
    copyArea.focus();
    copyArea.select();
    say("Couldn't copy it here: the text is selected, so press Ctrl+C.", true);
    return;
  }
  copyBox.hidden = true;
  const was = button.textContent;
  button.textContent = "Copied ✓";
  setTimeout(() => (button.textContent = was), 1500);
  say("Copied.");
}

/** Whether the browser copied it: by the clipboard API, or else the older way, which says. */
async function copied(text) {
  try {
    await navigator.clipboard.writeText(text); // needs a secure page, such as localhost
    return true;
  } catch (error) {
    // http://robot.local isn't secure: try the older way.
  }
  const area = h("textarea", { class: "offscreen" });
  area.value = text;
  document.body.append(area);
  area.select();
  let ok = false;
  try {
    ok = document.execCommand("copy");
  } catch (error) {
    ok = false;
  }
  area.remove();
  return ok;
}

/** Copies the robot's own report: every change with the builder call that commits it. */
async function copyReport(button) {
  const response = await fetch("/api/changes", { headers: { Accept: "text/plain" } })
      .catch(() => null);
  if (!response || !response.ok) {
    say("The robot didn't send its report" + (response ? " (" + response.status + ")" : "")
        + ": nothing was copied.", true);
    return;
  }
  copy(await response.text(), button);
}

/**
 * Returns every tuned value to committed, once asked, listing each. The history keeps what it
 * undid, so Restore brings it back.
 */
async function returnAll() {
  const lock = whyLocked();
  if (lock) {
    say(lock, true, true);
    return;
  }
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
 * as it was), what a return to committed undid (the field connecting, a restart, Return all), or
 * edits set aside unapplied. A restart also says what the new code changed, apart, as there's
 * nothing to restore of that.
 */
function renderHistory() {
  if (!historyList) return;
  const locked = Boolean(whyLocked());
  replace(historyList, entries.length === 0
    ? h("li", null, h("span", { class: "muted" }, "Nothing changed yet."))
    : entries.slice().reverse().map((entry, index) => h("li", null,
      h("span", { class: "t" }, clock(entry.time)),
      h("div", null, h("div", { class: "src" }, entry.source),
        entry.items.length > 0 && h("div", { class: "items" }, itemsText(entry.items)),
        entry.code && entry.code.length > 0 && h("div", { class: "items" },
          "The code changed (committed): " + itemsText(entry.code))),
      entry.items.length > 0 ? h("button", { type: "button",
        "data-entry": String(entries.length - 1 - index), disabled: locked },
        entry.kind === "staged" ? "Stage them again"
          : undid(entry) ? "Restore what it undid" : "Restore these") : h("span"))));
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
  // The robot keeps its latest few: forget what it no longer lists.
  for (const text of seen.keys()) if (!refused.includes(text)) seen.delete(text);
  replace(refusalList, refused.length === 0 ? h("li", null, h("span", { class: "muted" }, "None."))
    : refused.slice().reverse().map((text) => h("li", null,
      h("span", { class: "t" }, clock(seen.get(text))), h("span", { class: "items" }, text))));
}

export default { id: "changes", title: "Changes", mount };
