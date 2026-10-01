// Tune: every live configuration's tunable values, by mechanism, then record. Edits wait here
// until applied, then all change at once, or none do.

import { robot, onChange, send, shown, h } from "./core.js";
import { pending, applyEdits, sayResult, sayIn } from "./edits.js";

const rows = new Map(); // path -> {row, input, revert}
let shownTunables = "";
let say;
let statusText;
let toggle;
let banner;
let valuesBody;
let applyButton;
let discardButton;
let refused;

function mount(view) {
  statusText = h("span", { class: "muted" });
  toggle = h("button", { hidden: true, onclick: async () =>
    sayResult(await send("PATCH", "/api/tuning", { enabled: !robot.doc.tuning.enabled }), say) });
  banner = h("div", { class: "banner", hidden: true },
    "The field system is connected, so tuning is locked off.");
  valuesBody = h("tbody");
  applyButton = h("button", { class: "primary", disabled: true,
    onclick: () => applyEdits(tuningInputs(), say) }, "Apply");
  discardButton = h("button", { disabled: true, onclick: discard }, "Discard");
  const message = h("div", { class: "message", hidden: true });
  say = sayIn(message);
  refused = h("p", { class: "muted", style: "white-space: pre-line", hidden: true });
  view.append(
    h("h1", null, "Tune ", statusText, " ", toggle),
    banner,
    h("table", null,
      h("thead", null, h("tr", null,
        h("th", null, "Value"), h("th", { class: "committed" }, "Committed"), h("th", null, "Now"),
        h("th", null, "Unit"), h("th"))),
      valuesBody),
    h("div", { class: "controls", style: "margin-top: 12px" }, applyButton, discardButton,
      h("span", { class: "muted" },
        "Edits wait here until you apply them, then all change at once, or none do.")),
    message,
    refused);
  onChange(render);
}

const tuningInputs = () => [...rows].map(([path, { input }]) => [path, input]);

function discard() {
  for (const [path, { input }] of rows) {
    delete input.dataset.edited;
    input.value = shown(robot.doc.values[path]);
    input.classList.remove("invalid");
  }
  refreshControls();
  say("");
}

function refreshControls() {
  const { changes, valid } = pending(tuningInputs());
  const count = Object.keys(changes).length;
  for (const [path, { row }] of rows) row.classList.toggle("pending", path in changes);
  applyButton.disabled = !robot.doc.tuning.enabled || count === 0 || !valid;
  applyButton.textContent = count > 1 ? `Apply ${count} changes` : "Apply";
  discardButton.disabled = count === 0;
}

function buildTable() {
  valuesBody.replaceChildren();
  rows.clear();
  let group = null;
  for (const [path, { config, field, unit, limits, committed }] of
       Object.entries(robot.doc.tunables)) {
    const cut = field.lastIndexOf(".");
    const heading = cut < 0 ? config : config + "." + field.slice(0, cut);
    if (heading !== group) {
      group = heading;
      const row = valuesBody.insertRow();
      row.className = "group";
      const cell = row.insertCell();
      cell.colSpan = 5;
      cell.textContent = heading.replaceAll(".", " › ");
    }
    const row = valuesBody.insertRow();
    row.insertCell().textContent = field.slice(cut + 1);
    const committedCell = row.insertCell();
    committedCell.className = "committed number";
    committedCell.textContent = shown(committed);
    const input = h("input", { inputmode: "decimal", "aria-label": config + " " + field,
                               title: limits ? "Limits: " + limits : null });
    input.addEventListener("input", () => {
      input.dataset.edited = "yes";
      refreshControls();
    });
    input.addEventListener("keydown", (event) => {
      if (event.key === "Enter") applyEdits(tuningInputs(), say);
    });
    row.insertCell().append(input);
    row.insertCell().textContent = unit;
    const revert = h("button", { class: "small", title: "Back to the committed value",
      "aria-label": "Revert " + config + " " + field,
      onclick: async () => sayResult(await send("PATCH", "/api/values", { [path]: null }), say),
    }, "↺");
    row.insertCell().append(revert);
    rows.set(path, { row, input, revert, committedCell });
  }
}

function render(patch, snapshot) {
  const doc = robot.doc;
  const key = Object.keys(doc.tunables).join("|");
  // A snapshot rebuilds every row: after a redeploy, the committed values may differ.
  if (snapshot || key !== shownTunables) {
    shownTunables = key;
    buildTable();
  }
  const session = doc.tuning;
  statusText.textContent = session.enabled ? "Tuning is on" : "Tuning is off";
  toggle.hidden = false;
  toggle.disabled = session.fieldConnected;
  toggle.textContent = session.enabled ? "Switch off" : "Switch on";
  toggle.className = session.enabled ? "" : "primary";
  banner.hidden = !session.fieldConnected;
  const refusals = session.refused || [];
  refused.hidden = refusals.length === 0;
  refused.textContent = refusals.length === 0
    ? "" : "Refused from a dashboard: " + refusals[refusals.length - 1];

  for (const [path, { row, input, revert, committedCell }] of rows) {
    const changed = path in doc.changes;
    row.classList.toggle("changed", changed);
    committedCell.textContent = shown(doc.tunables[path].committed);
    revert.hidden = !changed;
    revert.disabled = !session.enabled;
    input.disabled = !session.enabled;
    // Leave alone what someone is typing, or has typed and not yet applied.
    if (!input.dataset.edited && document.activeElement !== input) {
      input.value = shown(doc.values[path]);
    }
  }
  refreshControls();
}

export default { id: "tune", title: "Tune", mount };
