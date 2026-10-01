// Pit: is the robot ready, and the checks to run. The status items, the actions, and the switches
// (every other WPILib tunable in the robot program).

import { robot, onChange, send, shown, h } from "./core.js";
import { sayResult, sayIn } from "./edits.js";

let statusBody;
let actionSection;
let actionBody;
let sayAction;
let otherSection;
let otherBody;
let sayOther;
let shownActions = "";
let shownOthers = "";
const others = new Map(); // path -> value cell

function mount(view) {
  statusBody = h("tbody");
  actionBody = h("tbody");
  const actionMessage = h("div", { class: "message bad", hidden: true });
  sayAction = sayIn(actionMessage);
  actionSection = h("section", { hidden: true },
    h("h2", null, "Actions"),
    h("p", { class: "muted" },
      "These run only with the robot enabled in utility mode, never on the field."),
    h("table", { class: "status" }, actionBody),
    actionMessage);
  otherBody = h("tbody");
  const otherMessage = h("div", { class: "message", hidden: true });
  sayOther = sayIn(otherMessage);
  otherSection = h("section", { hidden: true },
    h("h2", null, "Other tunables"),
    h("p", { class: "muted" }, "Every other WPILib tunable in the robot program. Type a value and "
        + "press Enter; it applies only while tuning is on, and dashboards see it too."),
    h("table", { class: "status" }, otherBody),
    otherMessage);
  view.append(
    h("h1", null, "Pit"),
    h("table", { class: "status" }, statusBody),
    actionSection,
    otherSection);
  onChange(render);
}

function render(patch, snapshot) {
  if (snapshot) {
    shownActions = "";
    shownOthers = "";
  }
  if (snapshot || "status" in patch || "units" in patch) renderStatus();
  if (snapshot || "actions" in patch) renderActions();
  if (snapshot || "wpilibTunables" in patch) renderOthers();
}

/** The status items: true and false as a check and a cross, a number with its unit. */
function renderStatus() {
  const doc = robot.doc;
  statusBody.replaceChildren();
  for (const [name, value] of Object.entries(doc.status)) {
    const row = statusBody.insertRow();
    row.insertCell().textContent = name;
    const cell = row.insertCell();
    const unit = doc.units[name];
    if (typeof value === "boolean") {
      cell.textContent = value ? "✓" : "✗";
      cell.className = value ? "ok" : "bad";
      cell.setAttribute("aria-label", value ? "yes" : "no");
    } else if (unit && typeof value === "number") {
      const decimals = Math.max(0, Math.ceil(-Math.log10(unit.resolution) - 1e-9));
      cell.textContent = value.toFixed(decimals) + (unit.unit ? " " + unit.unit : "");
    } else {
      cell.textContent = shown(value);
    }
  }
}

/** Each action with where it stands, and a button to run or cancel it. */
function renderActions() {
  const key = JSON.stringify(robot.doc.actions);
  if (key === shownActions) return; // rebuilding the rows mid-click would lose the click
  shownActions = key;
  const actions = Object.entries(robot.doc.actions);
  actionSection.hidden = actions.length === 0;
  actionBody.replaceChildren();
  for (const [name, { phase, detail }] of actions) {
    const row = actionBody.insertRow();
    row.insertCell().textContent = name;
    const state = row.insertCell();
    state.textContent = detail || "Ready";
    state.className = phase === "RUNNING" ? "ok" : phase === "REFUSED" ? "bad" : "";
    const running = phase === "RUNNING";
    row.insertCell().append(h("button", {
      class: running ? "" : "primary",
      onclick: async () => sayResult(await send("POST", "/api/actions",
          running ? { cancel: name } : { start: name }), sayAction),
    }, running ? "Cancel" : "Run"));
  }
}

/** Every other WPILib tunable: rows rebuilt only when the set changes, values on every change. */
function renderOthers() {
  const list = Object.entries(robot.doc.wpilibTunables);
  const key = list.map(([path, t]) => path + ":" + t.editable + ":" + t.limits).join("|");
  if (key !== shownOthers) {
    shownOthers = key;
    others.clear();
    otherBody.replaceChildren();
    for (const [path, { type, editable, limits }] of list) {
      const row = otherBody.insertRow();
      row.insertCell().textContent = path;
      const valueCell = row.insertCell();
      valueCell.className = "number";
      const inputCell = row.insertCell();
      if (editable) {
        const input = h("input", {
          "aria-label": path, placeholder: limits ? type + ", " + limits : type });
        input.addEventListener("keydown", async (event) => {
          if (event.key !== "Enter") return;
          const text = input.value.trim();
          const value = text === "true" ? true : text === "false" ? false : Number(text);
          if (text === "" || (typeof value === "number" && !Number.isFinite(value))) {
            sayOther(path + " takes a " + type + " value.", true);
            return;
          }
          const result = await send("PATCH", "/api/wpilib-tunables", { [path]: value });
          sayResult(result, sayOther);
          if (result.ok) input.value = "";
        });
        inputCell.append(input);
      }
      others.set(path, valueCell);
    }
    otherSection.hidden = list.length === 0;
  }
  for (const [path, { value }] of list) {
    const cell = others.get(path);
    if (cell) cell.textContent = shown(value);
  }
}

export default { id: "pit", title: "Pit", mount };
