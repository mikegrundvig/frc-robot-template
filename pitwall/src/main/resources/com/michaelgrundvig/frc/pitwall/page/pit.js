// Pit: is the robot ready, and the checks to run before a match. The status items, the actions,
// and the switches: every other WPILib tunable in the robot program, which acts at once. Beside
// the switches, a card for each device the robot reports on (devices.js). Each card folds away.

import { robot, onChange, onConnection, send, shown, h, say } from "./core.js";
import { tuningOffBanner } from "./bar.js";
import { mountDevices } from "./devices.js";
import { folds } from "./fold.js";

let statusSummary;
let statusGroups;
let actionCard;
let actionList;
let gate;
let switchCard;
let switchList;
let builtActions = "";
let builtSwitches = "";
let builtStatus = "";
const statusLists = new Map(); // group key ("" for ungrouped) -> its <ul>
const actions = new Map(); // name -> its row's parts
const switches = new Map(); // path -> its row's parts
const started = new Map(); // action -> when this page first saw it running

const cards = folds("pitwall.pit.folded");

/** A card that folds away under its title, kept folded by its title. */
function fold(title, ...content) {
  return cards.section(title, title, null, ...content);
}

function mount(view) {
  statusSummary = h("p", { class: "status-summary" });
  statusGroups = h("div", { class: "status-groups" });
  gate = h("p", { class: "gate", id: "pit-gate" });
  actionList = h("ul", { class: "alist" });
  actionCard = fold("Actions", gate, actionList);
  actionCard.hidden = true;
  switchList = h("ul", { class: "alist" });
  switchCard = fold("Switches",
    h("p", { class: "meta" }, "Every other WPILib tunable in the robot program, as a dashboard "
        + "sees it. They act at once, only while tuning is on, and keep their setting into a "
        + "match."),
    tuningOffBanner("Switches"),
    switchList);
  switchCard.hidden = true;
  const side = h("div", { class: "side" }, switchCard);
  view.append(
    h("h1", { class: "view-title" }, "Pit"),
    h("p", { class: "lede" }, "Is the robot ready, and the checks to run before a match."),
    h("div", { class: "pit" },
      h("div", { class: "wide" }, statusSummary),
      statusGroups,
      actionCard,
      side));
  mountDevices(side);
  onChange(render);
  onConnection(() => {
    if (!robot.doc) return;
    renderActions(false);
    renderSwitches(false);
  });
  // An action's running time, written in place each second.
  setInterval(tickActions, 1000);
}

function render(patch, snapshot) {
  if (snapshot || "status" in patch || "units" in patch) renderStatus();
  if (snapshot || "actions" in patch || "tuning" in patch) renderActions(snapshot);
  if (snapshot || "wpilibTunables" in patch || "tuning" in patch) renderSwitches(snapshot);
}

/**
 * A name's group and its own label: "Field/Cameras" is "Cameras" in the Field group; a name with
 * no "/" stands alone, in no group (see Status's javadoc).
 */
function statusParts(name) {
  const slash = name.indexOf("/");
  return slash === -1 ? [null, name] : [name.slice(0, slash), name.slice(slash + 1)];
}

/**
 * The status items, grouped under the heading each names: true and false as a mark and a word, a
 * number with its unit, anything else as text. One line above them says how many of the yes-or-no
 * checks need attention, so the page answers "is the robot ready?" before anyone reads a line.
 */
function renderStatus() {
  const doc = robot.doc;
  const entries = Object.entries(doc.status);
  const key = entries.map(([name]) => name).join("\n");
  if (key !== builtStatus) {
    builtStatus = key;
    statusLists.clear();
    const order = [];
    for (const [name] of entries) {
      const [group] = statusParts(name);
      if (!statusLists.has(group)) {
        const list = h("ul", { class: "slist" });
        statusLists.set(group, list);
        order.push(group === null ? h("section", { class: "card" }, list) : fold(group, list));
      }
    }
    statusGroups.replaceChildren(...order);
  }
  let passing = 0;
  let total = 0;
  const failing = [];
  const rows = new Map(); // group -> its rows, in order
  for (const [name, value] of entries) {
    const [group, label] = statusParts(name);
    const unit = doc.units[name];
    let mark = h("span", { class: "sev none", "aria-hidden": "true" }, "•");
    let text;
    if (typeof value === "boolean") {
      total++;
      if (value) passing++; else failing.push(label);
      mark = h("span", { class: "sev " + (value ? "ok" : "bad"), "aria-hidden": "true" },
        value ? "✓" : "✗");
      text = value ? "yes" : "no";
    } else if (unit && typeof value === "number") {
      const decimals = Math.max(0, Math.ceil(-Math.log10(unit.resolution) - 1e-9));
      text = value.toFixed(decimals) + (unit.unit ? " " + unit.unit : "");
    } else {
      text = shown(value);
    }
    const row = h("li", null, mark, h("span", { class: "k" }, label), h("span", null, text));
    if (!rows.has(group)) rows.set(group, []);
    rows.get(group).push(row);
  }
  // Every row is rebuilt each pass (cheap: a status page has few items), so a list's rows are
  // always exactly this pass's, never last pass's left over.
  for (const [group, list] of statusLists) {
    list.replaceChildren(...rows.get(group));
  }
  renderStatusSummary(passing, total, failing);
}

/** How many of the status page's yes-or-no checks need attention, and which. */
function renderStatusSummary(passing, total, failing) {
  if (total === 0) {
    statusSummary.hidden = true;
    return;
  }
  statusSummary.hidden = false;
  if (failing.length === 0) {
    statusSummary.className = "status-summary ok";
    statusSummary.replaceChildren("✓ All " + total + (total === 1 ? " check" : " checks")
      + " pass.");
  } else {
    statusSummary.className = "status-summary bad";
    statusSummary.replaceChildren("✗ " + failing.length + " of " + total
      + " checks need attention: " + failing.join(", ") + ".");
  }
}

/** Each action with where it stands, and one large button to run it or stop it. */
function renderActions(snapshot) {
  const doc = robot.doc;
  const names = Object.keys(doc.actions);
  actionCard.hidden = names.length === 0;
  const key = names.join("\n");
  if (snapshot || key !== builtActions) {
    builtActions = key;
    actions.clear();
    actionList.replaceChildren(...names.map((name) => {
      const parts = { detail: h("div", { class: "adetail" }) };
      parts.button = h("button", { type: "button", class: "big", onclick: () => runOrStop(name) });
      actions.set(name, parts);
      return h("li", null,
        h("div", null, h("div", { class: "aname" }, name), parts.detail), parts.button);
    }));
  }
  // Why Run is off, beside it: the gate's own reason, and the mode now.
  const why = !robot.connected ? "Not connected to the robot." : doc.robot.actionsLocked
    ? capital(doc.robot.actionsLocked) + (doc.robot.mode ? ". Now: " + doc.robot.mode : "") + "."
    : null;
  gate.className = "gate " + (why ? "locked" : "ok");
  gate.textContent = why ? "🔒 " + why
    : "✓ Ready" + (doc.robot.mode ? ": " + doc.robot.mode : "") + ". They stop when the robot "
      + "disables or leaves Utility mode.";
  for (const [name, { phase, detail, locked, when }] of Object.entries(doc.actions)) {
    const parts = actions.get(name);
    const running = phase === "RUNNING";
    if (running && !started.has(name)) started.set(name, Date.now());
    if (!running) started.delete(name);
    // An action with times of its own (one that runs only while disabled) says its own lock.
    const own = when !== undefined && when !== "ENABLED_IN_UTILITY";
    const ownWhy = own && locked ? capital(locked) + "." : null;
    parts.detail.className = "adetail" + (running ? " running"
        : phase === "REFUSED" || phase === "CANCELED" ? " refused" : ownWhy ? " locked" : "");
    parts.detail.textContent = running ? "Running · " + elapsed(name)
        : ownWhy && !detail ? "🔒 " + ownWhy : detail || "Ready";
    parts.button.className = running ? "big danger" : "big primary";
    parts.button.textContent = running ? "■ Stop" : "▶ Run";
    parts.button.setAttribute("aria-label", (running ? "Stop " : "Run ") + name);
    // Stop is never locked; Run is, with the reason beside it.
    parts.button.disabled = !running && (!robot.connected || (own ? Boolean(locked) : Boolean(why)));
    parts.button.setAttribute("aria-describedby", "pit-gate");
  }
}

function capital(text) {
  return text.charAt(0).toUpperCase() + text.slice(1);
}

async function runOrStop(name) {
  const running = robot.doc.actions[name].phase === "RUNNING";
  const result = await send("POST", "/api/actions", running ? { cancel: name } : { start: name });
  if (!result.ok) say(name + ": " + result.problem.detail, true);
}

function elapsed(name) {
  const seconds = Math.floor((Date.now() - started.get(name)) / 1000);
  return Math.floor(seconds / 60) + ":" + String(seconds % 60).padStart(2, "0");
}

function tickActions() {
  for (const [name, parts] of actions) {
    if (started.has(name)) parts.detail.textContent = "Running · " + elapsed(name);
  }
}

/**
 * Every other WPILib tunable. A true-or-false one is a toggle and a number has a box and Set: both
 * act at once, as a dashboard's change does. The rows are rebuilt only when the set changes.
 */
function renderSwitches(snapshot) {
  const doc = robot.doc;
  const list = Object.entries(doc.wpilibTunables);
  switchCard.hidden = list.length === 0;
  const key = list.map(([path, t]) => path + ":" + t.type + ":" + t.editable + ":" + t.limits)
    .join("\n");
  if (snapshot || key !== builtSwitches) {
    builtSwitches = key;
    switches.clear();
    switchList.replaceChildren(...list.map(([path, tunable]) => makeSwitch(path, tunable)));
  }
  const locked = !doc.tuning.enabled || !robot.connected;
  for (const [path, { value }] of list) {
    const parts = switches.get(path);
    if (!parts) continue;
    if (parts.toggle) {
      parts.toggle.setAttribute("aria-checked", String(value === true));
      parts.state.textContent = value ? "On" : "Off";
      parts.toggle.disabled = locked;
    } else {
      parts.now.textContent = typeof value === "boolean" ? (value ? "On" : "Off")
          : "Now " + shown(value);
      if (parts.input) parts.input.disabled = parts.set.disabled = locked;
    }
  }
}

function makeSwitch(path, { type, editable, limits }) {
  const parts = { message: h("div", { class: "message bad", hidden: true }) };
  const fail = (text) => {
    parts.message.hidden = !text;
    parts.message.textContent = text || "";
  };
  const change = async (value) => {
    const result = await send("PATCH", "/api/wpilib-tunables", { [path]: value });
    fail(result.ok ? "" : "✗ " + result.problem.detail);
    if (result.ok && parts.input) parts.input.value = "";
    return result.ok;
  };
  let control = null;
  if (editable && type === "boolean") {
    parts.state = h("span");
    parts.toggle = h("button", { type: "button", role: "switch", class: "switch",
      "aria-label": path, onclick: () => change(!(robot.doc.wpilibTunables[path].value === true)) },
      h("span", { class: "knob" }), parts.state);
    control = parts.toggle;
  } else if (editable) {
    parts.input = h("input", { type: "text", inputmode: "decimal", autocomplete: "off",
      "aria-label": path + (limits ? ", " + limits : "") });
    const set = () => {
      const number = Number(parts.input.value.trim());
      if (parts.input.value.trim() === "" || !Number.isFinite(number)) {
        fail("✗ " + path + " takes a number.");
        return;
      }
      change(number);
    };
    parts.input.addEventListener("keydown", (event) => {
      if (event.key === "Enter") set();
    });
    parts.set = h("button", { type: "button", onclick: set }, "Set");
    control = h("span", { class: "setter" }, parts.input, parts.set);
  }
  parts.now = h("span");
  switches.set(path, parts);
  return h("li", null,
    h("div", null, h("div", { class: "aname mono" }, path),
      h("div", { class: "adetail" }, parts.toggle ? null : parts.now,
        type !== "boolean" && limits ? " · limits " + limits : null,
        editable ? null : " · read only here"),
      parts.message),
    control || h("span"));
}

export default { id: "pit", title: "Pit", mount };
