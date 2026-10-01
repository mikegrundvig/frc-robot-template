// Sim's faults and scenarios: a card for each part of the simulated robot that can fail, each
// fault with what it does and what undoing it does, as the robot's simulation declares them
// (frc.robot.sim.Faults); and the scenarios, named sets of those faults. The robot's code is never
// told: it finds out from its sensors, as on a robot.
//
// A lasting fault (a camera unplugged) shows as on until it's undone; an event (a hit) happens
// once, so it's only a button. The cards are built once, and again only when the parts change, so
// a value being typed is kept.

import { send, shown, h, replace } from "./core.js";
import { note, outcome } from "./sim-outcome.js";

let cards;
let onNow;
let undoAllButton;
let undoAllMessage;
let scenarioList;
let builtFaults = "";
let builtScenarios = "";
let locked = false;
let undoing = false; // Undo all is waiting for its answer
let context = { state: () => null, paused: () => false };
const rows = new Map(); // "component\nfault" -> its row's parts
const cardParts = new Map(); // component -> its card's parts
const scenarioRows = new Map(); // scenario -> its row's parts

/**
 * What fails: what's on now with Undo all, the scenarios, then a card for each part with its
 * faults.
 *
 * @param from {state(), paused()}: the simulator's sections as Sim has them, and whether it's
 *     paused
 */
export function makeFaults(from) {
  context = from;
  onNow = h("p", { class: "onnow" });
  undoAllButton = h("button", { type: "button", onclick: undoAll }, "Undo all");
  undoAllMessage = h("p", { class: "message", hidden: true });
  cards = h("div", { class: "fgrid" });
  scenarioList = h("div", { class: "scens" });
  return h("section", { class: "faults", "aria-labelledby": "sim-faults" },
    h("h2", { class: "mech-h", id: "sim-faults" }, "What fails"),
    h("p", { class: "lede" }, "Fail the simulated robot's parts while it runs. Its code isn't "
        + "told: it finds out from its sensors, as on a robot. Pit's status shows what it makes of "
        + "it."),
    h("div", { class: "card onbar" }, h("div", { class: "acts" }, onNow, undoAllButton),
      undoAllMessage),
    // Open on a laptop, where they sit side by side; on a phone, a long list, closed.
    h("details", { class: "card fcard", open: !matchMedia("(max-width: 719px)").matches },
      h("summary", null, h("span", { class: "fcname" }, "Scenarios")),
      h("p", { class: "meta" }, "Named sets of the faults below, as ./gradlew run -Pscenario "
          + "starts in, applied here at once to the running robot. What lasts shows as on, and is "
          + "undone below, one at a time or all together."),
      scenarioList),
    cards);
}

/** Shows the parts and their faults as the robot sent them. */
export function renderFaults(components) {
  const key = JSON.stringify(Object.entries(components || {}).map(([name, part]) => [name,
    part.kind, Object.entries(part.faults).map(([fault, { does, lasting, undo, parameters }]) =>
      [fault, does, lasting, undo, parameters])]));
  if (key !== builtFaults) {
    builtFaults = key;
    buildCards(components || {});
  }
  const on = [];
  for (const [name, part] of Object.entries(components || {})) {
    let count = 0;
    for (const [fault, described] of Object.entries(part.faults)) {
      const parts = rows.get(name + "\n" + fault);
      if (parts) updateFault(parts, described);
      if (described.lasting && described.on) {
        count++;
        on.push(name + ": " + fault);
      }
    }
    const card = cardParts.get(name);
    if (card) {
      card.count.hidden = count === 0;
      card.count.textContent = "● " + count + " on";
    }
  }
  replace(onNow, on.length === 0
    ? h("span", { class: "muted" }, "Nothing on: the robot as built.")
    : [h("b", { class: "onword" }, "● " + on.length + " on: "), on.join(" · ")]);
  undoAllButton.disabled = locked || undoing || on.length === 0;
  undoAllButton.textContent = on.length > 0 ? "Undo all " + on.length : "Undo all";
}

/** Shows the scenarios, each fault of theirs marked on while it is. */
export function renderScenarios(scenarios = {}, components = {}) {
  const key = JSON.stringify(scenarios || {});
  if (key !== builtScenarios) {
    builtScenarios = key;
    scenarioRows.clear();
    const names = Object.keys(scenarios || {});
    scenarioList.replaceChildren(...names.map((name) => makeScenario(name, scenarios[name])));
    if (names.length === 0) {
      scenarioList.append(h("p", { class: "muted" }, "This robot's simulation declares none."));
    }
  }
  for (const [name, parts] of scenarioRows) {
    // Its faults, those it put on marked as on while they are.
    replace(parts.list, scenarios[name].faults.map((setting, index) => {
      const on = isOn(setting, components);
      return [index > 0 ? " · " : "", h("span", { class: on ? "son" : null },
        on ? "● " : "", settingText(setting, components), on ? ": on" : "")];
    }));
    parts.el.classList.toggle("on", scenarios[name].faults.some((s) => isOn(s, components)));
    parts.apply.disabled = locked || parts.busy;
  }
}

/** Locks every control (not connected, or stopped); values can still be typed. */
export function lockFaults(yes) {
  locked = yes;
  for (const parts of rows.values()) updateButtons(parts);
  for (const parts of scenarioRows.values()) parts.apply.disabled = yes || parts.busy;
}

// ---- The cards ----

function buildCards(components) {
  rows.clear();
  cardParts.clear();
  cards.replaceChildren(...Object.entries(components).map(([name, part]) => {
    const count = h("span", { class: "tag on", hidden: true });
    cardParts.set(name, { count });
    const anyOn = Object.values(part.faults).some((fault) => fault.lasting && fault.on);
    // The robot's own card is open: shoving, hitting, and placing it are the most used.
    return h("details", { class: "card fcard", open: anyOn || part.kind === "robot" },
      h("summary", null, h("span", { class: "fcname" }, name),
        h("span", { class: "meta" }, part.kind), count),
      Object.entries(part.faults).map(([fault, described]) => makeFault(name, fault, described)));
  }));
}

function makeFault(component, name, fault) {
  const parts = { component, name, lasting: fault.lasting, params: [], busy: false };
  const label = component + ", " + name;
  for (const parameter of fault.parameters) {
    const range = shown(parameter.min) + " to " + shown(parameter.max);
    const input = h("input", { type: "text", inputmode: "decimal", autocomplete: "off",
      spellcheck: "false", class: "pval", value: shown(parameter.initial),
      "aria-label": label + ": " + parameter.name + (parameter.unit ? " in " + parameter.unit : "")
          + ", " + range });
    input.addEventListener("keydown", (event) => {
      if (event.key === "Enter") {
        event.preventDefault();
        applyFault(parts);
      }
    });
    parts.params.push({ ...parameter, input, range });
  }
  parts.tag = h("span", { class: "tag on", hidden: true }, "● on");
  parts.values = h("p", { class: "fvalues", hidden: true });
  parts.apply = h("button", { type: "button", "aria-label": "Apply " + label,
                              onclick: () => applyFault(parts) }, "Apply");
  parts.undo = h("button", { type: "button", hidden: true, "aria-label": "Undo " + label,
                             onclick: () => undoFault(parts) }, "Undo");
  parts.message = h("p", { class: "message", hidden: true });
  parts.el = h("div", { class: "fault", role: "group", "aria-label": label },
    h("div", { class: "fhead" }, h("b", null, name), parts.tag,
      !fault.lasting && h("span", { class: "tag plain" }, "↯ happens once")),
    h("p", { class: "does" }, fault.does),
    parts.values,
    parts.params.length > 0 && h("div", { class: "params" }, parts.params.map((parameter) =>
      h("label", { class: "param" }, h("span", null, parameter.name), parameter.input,
        parameter.unit && h("span", { class: "unit" }, parameter.unit),
        h("span", { class: "lim" }, parameter.range)))),
    h("div", { class: "acts" }, parts.apply, parts.undo,
      h("span", { class: "meta" }, (fault.lasting ? "Undo: " : "Afterwards: ") + fault.undo)),
    parts.message);
  rows.set(component + "\n" + name, parts);
  return parts.el;
}

function updateFault(parts, fault) {
  parts.on = Boolean(fault.lasting && fault.on);
  parts.el.classList.toggle("on", parts.on);
  parts.tag.hidden = !parts.on;
  const values = parts.on ? parts.params.filter((parameter) => parameter.name in fault.values) : [];
  parts.values.hidden = values.length === 0;
  parts.values.textContent = "On with " + values.map((parameter) => parameter.name + " "
      + shown(fault.values[parameter.name]) + (parameter.unit ? " " + parameter.unit : ""))
    .join(", ");
  updateButtons(parts);
}

function updateButtons(parts) {
  parts.undo.hidden = !parts.on;
  // On, a fault with nothing to change has nothing more to apply.
  parts.apply.hidden = parts.on && parts.params.length === 0;
  parts.apply.disabled = parts.undo.disabled = locked || parts.busy;
}

/** Applies a fault with the values typed, each checked against its limits here first. */
async function applyFault(parts) {
  if (locked || parts.busy) return;
  const values = {};
  for (const parameter of parts.params) {
    const text = parameter.input.value.trim();
    const number = Number(text);
    if (text === "" || !Number.isFinite(number)) {
      note(parts.message, "✗ " + parameter.name + " takes a number.", true);
      parameter.input.focus();
      return;
    }
    if (number < parameter.min || number > parameter.max) {
      note(parts.message, "✗ " + parameter.name + " is from " + parameter.range
          + (parameter.unit ? " " + parameter.unit : "") + ". Checked here, before sending.", true);
      parameter.input.focus();
      return;
    }
    values[parameter.name] = number;
  }
  await act(parts, { component: parts.component, fault: parts.name, values },
    parts.component + ": " + parts.name + (parts.lasting ? ", on." : ", done."));
}

async function undoFault(parts) {
  if (locked || parts.busy) return;
  await act(parts, { component: parts.component, fault: parts.name, undo: true },
    parts.component + ": " + parts.name + ", undone.");
}

/**
 * Applies or undoes a fault, saying what became of it under its row. An event says, for a while,
 * that it happened, since nothing else on the page changes.
 */
async function act(parts, body, done) {
  parts.busy = true;
  updateButtons(parts);
  const result = await send("POST", "/api/simulator/faults", body);
  parts.busy = false;
  updateButtons(parts);
  outcome(parts.message, result,
    { done, shown: !parts.lasting && !body.undo, paused: context.paused() });
}

async function undoAll() {
  if (locked || undoing) return;
  undoing = true;
  undoAllButton.disabled = true;
  try {
    const result = await send("DELETE", "/api/simulator/faults");
    outcome(undoAllMessage, result,
      { done: "Every fault undone: the robot as built.", paused: context.paused() });
  } finally {
    undoing = false;
    const now = context.state();
    if (now && now.components) renderFaults(now.components); // Undo all's count, and whether any
    else undoAllButton.disabled = locked;
  }
}

// ---- Scenarios ----

function makeScenario(name, scenario) {
  const parts = { busy: false, list: h("p", { class: "sfaults" }),
                  message: h("p", { class: "message", hidden: true }) };
  parts.apply = h("button", { type: "button", "aria-label": "Apply " + name,
                              onclick: () => applyScenario(name, parts) }, "Apply");
  parts.el = h("div", { class: "scen", role: "group", "aria-label": name },
    h("div", { class: "fhead" }, h("b", { class: "mono grow" }, name), parts.apply),
    h("p", { class: "does" }, scenario.description), parts.list, parts.message);
  scenarioRows.set(name, parts);
  return parts.el;
}

async function applyScenario(name, parts) {
  if (locked || parts.busy) return;
  parts.busy = true;
  parts.apply.disabled = true;
  const result = await send("POST", "/api/simulator/scenarios", { apply: name });
  parts.busy = false;
  parts.apply.disabled = locked;
  // What it did, as the answer says: which faults it put on, which happened, and which it passed
  // over, the part being that way as built. Kept under it, beside the faults it marks as on.
  const now = context.state();
  const scenario = now && now.scenarios ? now.scenarios[name] : null;
  const said = result.ok && result.status === 200 && scenario ? didWhat(scenario, result.body) : "";
  const done = name + " applied" + (said ? ": " + said : ".");
  if (outcome(parts.message, result, { done, paused: context.paused() })) {
    note(parts.message, "✓ " + done);
  }
}

function didWhat(scenario, components) {
  const on = [];
  const happened = [];
  const passed = [];
  for (const setting of scenario.faults) {
    const fault = faultOf(setting, components || {});
    if (!fault) continue;
    const name = setting.component + ": " + setting.fault;
    if (!fault.lasting) happened.push(name);
    else if (fault.on) on.push(name);
    else passed.push(name);
  }
  return [on.length > 0 && "on now, " + on.join(", "),
    happened.length > 0 && "happened, " + happened.join(", "),
    passed.length > 0 && "passed over, as built, " + passed.join(", ")]
    .filter(Boolean).join("; ");
}

/**
 * A scenario's fault as people read it, its values with their units, those at zero left out:
 * "Camera front-left: Moved on its mount, Up -4 in".
 */
function settingText(setting, components) {
  const fault = faultOf(setting, components);
  const units = new Map((fault ? fault.parameters : []).map((p) => [p.name, p.unit]));
  const given = Object.entries(setting.values || {});
  const values = given.filter(([, value]) => value !== 0 || given.length === 1)
    .map(([name, value]) => name + " " + shown(value) + (units.get(name) ? " " + units.get(name)
      : ""));
  return setting.component + ": " + setting.fault + (values.length > 0 ? ", " + values.join(", ")
    : "");
}

function faultOf(setting, components) {
  return ((components[setting.component] || {}).faults || {})[setting.fault];
}

function isOn(setting, components) {
  const fault = faultOf(setting, components);
  return Boolean(fault && fault.lasting && fault.on);
}
