// Sim: the simulator's Driver Station, its joysticks, its clock, and what fails, in place of
// WPILib's simulator GUI. Shown only in the simulator: a robot program that isn't simulated never
// offers it, and has none of the routes it uses (docs/simulator.md).
//
// A view is a module whose default export is {id, title, mount(element), show(), hide(), route(),
// offered()}: mount builds it once, into its own section, and it then follows the robot through
// core.js's onChange; show and hide (optional) say when it's on screen; route (optional) takes the
// rest of the address, after "#sim/"; offered (optional) says whether the view is offered now,
// where simulatorOnly isn't enough to tell.
//
// The robot program sees a Driver Station change as its next loop begins, as from a real one; this
// view shows it as the simulator answers. A paused simulator runs no loops, so it sends nothing
// new: a page opened while it's paused gets no snapshot, and asks the simulator directly instead.
//
// Esc disables the robot in Sim, always, even in a field; and from any other view while this page
// drives it (the keyboard or a gamepad), outside a field, where Esc drops what was typed.
// Elsewhere Esc keeps its meaning: it stops a run or an action, and with nothing running drops a
// value's edit. Disabling there would turn "stop this run" into "stop it and enable again", since
// a run needs the robot enabled.

import {
  robot, onChange, onConnection, send, h, replace, announce, say, confirm, typing, stopping, shown,
} from "./core.js";
import { showDriving } from "./bar.js";
import * as drive from "./sim-drive.js";
import { makeFaults, renderFaults, renderScenarios, lockFaults } from "./sim-faults.js";
import { note, outcome, loopRan } from "./sim-outcome.js";

/** The sections of the robot's state this view reads. */
const READS = ["simulator", "components", "scenarios"];

const MODES = [["AUTONOMOUS", "Autonomous"], ["TELEOPERATED", "Teleoperated"],
  ["UTILITY", "Utility"]];
const STATIONS = ["RED_1", "RED_2", "RED_3", "BLUE_1", "BLUE_2", "BLUE_3"];

/**
 * The longest match time the simulator takes, in seconds: room for any match period. The server's
 * is PitwallServer's MAX_MATCH_SECONDS, which decides; this only checks before sending.
 */
const MAX_MATCH_SECONDS = 600;

let visible = false;
/**
 * The simulator as GET /api/simulator answered, used only while no snapshot has come: a page that
 * opened while the simulator was paused gets none until a loop runs.
 */
let fallback = null;
let lastEnabled = null;
let hiddenAt = 0; // when the tab was last hidden while this page drove, for the note on return
let driveStatus = drive.status();
let drawnDrive = ""; // what renderDrive last drew, beside the joysticks' axes
const ui = {};

function mount(view) {
  ui.word = h("span", { class: "word" });
  ui.sub = h("span", { class: "sub" });
  ui.paused = h("span", { class: "tag paused", hidden: true }, "❚❚ PAUSED");
  ui.state = h("div", { class: "dsstate", id: "sim-state" },
    h("span", { class: "line" }, ui.word, ui.paused), ui.sub);
  ui.enable = h("button", { type: "button", class: "big primary enable", onclick: enable,
                            "aria-describedby": "sim-state sim-enable-why" }, "▶ Enable");
  ui.disable = h("button", { type: "button", class: "big danger disable", onclick: disable,
                             "aria-describedby": "sim-state", "aria-keyshortcuts": "Escape" },
    "■ Disable", h("kbd", null, "Esc"));
  ui.enableWhy = h("p", { class: "gate locked", id: "sim-enable-why", hidden: true });
  ui.notice = h("div", { class: "banner lock", hidden: true });
  view.append(
    h("h1", { class: "view-title" }, "Sim"),
    h("p", { class: "lede" }, "Drive the simulator, change its Driver Station and its clock, and "
        + "fail its parts. WPILib's simulator window isn't needed: ./gradlew run -Pgui opens it, "
        + "and while it's open it overwrites these joysticks every frame."),
    h("div", { class: "dsbar" }, ui.state,
      h("div", { class: "dsbuttons" }, ui.enable, ui.disable), ui.enableWhy),
    ui.notice,
    h("div", { class: "sim" },
      h("div", null, stationCard(), timeCard()),
      h("div", null, driveCard())),
    makeFaults({ state, paused: isPaused }));
  onChange((patch, snapshot) => {
    if (snapshot) fallback = null;
    // Anything from the robot means a loop ran: what waited for one is done.
    loopRan();
    // Only what it reads: the status and the values change often, and say nothing here.
    if (!snapshot && !READS.some((section) => section in patch)) return;
    noteEnabled(snapshot);
    if (!visible) return;
    if (snapshot || "simulator" in patch) renderStation();
    if (snapshot || "components" in patch || "scenarios" in patch) renderFaultsAndScenarios();
  });
  onConnection((connected) => {
    if (visible) renderAll();
    if (connected && !robot.doc) askSoon();
  });
  drive.onDrive((status) => {
    if (driveStatus.keyboard.on || driveStatus.gamepad.on) {
      if (status.hidden && !driveStatus.hidden) hiddenAt = Date.now();
    }
    driveStatus = status;
    showPill(status);
    if (visible) renderDrive();
  });
  document.addEventListener("keydown", escape);
  askSoon();
}

function show() {
  visible = true;
  renderAll();
}

function hide() {
  visible = false;
}

/**
 * Offered where the robot is simulated and has the simulator's controls, and before any snapshot
 * once a paused simulator has answered.
 */
function offered() {
  const doc = robot.doc;
  return Boolean(doc ? doc.robot.simulated && doc.simulator : fallback);
}

// ---- The state ----

/** The simulator's sections, from the stream; or before a snapshot, as the simulator answered. */
function state() {
  return robot.doc && robot.doc.simulator ? robot.doc : fallback;
}

function sim() {
  const now = state();
  return now ? now.simulator : null;
}

function isPaused() {
  return Boolean(sim() && sim().paused);
}

/** Whether anything here can be asked of the simulator now. */
function canAct() {
  return !robot.stopped && state() !== null
      && (robot.connected === true || (robot.connected === null && fallback !== null));
}

/**
 * A page opened while the simulator is paused gets no snapshot: after a second and a half without
 * one, it asks the simulator, which answers at once while paused. Only a paused simulator's answer
 * stands in for a snapshot: anything else (a slow loop, a robot) waits for the stream.
 */
function askSoon() {
  setTimeout(async () => {
    if (robot.doc) return;
    const result = await send("GET", "/api/simulator");
    const answer = result.ok && result.body ? result.body.simulator : null;
    if (robot.doc || !answer || answer.paused !== true || !("enabled" in answer)) return;
    fallback = result.body;
    document.dispatchEvent(new Event("pitwall-offered"));
    if (visible) renderAll();
  }, 1500);
}

/** Says when the robot is enabled or disabled, whoever did it: a disable at once. */
function noteEnabled(snapshot) {
  const now = sim();
  if (!now) return;
  if (!snapshot && lastEnabled !== null && now.enabled !== lastEnabled) {
    if (now.enabled) announce("Enabled: " + modeName(now.mode) + ".");
    else announce("Disabled.", true);
  }
  lastEnabled = now.enabled;
}

function modeName(mode) {
  const found = MODES.find(([name]) => name === mode);
  return found ? found[1] : "no mode";
}

/** "RED_1" as people say it: "Red 1". */
function stationName(station) {
  if (!STATIONS.includes(station)) return "no alliance station";
  const [alliance, number] = station.split("_");
  return capital(alliance.toLowerCase()) + " " + number;
}

function capital(text) {
  return text.charAt(0).toUpperCase() + text.slice(1);
}

// ---- Requests ----

/**
 * Changes the Driver Station or the clock, saying what became of it where it was asked (see
 * sim-outcome.js).
 */
async function patch(body, message) {
  const result = await send("PATCH", "/api/simulator", body);
  answered(result, message);
  return result;
}

function answered(result, message) {
  outcome(message, result, { paused: isPaused() });
  if (result.ok && result.status === 200 && fallback && !robot.doc && result.body
      && "enabled" in result.body) {
    fallback.simulator = result.body; // no stream to say so until a loop runs
    renderAll();
  }
}

async function enable() {
  const now = sim();
  if (!now || now.mode === "UNKNOWN") return;
  await patch({ enabled: true }, ui.dsMessage);
}

/** Disables the robot. Failing to is said loudly, wherever it was asked from. */
async function disable() {
  const result = await patch({ enabled: false }, ui.dsMessage);
  if (!result.ok) {
    say("The simulator didn't disable: " + (result.problem.detail || result.problem.title), true,
      true);
  }
}

/** Esc disables: in Sim always, and anywhere while this page drives, outside a field. */
function escape(event) {
  if (event.key !== "Escape" || event.defaultPrevented || document.querySelector("dialog[open]")) {
    return;
  }
  const now = sim();
  if (!now || !now.enabled) return; // nothing to disable: Esc keeps its other meanings
  const driving = driveStatus.inputs.some(Boolean) && !typing(event.target);
  if (!visible && !driving) return;
  event.preventDefault();
  disable();
}

// ---- The Driver Station ----

function stationCard() {
  ui.modes = segmented("Mode", MODES.map(([name, label]) => [name, label]),
    (mode) => patch({ mode }, ui.dsMessage));
  ui.red = segmented("Red alliance", STATIONS.slice(0, 3).map((s) => [s, "Red " + s.slice(-1)]),
    (station) => patch({ allianceStation: station }, ui.dsMessage));
  ui.blue = segmented("Blue alliance", STATIONS.slice(3).map((s) => [s, "Blue " + s.slice(-1)]),
    (station) => patch({ allianceStation: station }, ui.dsMessage));
  ui.ds = toggle("Driver Station attached", () => patch({ dsAttached: !sim().dsAttached },
    ui.dsMessage));
  ui.fms = toggle("FMS attached", () => setField(!sim().fmsAttached));
  ui.matchNow = h("b", { class: "num" });
  ui.matchHow = h("span", { class: "meta" });
  ui.matchInput = h("input", { type: "text", inputmode: "numeric", autocomplete: "off",
    class: "pval", placeholder: "seconds", "aria-label": "Match time to set, in seconds, 0 to "
        + MAX_MATCH_SECONDS });
  ui.matchInput.addEventListener("keydown", (event) => {
    if (event.key === "Enter") {
      event.preventDefault();
      setMatchTime();
    }
  });
  ui.matchSet = h("button", { type: "button", onclick: setMatchTime }, "Set");
  ui.matchClear = h("button", { type: "button", onclick: () => patch({ matchTime: null },
    ui.dsMessage) }, "Clear");
  ui.dsMessage = h("p", { class: "message", hidden: true });
  return h("section", { class: "card" }, h("h2", null, "Driver Station"),
    h("div", { class: "simrow" }, h("span", { class: "label" }, "Mode"), ui.modes.el),
    h("div", { class: "simrow" }, h("span", { class: "label" }, "Alliance station"),
      h("span", { class: "stations" }, ui.red.el, ui.blue.el)),
    h("div", { class: "simrow" }, ui.ds.el),
    h("div", { class: "simrow" }, ui.fms.el,
      h("span", { class: "meta" }, "On the field, as at an event: tuning, actions, and experiments "
          + "lock, and tuned values go back to committed. Off here unlocks them.")),
    h("div", { class: "simrow" }, h("span", { class: "label" }, "Match time"), ui.matchNow,
      ui.matchHow),
    h("div", { class: "simrow" }, h("span", { class: "setter" }, ui.matchInput, ui.matchSet),
      ui.matchClear),
    ui.dsMessage);
}

/** Buttons for one choice of several, the chosen one pressed. */
function segmented(label, options, choose) {
  const buttons = new Map();
  const el = h("span", { class: "seg", role: "group", "aria-label": label },
    options.map(([value, text]) => {
      const button = h("button", { type: "button", "aria-pressed": "false",
                                   onclick: () => choose(value) }, text);
      buttons.set(value, button);
      return button;
    }));
  return { el, buttons };
}

function toggle(label, change) {
  const state = h("span");
  const el = h("button", { type: "button", role: "switch", class: "switch", "aria-checked": "false",
                           "aria-label": label, onclick: change },
    h("span", { class: "knob" }), h("span", null, label), state);
  return { el, state };
}

/** Attaching the field returns every tuned value to committed: asked first, listing them. */
async function setField(on) {
  const doc = robot.doc;
  const tuned = on && doc ? Object.keys(doc.changes || {}) : [];
  if (tuned.length > 0) {
    const yes = await confirm("Attach the field system?", [
      h("p", null, "As at an event, tuning, actions, and experiments lock, and "
          + (tuned.length === 1 ? "this tuned value goes" : "these " + tuned.length
            + " tuned values go") + " back to committed:"),
      tunedList(doc, tuned),
      h("p", { class: "meta" }, "What was tuned is printed to the log, and this session's history "
          + "keeps it: Restore, in Changes, stages it again."),
    ], "Attach the field");
    if (!yes) return;
  }
  await patch({ fmsAttached: on }, ui.dsMessage);
}

/** Tuned values as a list: each by its configuration and path, now and committed. */
function tunedList(doc, paths) {
  return h("ul", null, paths.map((path) => {
    const value = doc.tunables[path];
    return h("li", null, value ? value.config + " " + value.field + ": " + shown(doc.values[path])
        + " → " + shown(value.committed) : path);
  }));
}

async function setMatchTime() {
  const text = ui.matchInput.value.trim();
  const seconds = Number(text);
  if (text === "" || !Number.isFinite(seconds) || seconds < 0 || seconds > MAX_MATCH_SECONDS) {
    note(ui.dsMessage, "✗ A match time is seconds from 0 to " + MAX_MATCH_SECONDS
        + ". Checked here, before sending.", true);
    return;
  }
  const result = await patch({ matchTime: seconds }, ui.dsMessage);
  if (result.ok) ui.matchInput.value = "";
}

/** The time as a Driver Station shows it: 2:15. */
function clock(seconds) {
  const whole = Math.max(0, Math.round(seconds));
  return Math.floor(whole / 60) + ":" + String(whole % 60).padStart(2, "0");
}

function renderStation() {
  const now = sim();
  const act = canAct();
  const stopped = robot.stopped && robot.connected !== true;
  ui.state.className = "dsstate" + (stopped ? " stopped" : now && now.enabled ? " enabled" : "");
  if (stopped) {
    ui.word.textContent = "■ STOPPED";
    ui.sub.textContent = "This page stopped the simulator. Start it again (./gradlew run), and "
        + "this page reconnects by itself.";
  } else if (!now) {
    ui.word.textContent = "…";
    ui.sub.textContent = "Waiting for the simulator.";
  } else {
    ui.word.textContent = now.enabled ? "● ENABLED" : "○ DISABLED";
    ui.sub.textContent = [capital(modeName(now.mode)), capital(stationName(now.allianceStation)),
      now.fmsAttached ? "on the field (FMS)" : null,
      !now.dsAttached ? "no Driver Station" : null,
      now.matchTime >= 0 ? clock(now.matchTime) + " left" : null].filter(Boolean).join(" · ");
  }
  ui.paused.hidden = !now || !now.paused || stopped;
  // Enable needs a mode, as a Driver Station always has one; Disable is never locked but by a lost
  // link, and pressing it again changes nothing.
  const noMode = Boolean(now) && now.mode === "UNKNOWN";
  ui.enable.disabled = !act || !now || now.enabled || noMode;
  ui.enableWhy.hidden = !noMode || !act;
  ui.enableWhy.textContent = "🔒 Choose a mode to enable it in.";
  ui.disable.disabled = !act;
  ui.disable.classList.toggle("armed", Boolean(now && now.enabled));
  ui.notice.hidden = !(fallback && !robot.doc);
  replace(ui.notice, h("span", { class: "grow" }, "❚❚ ",
    h("b", null, "The simulator is paused, "),
    h("span", { class: "why" }, "and this page opened while it was: the rest of the page fills in "
        + "when a loop runs. Step or Resume, under Time.")));
  if (!now) return;
  for (const [mode, button] of ui.modes.buttons) {
    button.setAttribute("aria-pressed", String(now.mode === mode));
    button.disabled = !act;
  }
  for (const seg of [ui.red, ui.blue]) {
    for (const [station, button] of seg.buttons) {
      button.setAttribute("aria-pressed", String(now.allianceStation === station));
      button.disabled = !act;
    }
  }
  for (const [part, on] of [[ui.ds, now.dsAttached], [ui.fms, now.fmsAttached]]) {
    part.el.setAttribute("aria-checked", String(on));
    part.state.textContent = on ? " · on" : " · off";
    part.el.disabled = !act;
  }
  ui.matchNow.textContent = now.matchTime >= 0 ? clock(now.matchTime) : "none";
  ui.matchHow.textContent = now.matchTime < 0 ? "set one to count it down while enabled"
    : now.matchTime === 0 ? "run out" : now.enabled ? "counting down"
    : "counts down while enabled";
  ui.matchSet.disabled = !act;
  ui.matchClear.disabled = !act || now.matchTime < 0;
  renderTime();
  renderJoysticks();
}

// ---- Time ----

function timeCard() {
  ui.timeState = h("p", { class: "state" });
  ui.pause = h("button", { type: "button", onclick: pauseOrResume });
  ui.step = h("button", { type: "button", onclick: step, "aria-describedby": "sim-step-why" },
    "Step one loop");
  ui.stepWhy = h("span", { class: "meta", id: "sim-step-why" });
  ui.stop = h("button", { type: "button", class: "danger", onclick: stop },
    "Stop the simulator…");
  ui.timeMessage = h("p", { class: "message", hidden: true });
  return h("section", { class: "card" }, h("h2", null, "Time"), ui.timeState,
    h("div", { class: "acts" }, ui.pause, ui.step, ui.stepWhy),
    ui.timeMessage,
    h("div", { class: "acts stoprow" }, ui.stop,
      h("span", { class: "meta" }, "Ends the robot program, as closing WPILib's window did.")));
}

function renderTime() {
  const now = sim();
  const act = canAct();
  const paused = Boolean(now && now.paused);
  ui.timeState.textContent = !now ? "" : paused
    ? "❚❚ Paused: no loops run, so the robot's clock stands still. What's asked waits for a "
      + "Step or Resume."
    : "▶ Running, in step with this computer's clock.";
  ui.pause.textContent = paused ? "▶ Resume" : "❚❚ Pause";
  ui.pause.className = paused ? "primary" : "";
  ui.pause.disabled = !act;
  ui.step.disabled = !act || !paused;
  ui.stepWhy.textContent = paused ? "" : "Step runs one loop of a paused simulator.";
  ui.stop.disabled = !act;
}

async function pauseOrResume() {
  const now = sim();
  if (!now) return;
  await patch({ paused: !now.paused }, ui.timeMessage);
}

async function step() {
  const result = await send("POST", "/api/simulator/step");
  answered(result, ui.timeMessage);
}

/** Stops the simulator, after asking: it ends the robot program, and what wasn't committed. */
async function stop() {
  const doc = robot.doc;
  const tuned = doc ? Object.keys(doc.changes || {}) : [];
  const yes = await confirm("Stop the simulator?", [
    h("p", null, "The robot program ends, as closing WPILib's simulator window did. AdvantageKit's "
        + "log keeps its last loops. Start it again with ./gradlew run; this page reconnects by "
        + "itself."),
    tuned.length > 0 && [
      h("p", { class: "message bad" }, "● " + (tuned.length === 1 ? "This tuned value isn't"
        : "These " + tuned.length + " tuned values aren't") + " committed, and "
          + (tuned.length === 1 ? "ends" : "end") + " with the program:"),
      tunedList(doc, tuned),
      h("p", { class: "meta" }, "Changes has the lines that commit "
          + (tuned.length === 1 ? "it." : "them.")),
    ],
  ], "Stop the simulator");
  if (!yes) return;
  stopping(); // its stream ending is the program ending, not a lost link
  drive.release();
  const result = await send("POST", "/api/simulator/stop");
  if (!result.ok) {
    stopping(false);
    answered(result, ui.timeMessage);
    return;
  }
  announce("Stopping the simulator.", true);
}

// ---- Driving ----

function driveCard() {
  ui.keys = toggle("Drive with the keyboard", () => drive.setKeyboard(!driveStatus.keyboard.on));
  ui.keyState = h("p", { class: "state" });
  ui.pad = toggle("Drive with a gamepad", () => drive.setGamepad(!driveStatus.gamepad.on));
  ui.padState = h("div", { class: "state" });
  ui.slot = segmented("The first gamepad drives", [[0, "Joystick 0 · driver"],
    [1, "Joystick 1 · operator"]], (slot) => drive.setSlot(slot));
  ui.hiddenNote = h("div", { class: "banner warn", hidden: true },
    h("span", { class: "grow" }, "While this tab was hidden, it didn't drive: its joysticks were "
        + "centered. To drive while you watch AdvantageScope, put the two windows side by side, "
        + "not one over the other."),
    h("button", { type: "button", class: "small", onclick: () => {
      hiddenAt = 0;
      renderDrive();
    } }, "OK"));
  ui.driveMessage = h("p", { class: "message bad", hidden: true });
  ui.joysticks = [makeJoystick(0), makeJoystick(1)];
  return h("section", { class: "card" }, h("h2", null, "Drive"),
    ui.hiddenNote,
    h("div", { class: "simrow" }, ui.keys.el), ui.keyState,
    h("table", { class: "keymap" }, h("tbody", null, drive.LAYOUT.map(([keys, does]) =>
      h("tr", null, h("td", null, keys.map((key, i) => [i > 0 ? " " : "", h("kbd", null, key)])),
        h("td", null, does))))),
    h("p", { class: "meta" }, "Keys type as usual in a field, and drive again outside it."),
    h("div", { class: "simrow" }, ui.pad.el), ui.padState,
    h("div", { class: "simrow" }, h("span", { class: "label" }, "The first gamepad drives"),
      ui.slot.el),
    h("p", { class: "meta" }, "A second gamepad drives the other joystick."),
    ui.driveMessage,
    ui.joysticks.map((joystick) => joystick.el));
}

function makeJoystick(port) {
  const parts = { axes: [], buttonsShown: "" };
  parts.source = h("span", { class: "meta" });
  parts.flag = h("p", { class: "meta" });
  parts.pressed = h("p", { class: "pressed" });
  parts.axesEl = h("div", { class: "axes" }, drive.AXIS_NAMES.map((name) => {
    const fill = h("span", { class: "fill" });
    const value = h("span", { class: "num v" }, "0.00");
    parts.axes.push({ fill, value, last: null });
    return h("div", { class: "axis" }, h("span", { class: "an" }, name),
      h("span", { class: "track", "aria-hidden": "true" }, fill), value);
  }));
  parts.el = h("div", { class: "joy", role: "group", "aria-label": "Joystick " + port },
    h("h3", null, "Joystick " + port + (port === 0 ? " · the driver's" : " · the operator's"),
      " ", parts.source),
    parts.flag, parts.axesEl, parts.pressed);
  return parts;
}

/**
 * The driving card. A gamepad changes it every frame, but only its joysticks' axes and buttons:
 * the rest is drawn only when it changed.
 */
function renderDrive() {
  const status = driveStatus;
  const key = JSON.stringify([status.keyboard, status.gamepad, status.hidden, status.problem,
    hiddenAt]);
  if (key !== drawnDrive) {
    drawnDrive = key;
    ui.keys.el.setAttribute("aria-checked", String(status.keyboard.on));
    ui.keys.state.textContent = status.keyboard.on ? " · on" : " · off";
    ui.keyState.className = "state"
        + (status.keyboard.on && status.keyboard.paused ? " paused" : "");
    ui.keyState.textContent = keyText(status);
    ui.pad.el.setAttribute("aria-checked", String(status.gamepad.on));
    ui.pad.state.textContent = status.gamepad.on ? " · on" : " · off";
    replace(ui.padState, padText(status));
    for (const [slot, button] of ui.slot.buttons) {
      button.setAttribute("aria-pressed", String(status.gamepad.slot === slot));
    }
    ui.hiddenNote.hidden = status.hidden || hiddenAt === 0
        || !(status.keyboard.on || status.gamepad.on);
    ui.driveMessage.hidden = !status.problem;
    ui.driveMessage.textContent = status.problem ? "✗ " + status.problem : "";
  }
  renderJoysticks();
}

function keyText(status) {
  if (!status.keyboard.on) return "Off: keys type as usual.";
  switch (status.keyboard.paused) {
    case "hidden": return "Not driving: this tab is hidden.";
    case "window": return "Paused: this window doesn't have the keyboard. Click the page to drive.";
    case "field": return "Paused: the focus is in a field, so keys type there. Click outside it "
        + "to drive.";
    default: return "Driving joystick 0: hold the keys below. Esc disables the robot, from any "
        + "view.";
  }
}

function padText(status) {
  if (!status.gamepad.on) return "Off.";
  if (status.hidden) {
    return "Not driving: this tab is hidden, and a hidden tab can't read a gamepad. Put Pitwall "
        + "beside AdvantageScope, not behind it.";
  }
  if (status.gamepad.pads.length === 0) {
    return "No gamepad yet: press a button on it. A browser shows a page its gamepads only after "
        + "a button press.";
  }
  return h("ul", { class: "pads" }, status.gamepad.pads.map((pad) => h("li", null,
    h("span", { class: "mono" }, pad.id.length > 48 ? pad.id.slice(0, 47) + "…" : pad.id), ": ",
    pad.drives !== null ? "drives joystick " + pad.drives
      : pad.standard ? "not driving: two gamepads drive at most"
      : h("span", { class: "bad" }, "✗ this browser doesn't know its layout (no standard "
          + "mapping), so it can't drive. Chrome knows most gamepads."))));
}

/**
 * Each joystick as this page drives it: its axes and buttons, and whether the robot centered it.
 */
function renderJoysticks() {
  const now = sim();
  for (const [port, parts] of ui.joysticks.entries()) {
    const input = driveStatus.inputs[port];
    const fromServer = now && now.joysticks ? now.joysticks[String(port)] : null;
    // Another page's input was the latest the robot took: two pages drive this joystick.
    const another = Boolean(canAct() && fromServer && fromServer.fromPage && fromServer.stream
        && fromServer.stream !== robot.stream);
    const sources = [port === 0 && driveStatus.keyboard.on && "the keyboard",
      driveStatus.gamepad.on && driveStatus.gamepad.pads.some((pad) => pad.drives === port)
        && "a gamepad"].filter(Boolean);
    const source = input ? "· from " + sources.join(" and ") : "";
    if (parts.source.textContent !== source) parts.source.textContent = source;
    parts.axesEl.hidden = !input;
    parts.pressed.hidden = !input;
    parts.el.classList.toggle("idle", !input);
    let flag;
    let bad = false;
    if (input && another) {
      bad = true;
      flag = "✗ Another Pitwall page drives this joystick too: the robot takes whichever input "
          + "came last. Switch driving off in one of them.";
    } else if (input && fromServer && fromServer.centeredByDeadman) {
      bad = true;
      flag = "✗ Centered by the robot: it heard nothing from this page for half a second (the "
          + "deadman). It drives again with the next input.";
    } else if (input) {
      flag = "";
    } else if (another) {
      flag = "Driven from another Pitwall page.";
    } else {
      flag = "Not driven from this page: a Driver Station or WPILib's simulator window may drive "
          + "it.";
    }
    if (parts.flag.textContent !== flag) {
      parts.flag.className = bad ? "message bad" : "meta";
      parts.flag.textContent = flag;
    }
    if (!input) continue;
    input.axes.forEach((value, axis) => {
      const shownAxis = parts.axes[axis];
      if (!shownAxis || shownAxis.last === value) return;
      shownAxis.last = value;
      // Drawn from the middle: left for negative, right for positive.
      shownAxis.fill.style.left = (50 + Math.min(0, value) * 50) + "%";
      shownAxis.fill.style.width = (Math.abs(value) * 50) + "%";
      shownAxis.value.textContent = value.toFixed(2);
    });
    const pressed = input.buttons.map((down, button) => (down ? button : -1))
      .filter((button) => button >= 0)
      .map((button) => (drive.BUTTON_NAMES[button] || "Button") + " (" + button + ")");
    const text = pressed.length === 0 ? "No buttons pressed." : "Pressed: " + pressed.join(", ");
    if (text !== parts.buttonsShown) {
      parts.buttonsShown = text;
      parts.pressed.textContent = text;
    }
  }
}

/**
 * The state bar's word on driving: ⌨ for the keys, 🎮 for a gamepad, and why not when it doesn't.
 */
function showPill(status) {
  const marks = [status.keyboard.on && "⌨", status.gamepad.on && "🎮"].filter(Boolean)
    .join(" ");
  if (!marks) {
    showDriving(null, false, false);
    return;
  }
  const keysOnly = status.keyboard.on && !status.gamepad.on;
  const why = status.hidden ? "Not driving: tab hidden"
    : keysOnly && status.keyboard.paused === "field" ? "Paused: typing"
    : keysOnly && status.keyboard.paused === "window" ? "Paused: window"
    : null;
  showDriving(marks + " " + (why || "Driving"), !why, status.hidden);
}

// ---- Everything ----

function renderFaultsAndScenarios() {
  const now = state();
  lockFaults(!canAct());
  renderFaults(now ? now.components : {});
  renderScenarios(now ? now.scenarios : {}, now ? now.components : {});
}

function renderAll() {
  renderStation();
  renderDrive();
  renderFaultsAndScenarios();
}

export default { id: "sim", title: "Sim", simulatorOnly: true, mount, show, hide, offered };
