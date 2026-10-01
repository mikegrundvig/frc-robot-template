// Driving the simulated robot from this page, in place of WPILib's simulator GUI: the keyboard,
// once switched on, and gamepads through the browser's Gamepad API, as WPILib's joysticks 0 (the
// driver's) and 1 (the operator's). Nothing here draws; the Sim view and the state bar show what
// it says (onDrive).
//
// What it sends, and when (docs/simulator.md): each joystick's whole input, as WPILib's Gamepad
// (org.wpilib.driverstation.Gamepad: 6 axes, 26 buttons, 1 POV), when it changes, at most once a
// frame, and again every 200 ms while anything is pressed, naming this page's stream. The robot
// centers a joystick whose page goes quiet for 500 ms of its own time (the deadman), and lets it go
// when the page's stream closes. A joystick this page never drove is never touched.
//
// Keys reach only a focused page, so keyboard driving pauses, centered, whenever the focus is in a
// field (typing "w" into a value never drives), the window loses the keyboard, or the tab is
// hidden. A hidden tab can't be trusted to read a gamepad either (browsers throttle it), so the
// page polls gamepads only while it's visible, and centers its joysticks when it's hidden. Nothing
// here listens to anything until driving is first switched on, which only Sim, in the simulator,
// can do: on a robot it never runs.

import { robot, api, onConnection, typing } from "./core.js";

/** WPILib's Gamepad's axes, by index. Sticks are -1 to 1, negative pushed up or left. */
export const AXIS_NAMES = ["Left X", "Left Y", "Right X", "Right Y", "Left trigger",
  "Right trigger"];

/** Its buttons, by index, as far as a standard gamepad has them (it declares 26). */
export const BUTTON_NAMES = ["Face down", "Face right", "Face left", "Face up", "Back", "Guide",
  "Start", "Left stick", "Right stick", "Left bumper", "Right bumper", "D-pad up", "D-pad down",
  "D-pad left", "D-pad right"];

const AXES = 6;
const BUTTONS = 26;

/** How often each joystick's input that isn't centered is sent again: well inside 500 ms. */
const REPEAT_MS = 200;

/** How soon input the robot didn't answer is sent again. */
const RETRY_MS = 100;

/**
 * Keys that can swallow another key's release: on macOS a key let go while Cmd is down never says
 * so, and Alt can take the keyboard to a menu. Pressed or let go, every key counts as released.
 */
const MODIFIERS = new Set(["Meta", "Alt", "OS"]);

/**
 * The keyboard's one layout, for this template's driving, by physical key (KeyboardEvent.code, so
 * it's where WASD is on any layout): W and S the left stick's Y, A and D its X, J and L or the
 * arrows the right stick's X, and R the Back button, which resets the heading.
 */
const KEYS = {
  KeyW: { axis: 1, value: -1 }, KeyS: { axis: 1, value: 1 },
  KeyA: { axis: 0, value: -1 }, KeyD: { axis: 0, value: 1 },
  KeyJ: { axis: 2, value: -1 }, KeyL: { axis: 2, value: 1 },
  ArrowLeft: { axis: 2, value: -1 }, ArrowRight: { axis: 2, value: 1 },
  KeyR: { button: 4 },
};

/** The layout as the page lists it: keys, then what they do. */
export const LAYOUT = [
  [["W", "S"], "Left stick forward, back (axis 1)"],
  [["A", "D"], "Left stick left, right (axis 0)"],
  [["J", "L"], "Right stick left, right (axis 2): turn"],
  [["←", "→"], "The same"],
  [["R"], "Back (button 4): the template resets the heading"],
];

/**
 * A standard gamepad (W3C) to WPILib's: each button of the standard layout to WPILib's button,
 * with the triggers (6, 7) as axes 4 and 5, 0 to 1. Axes 0 to 3 are the same, with the same sign.
 */
const PAD_BUTTONS = [[0, 0], [1, 1], [2, 2], [3, 3], [4, 9], [5, 10], [8, 4], [9, 6], [10, 7],
  [11, 8], [12, 11], [13, 12], [14, 13], [15, 14], [16, 5]];
const PAD_TRIGGERS = [[6, 4], [7, 5]];

/** Where the joystick the first gamepad drives is kept: the second drives the other. */
const SLOT_KEY = "pitwall.gamepadSlot";

const keyboard = { on: false, held: new Set() };
const gamepad = { on: false, pads: [] };
const listeners = [];
let slot = storedSlot();
let armed = false;
let hidden = false;
let focused = true;
let inField = false;

// Each joystick: what was sent (its input as JSON, or null), when (its own repeat clock), and
// whether this page drives it now, as the robot last heard.
const sent = [null, null];
const sentAt = [0, 0];
const claimed = [false, false];
let sentStream = "";
let inFlight = false;
let again = false;
let retrying = 0;
let repeating = 0;
let framePending = false;
let polling = 0;
let problem = null;
let inputs = [null, null];
let told = "";

/** Calls {@code listener(status)} whenever what this page drives, or how, changes. */
export function onDrive(listener) {
  listeners.push(listener);
}

/**
 * What this page drives now: {keyboard: {on, paused}, gamepad: {on, slot, pads: [{id, standard,
 * drives}]}, hidden, inputs: [input or null, for joystick 0 and 1], problem}. {@code paused} says
 * why the keys don't drive now ("field", "window", "hidden"), or is null; a gamepad {@code drives}
 * joystick 0 or 1, or null when it can't drive. A joystick's input isn't null while this page
 * drives it.
 */
export function status() {
  return {
    keyboard: { on: keyboard.on, paused: keyboard.on ? paused() : null },
    gamepad: { on: gamepad.on, slot, pads: gamepad.pads.map(({ id, standard, drives }) => ({
      id, standard, drives })) },
    hidden,
    inputs,
    problem,
  };
}

/** Switches keyboard driving on or off. Off lets the joystick go, for anything else to drive. */
export function setKeyboard(on) {
  arm();
  keyboard.on = on;
  keyboard.held.clear();
  update();
}

/** Switches gamepad driving on or off. */
export function setGamepad(on) {
  arm();
  gamepad.on = on;
  if (on && !hidden) readPads();
  if (!on) gamepad.pads = [];
  poll();
  update();
}

/** Which joystick the first gamepad drives, 0 or 1, kept in this browser. */
export function setSlot(chosen) {
  slot = chosen === 1 ? 1 : 0;
  try {
    localStorage.setItem(SLOT_KEY, String(slot));
  } catch (error) {
    // Kept for this page only: storage is off in this browser.
  }
  if (gamepad.on && !hidden) readPads();
  update();
}

/** Stops driving, keyboard and gamepad: the simulator is stopping. */
export function release() {
  keyboard.on = false;
  keyboard.held.clear();
  gamepad.on = false;
  gamepad.pads = [];
  update();
}

function storedSlot() {
  try {
    return localStorage.getItem(SLOT_KEY) === "1" ? 1 : 0;
  } catch (error) {
    return 0;
  }
}

/** Why the keys don't drive now, or null when they do. */
function paused() {
  if (hidden) return "hidden";
  if (!focused) return "window";
  if (inField) return "field";
  return null;
}

/** Starts listening, the first time driving is switched on: the page as it is now, then changes. */
function arm() {
  if (armed) return;
  armed = true;
  hidden = document.visibilityState === "hidden";
  focused = document.hasFocus();
  inField = typing(document.activeElement);
  document.addEventListener("keydown", keyDown);
  document.addEventListener("keyup", keyUp);
  document.addEventListener("contextmenu", letGo);
  // Focus in a field pauses the keys, and so does the window losing the keyboard: a key released
  // meanwhile is never heard, so they all count as released, and the joystick is centered.
  document.addEventListener("focusin", (event) => pause(() => (inField = typing(event.target))));
  document.addEventListener("focusout", () => pause(() => (inField = false)));
  window.addEventListener("blur", () => pause(() => (focused = false)));
  window.addEventListener("focus", () => pause(() => (focused = true)));
  document.addEventListener("visibilitychange", () => {
    const nowHidden = document.visibilityState === "hidden";
    // Back in view, the gamepads are read afresh first: what was read before is stale.
    if (!nowHidden && gamepad.on) readPads();
    pause(() => (hidden = nowHidden));
    poll();
  });
  // A link back sends everything again, to the new stream; a link lost sends nothing until then.
  onConnection((connected) => {
    if (connected) transmit();
  });
}

// ---- Keys ----

function keyDown(event) {
  if (MODIFIERS.has(event.key) || event.metaKey) {
    letGo();
    return;
  }
  const key = KEYS[event.code];
  if (!keyboard.on || !key || event.ctrlKey || event.altKey || typing(event.target)
      || document.querySelector("dialog[open]")) {
    return;
  }
  event.preventDefault(); // the arrows don't scroll the page while they drive
  if (paused() || keyboard.held.has(event.code)) return;
  keyboard.held.add(event.code);
  soon();
}

function keyUp(event) {
  if (MODIFIERS.has(event.key)) letGo();
  else if (keyboard.held.delete(event.code)) soon();
}

/** Every key counts as released: one may have been let go unheard. Pressed again, it drives. */
function letGo() {
  if (keyboard.held.size === 0) return;
  keyboard.held.clear();
  soon();
}

function pause(change) {
  const before = paused();
  change();
  const now = paused();
  if (now === before) return;
  if (now) keyboard.held.clear();
  update(); // at once: a hidden tab runs no frames
}

// ---- Gamepads ----

/** Polls the gamepads each frame while gamepad driving is on and the tab is visible. */
function poll() {
  if (polling || !gamepad.on || hidden) return;
  const frame = () => {
    polling = 0;
    if (!gamepad.on || hidden) return;
    readPads();
    update();
    polling = requestAnimationFrame(frame);
  };
  polling = requestAnimationFrame(frame);
}

function readPads() {
  const found = [];
  for (const pad of navigator.getGamepads ? navigator.getGamepads() : []) {
    if (!pad || !pad.connected) continue;
    found.push({ id: pad.id, standard: pad.mapping === "standard", pad });
  }
  // The first standard gamepad drives the chosen joystick, the second the other; one without the
  // standard layout can't drive, since its buttons could be anything.
  let next = 0;
  gamepad.pads = found.map((each) => ({
    id: each.id,
    standard: each.standard,
    drives: each.standard && next < 2 ? (next++ === 0 ? slot : 1 - slot) : null,
    pad: each.pad,
  }));
}

function padInput(pad) {
  const input = centered();
  for (let axis = 0; axis < 4; axis++) input.axes[axis] = level(pad.axes[axis] || 0);
  for (const [from, to] of PAD_TRIGGERS) input.axes[to] = level(pad.buttons[from]?.value || 0);
  for (const [from, to] of PAD_BUTTONS) input.buttons[to] = Boolean(pad.buttons[from]?.pressed);
  return input;
}

// ---- Input ----

function centered() {
  return { axes: new Array(AXES).fill(0), buttons: new Array(BUTTONS).fill(false), povs: [-1] };
}

/** An axis as a Driver Station sends it: in steps of 1/127. */
function level(value) {
  return Math.round(Math.max(-1, Math.min(1, value)) * 127) / 127;
}

function keyInput() {
  const input = centered();
  for (const code of keyboard.held) {
    const key = KEYS[code];
    if (key.button !== undefined) input.buttons[key.button] = true;
    else input.axes[key.axis] = Math.max(-1, Math.min(1, input.axes[key.axis] + key.value));
  }
  return input;
}

/** Two inputs on one joystick: an axis the keys move, else the gamepad's; a button either's. */
function combine(into, input) {
  if (!into) return input;
  return {
    axes: into.axes.map((value, axis) => (value !== 0 ? value : input.axes[axis])),
    buttons: into.buttons.map((down, button) => down || input.buttons[button]),
    povs: into.povs,
  };
}

function pressed(input) {
  return Boolean(input) && (input.axes.some((value) => value !== 0)
      || input.buttons.some(Boolean) || input.povs.some((pov) => pov !== -1));
}

/** Works out each joystick's input, and sends it when it changed. */
function update() {
  const next = [null, null];
  if (keyboard.on) next[0] = paused() ? centered() : keyInput();
  if (gamepad.on) {
    for (const each of gamepad.pads) {
      if (each.drives === null) continue;
      next[each.drives] = combine(next[each.drives], hidden ? centered() : padInput(each.pad));
    }
  }
  inputs = next;
  transmit();
  // Each joystick held is sent again every 200 ms, so the robot's deadman never centers a stick
  // held still; with nothing held, nothing ticks.
  const held = inputs.some(pressed);
  if (held && !repeating) repeating = setInterval(transmit, REPEAT_MS / 4);
  if (!held && repeating) {
    clearInterval(repeating);
    repeating = 0;
  }
  notify();
}

/** Tells the listeners, when anything they show changed: a gamepad is read every frame. */
function notify() {
  const now = status();
  const key = JSON.stringify(now);
  if (key === told) return;
  told = key;
  for (const listener of listeners) listener(now);
}

/** An update before the next frame, for a key: at most one a frame. */
function soon() {
  if (framePending) return;
  framePending = true;
  requestAnimationFrame(() => {
    framePending = false;
    update();
  });
}

/**
 * Sends each joystick's input that differs from what was sent, or that's held and due again by its
 * own 200 ms clock, in one request at a time, the latest input winning. A joystick this page no
 * longer drives is let go, once. What the robot didn't answer is sent again shortly.
 */
function transmit() {
  if (!robot.stream || !robot.connected || robot.stopped) return;
  if (robot.stream !== sentStream) {
    // A new stream: the robot let go of the old one's joysticks, so everything is new to it.
    sentStream = robot.stream;
    sent.fill(null);
    claimed.fill(false);
  }
  const now = Date.now();
  const body = {};
  for (let port = 0; port < 2; port++) {
    const input = inputs[port];
    if (input) {
      const text = JSON.stringify(input);
      if (text !== sent[port] || (pressed(input) && now - sentAt[port] >= REPEAT_MS)) {
        body[port] = input;
      }
    } else if (claimed[port]) {
      body[port] = null;
    }
  }
  if (Object.keys(body).length === 0) return;
  if (inFlight) {
    again = true;
    return;
  }
  inFlight = true;
  clearTimeout(retrying);
  for (const [port, input] of Object.entries(body)) {
    sent[port] = input ? JSON.stringify(input) : null;
    claimed[port] = Boolean(input);
    sentAt[port] = now;
  }
  // One that hangs is dropped, and sent again: the robot's deadman covers the gap.
  const options = { keepalive: hidden, signal: AbortSignal.timeout(400) };
  api("PUT", "/api/simulator/joysticks", body, options)
    .then(async (response) => {
      if (response.status >= 500) {
        unanswered(body);
        return;
      }
      problem = response.ok ? null
        : ((await response.json().catch(() => null)) || {}).detail || response.statusText;
      if (response.status === 404) release(); // not a simulator after all
      notify();
    })
    .catch(() => unanswered(body))
    .finally(() => {
      inFlight = false;
      if (again) {
        again = false;
        transmit();
      }
    });
}

/** What a request carried wasn't heard: it's sent again, a joystick let go included. */
function unanswered(body) {
  for (const [port, input] of Object.entries(body)) {
    sent[port] = null;
    if (input === null) claimed[port] = true;
  }
  problem = "The robot didn't answer: sending again.";
  notify();
  clearTimeout(retrying);
  retrying = setTimeout(transmit, RETRY_MS);
}
