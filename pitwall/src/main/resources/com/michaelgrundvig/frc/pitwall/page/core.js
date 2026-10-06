// The page's core: the robot's state as its event stream sends it, requests to its API, and the
// small helpers every view shares. The views (pit.js, tune.js, experiment.js, changes.js, sim.js)
// listen here and draw only what changed.
//
// Plain JavaScript modules, served from the robot program's jar: nothing to build, nothing loaded
// from anywhere else (there's no internet at an event). Text from the robot is always set as text
// (textContent), never as HTML.

/**
 * The robot as this page knows it. {@code doc} is its state as sent: the snapshot, with every patch
 * since merged in. {@code stream} names this page's event stream in every request, so the robot
 * knows who's asking (the deadman). {@code connected} is null until the first answer, or the first
 * failure; {@code lastHeard} is when anything last arrived (0 for never). {@code stopped} is true
 * once this page stopped the simulator, until a robot program answers again: its stream ending
 * then is the program ending, not a lost link.
 */
export const robot = { doc: null, stream: "", connected: null, lastHeard: 0, stopped: false };

const changeListeners = [];
const sampleListeners = [];
const connectionListeners = [];
let source = null;
let opened = 0; // when the stream was last opened
let lastRequest = 0;
let retry = null;

/**
 * Calls {@code listener(patch, snapshot, replaced)} after each snapshot and patch: the patch is what
 * changed (the whole document for a snapshot), so a view redraws only that. {@code replaced} is true
 * when a part of the document was put in place whole (a scene's state, as a page starts showing it):
 * the patch then holds all of that part, and what it doesn't hold is gone.
 */
export function onChange(listener) {
  changeListeners.push(listener);
}

/**
 * The views this page shows that the robot sends only to pages showing them (a scene's id), and
 * whether the tab is on screen: a hidden tab shows nothing, so the robot sends it none of them.
 */
let views = [];

/**
 * Says which of those views this page shows now: the robot starts sending what they need, and stops
 * sending what they don't. Kept across reconnecting.
 */
export function showViews(shown) {
  views = [...shown];
  sendViews();
}

function shownViews() {
  return document.hidden ? [] : views;
}

let viewsSent = null; // as the robot last heard them for this page's stream; null for not known
let viewsSending = false; // a PUT of them is on its way

/**
 * Tells the robot the views shown, one request at a time: two on their way at once could land in
 * either order, and a quick World, Pit, World could leave the robot holding none. Whatever changes
 * while one is on its way goes when it's answered, the latest only.
 */
function sendViews() {
  if (viewsSending) return;
  const now = shownViews().join(",");
  if (!robot.stream || now === viewsSent) return;
  viewsSent = now;
  viewsSending = true;
  const stream = robot.stream;
  api("PUT", "/api/views", { views: shownViews() })
    .then((response) => {
      // Refused: a stream just replaced, whose snapshot says them again.
      if (!response.ok && robot.stream === stream) viewsSent = null;
    })
    .catch(() => {
      if (robot.stream === stream) viewsSent = null;
    })
    .finally(() => {
      viewsSending = false;
      // Sent, or the stream was replaced meanwhile: the views may have changed since.
      if (viewsSent !== null || robot.stream !== stream) sendViews();
    });
}

document.addEventListener("visibilitychange", sendViews);

/** Calls {@code listener(trace, fromStart)} with a run's samples as they arrive. */
export function onSamples(listener) {
  sampleListeners.push(listener);
}

/** Calls {@code listener(connected)} whenever the link to the robot comes or goes. */
export function onConnection(listener) {
  connectionListeners.push(listener);
}

/** A JSON merge patch (RFC 7396): what changed, with null for what's gone. */
function merge(target, patch) {
  if (patch === null || typeof patch !== "object" || Array.isArray(patch)) return patch;
  if (target === null || typeof target !== "object" || Array.isArray(target)) target = {};
  for (const [key, value] of Object.entries(patch)) {
    if (key === "__proto__") continue; // a key JSON may carry, and an object mustn't take
    if (value === null) delete target[key];
    else target[key] = merge(target[key], value);
  }
  return target;
}

// ---- The stream ----

/** Follows the robot: a snapshot, then patches, then the run's samples as they're taken. */
export function connect() {
  clearTimeout(retry);
  if (source) source.close();
  // Reconnecting, it names the stream it had, which the robot then closes rather than keep a place
  // for it until a write to it fails.
  const asked = [];
  if (robot.stream) asked.push("replaces=" + encodeURIComponent(robot.stream));
  if (shownViews().length) asked.push("views=" + shownViews().map(encodeURIComponent).join(","));
  source = new EventSource("/api/events" + (asked.length ? "?" + asked.join("&") : ""));
  const asking = shownViews().join(",");
  opened = Date.now();
  source.addEventListener("snapshot", (event) => {
    const snapshot = JSON.parse(event.data);
    robot.stream = snapshot.stream;
    viewsSent = asking; // what the address asked for, unless they changed while it opened
    sendViews();
    robot.doc = snapshot.document;
    robot.stopped = false; // a robot program is running again
    heard();
    tell(changeListeners, robot.doc, true);
    if (snapshot.trace) tell(sampleListeners, snapshot.trace, true);
  });
  source.addEventListener("patch", (event) => {
    heard();
    if (robot.doc === null) return;
    const patch = JSON.parse(event.data);
    robot.doc = merge(robot.doc, patch);
    tell(changeListeners, patch, false);
  });
  // A part of the document put in place whole: a scene's state, when this page starts showing it.
  source.addEventListener("replace", (event) => {
    heard();
    if (robot.doc === null) return;
    const { at, value } = JSON.parse(event.data);
    if (!Array.isArray(at) || at.length === 0 || at.includes("__proto__")) return;
    let parent = robot.doc;
    for (const key of at.slice(0, -1)) {
      if (parent[key] === null || typeof parent[key] !== "object") parent[key] = {};
      parent = parent[key];
    }
    parent[at[at.length - 1]] = value;
    const patch = {};
    let into = patch;
    for (const key of at.slice(0, -1)) into = into[key] = {};
    into[at[at.length - 1]] = value;
    tell(changeListeners, patch, false, true);
  });
  source.addEventListener("samples", (event) => {
    heard();
    tell(sampleListeners, JSON.parse(event.data), false);
  });
  source.addEventListener("heartbeat", heard);
  source.onerror = () => {
    setConnected(false);
    // A browser retries a dropped connection by itself, but gives up on an answer that isn't a
    // stream (a robot program stopping says 503): start over soon, rather than at the silence
    // check.
    if (source.readyState === EventSource.CLOSED) retry = setTimeout(connect, 2000);
  };
}

function heard() {
  robot.lastHeard = Date.now();
  setConnected(true);
}

function setConnected(connected) {
  if (robot.connected === connected) return;
  robot.connected = connected;
  tell(connectionListeners, connected);
}

/**
 * Says this page is stopping the simulator (or isn't, after all: its Stop failed). The stream's end
 * that follows shows as "stopped", not as a lost link; the page still reconnects by itself when
 * the simulator starts again.
 */
export function stopping(yes = true) {
  robot.stopped = yes;
  tell(connectionListeners, robot.connected);
}

function tell(listeners, ...args) {
  for (const listener of listeners) {
    try {
      listener(...args);
    } catch (error) {
      console.error(error); // one view's bug doesn't stop the others
    }
  }
}

// The robot sends something at least every 5 s. Ten seconds of silence is a lost link, even one
// the browser hasn't noticed (dropped Wi-Fi): start over with a fresh snapshot.
setInterval(() => {
  if (source && Date.now() - Math.max(robot.lastHeard, opened) > 10000) {
    setConnected(false);
    connect();
  }
}, 1000);

// ---- Requests ----

/** A request to the API, naming this page's stream. Sent counts for the deadman, not answered. */
export function api(method, path, body, more = {}) {
  lastRequest = Date.now();
  const headers = { "Pitwall-Stream": robot.stream };
  const options = { method, headers, ...more };
  if (body instanceof Blob) {
    headers["Content-Type"] = "application/octet-stream"; // a file, sent as it is
    options.body = body;
  } else if (body !== undefined) {
    headers["Content-Type"] = "application/merge-patch+json";
    options.body = JSON.stringify(body);
  }
  return fetch(path, options);
}

/**
 * What's running on the robot, which Esc stops: the experiment's run, and each action, as
 * {kind, name, run}.
 */
export function running() {
  const doc = robot.doc;
  if (!doc) return [];
  const now = [];
  if (doc.run && doc.run.phase === "RUNNING") {
    now.push({ kind: "experiment", name: doc.run.experiment, run: doc.run.id });
  }
  for (const [name, action] of Object.entries(doc.actions || {})) {
    if (action.phase === "RUNNING") now.push({ kind: "action", name });
  }
  return now;
}

/** When this page last sent the robot anything. */
export function lastRequestAt() {
  return lastRequest;
}

/**
 * Sends a request and says what became of it: {ok, status, body, problem}. The body is the answer's
 * JSON; a problem is the robot's problem details ({title, detail, errors}), or one made up here
 * when the robot isn't answering.
 */
export async function send(method, path, body) {
  let response;
  try {
    response = await api(method, path, body);
  } catch (error) {
    return { ok: false, status: 0, body: null,
             problem: { title: "Not answering", detail: "The robot isn't answering." } };
  }
  const json = await response.json().catch(() => null);
  if (response.ok) return { ok: true, status: response.status, body: json, problem: null };
  return { ok: false, status: response.status, body: null,
           problem: json || { title: response.statusText, detail: response.statusText } };
}

// ---- Helpers ----

/** Whether two numbers are the same value, as the robot's report judges it. */
export function near(a, b) {
  return Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(b));
}

/** A number as people write it: whole numbers plainly, others to six figures, as the robot does. */
export function shown(number) {
  if (typeof number !== "number") return String(number);
  return Number.isInteger(number) ? String(number) : String(Number(number.toPrecision(6)));
}

/**
 * An element, with its properties and children: {@code h("button", {class: "primary", onclick:
 * run}, "Run")}. Text children are text nodes, never parsed as HTML.
 */
export function h(tag, props, ...children) {
  const made = document.createElement(tag);
  for (const [key, value] of Object.entries(props || {})) {
    if (value === null || value === undefined || value === false) continue;
    if (key === "class") made.className = value;
    else if (key === "text") made.textContent = value;
    else if (key.startsWith("on") && typeof value === "function") {
      made.addEventListener(key.slice(2), value);
    } else made.setAttribute(key, value === true ? "" : String(value));
  }
  for (const child of children.flat(Infinity)) {
    if (child === null || child === undefined || child === false) continue;
    made.append(child instanceof Node ? child : String(child));
  }
  return made;
}

/**
 * Goes to a place on the page by its address (#tune/drivetrain/gains.steerKp), even when it's the
 * address already: the view then shows the place again, focused.
 */
export function go(hash) {
  if (location.hash === hash) window.dispatchEvent(new HashChangeEvent("hashchange"));
  else location.hash = hash;
}

/**
 * Replaces an element's children, but only when their markup differs: a part updated with nothing
 * new keeps its elements, so a button isn't rebuilt between a press and its release. (Comparing
 * markup reads it; nothing is ever written as HTML.)
 *
 * <p>The rule that follows: what the markup doesn't show isn't compared, so new elements whose
 * markup matches the old are thrown away, their handlers with them. A handler given here must
 * therefore read what it needs when it runs (from {@link robot}, or a data- attribute), not from
 * what was current when it was made.
 */
export function replace(element, ...children) {
  const next = children.flat(Infinity).filter((child) => child !== null && child !== undefined
      && child !== false);
  const scratch = document.createElement("div");
  for (const child of next) scratch.append(child instanceof Node ? child : String(child));
  if (scratch.innerHTML === element.innerHTML) return false;
  element.replaceChildren(...scratch.childNodes);
  return true;
}

/** A number with its value's unit, as the page shows it: "40 A", or "16" for a plain number. */
export function withUnit(number, value) {
  return shown(number) + (value.unit ? " " + value.unit : "");
}

/**
 * Whether keys pressed in this element are typing: a value, a filter, a choice. Shortcuts and the
 * simulator's keyboard driving leave those keys alone.
 */
export function typing(target) {
  return target instanceof HTMLInputElement || target instanceof HTMLTextAreaElement
      || target instanceof HTMLSelectElement;
}

/** The element with this id. */
export function byId(id) {
  return document.getElementById(id);
}

/** Tells a screen reader, through a live region: urgently for a lock or a lost link. */
export function announce(text, urgent = false) {
  const live = byId(urgent ? "live-urgent" : "live");
  live.textContent = "";
  setTimeout(() => (live.textContent = text), 30); // a change, so it's read even when repeated
}

/**
 * Asks before something that can't be taken back with one press, in a dialog listing what it does.
 *
 * @param body the dialog's content, under its title
 * @return whether it was confirmed
 */
export function confirm(title, body, yes) {
  const dialog = byId("dialog");
  return new Promise((resolve) => {
    dialog.replaceChildren(h("form", { method: "dialog", class: "dlg" },
      h("h2", null, title), body,
      h("div", { class: "dlg-actions" },
        h("button", { value: "no" }, "Cancel"),
        h("button", { value: "yes", class: "primary" }, yes))));
    dialog.returnValue = "";
    dialog.onclose = () => resolve(dialog.returnValue === "yes");
    dialog.showModal();
  });
}

let toastTimer = null;

/**
 * Says what became of something done here: to a screen reader, and on screen for a while when it
 * went wrong (a refusal of the request, say), since the page then looks as it did before.
 *
 * @param bad whether it went wrong: shown as well as read out
 * @param urgent read out at once, interrupting: for a lock, a lost link, or a run stopped; a result
 *     (applied, refused) is read out when the reader is free
 */
export function say(text, bad = false, urgent = false) {
  announce(text, urgent);
  if (!bad) return;
  const toast = byId("toast");
  toast.replaceChildren(h("span", null, "✗ " + text),
    h("button", { type: "button", class: "link", "aria-label": "Close",
                  onclick: () => (toast.hidden = true) }, "✕"));
  toast.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => (toast.hidden = true), 8000);
}
