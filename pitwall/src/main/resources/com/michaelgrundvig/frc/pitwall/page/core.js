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
 * failure; {@code lastHeard} is when anything last arrived (0 for never).
 */
export const robot = { doc: null, stream: "", connected: null, lastHeard: 0 };

const changeListeners = [];
const sampleListeners = [];
const connectionListeners = [];
let source = null;
let opened = 0; // when the stream was last opened
let lastRequest = 0;
let retry = null;

/**
 * Calls {@code listener(patch, snapshot)} after each snapshot and patch: the patch is what changed
 * (the whole document for a snapshot), so a view redraws only that.
 */
export function onChange(listener) {
  changeListeners.push(listener);
}

/** Calls {@code listener(trace, fromStart)} with a run's samples as they arrive. */
export function onSamples(listener) {
  sampleListeners.push(listener);
}

/** Calls {@code listener(connected)} whenever the link to the robot comes or goes. */
export function onConnection(listener) {
  connectionListeners.push(listener);
}

/** A JSON merge patch (RFC 7396): what changed, with null for what's gone. */
export function merge(target, patch) {
  if (patch === null || typeof patch !== "object" || Array.isArray(patch)) return patch;
  if (target === null || typeof target !== "object" || Array.isArray(target)) target = {};
  for (const [key, value] of Object.entries(patch)) {
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
  source = new EventSource(robot.stream
    ? "/api/events?replaces=" + encodeURIComponent(robot.stream) : "/api/events");
  opened = Date.now();
  source.addEventListener("snapshot", (event) => {
    const snapshot = JSON.parse(event.data);
    robot.stream = snapshot.stream;
    robot.doc = snapshot.document;
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
  if (body !== undefined) {
    headers["Content-Type"] = "application/merge-patch+json";
    options.body = JSON.stringify(body);
  }
  return fetch(path, options);
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

let toastTimer = null;

/**
 * Says what became of something done here: always to a screen reader, and on screen for a while
 * when it went wrong (a refusal of the request, say), since the page then looks as it did before.
 */
export function say(text, bad = false) {
  announce(text, bad);
  if (!bad) return;
  const toast = byId("toast");
  toast.replaceChildren(h("span", null, "✗ " + text),
    h("button", { type: "button", class: "link", "aria-label": "Close",
                  onclick: () => (toast.hidden = true) }, "✕"));
  toast.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => (toast.hidden = true), 8000);
}
