// The state bar, on every view: which robot (the simulator or a real one), whether this page hears
// it and how long since it last did, its mode, whether tuning is on, and how many tuned values
// aren't committed yet; in the simulator, whether this page drives it. It answers "can I, and on
// what?", the question asked most, whose wrong answer costs most (tuning the simulator while the
// robot sits unchanged).

import { robot, onChange, onConnection, send, h, byId, say, announce } from "./core.js";

let name;
let badge;
let code;
let connection;
let since;
let mode;
let driving;
let notDriving = false;
let title = "Pitwall";
let tuningPill;
let tuningButton;
let uncommitted;
let band;
let bandSince;
let themeButton;

/** The theme: the system's, or light or dark whatever the system says (a bright pit). */
const THEMES = { system: "◐ System", light: "☀ Light", dark: "☾ Dark" };

/** Builds the bar into the page's header, and the band below it for a lost link. */
export function mountBar() {
  name = h("span", { class: "robot-name" });
  badge = h("span", { class: "badge", hidden: true });
  code = h("span", { class: "code mono muted", title: "The code running: its git commit, branch, "
      + "and whether it had uncommitted changes" });
  since = h("span");
  connection = h("span", { class: "conn" }, "Connecting to the robot…");
  mode = h("a", { class: "mode", hidden: true });
  driving = h("a", { href: "#sim", class: "pill driving", hidden: true });
  tuningButton = h("button", { type: "button", onclick: switchTuning });
  tuningPill = h("span", { class: "pill", hidden: true });
  uncommitted = h("a", { href: "#changes", class: "uncommitted", hidden: true });
  themeButton = h("button", { type: "button", class: "theme", onclick: nextTheme });
  showTheme();
  byId("bar").append(
    h("span", { class: "brand" }, "Pitwall", name, badge),
    code,
    h("span", { class: "spacer" }),
    connection, mode, driving, tuningPill, uncommitted, themeButton);
  bandSince = h("span");
  band = byId("band");
  onChange(render);
  onConnection(renderConnection);
  // How long since the robot was last heard, written in place each second: nothing around it is
  // rebuilt.
  setInterval(renderSince, 1000);
}

function render(patch, snapshot) {
  const doc = robot.doc;
  if (snapshot) renderConnection(robot.connected);
  if (snapshot || "robot" in patch) {
    const simulated = doc.robot.simulated;
    name.textContent = doc.robot.name;
    badge.hidden = false;
    badge.textContent = simulated ? "SIM" : "ROBOT";
    badge.className = "badge " + (simulated ? "sim" : "real");
    badge.title = simulated ? "The simulator" : "A real robot";
    code.textContent = doc.robot.code;
    mode.hidden = !doc.robot.mode;
    mode.textContent = doc.robot.mode;
    // Highlighted when it keeps actions and experiments from running: the way out is here.
    mode.classList.toggle("gated", Boolean(doc.robot.actionsLocked));
    // In the simulator, Sim is the Driver Station: the mode is a way there.
    if (simulated) mode.setAttribute("href", "#sim"); else mode.removeAttribute("href");
    mode.title = simulated ? "The Driver Station's mode: change it in Sim" : "The Driver Station's mode";
    title = "Pitwall · " + doc.robot.name + " · " + (simulated ? "SIM" : "ROBOT");
    showTitle();
  }
  if (snapshot || "robot" in patch || "simulator" in patch) showEnabled();
  if (snapshot || "tuning" in patch) {
    renderTuning();
    renderBand();
  }
  if (snapshot || "changes" in patch) {
    const count = Object.keys(doc.changes).length;
    uncommitted.hidden = count === 0;
    uncommitted.replaceChildren(h("b", null, "● " + count), "to commit");
    uncommitted.title = count + (count === 1 ? " tuned value differs" : " tuned values differ")
        + " from the committed configuration: see Changes";
  }
}

/**
 * Says in the bar whether this page drives the simulator: "⌨ Driving", or why it isn't now. While
 * the tab is hidden, its title says so too: that's all of the page anyone sees then.
 *
 * @param text what to say; null when this page doesn't drive
 * @param live whether it drives right now
 * @param hidden whether it would, but the tab is hidden
 */
export function showDriving(text, live, hidden) {
  const className = "pill driving " + (live ? "on" : "locked");
  if (driving.textContent !== (text || "") || driving.className !== className) {
    driving.hidden = !text;
    driving.textContent = text || "";
    driving.className = className;
    driving.title = "Driving the simulator from this page: see Sim";
  }
  notDriving = Boolean(text) && hidden;
  showTitle();
}

/**
 * In the simulator an enabled robot's mode is filled in, unmistakable from across the room; never
 * while the robot isn't heard, when it may be anything.
 */
function showEnabled() {
  const doc = robot.doc;
  mode.classList.toggle("enabled", Boolean(robot.connected && doc && doc.robot.simulated
      && doc.simulator && doc.simulator.enabled));
}

function showTitle() {
  const next = (notDriving ? "⏸ Not driving · " : "") + title;
  if (document.title !== next) document.title = next;
}

function renderTuning() {
  const session = robot.doc.tuning;
  tuningPill.hidden = false;
  if (session.fieldConnected) {
    tuningPill.className = "pill locked";
    tuningPill.replaceChildren("🔒 Tuning locked: field");
  } else if (session.enabled) {
    tuningPill.className = "pill on";
    tuningButton.className = "";
    tuningButton.textContent = "Switch off";
    tuningPill.replaceChildren("✓ Tuning on", tuningButton);
  } else {
    tuningPill.className = "pill locked";
    tuningButton.className = "primary";
    tuningButton.textContent = "Switch on";
    tuningPill.replaceChildren("🔒 Tuning off", tuningButton);
  }
}

async function switchTuning() {
  await setTuning(!robot.doc.tuning.enabled);
}

/** Switches tuning on or off, from the bar or from a banner where it's off. */
export async function setTuning(on) {
  const result = await send("PATCH", "/api/tuning", { enabled: on });
  if (result.ok) say(on ? "Tuning is on." : "Tuning is off.");
  else say(result.problem.detail, true);
}

/**
 * A banner where tuning's being off bites, with its own way out: "Tuning is off: values can't
 * change until it's on", and Switch tuning on. Shown only while it's off and the field isn't
 * connected (the field's band says that, and has no way out here).
 *
 * @param what what can't change: "values", "gains", "switches"
 */
export function tuningOffBanner(what) {
  const button = h("button", { type: "button", class: "primary", onclick: () => setTuning(true) },
    "Switch tuning on");
  const banner = h("div", { class: "banner lock", hidden: true },
    h("span", { class: "grow" }, "🔒 ", h("b", null, "Tuning is off: "),
      h("span", { class: "why" }, what + " can't change until it's on. On a robot it starts off; "
          + "switch it on in the pit.")),
    button);
  const update = () => {
    const doc = robot.doc;
    banner.hidden = !doc || doc.tuning.enabled || doc.tuning.fieldConnected;
    button.disabled = !robot.connected;
  };
  onChange((patch, snapshot) => {
    if (snapshot || "tuning" in patch) update();
  });
  onConnection(update);
  return banner;
}

function renderConnection(connected) {
  document.body.classList.toggle("stale", !connected);
  tuningButton.disabled = !connected;
  showEnabled();
  if (robot.stopped) {
    // Stopped from this page: the program ended on purpose, so it isn't a lost link.
    connection.className = "conn stopped";
    connection.replaceChildren(connected ? "■ Stopping…" : "■ Stopped");
    if (!connected) announce("The simulator stopped.", true);
  } else if (connected) {
    connection.className = "conn up";
    // The robot program answers before its loop first runs: until then, there's nothing to show.
    connection.replaceChildren(robot.doc ? "● Connected" : "● Connected, waiting for the robot loop");
    if (robot.doc) announce("Connected to the robot.");
  } else {
    connection.className = "conn down";
    connection.replaceChildren("✗ Not connected · ", since, " · retrying");
    renderSince();
    announce("Not connected to the robot.", true);
  }
  renderBand();
}

let fieldShown = false;

/**
 * The band under the bar, for a lock that covers everything: a lost link, or the field. The
 * field's is striped and large, unmistakable from across the pit.
 */
function renderBand() {
  const field = Boolean(robot.connected && robot.doc && robot.doc.tuning.fieldConnected);
  if (robot.connected === false && robot.stopped) {
    band.className = "band stopped";
    band.replaceChildren(h("b", null, "■ Stopped"), h("span", { class: "sub" },
      "This page stopped the simulator. Start it again (./gradlew run), and this page reconnects "
        + "by itself."));
    band.hidden = false;
  } else if (robot.connected === false) {
    band.className = "band down";
    band.replaceChildren(h("b", null, "✗ Not connected"), h("span", { class: "sub" }, bandSince));
    band.hidden = false;
  } else if (field) {
    band.className = "band field";
    // In the simulator, Sim is the field: the way out is its FMS attached switch.
    const simulated = robot.doc.robot.simulated;
    band.replaceChildren(h("b", null, "🔒 FIELD CONNECTED"), h("span", { class: "sub" },
      "Locked: every value is back to committed, and what was tuned was printed to the log. "
        + "Tuning, actions, and experiments are off until the field disconnects"
        + (simulated ? ": in the simulator, switch off " : "."),
      simulated && h("a", { href: "#sim" }, "FMS attached in Sim"), simulated && "."));
    band.hidden = false;
  } else {
    band.hidden = true;
  }
  if (field !== fieldShown) {
    fieldShown = field;
    if (field) announce("The field system is connected: everything is locked and back to "
        + "committed.", true);
  }
}

function renderSince() {
  if (robot.connected) return;
  const text = robot.lastHeard === 0 ? "never"
      : ago(Math.round((Date.now() - robot.lastHeard) / 1000));
  since.textContent = robot.lastHeard === 0 ? "never heard" : "last heard " + text;
  bandSince.textContent = robot.lastHeard === 0 ? "The robot hasn't answered yet. Retrying."
      : "The robot was last heard " + text + ". Retrying. What you see is what it last sent; "
        + "nothing can change until it's back.";
}

/** A time since, coarse past a minute so the text settles. */
function ago(seconds) {
  if (seconds < 60) return seconds + " s ago";
  const minutes = Math.floor(seconds / 60);
  return minutes < 60 ? minutes + " min ago" : Math.floor(minutes / 60) + " h ago";
}

/** The theme chosen here, kept in this browser; the system's until someone chooses. */
function theme() {
  try {
    const chosen = localStorage.getItem("pitwall.theme");
    return chosen in THEMES ? chosen : "system";
  } catch (error) {
    return "system";
  }
}

function nextTheme() {
  const names = Object.keys(THEMES);
  const next = names[(names.indexOf(theme()) + 1) % names.length];
  try {
    localStorage.setItem("pitwall.theme", next);
  } catch (error) {
    // Kept for this page only: storage is off in this browser.
  }
  showTheme(next);
}

function showTheme(chosen = theme()) {
  if (chosen === "system") delete document.documentElement.dataset.theme;
  else document.documentElement.dataset.theme = chosen;
  const [mark, word] = THEMES[chosen].split(" ");
  themeButton.replaceChildren(h("span", { "aria-hidden": "true" }, mark),
    h("span", { class: "word" }, " " + word));
  themeButton.title = "Theme: " + word;
  themeButton.setAttribute("aria-label", "Theme: " + chosen + ". Change it.");
  document.dispatchEvent(new Event("pitwall-theme"));
}
