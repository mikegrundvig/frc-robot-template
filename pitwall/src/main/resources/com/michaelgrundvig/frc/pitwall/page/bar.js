// The state bar, on every view: which robot (the simulator or a real one), whether this page hears
// it and how long since it last did, its mode, whether tuning is on, and how many tuned values
// aren't committed yet. It answers "can I, and on what?", the question asked most, whose wrong
// answer costs most (tuning the simulator while the robot sits unchanged).

import { robot, onChange, onConnection, send, h, byId, say, announce } from "./core.js";

let name;
let badge;
let code;
let connection;
let since;
let mode;
let tuningPill;
let tuningButton;
let uncommitted;
let band;
let bandSince;

/** Builds the bar into the page's header, and the band below it for a lost link. */
export function mountBar() {
  name = h("span", { class: "robot-name" });
  badge = h("span", { class: "badge", hidden: true });
  code = h("span", { class: "code mono muted", title: "The code running: its git commit, branch, "
      + "and whether it had uncommitted changes" });
  since = h("span");
  connection = h("span", { class: "conn" }, "Connecting to the robot…");
  mode = h("a", { class: "mode", hidden: true });
  tuningButton = h("button", { type: "button", onclick: switchTuning });
  tuningPill = h("span", { class: "pill", hidden: true });
  uncommitted = h("a", { href: "#changes", class: "uncommitted", hidden: true });
  byId("bar").append(
    h("span", { class: "brand" }, "Pitwall", name, badge),
    code,
    h("span", { class: "spacer" }),
    connection, mode, tuningPill, uncommitted);
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
    // In the simulator, Sim is the Driver Station: the mode is a way there.
    if (simulated) mode.setAttribute("href", "#sim"); else mode.removeAttribute("href");
    mode.title = simulated ? "The Driver Station's mode: change it in Sim" : "The Driver Station's mode";
    document.title = "Pitwall · " + doc.robot.name + " · " + (simulated ? "SIM" : "ROBOT");
  }
  if (snapshot || "tuning" in patch) renderTuning();
  if (snapshot || "changes" in patch) {
    const count = Object.keys(doc.changes).length;
    uncommitted.hidden = count === 0;
    uncommitted.textContent = "● " + count + " to commit";
    uncommitted.title = count + (count === 1 ? " tuned value differs" : " tuned values differ")
        + " from the committed configuration: see Changes";
  }
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
  const on = !robot.doc.tuning.enabled;
  const result = await send("PATCH", "/api/tuning", { enabled: on });
  if (result.ok) say(on ? "Tuning is on." : "Tuning is off.");
  else say(result.problem.detail, true);
}

function renderConnection(connected) {
  document.body.classList.toggle("stale", !connected);
  tuningButton.disabled = !connected;
  if (connected) {
    connection.className = "conn up";
    // The robot program answers before its loop first runs: until then, there's nothing to show.
    connection.replaceChildren(robot.doc ? "● Connected" : "● Connected, waiting for the robot loop");
    band.hidden = true;
    if (robot.doc) announce("Connected to the robot.");
    return;
  }
  connection.className = "conn down";
  connection.replaceChildren("✗ Not connected · ", since, " · retrying");
  band.className = "band down";
  band.replaceChildren(h("b", null, "✗ Not connected"), h("span", { class: "sub" }, bandSince));
  band.hidden = false;
  renderSince();
  announce("Not connected to the robot.", true);
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
