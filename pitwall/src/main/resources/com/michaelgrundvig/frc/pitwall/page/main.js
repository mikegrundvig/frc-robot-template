// The page: one view at a time, chosen by the address's hash (#pit, #tune, ...), so a reload comes
// back to the same place and a link can be sent. Each view is its own module; this starts them
// and follows the robot.

import { robot, connect, onChange, byId, h, running, send, announce, say } from "./core.js";
import { mountBar } from "./bar.js";
import { mountApplyBar, showIn } from "./applybar.js";
import pit from "./pit.js";
import tune from "./tune.js";
import experiment from "./experiment.js";
import changes from "./changes.js";
import sim from "./sim.js";

const views = [pit, tune, experiment, changes, sim];
mountBar();
mountApplyBar();
const sections = new Map(); // view id -> its section
const tabs = new Map(); // view id -> its tab
let current = null;

for (const view of views) {
  const section = h("section", { id: "view-" + view.id, class: "view", hidden: true,
                                 "aria-label": view.title });
  byId("views").append(section);
  sections.set(view.id, section);
  const tab = h("a", { href: "#" + view.id, class: "tab", hidden: Boolean(view.simulatorOnly) },
    view.title);
  byId("tabs").append(tab);
  tabs.set(view.id, tab);
  view.mount(section);
}

/** The view the address names, and the rest of the address after it: #tune/drivetrain/... */
function route() {
  const [id, ...rest] = decodeURIComponent(location.hash.slice(1)).split("/");
  const known = views.find((view) => view.id === id && offered(view));
  show(known || pit, known ? rest.join("/") : "");
}

/** Whether a view is offered: the simulator's only in the simulator. */
function offered(view) {
  return !view.simulatorOnly || Boolean(robot.doc && robot.doc.robot.simulated);
}

function show(view, rest) {
  if (current !== view) {
    if (current && current.hide) current.hide();
    for (const [id, section] of sections) section.hidden = id !== view.id;
    for (const [id, tab] of tabs) {
      if (id === view.id) tab.setAttribute("aria-current", "page");
      else tab.removeAttribute("aria-current");
    }
    current = view;
    if (view.show) view.show();
    showIn(view.id);
  }
  if (view.route) view.route(rest);
}

const count = h("span", { class: "count", hidden: true });
tabs.get("changes").append(count);

onChange((patch, snapshot) => {
  if (snapshot || "changes" in patch) {
    const tuned = Object.keys(robot.doc.changes).length;
    count.hidden = tuned === 0;
    count.textContent = String(tuned);
    count.setAttribute("aria-label", tuned + " to commit");
  }
  if (snapshot || "robot" in patch) {
    for (const view of views) tabs.get(view.id).hidden = !offered(view);
    // The address may name a view offered only now, such as #sim, or one no longer offered.
    const named = views.find((view) => view.id === location.hash.slice(1).split("/")[0]);
    if ((named || pit) !== current || snapshot) route();
  }
});

// Esc stops whatever is running on the robot, anywhere on the page: an experiment's run, and any
// action. (With nothing running, Esc in a value drops its edit: see rows.js.)
document.addEventListener("keydown", async (event) => {
  if (event.key !== "Escape" || event.defaultPrevented || document.querySelector("dialog[open]")) {
    return;
  }
  const now = running();
  if (now.length === 0) return;
  event.preventDefault();
  announce("Stopping " + now.map((thing) => thing.name).join(", ") + ".", true);
  for (const thing of now) {
    const result = thing.kind === "experiment"
      ? await send("POST", "/api/experiments", { cancel: thing.name, run: thing.run })
      : await send("POST", "/api/actions", { cancel: thing.name });
    if (!result.ok && result.status !== 409) say(thing.name + ": " + result.problem.detail, true);
  }
});

window.addEventListener("hashchange", route);
route();
connect();
