// World: the robot program's scenes, drawn in 3D: what the code believes about where things are,
// such as the field, the robot on it, and what its cameras see. Offered only when the robot program
// adds a scene (pitwall.scenes()); Pitwall without one has no such view.
//
// The robot describes a scene once (frames and shapes, in layers) and then sends only what moves,
// and only while this view is on screen: the page names the scene it shows on its event stream
// (core.js's showViews). Hiding the view, or the tab, stops it. three.js and the renderer load the
// first time the view is shown (scene-render.js), so a page that never opens it never loads them.
//
// A click on a shape the robot made inspectable asks the robot what it is (GET
// /api/scenes/<id>/inspect/<shape>) and shows the answer beside the drawing, ending in what can be
// done about it: the page's actions, run as from Pit. Hold freezes the drawing so something
// fleeting, a sight line that came and went, stays to be clicked; the robot keeps what it says
// about such a shape for two seconds after it's gone.

import { robot, onChange, onConnection, showViews, send, h, replace, say } from "./core.js";

let visible = false;
let sceneId = null; // the scene shown
let renderer = null; // the renderer, once loaded
let loading = null; // loading it
let unavailable = ""; // why it can't draw, if it can't
let describedAs = ""; // the description the renderer was built from
let held = false;
let following = false;
let inspected = null; // {id, layer} of the shape whose details are shown
let inspection = null; // what the robot said of it
let clickedX = null; // where it was clicked, in CSS pixels from the canvas's left
let wasOffered = false;
const ui = {};

function scenes() {
  return (robot.doc && robot.doc.scenes) || {};
}

function scene() {
  return sceneId ? scenes()[sceneId] || null : null;
}

/** Offered once the robot program has added a scene. */
function offered() {
  return Object.keys(scenes()).length > 0;
}

function mount(view) {
  ui.title = h("h1", { class: "view-title" }, "World");
  ui.pick = h("select", { class: "world-scene", "aria-label": "Scene", hidden: true,
                          onchange: () => choose(ui.pick.value) });
  ui.top = h("button", { type: "button", class: "chip", "aria-pressed": "true",
                         onclick: () => camera("top") }, "Top");
  ui.around = h("button", { type: "button", class: "chip", "aria-pressed": "false",
                            onclick: () => camera("around") }, "3D");
  ui.follow = h("button", { type: "button", class: "chip", "aria-pressed": "false", hidden: true,
                            onclick: toggleFollow }, "Follow");
  ui.fit = h("button", { type: "button", class: "chip", onclick: () => renderer && renderer.fit() },
    "Fit");
  ui.hold = h("button", { type: "button", class: "chip", "aria-pressed": "false",
                          onclick: toggleHold }, "❚❚ Hold");
  ui.layers = h("div", { class: "world-layers", role: "group", "aria-label": "Layers" });
  ui.canvas = h("canvas", { class: "world-gl", tabindex: "0",
                            "aria-label": "The scene, drawn in 3D. Drag to move, scroll to zoom." });
  ui.message = h("p", { class: "world-message", hidden: true });
  ui.held = h("p", { class: "world-held", hidden: true },
    "❚❚ Held: the drawing waits while the robot moves on. Release follows it again.");
  ui.details = h("aside", { class: "world-details card", hidden: true, "aria-live": "polite" });
  ui.stage = h("div", { class: "world-stage" }, ui.canvas, ui.message, ui.held);
  view.classList.add("world");
  view.append(
    h("div", { class: "world-bar" }, ui.title, ui.pick,
      h("div", { class: "world-cameras", role: "group", "aria-label": "Camera" }, ui.top,
        ui.around),
      ui.follow, ui.fit, ui.hold, ui.layers),
    h("div", { class: "world-main" }, ui.stage, ui.details));

  let down = null;
  ui.canvas.addEventListener("pointerdown", (event) => {
    down = { x: event.clientX, y: event.clientY };
  });
  ui.canvas.addEventListener("pointerup", (event) => {
    // A click, not a drag: the controls turn and pan on drags.
    if (down && Math.hypot(event.clientX - down.x, event.clientY - down.y) < 6) {
      clicked(event.clientX, event.clientY);
    }
    down = null;
  });
  new ResizeObserver(() => renderer && visible && renderer.resize()).observe(ui.stage);
  const recolor = () => renderer && renderer.recolor();
  document.addEventListener("pitwall-theme", recolor);
  window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", recolor);
  onChange(changed);
  // Run needs the robot there, as on Pit: the details' button follows the link.
  onConnection(() => inspection && showDetails());
}

function show() {
  visible = true;
  chooseScene();
  showViews(sceneId ? [sceneId] : []);
  load().then(redraw);
}

function hide() {
  visible = false;
  showViews([]);
}

/** #world/<scene> shows that scene. */
function route(rest) {
  if (rest && scenes()[rest]) choose(rest);
}

/** Keeps the scene shown, or picks the first when it's gone (or none was chosen). */
function chooseScene() {
  const ids = Object.keys(scenes());
  if (!sceneId || !ids.includes(sceneId)) sceneId = ids[0] || null;
  ui.pick.hidden = ids.length < 2;
  if (ids.length >= 2) {
    replace(ui.pick, ids.map((id) =>
      h("option", { value: id, selected: id === sceneId }, scenes()[id].title || id)));
  }
}

function choose(id) {
  if (id === sceneId) return;
  sceneId = id;
  describedAs = "";
  closeDetails();
  if (visible) {
    showViews([sceneId]);
    redraw();
  }
}

// ---- Keeping up with the robot program ----

function changed(patch, snapshot, replaced) {
  if (snapshot || "scenes" in patch) {
    const now = offered();
    if (now !== wasOffered) {
      wasOffered = now;
      document.dispatchEvent(new Event("pitwall-offered"));
    }
  }
  if (!visible || !renderer) return;
  if (snapshot) {
    chooseScene();
    redraw();
    if (inspection) showDetails();
    return;
  }
  // An action's phase can change in any patch, one carrying the scene's state too (a calibration
  // asks for driving while it runs): the details' button says what it does, so it's kept current.
  if (inspection && "actions" in patch) showDetails();
  const mine = patch.scenes && sceneId ? patch.scenes[sceneId] : null;
  if (!mine) return;
  if ("layers" in mine || "title" in mine || "focus" in mine) {
    redraw();
    return;
  }
  if ("state" in mine && !held) {
    renderer.apply(scene().state || {}, replaced ? null : mine.state);
  }
}

/** Draws the scene afresh: described again if its description changed, then all of its state. */
function redraw() {
  if (!visible) return;
  if (!renderer) {
    showMessage(unavailable || (offered() ? "Loading the 3D view…" : "No scene."));
    return;
  }
  const now = scene();
  if (!now) {
    showMessage("The robot program has no scene to show.");
    return;
  }
  showMessage("");
  ui.title.textContent = now.title || "World";
  const description = JSON.stringify({ focus: now.focus, layers: now.layers });
  if (description !== describedAs) {
    describedAs = description;
    renderer.describe(now);
    layerSwitches(now);
    ui.follow.hidden = !now.focus;
    renderer.follow(following && now.focus ? now.focus : null);
  }
  if (!held) renderer.apply(now.state || {}, null);
  renderer.resize();
}

function layerSwitches(now) {
  const switches = [];
  for (const [id, layer] of Object.entries(now.layers || {})) {
    const on = renderer.layers.get(id) ? renderer.layers.get(id).shown : layer.shown !== false;
    const button = h("button", { type: "button", class: "chip", "aria-pressed": String(on),
                                 "data-layer": id }, (on ? "✓ " : "") + (layer.title || id));
    button.addEventListener("click", () => {
      const shown = button.getAttribute("aria-pressed") !== "true";
      renderer.showLayer(id, shown);
      button.setAttribute("aria-pressed", String(shown));
      button.textContent = (shown ? "✓ " : "") + (layer.title || id);
      if (!shown && inspected && inspected.layer === id) closeDetails();
    });
    switches.push(button);
  }
  ui.layers.replaceChildren(...switches);
}

// ---- The renderer ----

/** Loads three.js and the renderer, once: or says why this browser can't draw them. */
function load() {
  if (loading) return loading;
  loading = import("./scene-render.js").then((module) => {
    if (!module.supported()) {
      unavailable = "This browser can't draw 3D: it has no WebGL 2. Everything else in Pitwall"
          + " works; try another browser, or turn on hardware acceleration.";
      return;
    }
    try {
      renderer = new module.SceneRenderer(ui.canvas, () => {
        showMessage("The browser took the 3D drawing away (its graphics were reset). Reload the"
            + " page to draw it again.");
      });
    } catch (error) {
      unavailable = "The 3D view couldn't start: " + error.message;
    }
  }).catch((error) => {
    unavailable = "The 3D view couldn't load: " + error.message;
  });
  return loading;
}

function showMessage(text) {
  ui.message.textContent = text;
  ui.message.hidden = !text;
}

function camera(view) {
  if (!renderer) return;
  renderer.use(view);
  ui.top.setAttribute("aria-pressed", String(view === "top"));
  ui.around.setAttribute("aria-pressed", String(view !== "top"));
}

function toggleFollow() {
  const now = scene();
  following = !following && Boolean(now && now.focus);
  ui.follow.setAttribute("aria-pressed", String(following));
  if (renderer) renderer.follow(following ? now.focus : null);
}

function toggleHold() {
  held = !held;
  ui.hold.setAttribute("aria-pressed", String(held));
  ui.hold.textContent = held ? "▶ Release" : "❚❚ Hold";
  ui.held.hidden = !held;
  // Let go, it catches up with everything the robot said meanwhile, at once.
  if (!held && renderer && scene()) renderer.apply(scene().state || {}, null);
}

// ---- Inspecting ----

async function clicked(x, y) {
  if (!renderer) return;
  const picked = renderer.pick(x, y);
  if (!picked) {
    closeDetails();
    return;
  }
  inspected = picked;
  inspection = null;
  clickedX = x - ui.canvas.getBoundingClientRect().left;
  renderer.select(picked.id);
  showDetails("Asking the robot…");
  const result = await send("GET", "/api/scenes/" + encodeURIComponent(sceneId) + "/inspect/"
      + encodeURIComponent(picked.id));
  if (inspected !== picked) return;
  if (!result.ok) {
    showDetails(result.problem.detail || "The robot has nothing to say about it now.");
    return;
  }
  inspection = result.body;
  showDetails();
}

/** The details panel: what the robot said of the shape clicked, ending in what can be done. */
function showDetails(waiting = "") {
  if (!inspected) return;
  detailsShown(true);
  const close = h("button", { type: "button", class: "link world-close", "aria-label": "Close",
                              onclick: closeDetails }, "✕");
  if (!inspection) {
    replace(ui.details, h("div", { class: "world-details-head" },
      h("h2", null, inspected.id), close), h("p", { class: "muted" }, waiting));
    return;
  }
  const details = h("dl", { class: "world-facts" },
    inspection.details.map((detail) => [h("dt", null, detail.label), h("dd", null, detail.value)]));
  const actions = inspection.actions.map((asked) => {
    const now = (robot.doc && robot.doc.actions && robot.doc.actions[asked.name]) || asked;
    const running = now.phase === "RUNNING";
    // Stop is never locked; Run is, with why beside it, as on Pit.
    const locked = !running && (robot.connected ? now.locked : "Not connected to the robot.");
    return h("div", { class: "world-action" },
      h("button", { type: "button", class: running ? "danger" : "primary", disabled: Boolean(locked),
                    "data-action": asked.name, "data-run": running ? "cancel" : "start",
                    onclick: runAction },
        running ? "■ Stop" : "▶ Run"),
      h("span", null, asked.name),
      locked ? h("span", { class: "gate locked" }, "🔒 " + locked) : null);
  });
  replace(ui.details,
    h("div", { class: "world-details-head" }, h("h2", null, inspection.title), close),
    details,
    actions.length ? h("div", { class: "world-actions" }, actions) : null);
}

/**
 * Runs or stops an action from the panel, as Pit would: what the button says, its name and whether
 * it starts or stops it on the button itself. Never the opposite of what it shows: a Stop pressed
 * just as the action ended asks to cancel it, which leaves it stopped, rather than start it again.
 */
async function runAction(event) {
  const { action: name, run } = event.currentTarget.dataset;
  const stopping = run === "cancel";
  const result = await send("POST", "/api/actions", stopping ? { cancel: name } : { start: name });
  if (!result.ok) say(name + ": " + result.problem.detail, true);
  else say((stopping ? "Stopped: " : "Started: ") + name + ".");
}

function closeDetails() {
  inspected = null;
  inspection = null;
  detailsShown(false);
  if (renderer) renderer.select(null);
}

/**
 * Shows or hides the details beside the drawing, which takes or gives back their width: the
 * drawing is fitted to what it has then, keeping the point clicked in sight.
 */
function detailsShown(shown) {
  if (ui.details.hidden === !shown) return;
  ui.details.hidden = !shown;
  if (renderer && visible) renderer.refit(shown ? clickedX : null);
}

export default { id: "world", title: "World", mount, show, hide, route, offered };
