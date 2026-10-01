// Experiment: run a test motion on a rig, see how it answered, change a gain, run again, compare.
// Runs stay in this tab: they outlive reconnects and restarts of the robot.

import { robot, onChange, onSamples, api, send, lastRequestAt, shown, h, say } from "./core.js";
import { onEdits } from "./edits.js";
import { makeRow, updateRow, replace } from "./rows.js";

// Every run this tab has seen, oldest first.
const runs = [];
// Whether this page is starting a run: from the click on Run until the robot answers.
let starting = false;
// How many of a rig's runs its plot lays over each other, the latest in bold.
const OVERLAID = 4;
// How many of a rig's runs keep their traces, so another signal can be plotted; scores stay.
const TRACES_KEPT = 8;

const rigViews = new Map(); // rig name -> its panel's parts
let builtFrom = "";
let chips;
let panels;
let empty;
let chosen = null; // the rig shown, by name
let visible = false;

function mount(view) {
  chips = h("div", { class: "chips", role: "group", "aria-label": "Rig" });
  panels = h("div");
  empty = h("p", { class: "muted", hidden: true }, "This robot has no rigs to experiment on yet. "
      + "A mechanism adds one in code: pitwall.experiments().add(new Experiment(...)).");
  view.append(
    h("h1", { class: "view-title" }, "Experiment"),
    h("p", { class: "lede" }, "Change a gain, run a test motion through the mechanism's own "
        + "command, and compare the runs. Runs stay in this tab until it closes."),
    chips, empty, panels);
  onChange((patch, snapshot) => {
    noteRun();
    render(patch, snapshot);
  });
  onSamples((trace, fromStart) => {
    takeSamples(trace, fromStart);
    drawSoon();
  });
  onEdits(() => {
    for (const view of rigViews.values()) view.rows.forEach((row) => updateGain(view, row));
  });
  // Plots take their colors from the theme: a new theme redraws them.
  document.addEventListener("pitwall-theme", redrawAll);
  matchMedia("(prefers-color-scheme: dark)").addEventListener("change", redrawAll);
}

function show() {
  visible = true;
  redrawAll();
}

function hide() {
  visible = false;
}

/** The rest of the address names a rig: #experiment/Steering. */
function route(rest) {
  if (rest && rest !== chosen) {
    chosen = rest;
    if (robot.doc) render({}, false);
  }
}

function redrawAll() {
  for (const [name, view] of rigViews) drawRig(name, view, true);
}

// ---- The deadman ----

/** Whether this page keeps a run going: it's starting one, or one it started is running. */
function inCharge() {
  const doc = robot.doc;
  return starting || Boolean(doc && doc.run && doc.run.phase === "RUNNING"
      && robot.stream !== "" && doc.run.controller === robot.stream);
}

// While this page keeps a run going, the robot must hear from it every 500 ms, so it sends a
// heartbeat whenever it has been quiet for 100 ms. One that hangs is dropped.
setInterval(() => {
  if (inCharge() && Date.now() - lastRequestAt() >= 100) {
    api("POST", "/api/heartbeat", undefined, { signal: AbortSignal.timeout(400) }).catch(() => {});
  }
}, 25);

// A hidden tab's timers slow to once a second, too slow to keep a run going. So a run this page
// keeps going stops as the tab is hidden, saying so, rather than when the deadman runs out.
document.addEventListener("visibilitychange", () => {
  if (document.hidden && inCharge() && robot.doc.run) {
    api("POST", "/api/experiments",
        { cancel: robot.doc.run.experiment, run: robot.doc.run.id, reason: "this tab was hidden" },
        { keepalive: true }).catch(() => {});
  }
});

// ---- Runs ----

function runKey(program, id) {
  return program + ":" + id;
}

/** Keeps the run the robot has now, or updates it: where it stands, and its scores. */
function noteRun() {
  const doc = robot.doc;
  const now = doc.run;
  if (!now || now.id === undefined) return;
  const key = runKey(doc.robot.program, now.id);
  let run = runs.find((r) => r.key === key);
  if (!run) {
    run = { key, program: doc.robot.program, code: doc.robot.code, robot: doc.robot.name,
            simulated: doc.robot.simulated, trace: {} };
    runs.push(run);
    // Scores for every run, but traces only for a rig's latest few: the plot overlays fewer.
    runs.filter((r) => r.rig === now.rig && r !== run).slice(0, -(TRACES_KEPT - 1))
      .forEach((r) => (r.trace = {}));
  }
  // A copy: the next run's patches change the document's run in place.
  Object.assign(run, structuredClone({
    id: now.id, experiment: now.experiment, rig: now.rig, settings: now.settings,
    gains: now.gains, signals: now.signals, phase: now.phase, detail: now.detail,
    scores: now.scores, gainsChanged: now.gainsChanged,
  }));
}

/** Adds a run's samples to its trace; a snapshot's start it over. */
function takeSamples(trace, fromStart) {
  if (!robot.doc) return;
  const run = runs.find((r) => r.key === runKey(robot.doc.robot.program, trace.run));
  if (!run) return;
  if (fromStart) run.trace = {};
  for (const [name, { t, v }] of Object.entries(trace.signals)) {
    const series = run.trace[name] || (run.trace[name] = { t: [], v: [] });
    for (let i = 0; i < t.length; i++) {
      series.t.push(t[i]);
      series.v.push(v[i]);
    }
  }
}

// ---- Rendering ----

function render(patch, snapshot) {
  const doc = robot.doc;
  const key = JSON.stringify([doc.rigs, doc.experiments, Object.keys(doc.tunables)]);
  if (snapshot || key !== builtFrom) {
    builtFrom = key;
    buildRigs();
  }
  const names = [...rigViews.keys()];
  if (!rigViews.has(chosen)) chosen = names[0] || null;
  empty.hidden = names.length > 0;
  chips.hidden = names.length < 2;
  replace(chips, names.map((name) => h("a", { class: "chip", href: "#experiment/" + name,
    "aria-current": name === chosen ? "true" : null }, name)));
  for (const [name, view] of rigViews) {
    view.panel.hidden = name !== chosen;
    updateRig(name, view);
  }
}

function buildRigs() {
  const doc = robot.doc;
  panels.replaceChildren();
  for (const built of rigViews.values()) if (built.plot) built.plot.destroy();
  rigViews.clear();
  for (const [name, rig] of Object.entries(doc.rigs)) {
    const view = { rig, rows: [], plot: null, plotKey: "", scoresKey: "", runsPlotted: "",
                   expanded: new Set() };

    // Its gains, beside its plot: the same values, and the same rows, as Tune's.
    const gains = h("div", { class: "rowlist" });
    const unpublished = [];
    for (const path of rig.gains) {
      if (!doc.tunables[path]) {
        unpublished.push(path);
        continue;
      }
      const row = makeRow(path, "exp");
      view.rows.push(row);
      gains.append(row.el);
    }

    // Pick an experiment; run it, or stop it.
    view.choice = h("select", { "aria-label": name + ": experiment",
                                onchange: () => updateRig(name, view) });
    for (const [experiment, described] of Object.entries(doc.experiments)) {
      if (described.rig === name) view.choice.append(h("option", { value: experiment }, experiment));
    }
    view.runButton = h("button", { type: "button", class: "big primary",
                                   onclick: () => runOrStop(name, view) });
    view.state = h("p", { class: "state" });
    view.calls = h("pre");
    view.commit = h("div", { class: "commit" },
      h("p", { class: "meta" }, "To keep the tuned gains, write these into the configuration:"),
      view.calls);

    // Its plot: the chosen signal of its last few runs, the latest in bold, over the setpoint.
    view.pick = h("select", { "aria-label": name + ": signal to plot",
                              onchange: () => drawRig(name, view, true) });
    for (const signal of [...rig.measurements, ...rig.signals.map((s) => s.name)]) {
      view.pick.append(h("option", { value: signal }, signal));
    }
    view.plotElement = h("div", { class: "plot" });
    view.scores = h("table", { class: "scores" });

    view.panel = h("div", { class: "exp", hidden: true },
      h("section", { class: "card" },
        h("h2", null, name, h("span", { class: "meta" }, "within " + rig.limits)),
        h("label", { class: "field" }, "Experiment", view.choice),
        h("h3", { class: "meta" }, "Gains"),
        gains,
        unpublished.length > 0 && h("p", { class: "meta" }, "Not tunable, so not shown: "
            + unpublished.join(", ")),
        h("div", { class: "runbox" }, view.runButton, view.state,
          h("p", { class: "dead" }, "Stops if this page goes quiet for half a second, or the "
              + "robot disables. The gains at the start are recorded with the run.")),
        view.commit),
      h("section", { class: "card" },
        h("div", { class: "plotbar" }, h("h2", null, "Runs"),
          h("label", { class: "meta" }, "Plot ", view.pick)),
        view.plotElement,
        h("div", { class: "scroll" }, view.scores)));
    panels.append(view.panel);
    rigViews.set(name, view);
  }
}

/** Starts a run, keeping it going from the click; or stops the one running. */
async function runOrStop(name, view) {
  const run = robot.doc.run;
  if (run && run.rig === name && run.phase === "RUNNING") {
    const result = await send("POST", "/api/experiments", { cancel: run.experiment, run: run.id });
    if (!result.ok) say(result.problem.detail, true);
    return;
  }
  starting = true;
  try {
    const result = await send("POST", "/api/experiments", { start: view.choice.value });
    if (!result.ok) say(result.problem.detail, true);
  } finally {
    starting = false;
  }
}

/** A rig's gain: as in Tune, except that it stays put while its run runs. */
function updateGain(view, row) {
  updateRow(row);
  if (view.busy) row.input.disabled = true;
}

function updateRig(name, view) {
  const doc = robot.doc;
  // While its run runs, its gains stay put here: a run is scored for the gains it started with.
  view.busy = Boolean(doc.run && doc.run.rig === name && doc.run.phase === "RUNNING");
  view.rows.forEach((row) => updateGain(view, row));
  const current = doc.run && doc.run.rig === name ? doc.run : null;
  const running = view.busy;
  view.runButton.className = running ? "big danger" : "big primary";
  view.runButton.textContent = running ? "■ Stop" : "▶ Run";
  const described = doc.experiments[view.choice.value];
  view.state.textContent = current
    ? `Run ${current.id}, ${current.experiment}: ${current.detail}`
    : described ? described.settings : "";
  const lines = view.rig.gains.filter((path) => doc.changes[path])
    .map((path) => doc.changes[path].builderCall + "   // was " + doc.changes[path].committed);
  view.calls.textContent = lines.join("\n");
  view.commit.hidden = lines.length === 0;
  // The scores only when a run or the lock changed; the plot only when a run came or went
  // (samples arriving redraw it too: see drawSoon).
  const mine = runs.filter((run) => run.rig === name);
  const scoresKey = JSON.stringify([
    mine.map((run) => [run.key, run.phase, run.detail, Boolean(run.scores), run.gainsChanged]),
    doc.tuning.enabled, view.busy, doc.robot.program, doc.robot.code, [...view.expanded]]);
  if (scoresKey !== view.scoresKey) {
    view.scoresKey = scoresKey;
    renderScores(name, view);
  }
  const plotKey = mine.map((run) => run.key).join(",");
  if (plotKey !== view.runsPlotted) {
    view.runsPlotted = plotKey;
    drawRig(name, view, false);
  }
}

/**
 * A run's scores, the worst of every jump: rise, overshoot, settling, and steady-state error (by
 * its size), and each peak. A jump that never got there, or never settled, is worse than any that
 * did. With {@code only}, a measurement's own jumps and signals (those named after it, such as
 * "Front left voltage" for "Front left"); without it, the worst measurement in each column.
 */
function summary(run, only) {
  if (!run.scores) return null;
  const jumps = Object.entries(run.scores.steps)
    .filter(([measurement]) => only === undefined || measurement === only)
    .flatMap(([, steps]) => steps);
  const worst = (key, size = (x) => x) => jumps.length === 0 ? NaN
      : Math.max(...jumps.map((jump) => typeof jump[key] === "number" ? size(jump[key]) : Infinity));
  const peaks = {};
  for (const signal of run.signals || []) {
    if (only !== undefined && !signal.name.startsWith(only + " ")) continue;
    const peak = run.scores.peaks[signal.name];
    if (typeof peak === "number") peaks[signal.unit] = Math.max(peaks[signal.unit] ?? 0, peak);
  }
  return {
    rise: worst("rise"),
    overshoot: worst("overshoot"),
    settling: worst("settling"),
    error: worst("error", Math.abs),
    peaks,
  };
}

/** A score to three figures, as {@link shown} writes numbers; anything under a thousandth so. */
function small(x) {
  return Math.abs(x) < 0.001 ? "< 0.001" : shown(Number(x.toPrecision(3)));
}

function renderScores(name, view) {
  const doc = robot.doc;
  const mine = runs.filter((run) => run.rig === name).reverse();
  const table = view.scores;
  table.replaceChildren();
  if (mine.length === 0) {
    table.append(h("caption", { class: "muted" }, "No runs yet: press Run."));
    return;
  }
  const summaries = mine.map((run) => summary(run));
  const units = [...new Set(summaries.flatMap((s) => (s ? Object.keys(s.peaks) : [])))];
  const unit = view.rig.unit;
  const columns = [
    { label: "Rise", get: (s) => s.rise, show: (x) => (x * 1000).toFixed(0) + " ms" },
    { label: "Overshoot", get: (s) => s.overshoot, show: (x) => (x * 100).toFixed(1) + "%" },
    { label: "Settling", get: (s) => s.settling, show: (x) => (x * 1000).toFixed(0) + " ms" },
    { label: "Error", get: (s) => s.error, show: (x) => small(x) + " " + unit },
    ...units.map((u) => ({ label: "Peak " + u, get: (s) => s.peaks[u],
                           show: (x) => small(x) + " " + u })),
  ];
  const head = table.createTHead().insertRow();
  head.append(h("th", { scope: "col" }, "Run"), h("th", { scope: "col" }, "Gains"));
  for (const column of columns) {
    head.append(h("th", { scope: "col", title: "The worst of every jump of every "
        + (view.rig.measurements.length > 1
          ? "measurement (" + view.rig.measurements.join(", ") + "): ▸ shows each" : "jump") },
      column.label));
  }
  head.append(h("th"));
  // The best in each column, when there's more than one fair run to compare: a run whose gains
  // changed while it ran answers for more than one set of them.
  const best = columns.map((column) => {
    const values = summaries
      .map((s, i) => (s && !mine[i].gainsChanged ? column.get(s) : NaN))
      .filter(Number.isFinite);
    return values.length > 1 ? Math.min(...values) : NaN;
  });
  const body = table.createTBody();
  mine.forEach((run, index) => {
    const row = body.insertRow();
    if (index === 0) row.className = "sel";
    const label = row.insertCell();
    if (summaries[index] && view.rig.measurements.length > 1) {
      const open = view.expanded.has(run.key);
      label.append(h("button", { type: "button", class: "small",
        "aria-label": (open ? "Hide" : "Show") + " each measurement of run " + run.id,
        "aria-expanded": String(open),
        onclick: () => {
          if (open) view.expanded.delete(run.key); else view.expanded.add(run.key);
          updateRig(name, view);
        } }, open ? "▾" : "▸"), " ");
    }
    label.append(`#${run.id} ${run.experiment}`);
    if (run.gainsChanged) {
      label.append(" ", h("span", { class: "tag refused", title: run.detail }, "gains changed"));
    }
    if (run.program !== doc.robot.program) {
      label.append(" ", h("span", { class: "tag plain",
        title: "Ran on " + run.robot + (run.simulated ? " (simulator)" : "") + ", code " + run.code },
        run.code !== doc.robot.code ? "other code" : "before a restart"));
    }
    row.insertCell().textContent = Object.entries(run.gains)
      .map(([path, value]) => path.slice(path.lastIndexOf("/") + 1) + " " + shown(value))
      .join(", ");
    const s = summaries[index];
    const show = (values, cells) => columns.forEach((column, c) => {
      const value = column.get(values);
      const cell = cells.insertCell();
      const isBest = cells === row && !run.gainsChanged && Number.isFinite(value) && value === best[c];
      if (isBest) {
        cell.className = "best";
        cell.title = "The best of these runs";
      }
      cell.textContent = Number.isFinite(value) ? column.show(value)
          : value === Infinity ? "never" : "—";
    });
    if (!s) {
      const cell = row.insertCell();
      cell.colSpan = columns.length;
      cell.className = run.phase === "RUNNING" ? "muted" : "stopped";
      cell.textContent = run.phase === "RUNNING" ? "Running…" : run.detail;
    } else {
      show(s, row);
    }
    row.insertCell().append(h("button", { type: "button", class: "small",
      title: "Put this run's gains back", disabled: !doc.tuning.enabled || view.busy,
      onclick: async () => {
        const result = await send("PATCH", "/api/values", run.gains);
        if (!result.ok) say(result.problem.detail, true);
      } }, "Restore"));
    if (s && view.expanded.has(run.key)) {
      for (const measurement of view.rig.measurements) {
        const wheel = body.insertRow();
        wheel.className = "sub";
        wheel.insertCell();
        wheel.insertCell().textContent = measurement;
        show(summary(run, measurement), wheel);
        wheel.insertCell();
      }
    }
  });
}

let drawPending = false;

/** Redraws the plots once, before the next frame: samples arrive every loop. */
function drawSoon() {
  if (drawPending) return;
  drawPending = true;
  requestAnimationFrame(() => {
    drawPending = false;
    for (const [name, view] of rigViews) drawRig(name, view, false);
  });
}

function color(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

/** Lines on one time axis, as uPlot takes them: each line's value where it has one, else null. */
function aligned(lines) {
  const xs = [...new Set(lines.flatMap((line) => line.t))].sort((a, b) => a - b);
  const at = new Map(xs.map((x, i) => [x, i]));
  return [xs, ...lines.map((line) => {
    const ys = new Array(xs.length).fill(null);
    line.t.forEach((t, i) => (ys[at.get(t)] = line.v[i]));
    return ys;
  })];
}

function drawRig(name, view, rebuild) {
  // Drawn when it's on screen: a hidden plot has no width.
  if (!visible || name !== chosen) return;
  if (typeof uPlot === "undefined") {
    if (!view.plotElement.firstChild) {
      view.plotElement.append(h("div", { class: "notice muted" },
          "Plots need uPlot, which isn't bundled with this build of Pitwall. "
          + "The scores below are complete without it."));
    }
    return;
  }
  const signal = view.pick.value;
  const overlaid = runs.filter((run) => run.rig === name && run.trace[signal]).slice(-OVERLAID);
  if (overlaid.length === 0) return;
  const latest = overlaid[overlaid.length - 1];
  const lines = overlaid.map((run) => ({
    label: `#${run.id}`, t: run.trace[signal].t, v: run.trace[signal].v, latest: run === latest }));
  const setpoint = view.rig.measurements.includes(signal) && latest.trace.Setpoint;
  if (setpoint) lines.unshift({ label: "Setpoint", t: setpoint.t, v: setpoint.v, setpoint: true });
  const data = aligned(lines);
  const key = signal + "|" + lines.map((line) => line.label).join(",");
  if (view.plot && view.plotKey === key && !rebuild) {
    view.plot.setData(data);
    return;
  }
  if (view.plot) view.plot.destroy();
  view.plotElement.replaceChildren();
  const unit = (latest.signals || []).find((s) => s.name === signal)?.unit || "";
  const stepped = uPlot.paths && uPlot.paths.stepped ? uPlot.paths.stepped({ align: 1 }) : undefined;
  const axis = { stroke: color("--muted"), grid: { stroke: color("--line"), width: 1 },
                 ticks: { stroke: color("--line"), width: 1 } };
  view.plot = new uPlot({
    width: Math.max(280, view.plotElement.clientWidth),
    height: 280,
    scales: { x: { time: false } },
    axes: [{ ...axis, label: "s" }, { ...axis, label: unit }],
    cursor: { drag: { x: true, y: false } },
    series: [{ label: "t" }, ...lines.map((line) => ({
      label: line.label,
      stroke: line.setpoint ? color("--text-2") : line.latest ? color("--accent") : color("--run-other"),
      width: line.latest ? 2.5 : line.setpoint ? 1.5 : 1,
      dash: line.setpoint ? [6, 4] : undefined,
      paths: line.setpoint ? stepped : undefined,
      spanGaps: true,
      points: { show: false },
    }))],
  }, data, view.plotElement);
  view.plotKey = key;
}

window.addEventListener("resize", () => {
  for (const view of rigViews.values()) {
    if (view.plot) view.plot.setSize({ width: Math.max(280, view.plotElement.clientWidth), height: 280 });
  }
});

export default { id: "experiment", title: "Experiment", mount, show, hide, route };
