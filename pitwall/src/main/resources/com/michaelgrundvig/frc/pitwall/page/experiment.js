// Experiment: run a test motion on a rig, see how it answered, change a gain, run again, compare.
// Runs stay in this tab: they outlive reconnects and restarts of the robot.

import { robot, onChange, onSamples, api, send, lastRequestAt, shown, h } from "./core.js";
import { applyEdits, builderLines, sayResult } from "./edits.js";

// Every run this tab has seen, oldest first.
const runs = [];
// Whether this page is starting a run: from the click on Run until the robot answers.
let starting = false;
// How many of a rig's runs its plot lays over each other, the latest in bold.
const OVERLAID = 4;
// How many of a rig's runs keep their traces, so another signal can be plotted; scores stay.
const TRACES_KEPT = 8;

const rigViews = new Map(); // rig name -> its panel's parts
let shownRigs = "";
let container;
let empty;
let visible = false;

function mount(view) {
  container = h("div");
  empty = h("p", { class: "muted", hidden: true }, "This robot has no rigs to experiment on yet. "
      + "A mechanism adds one in code: pitwall.experiments().add(new Experiment(...)).");
  view.append(
    h("h1", null, "Experiment"),
    h("p", { class: "muted" }, "Each runs a test motion on its mechanism and scores how it "
        + "answered. They run only with the robot enabled in utility mode, never on the field, "
        + "and only while this page stays open and in touch: close it, or lose the link, and the "
        + "run stops. Change a gain, run again, and compare. Runs stay here until this tab closes."),
    empty,
    container);
  onChange((patch, snapshot) => {
    noteRun();
    render(patch, snapshot);
  });
  onSamples((trace, fromStart) => {
    takeSamples(trace, fromStart);
    drawSoon();
  });
}

function show() {
  visible = true;
  for (const [name, view] of rigViews) drawRig(name, view, true);
}

function hide() {
  visible = false;
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
  const key = JSON.stringify([doc.rigs, doc.experiments]);
  if (snapshot || key !== shownRigs) {
    shownRigs = key;
    buildRigs();
  }
  empty.hidden = rigViews.size > 0;
  for (const [name, view] of rigViews) updateRig(name, view);
}

function buildRigs() {
  const doc = robot.doc;
  container.replaceChildren();
  for (const built of rigViews.values()) if (built.plot) built.plot.destroy();
  rigViews.clear();
  for (const [name, rig] of Object.entries(doc.rigs)) {
    const view = { rig, gains: new Map(), plot: null, plotKey: "", scoresKey: "", runsPlotted: "",
                   expanded: new Set() };
    const gainInputs = () => [...view.gains].map(([path, gain]) => [path, gain.input]);

    // Its gains, beside its plot: the same values as the tuning table, checked the same way.
    const body = h("tbody");
    for (const path of rig.gains) {
      const described = doc.tunables[path] || { field: path, unit: "", limits: "" };
      const committed = h("td", { class: "number" });
      const input = h("input", { inputmode: "decimal", "aria-label": name + " " + described.field,
                                 title: described.limits ? "Limits: " + described.limits : null });
      input.addEventListener("input", () => (input.dataset.edited = "yes"));
      input.addEventListener("keydown", (event) => {
        if (event.key === "Enter") applyEdits(gainInputs(), view.say);
      });
      const row = h("tr", null,
        h("td", null, described.field.slice(described.field.lastIndexOf(".") + 1)),
        committed, h("td", null, input), h("td", null, described.unit));
      body.append(row);
      view.gains.set(path, { row, input, committed });
    }
    view.apply = h("button", { class: "primary", onclick: () => applyEdits(gainInputs(), view.say) },
      "Apply gains");

    // Pick an experiment; run it; cancel it.
    view.choice = h("select", { "aria-label": name + " experiment",
                                onchange: () => updateRig(name, view) });
    for (const [experiment, described] of Object.entries(doc.experiments)) {
      if (described.rig === name) view.choice.append(h("option", { value: experiment }, experiment));
    }
    view.runButton = h("button", { class: "primary", onclick: () => startRun(view.choice.value, view) },
      "Run");
    // Cancels the run in progress, whichever experiment is chosen now.
    view.cancelButton = h("button", { onclick: async () => {
      const run = robot.doc.run;
      if (run && run.phase === "RUNNING") {
        sayResult(await send("POST", "/api/experiments", { cancel: run.experiment, run: run.id }),
            view.say);
      }
    } }, "Cancel");
    view.settings = h("div", { class: "muted" });
    view.state = h("div", { style: "margin-top: 8px" });
    const message = h("div", { class: "message bad", hidden: true });
    view.say = (text, bad) => {
      message.hidden = !text;
      message.textContent = text || "";
      message.classList.toggle("bad", Boolean(bad));
    };

    // The builder calls that commit its tuned gains.
    view.commitTitle = h("div", { class: "muted", style: "margin-top: 16px" },
      "To keep the tuned gains, write these into the configuration:");
    view.calls = h("pre");

    // Its plot: the chosen signal of its last few runs, the latest in bold, over the setpoint.
    view.pick = h("select", { "aria-label": name + " signal to plot",
                              onchange: () => drawRig(name, view, true) });
    for (const signal of [...rig.measurements, ...rig.signals.map((s) => s.name)]) {
      view.pick.append(h("option", { value: signal }, signal));
    }
    view.plotElement = h("div", { class: "plot" });

    // Every run of this rig, the latest first, with the best in each column marked.
    view.scores = h("table", { class: "scores" });

    container.append(h("section", { class: "rig" },
      h("h3", null, name, h("span", { class: "muted" }, "limits " + rig.limits)),
      h("div", { class: "rig-grid" },
        h("div", null,
          h("table", null,
            h("thead", null, h("tr", null, h("th", null, "Gain"), h("th", null, "Committed"),
              h("th", null, "Now"), h("th"))),
            body),
          h("div", { class: "controls", style: "margin-top: 8px" }, view.apply),
          h("div", { class: "controls", style: "margin-top: 16px" },
            view.choice, view.runButton, view.cancelButton),
          view.settings, view.state, message, view.commitTitle, view.calls),
        h("div", null,
          h("label", { class: "controls muted" }, "Plot ", view.pick),
          view.plotElement)),
      h("div", { class: "scroll" }, view.scores)));
    rigViews.set(name, view);
  }
}

/** Starts a run, keeping it going from the click: the robot answers once it has started. */
async function startRun(experiment, view) {
  starting = true;
  try {
    sayResult(await send("POST", "/api/experiments", { start: experiment }), view.say);
  } finally {
    starting = false;
  }
}

function updateRig(name, view) {
  const doc = robot.doc;
  const session = doc.tuning;
  // While its run runs, its gains stay put here: a run is scored for the gains it started with.
  const busy = Boolean(doc.run && doc.run.rig === name && doc.run.phase === "RUNNING");
  view.apply.disabled = !session.enabled || busy;
  for (const [path, { row, input, committed }] of view.gains) {
    const described = doc.tunables[path];
    committed.textContent = described ? shown(described.committed) : "";
    row.classList.toggle("changed", path in doc.changes);
    input.disabled = !session.enabled || busy;
    if (!input.dataset.edited && document.activeElement !== input) {
      input.value = shown(doc.values[path]);
    }
  }
  const described = doc.experiments[view.choice.value];
  view.settings.textContent = described ? described.settings : "";
  const current = doc.run && doc.run.rig === name ? doc.run : null;
  const running = Boolean(current) && current.phase === "RUNNING";
  view.runButton.disabled = running;
  view.cancelButton.disabled = !running;
  view.state.textContent = current ? `Run ${current.id}, ${current.experiment}: ${current.detail}` : "";
  const lines = builderLines(view.rig.gains);
  view.calls.textContent = lines.join("\n");
  view.calls.hidden = lines.length === 0;
  view.commitTitle.hidden = lines.length === 0;
  // The scores only when a run or the lock changed; the plot only when a run came or went
  // (samples arriving redraw it too: see drawSoon).
  const mine = runs.filter((run) => run.rig === name);
  const scoresKey = JSON.stringify([
    mine.map((run) => [run.key, run.phase, run.detail, Boolean(run.scores), run.gainsChanged]),
    session.enabled, busy, doc.robot.program, doc.robot.code, [...view.expanded]]);
  view.busy = busy;
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
  if (mine.length === 0) return;
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
  head.append(h("th", null, "Run"), h("th", null, "Gains"));
  for (const column of columns) {
    head.append(h("th", { class: "number", title: "The worst of every jump of every "
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
    if (index === 0) row.className = "latest";
    const label = row.insertCell();
    if (summaries[index] && view.rig.measurements.length > 1) {
      const open = view.expanded.has(run.key);
      label.append(h("button", { class: "small",
        "aria-label": (open ? "Hide" : "Show") + " each measurement of run " + run.id,
        onclick: () => {
          if (open) view.expanded.delete(run.key); else view.expanded.add(run.key);
          updateRig(name, view);
        } }, open ? "▾" : "▸"), " ");
    }
    label.append(`${run.id} ${run.experiment}`);
    if (run.gainsChanged) label.append(" ", h("span", { class: "tag", title: run.detail }, "gains changed"));
    if (run.program !== doc.robot.program) {
      label.append(" ", h("span", { class: "tag",
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
      cell.className = "number" + (cells === row && !run.gainsChanged && Number.isFinite(value)
          && value === best[c] ? " best" : "");
      cell.textContent = Number.isFinite(value) ? column.show(value)
          : value === Infinity ? "never" : "—";
    });
    if (!s) {
      const cell = row.insertCell();
      cell.colSpan = columns.length;
      cell.className = "muted";
      cell.textContent = run.phase === "RUNNING" ? "Running…" : run.detail;
    } else {
      show(s, row);
    }
    row.insertCell().append(h("button", { class: "small", title: "Put this run's gains back",
      disabled: !doc.tuning.enabled || view.busy,
      onclick: async () => sayResult(await send("PATCH", "/api/values", run.gains), view.say) },
      "Restore"));
    if (s && view.expanded.has(run.key)) {
      for (const measurement of view.rig.measurements) {
        const wheel = body.insertRow();
        wheel.className = "muted";
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
  if (!visible) return; // drawn when the view is shown: a hidden plot has no width
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
    label: `Run ${run.id}`, t: run.trace[signal].t, v: run.trace[signal].v, latest: run === latest }));
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
  const axis = { stroke: color("--muted"), grid: { stroke: color("--line") },
                 ticks: { stroke: color("--line") } };
  view.plot = new uPlot({
    width: Math.max(280, view.plotElement.clientWidth),
    height: 280,
    scales: { x: { time: false } },
    axes: [{ ...axis, label: "s" }, { ...axis, label: unit }],
    cursor: { drag: { x: true, y: false } },
    series: [{ label: "t" }, ...lines.map((line) => ({
      label: line.label,
      stroke: line.setpoint ? color("--setpoint") : line.latest ? color("--accent") : color("--older"),
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

export default { id: "experiment", title: "Experiment", mount, show, hide };
