// Experiment: run a test motion on a rig, see how it answered, change a gain, run again, compare.
// Runs stay in this tab: they outlive reconnects and restarts of the robot, and a reload, kept in
// the tab's sessionStorage.

import {
  robot, onChange, onConnection, onSamples, api, send, lastRequestAt, shown, h, say, announce,
  replace,
} from "./core.js";
import { tuningOffBanner } from "./bar.js";
import { onEdits, stageAll, pending, apply, edited, whyLocked } from "./edits.js";
import { load, trySave, cannotKeep } from "./history.js";
import { makeRow, updateRow } from "./rows.js";
import { valueDocs } from "./docs.js";
import { folds } from "./fold.js";

const KEY = "pitwall.runs";
// Every run this tab has seen, oldest first: kept through a reload, until the tab closes.
const runs = load(KEY, [], (kept) => Array.isArray(kept) && kept.every((run) => run
    && typeof run.key === "string" && typeof run.trace === "object" && run.gains));
// Element ids, each its own.
let ids = 0;
// Whether this page is starting a run: from the click on Run until the robot answers.
let starting = false;
// How many of a rig's runs its plot lays over each other, the latest in bold.
const OVERLAID = 4;
// How many of a rig's runs keep their traces, so another signal can be plotted; scores stay.
const TRACES_KEPT = 8;

/** The sections of the robot's state this view reads. */
const READS = ["run", "rigs", "experiments", "tunables", "values", "changes", "tuning", "robot"];

const rigViews = new Map(); // rig name -> its panel's parts
// The measurement each rig's plot shows, kept as the page rebuilds (a reconnect, a redeploy).
const picked = new Map(); // rig name -> measurement
const sections = folds("pitwall.experiment.folded");
// A signal's panel: shorter than the measurement's, which is drawn at PLOT_HEIGHT.
const PLOT_HEIGHT = 280;
const SIGNAL_HEIGHT = 130;
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
    tuningOffBanner("Gains"),
    chips, empty, panels);
  onChange((patch, snapshot) => {
    // Only what it reads: other sections (the status, the simulator's) change often, and say
    // nothing to a rig.
    if (!snapshot && !READS.some((section) => section in patch)) return;
    if (snapshot || "run" in patch || "robot" in patch) noteRun();
    render(patch, snapshot);
  });
  onSamples((trace, fromStart) => {
    takeSamples(trace, fromStart);
    // The time on Stop as the samples come, not with the plot's frames, which a hidden tab skips.
    for (const [name, view] of rigViews) {
      if (view.busy) view.runButton.textContent = "■ Stop  (Esc)" + countdown(name);
    }
    drawSoon();
  });
  onEdits(() => {
    for (const [name, view] of rigViews) updateRig(name, view);
  });
  onConnection(() => {
    if (robot.doc) render({}, false);
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
  const created = !run;
  if (!run) {
    run = { key, program: doc.robot.program, code: doc.robot.code, robot: doc.robot.name,
            simulated: doc.robot.simulated, trace: {} };
    runs.push(run);
    // Scores for every run, but traces only for a rig's latest few: the plot overlays fewer.
    runs.filter((r) => r.rig === now.rig && r !== run).slice(0, -(TRACES_KEPT - 1))
      .forEach((r) => (r.trace = {}));
  }
  const ended = run.phase === "RUNNING" && now.phase !== "RUNNING";
  // A copy: the next run's patches change the document's run in place.
  Object.assign(run, structuredClone({
    id: now.id, experiment: now.experiment, rig: now.rig, settings: now.settings,
    gains: now.gains, signals: now.signals, phase: now.phase, detail: now.detail,
    scores: now.scores, gainsChanged: now.gainsChanged,
  }));
  if (ended) {
    // A run that stopped short (the deadman, the robot disabling) is said at once; one that ran
    // its course, when the reader is free.
    if (now.phase === "ENDED") announce("Run " + now.id + " ended: " + now.detail + ".");
    else say("Run " + now.id + " stopped. " + now.detail + ".", true, true);
  }
  if (ended || (created && now.phase !== "RUNNING")) keepSoon();
}

let keeping = null;

/** Keeps the runs once a run's last samples are in: they arrive just after it ends. */
function keepSoon() {
  clearTimeout(keeping);
  keeping = setTimeout(keepRuns, 1000);
}

/**
 * Keeps the runs for this tab, as many as fit: the browser gives a page about 5 MB, and a trace is
 * most of a run, so the oldest runs' traces go first, their scores kept.
 */
function keepRuns() {
  // A copy is trimmed to fit, never the runs on the page: they keep their traces here.
  const copy = runs.map((run) => ({ ...run }));
  while (!trySave(KEY, copy)) {
    const oldest = copy.find((run) => Object.keys(run.trace).length > 0);
    if (!oldest) {
      cannotKeep();
      return;
    }
    oldest.trace = {};
  }
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
  for (const built of rigViews.values()) destroyPlots(built);
  rigViews.clear();
  for (const [name, rig] of Object.entries(doc.rigs)) {
    const view = { rig, rows: [], plot: null, plotKey: "", scoresKey: "", runsPlotted: "",
                   expanded: new Set(), signalPlots: new Map(), signalsKey: "" };

    // Its gains, beside its plot: the same values, and the same rows, as Tune's.
    const gains = h("div", { class: "rowlist" });
    const unpublished = [];
    for (const path of rig.gains) {
      if (!doc.tunables[path]) {
        unpublished.push(path);
        continue;
      }
      const row = makeRow(path, "exp", { select: openDocs });
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
    view.gate = h("p", { class: "gatemsg", id: "exp-gate-" + ++ids });
    view.staged = h("div", { class: "staged-note", id: "exp-staged-" + ++ids });
    view.calls = h("pre");
    view.commit = h("div", { class: "commit" },
      h("p", { class: "meta" }, "To keep the tuned gains, write these into the configuration:"),
      view.calls);

    // Its plot: the chosen measurement of its last few runs, the latest in bold, over the
    // setpoint; under it, each of the measurement's signals in the latest run, on the same time.
    view.pick = h("select", { "aria-label": name + ": measurement to plot", onchange: () => {
      picked.set(name, view.pick.value);
      drawRig(name, view, true);
    } });
    for (const measurement of rig.measurements) {
      view.pick.append(h("option", { value: measurement }, measurement));
    }
    if (rig.measurements.includes(picked.get(name))) view.pick.value = picked.get(name);
    // With one measurement there's nothing to choose.
    view.pickLabel = h("label", { class: "meta", hidden: rig.measurements.length < 2 }, "Plot ",
      view.pick);
    view.plotElement = h("div", { class: "plot" });
    view.signalsElement = h("div", { class: "signal-plots" });
    view.signals = sections.section("signals", "Signals, latest run",
      { className: "fold signals" }, view.signalsElement);
    view.signals.hidden = rig.signals.length === 0;
    // Folded, its plots have no width: drawn again as it opens.
    view.signals.addEventListener("toggle", () => {
      if (view.signals.open) drawRig(name, view, true);
    });
    view.scores = h("table", { class: "scores" });

    view.panel = h("div", { class: "exp", hidden: true },
      h("section", { class: "card" },
        h("h2", null, name, h("span", { class: "meta" }, "within " + rig.limits)),
        h("label", { class: "field" }, "Experiment", view.choice),
        h("h3", { class: "meta" }, "Gains"),
        gains,
        unpublished.length > 0 && h("p", { class: "meta" }, "Not tunable, so not shown: "
            + unpublished.join(", ")),
        h("div", { class: "runbox" }, view.runButton, view.gate, view.staged, view.state,
          h("p", { class: "dead" }, "Stops if this page goes quiet for half a second, or the "
              + "robot disables. The gains at the start are recorded with the run.")),
        view.commit),
      h("section", { class: "card" },
        h("div", { class: "plotbar" }, h("h2", null, "Runs"), view.pickLabel),
        view.plotElement,
        view.signals,
        h("div", { class: "scroll" }, view.scores)));
    panels.append(view.panel);
    rigViews.set(name, view);
  }
}

/**
 * Starts a run, keeping it going from the click; or stops the one running. With the rig's gains
 * staged, it applies them first, through the same checks, all or none, and runs only if they
 * applied: one press, so a run is never of the gains before the edit. Only its gains: other
 * values staged in Tune, which can't be seen from here, stay staged.
 */
async function runOrStop(name, view) {
  const run = robot.doc.run;
  if (run && run.rig === name && run.phase === "RUNNING") {
    const result = await send("POST", "/api/experiments", { cancel: run.experiment, run: run.id });
    if (!result.ok) say(result.problem.detail, true);
    return;
  }
  const gains = view.rig.gains;
  if (pending(gains).count > 0 && !(await apply({ only: gains }))) return; // the bar says why
  starting = true;
  try {
    const result = await send("POST", "/api/experiments", { start: view.choice.value });
    if (!result.ok) say(result.problem.detail, true);
  } finally {
    starting = false;
  }
}

function capital(text) {
  return text.charAt(0).toUpperCase() + text.slice(1);
}

/** A value's name in code, steerKp; its configuration and path, for one not of this rig. */
function nameOf(path) {
  const value = robot.doc.tunables[path];
  return value ? value.field.slice(value.field.lastIndexOf(".") + 1) : path;
}

function fullName(path) {
  const value = robot.doc.tunables[path];
  return value ? value.config + " " + value.field : path;
}

/**
 * How far the run on a rig has got, by its samples' own times, and how long it has left: " · 0.8 s
 * of 2.1 s, 1.3 s left". Empty before its first sample.
 */
function countdown(name) {
  const run = robot.doc.run;
  if (!run || run.rig !== name) return "";
  const kept = runs.find((r) => r.key === runKey(robot.doc.robot.program, run.id));
  const seconds = (robot.doc.experiments[run.experiment] || {}).seconds;
  let elapsed = 0;
  for (const series of Object.values((kept && kept.trace) || {})) {
    if (series.t.length > 0) elapsed = Math.max(elapsed, series.t[series.t.length - 1]);
  }
  if (typeof seconds !== "number") return elapsed > 0 ? " · " + elapsed.toFixed(1) + " s" : "";
  const left = Math.max(0, seconds - elapsed);
  return " · " + elapsed.toFixed(1) + " s of " + seconds.toFixed(1) + " s, " + left.toFixed(1)
      + " s left";
}

/** A rig's gain: as in Tune, except that it stays put while its run runs. */
function updateGain(view, row) {
  updateRow(row);
  if (view.busy) row.input.disabled = true;
  if (row.docs) replace(row.docs, valueDocs(row.path, true));
}

/** What a gain means, under its row: its name opens it, and again closes it. */
function openDocs(row, how) {
  if (how === "focus") return;
  if (row.docs) {
    row.docs.remove();
    row.docs = null;
    return;
  }
  row.docs = h("div", { class: "docs docs-inline", "aria-label": "What it means" },
    valueDocs(row.path, true));
  row.el.append(row.docs);
}

function updateRig(name, view) {
  const doc = robot.doc;
  // While its run runs, its gains stay put here: a run is scored for the gains it started with.
  view.busy = Boolean(doc.run && doc.run.rig === name && doc.run.phase === "RUNNING");
  view.rows.forEach((row) => updateGain(view, row));
  const current = doc.run && doc.run.rig === name ? doc.run : null;
  const running = view.busy;
  // Run applies the rig's staged gains first; anything else staged stays, and says so here.
  const gains = view.rig.gains;
  const own = pending(gains);
  const others = edited().filter((path) => !gains.includes(path));
  const lock = own.count > 0 ? whyLocked() : null;
  const bad = [...own.invalid, ...own.outside];
  view.runButton.className = "big " + (running ? "danger pinned" : "primary");
  view.runButton.textContent = running ? "■ Stop  (Esc)" + countdown(name)
    : own.count > 0 ? "Apply " + own.count + (own.big.length > 0 ? ", including "
      + own.big.length + (own.big.length === 1 ? " big step," : " big steps,") : "") + " and run"
    : "▶ Run";
  // Run is off beside the gate's own reason, and the mode now. Stop is never locked.
  const why = !robot.connected ? "Not connected to the robot." : doc.robot.experimentsLocked
    ? capital(doc.robot.experimentsLocked) + (doc.robot.mode ? ". Now: " + doc.robot.mode : "") + "."
    : null;
  view.gate.hidden = running || !why;
  view.gate.textContent = why ? "🔒 " + why : "";
  view.runButton.disabled = !running && (Boolean(why) || bad.length > 0 || Boolean(lock));
  replace(view.staged, running ? [] : [
    bad.length > 0 && h("p", { class: "why" }, "✗ " + bad.map(nameOf).join(", ")
        + (bad.length === 1 ? " is" : " are") + " outside " + (bad.length === 1 ? "its" : "their")
        + " limits or not a number: fix " + (bad.length === 1 ? "it" : "them") + " to run."),
    !why && lock && h("p", { class: "gatemsg" }, "🔒 " + lock),
    own.big.length > 0 && h("p", { class: "big" }, "▲ " + own.big.map((step) =>
      nameOf(step.path) + " " + shown(step.from) + " → " + shown(step.to) + ": "
        + Math.round(step.share * 100) + "% of its range").join("; ")),
    others.length > 0 && h("p", { class: "meta" }, others.length
        + (others.length === 1 ? " other edit is" : " other edits are") + " staged in Tune ("
        + others.map(fullName).join(", ") + "): Run leaves "
        + (others.length === 1 ? "it" : "them") + " staged."),
  ]);
  view.runButton.setAttribute("aria-describedby", view.gate.id + " " + view.staged.id);
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
  // A gain a column, so each lines up down the runs; a run says its experiment only when the rig
  // has more than one to tell apart.
  const gains = view.rig.gains;
  const several = Object.values(doc.experiments).filter((e) => e.rig === name).length > 1;
  const head = table.createTHead().insertRow();
  head.append(h("th", { scope: "col" }, "Run"),
    ...gains.map((path) => h("th", { scope: "col", class: "gain", title: fullName(path) },
      nameOf(path))));
  for (const column of columns) {
    head.append(h("th", { scope: "col", class: column === columns[0] ? "first-score" : null,
      title: "The worst of every jump of every "
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
    label.append(`#${run.id}` + (several ? " " + run.experiment : ""));
    if (run.gainsChanged) {
      label.append(" ", h("span", { class: "tag refused", title: run.detail }, "gains changed"));
    }
    if (run.program !== doc.robot.program) {
      label.append(" ", h("span", { class: "tag plain",
        title: "Ran on " + run.robot + (run.simulated ? " (simulator)" : "") + ", code " + run.code },
        run.code !== doc.robot.code ? "other code" : "before a restart"));
    }
    for (const path of gains) {
      const cell = row.insertCell();
      cell.className = "gain";
      cell.textContent = path in run.gains ? shown(run.gains[path]) : "—";
    }
    const s = summaries[index];
    const show = (values, cells) => columns.forEach((column, c) => {
      const value = column.get(values);
      const cell = cells.insertCell();
      if (c === 0) cell.classList.add("first-score");
      const isBest = cells === row && !run.gainsChanged && Number.isFinite(value) && value === best[c];
      if (isBest) {
        cell.classList.add("best");
        cell.title = "The best of these runs";
      }
      cell.textContent = Number.isFinite(value) ? column.show(value)
          : value === Infinity ? "never" : "—";
    });
    if (!s) {
      const cell = row.insertCell();
      cell.colSpan = columns.length;
      cell.className = (run.phase === "RUNNING" ? "muted" : "stopped") + " first-score";
      cell.textContent = run.phase === "RUNNING" ? "Running…" : run.detail;
    } else {
      show(s, row);
    }
    // Stages the run's gains, as any other edit: they apply with the rest, or with the next run.
    row.insertCell().append(h("button", { type: "button", class: "small revert",
      title: "Stage this run's gains", "aria-label": "Stage run " + run.id + "'s gains",
      disabled: !doc.tuning.enabled || view.busy, onclick: () => stageAll(run.gains) }, "↺"));
    if (s && view.expanded.has(run.key)) {
      for (const measurement of view.rig.measurements) {
        const wheel = body.insertRow();
        wheel.className = "sub";
        wheel.insertCell().textContent = measurement;
        for (let g = 0; g < gains.length; g++) wheel.insertCell();
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

/**
 * Lines on one time axis, as uPlot takes them: each line's value where it has one, else null.
 * {@code until} stretches the axis to that time, so plots of one run share it.
 */
function aligned(lines, until) {
  const times = lines.flatMap((line) => line.t);
  if (until !== undefined) times.push(0, until);
  const xs = [...new Set(times)].sort((a, b) => a - b);
  const at = new Map(xs.map((x, i) => [x, i]));
  return [xs, ...lines.map((line) => {
    const ys = new Array(xs.length).fill(null);
    line.t.forEach((t, i) => (ys[at.get(t)] = line.v[i]));
    return ys;
  })];
}

/**
 * A measurement's own signals: those named after it ("Front left voltage" for "Front left"), or,
 * for a rig whose signals name no measurement (a joint's Voltage and Current), every one.
 */
function signalsFor(rig, measurement) {
  const names = rig.signals.map((signal) => signal.name);
  const own = names.filter((signal) => signal.startsWith(measurement + " "));
  if (own.length > 0) return own;
  return names.filter((signal) => !rig.measurements.some((m) => signal.startsWith(m + " ")));
}

function destroyPlots(view) {
  if (view.plot) view.plot.destroy();
  view.plot = null;
  for (const panel of view.signalPlots.values()) panel.plot.destroy();
  view.signalPlots.clear();
  view.signalsKey = "";
}

/** A plot, styled as the page is: every one of a rig's shares its cursor, and dragging zooms all. */
function makePlot(name, element, lines, unit, height) {
  const stepped = uPlot.paths && uPlot.paths.stepped ? uPlot.paths.stepped({ align: 1 }) : undefined;
  const axis = { stroke: color("--muted"), grid: { stroke: color("--line"), width: 1 },
                 ticks: { stroke: color("--line"), width: 1 } };
  return new uPlot({
    width: Math.max(280, element.clientWidth),
    height,
    scales: { x: { time: false } },
    axes: [{ ...axis, label: "s" }, { ...axis, label: unit }],
    cursor: { drag: { x: true, y: false }, sync: { key: "exp-" + name, scales: ["x", null] } },
    series: [{ label: "t" }, ...lines.map((line) => ({
      label: line.label,
      stroke: line.setpoint ? color("--text-2") : line.latest ? color("--accent") : color("--run-other"),
      width: line.latest ? 2.5 : line.setpoint ? 1.5 : 1,
      dash: line.setpoint ? [6, 4] : undefined,
      paths: line.setpoint ? stepped : undefined,
      spanGaps: true,
      points: { show: false },
    }))],
  }, aligned(lines), element);
}

function unitOf(run, signal) {
  return (run.signals || []).find((s) => s.name === signal)?.unit || "";
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
  const measurement = view.pick.value;
  const overlaid = runs.filter((run) => run.rig === name && run.trace[measurement])
    .slice(-OVERLAID);
  if (overlaid.length === 0) return;
  const latest = overlaid[overlaid.length - 1];
  const lines = overlaid.map((run) => ({ label: `#${run.id}`, t: run.trace[measurement].t,
    v: run.trace[measurement].v, latest: run === latest }));
  const setpoint = latest.trace.Setpoint;
  if (setpoint) lines.unshift({ label: "Setpoint", t: setpoint.t, v: setpoint.v, setpoint: true });
  // Every plot of the rig on one time axis, as long as the longest line on any of them.
  const signals = signalsFor(view.rig, measurement).filter((signal) => latest.trace[signal]);
  const until = Math.max(0, ...lines.flatMap((line) => line.t),
    ...signals.flatMap((signal) => latest.trace[signal].t));

  const key = measurement + "|" + lines.map((line) => line.label).join(",");
  if (view.plot && view.plotKey === key && !rebuild) {
    view.plot.setData(aligned(lines, until));
  } else {
    if (view.plot) view.plot.destroy();
    view.plotElement.replaceChildren();
    view.plot = makePlot(name, view.plotElement, lines, unitOf(latest, measurement), PLOT_HEIGHT);
    view.plot.setData(aligned(lines, until));
    view.plotKey = key;
  }

  // Each of the measurement's signals in the latest run: shorter, one a panel, while open.
  if (!view.signals.open) return;
  const signalsKey = signals.join(",") + "|" + latest.key;
  if (rebuild || signalsKey !== view.signalsKey) {
    for (const panel of view.signalPlots.values()) panel.plot.destroy();
    view.signalPlots.clear();
    view.signalsElement.replaceChildren();
    for (const signal of signals) {
      const element = h("div", { class: "signal-plot", "aria-label": signal });
      view.signalsElement.append(h("div", { class: "meta" }, signal), element);
      const line = { label: `#${latest.id}`, t: [], v: [], latest: true };
      view.signalPlots.set(signal,
        { element, plot: makePlot(name, element, [line], unitOf(latest, signal), SIGNAL_HEIGHT) });
    }
    view.signalsKey = signalsKey;
  }
  for (const [signal, panel] of view.signalPlots) {
    const trace = latest.trace[signal];
    panel.plot.setData(aligned([{ t: trace.t, v: trace.v }], until));
  }
}

window.addEventListener("resize", () => {
  for (const view of rigViews.values()) {
    if (view.plot) {
      view.plot.setSize({ width: Math.max(280, view.plotElement.clientWidth), height: PLOT_HEIGHT });
    }
    for (const panel of view.signalPlots.values()) {
      panel.plot.setSize({ width: Math.max(280, panel.element.clientWidth), height: SIGNAL_HEIGHT });
    }
  }
});

export default { id: "experiment", title: "Experiment", mount, show, hide, route };
