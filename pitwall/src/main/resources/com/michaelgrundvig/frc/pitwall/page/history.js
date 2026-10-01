// The session's history: every change to a configuration value this page saw, newest last, with
// where it came from (this page, a dashboard or another page, the field connecting, the robot
// restarting) and each value's before and after, so any of it can be staged again.
//
// Kept in this tab's sessionStorage, so a reload (a phone's pull to refresh) keeps it; closing the
// tab ends it, as nothing else of the session outlives the tab.

import { robot, onChange, near } from "./core.js";

const KEY = "pitwall.history";
/** The most entries kept: a long session's oldest go first. */
const KEPT = 200;
/** How long a change this page made waits to be seen on the robot as its own, not another's. */
const EXPECTED_MS = 5000;

/** Every entry, oldest first: {time, source, kind, items: [{path, config, field, from, to}]}. */
export const entries = load(KEY, []);
const listeners = [];
/** What this page applied and hasn't yet seen on the robot: path -> {value, until}. */
const expected = new Map();
/** Each value as this page last saw it, and the program it saw it in: to tell what changed. */
let known = load(KEY + ".known", { program: null, values: {}, tunables: {} });

/** Calls {@code listener()} whenever an entry is added. */
export function onHistory(listener) {
  listeners.push(listener);
}

/**
 * Notes a change this page is about to ask for, {path: value}, before it's sent: the robot may
 * report it before it answers, and it mustn't be taken for another's. Record it once it's applied.
 *
 * @return its items, each value before and after, for {@link #record}
 */
export function expect(changes) {
  const items = [];
  for (const [path, to] of Object.entries(changes)) {
    const value = robot.doc.tunables[path];
    if (!value) continue;
    items.push({ path, config: value.config, field: value.field, from: robot.doc.values[path], to });
    expected.set(path, { value: to, until: Date.now() + EXPECTED_MS });
  }
  return items;
}

/** Records a change this page made, applied: where it came from, and its items from expect. */
export function record(source, kind, items) {
  add(source, kind, items);
}

function add(source, kind, items) {
  if (items.length === 0) return;
  entries.push({ time: Date.now(), source, kind, items });
  while (entries.length > KEPT) entries.shift();
  save(KEY, entries);
  for (const listener of listeners) listener();
}

// Changes this page didn't make: from a dashboard, another page, the field connecting (which
// returns every value to committed), or a restart of the robot program (a redeploy), which this
// page sees as a snapshot from a new program.
onChange((patch, snapshot) => {
  const doc = robot.doc;
  if (snapshot) {
    const restarted = known.program !== null && known.program !== doc.robot.program;
    noteChanges(doc.values, restarted ? "the robot restarted" : "while this page wasn't connected",
        restarted ? "restart" : "elsewhere");
  } else if ("values" in patch) {
    const field = Boolean(doc.tuning && doc.tuning.fieldConnected);
    noteChanges(patch.values, field ? "the field connected" : "a dashboard or another page",
        field ? "field" : "elsewhere");
  }
  if (snapshot || "values" in patch || "tunables" in patch) {
    known = { program: doc.robot.program, values: { ...doc.values }, tunables: describe(doc) };
    save(KEY + ".known", known);
  }
});

/** Records the values that changed from what this page last saw, but not those it expected. */
function noteChanges(values, source, kind) {
  const now = Date.now();
  const items = [];
  for (const [path, to] of Object.entries(values)) {
    const from = known.values[path];
    const value = robot.doc.tunables[path] || known.tunables[path];
    if (typeof to !== "number" || typeof from !== "number" || near(to, from) || !value) continue;
    const mine = expected.get(path);
    if (mine && mine.until >= now && near(mine.value, to)) {
      expected.delete(path);
      continue;
    }
    items.push({ path, config: value.config, field: value.field, from, to });
  }
  for (const [path, mine] of expected) if (mine.until < now) expected.delete(path);
  add(source, kind, items);
}

/** Enough of each value's description to name it in the history after it's gone. */
function describe(doc) {
  const described = {};
  for (const [path, value] of Object.entries(doc.tunables)) {
    described[path] = { config: value.config, field: value.field };
  }
  return described;
}

/** Whether an entry took values back to committed: restoring it stages what it undid. */
export function undid(entry) {
  return entry.kind === "reset" || entry.kind === "field" || entry.kind === "restart";
}

/** An entry's items as people read them: "drivetrain gains.steerKp 16 → 18.5". */
export function itemsText(entry) {
  return entry.items.map((item) => item.config + " " + item.field + " " + shortly(item.from)
      + " → " + shortly(item.to)).join(", ");
}

function shortly(number) {
  return typeof number === "number" ? String(Number(number.toPrecision(6))) : String(number);
}

/** The time of day of an entry, as the list shows it. */
export function clock(time) {
  return new Date(time).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
}

/** Reads what this tab kept; nothing, if storage is off or what's kept is unreadable. */
export function load(key, otherwise) {
  try {
    const kept = sessionStorage.getItem(key);
    return kept ? JSON.parse(kept) : otherwise;
  } catch (error) {
    return otherwise;
  }
}

/**
 * Keeps something for this tab.
 *
 * @return whether it was kept: false when storage is off, or full
 */
export function save(key, value) {
  try {
    sessionStorage.setItem(key, JSON.stringify(value));
    return true;
  } catch (error) {
    return false;
  }
}
