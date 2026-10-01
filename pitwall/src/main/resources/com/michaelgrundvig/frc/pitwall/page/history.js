// The session's history: every change to a configuration value this page saw, newest last, with
// where it came from (this page, a dashboard or another page, the field connecting, the robot
// restarting) and each value's before and after, so any of it can be staged again.
//
// Kept in this tab's sessionStorage, so a reload (a phone's pull to refresh) keeps it; closing the
// tab ends it, as nothing else of the session outlives the tab. A browser that won't keep it (a
// private window, storage off, or full) is told once, and the page carries on without it.

import { robot, onChange, near, say } from "./core.js";

const KEY = "pitwall.history";
const KNOWN = "pitwall.known";
/** The shape of what's kept: a reload after a page of another shape starts afresh. */
const VERSION = 2;
/** The most entries kept: a long session's oldest go first. */
const KEPT = 200;
/** How long a change this page made waits to be seen on the robot as its own, not another's. */
const EXPECTED_MS = 5000;

/** Whether this tab can keep anything, checked once. */
const STORAGE = (() => {
  try {
    sessionStorage.setItem("pitwall.probe", "1");
    sessionStorage.removeItem("pitwall.probe");
    return true;
  } catch (error) {
    return false;
  }
})();
let warned = false;

/**
 * Every entry, oldest first: {time, source, kind, items: [{path, config, field, from, to}], code}.
 * Its items are what Restore stages; code, for a restart, is the committed values the new code
 * changed, which there's nothing to restore of.
 */
export const entries = load(KEY, [], (kept) => Array.isArray(kept)
    && kept.every((entry) => entry && typeof entry.time === "number" && Array.isArray(entry.items)));
const listeners = [];
/** What this page applied and hasn't yet seen on the robot: path -> {value, until, source, item}. */
const expected = new Map();
/**
 * Each value as this page last saw it, its committed value, and the program it saw them in: to
 * tell what changed, and on a restart, what the restart undid from what the code changed.
 */
let known = load(KNOWN, { program: null, values: {}, tunables: {} }, (kept) => Boolean(kept)
    && typeof kept.values === "object" && typeof kept.tunables === "object");

/** Calls {@code listener(entry)} whenever an entry is added. */
export function onHistory(listener) {
  listeners.push(listener);
}

/**
 * Notes a change this page is about to ask for, {path: value}, before it's sent: the robot may
 * report it before it answers, and it mustn't be taken for another's. Record it once it's applied,
 * or with {@link #recordWhenSeen} if the robot only queued it.
 *
 * @return its items, each value before and after
 */
export function expect(changes) {
  const items = [];
  for (const [path, to] of Object.entries(changes)) {
    const value = robot.doc.tunables[path];
    if (!value) continue;
    const item = { path, config: value.config, field: value.field, from: robot.doc.values[path], to };
    items.push(item);
    expected.set(path, { value: to, until: Date.now() + EXPECTED_MS, source: null, item });
  }
  return items;
}

/** Records a change this page made, applied: where it came from, and its items from expect. */
export function record(source, kind, items) {
  add({ source, kind, items });
}

/**
 * Records a change the robot queued but hasn't answered for (a 202), as each value is seen: what
 * isn't seen in time was never applied, and isn't recorded.
 */
export function recordWhenSeen(source, items) {
  for (const item of items) {
    const asked = expected.get(item.path);
    if (asked) asked.source = source;
  }
}

function add(entry) {
  if (entry.items.length === 0 && !(entry.code && entry.code.length > 0)) return;
  entry.time = Date.now();
  entries.push(entry);
  while (entries.length > KEPT) entries.shift();
  save(KEY, entries);
  for (const listener of listeners) listener(entry);
}

// Changes this page didn't make: from a dashboard, another page, the field connecting (which
// returns every value to committed), or a restart of the robot program (a redeploy), which this
// page sees as a snapshot from a new program.
onChange((patch, snapshot) => {
  const doc = robot.doc;
  if (snapshot && known.program !== null && known.program !== doc.robot.program) {
    restarted();
  } else if (snapshot) {
    noteChanges(doc.values, "while this page wasn't connected", "elsewhere");
  } else if ("values" in patch) {
    const field = Boolean(doc.tuning && doc.tuning.fieldConnected);
    noteChanges(patch.values, field ? "the field connected" : "a dashboard or another page",
        field ? "field" : "elsewhere");
  }
  if (snapshot || "values" in patch || "tunables" in patch) {
    known = { program: doc.robot.program, values: { ...doc.values }, tunables: describe(doc) };
    save(KNOWN, known);
  }
});

/**
 * The robot program restarted (a redeploy, say): every tuned value is back to committed. What
 * that undid (each value that was tuned) can be restored; what the new code changed (a committed
 * value that differs) is said apart, as there's nothing to restore of it.
 */
function restarted() {
  const doc = robot.doc;
  const items = [];
  const code = [];
  for (const [path, before] of Object.entries(known.values)) {
    const was = known.tunables[path];
    if (!was || typeof before !== "number") continue;
    const after = doc.values[path];
    const value = doc.tunables[path];
    // Tuned before, and not to what it is now: a value committed in the new code needs nothing.
    if (typeof was.committed === "number" && !near(before, was.committed)
        && !(typeof after === "number" && near(before, after))) {
      items.push({ path, config: was.config, field: was.field, from: before, to: after });
    }
    if (value && typeof was.committed === "number" && !near(was.committed, value.committed)) {
      code.push({ path, config: was.config, field: was.field, from: was.committed,
                  to: value.committed });
    }
  }
  expected.clear();
  add({ source: "the robot restarted", kind: "restart", items, code });
}

/** Records the values that changed from what this page last saw, but not those it expected. */
function noteChanges(values, source, kind) {
  const now = Date.now();
  const items = [];
  const mine = new Map(); // a queued change of this page's, seen at last: its source -> items
  for (const [path, to] of Object.entries(values)) {
    const from = known.values[path];
    const value = robot.doc.tunables[path] || known.tunables[path];
    if (typeof to !== "number" || typeof from !== "number" || near(to, from) || !value) continue;
    const asked = expected.get(path);
    if (asked && asked.until >= now && near(asked.value, to)) {
      expected.delete(path);
      if (asked.source) {
        if (!mine.has(asked.source)) mine.set(asked.source, []);
        mine.get(asked.source).push(asked.item);
      }
      continue;
    }
    items.push({ path, config: value.config, field: value.field, from, to });
  }
  for (const [path, asked] of expected) if (asked.until < now) expected.delete(path);
  for (const [from, applied] of mine) add({ source: from, kind: "apply", items: applied });
  add({ source, kind, items });
}

/** Enough of each value to name it after it's gone, and its committed value. */
function describe(doc) {
  const described = {};
  for (const [path, value] of Object.entries(doc.tunables)) {
    described[path] = { config: value.config, field: value.field, committed: value.committed };
  }
  return described;
}

/** Whether an entry took values back to committed: restoring it stages what it undid. */
export function undid(entry) {
  return entry.kind === "reset" || entry.kind === "field" || entry.kind === "restart";
}

/** Items as people read them: "drivetrain gains.steerKp 16 → 18.5". */
export function itemsText(items) {
  return items.map((item) => item.config + " " + item.field + " " + shortly(item.from)
      + " → " + shortly(item.to)).join(", ");
}

function shortly(number) {
  return typeof number === "number" ? String(Number(number.toPrecision(6))) : "gone";
}

/** The time of day of an entry, as the list shows it. */
export function clock(time) {
  return new Date(time).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
}

/**
 * Reads what this tab kept, if it's of this page's shape: what an older page kept, or what isn't
 * readable, is dropped.
 *
 * @param check whether the data kept has the shape expected
 */
export function load(key, otherwise, check) {
  if (!STORAGE) return otherwise;
  try {
    const kept = JSON.parse(sessionStorage.getItem(key) || "null");
    if (kept && kept.version === VERSION && check(kept.data)) return kept.data;
  } catch (error) {
    // Unreadable: start afresh.
  }
  return otherwise;
}

/**
 * Keeps something for this tab, as {@link #load} reads it.
 *
 * @return whether it was kept: false when storage is off, or full, and then the page says once
 *     that this tab can't keep the session
 */
function save(key, data) {
  if (trySave(key, data)) return true;
  cannotKeep();
  return false;
}

/** Keeps something for this tab, saying nothing when it can't: for a caller that can try less. */
export function trySave(key, data) {
  if (!STORAGE) return false;
  try {
    sessionStorage.setItem(key, JSON.stringify({ version: VERSION, data }));
    return true;
  } catch (error) {
    return false;
  }
}

/** Says, once, that this tab can't keep the session: a reload would lose it. */
export function cannotKeep() {
  if (warned) return;
  warned = true;
  say("This tab can't keep the session (its storage is off or full): a reload would lose its "
      + "history and runs.", true);
}
