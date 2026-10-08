// The apply bar, at the bottom of Tune and Experiment while anything is staged: how many edits,
// and Apply and Discard (Enter and Esc). Applying sends them all at once, and the robot applies
// all or none. Edits outside their limits are caught here first. When it refuses, every reason is
// listed here, grouped by the check that gave it, each a way to its value; the same reasons show
// beside the values.
//
// A big step changes the bar, and nothing pops up: while any staged edit moves its value by more
// than a quarter of its tuning range, the bar turns to a warning, names each, and its button says
// so. One press still applies everything, as for any change: the warning is there from the moment
// the edit is staged, not sprung at the press, and a second press would only teach people to press
// twice.

import { robot, onChange, onConnection, h, byId, go, replace, shown } from "./core.js";
import { onEdits, pending, apply, discard, closeRefusal, outcome, whyLocked } from "./edits.js";
import { setTuning } from "./bar.js";

/** The views it shows in: those with values to stage. */
const VIEWS = new Set(["tune", "experiment"]);

/** The checks, in the order the robot runs them, as the refusal list names them. */
const LAYERS = {
  value: "The values",
  limit: "Tuning limits",
  record: "The records' own checks",
  tunable: "A tunable's own setter",
};

let bar;
let view = null;
let refusalBox;
let line;
let applyButton;
let discardButton;
let switchOn;
let what;
let bigSteps;

export function mountApplyBar() {
  bar = byId("applybar");
  refusalBox = h("div", { class: "refusal", hidden: true });
  what = h("span", { class: "what" });
  discardButton = h("button", { type: "button", onclick: () => discard() }, "Discard");
  applyButton = h("button", { type: "button", class: "primary", onclick: () => apply() });
  switchOn = h("button", { type: "button", class: "primary", hidden: true,
                           onclick: () => setTuning(true) }, "Switch tuning on");
  line = h("div", { class: "ab-line" }, what, discardButton, switchOn, applyButton);
  bigSteps = h("div", { class: "bigsteps", role: "status" });
  bar.append(refusalBox, bigSteps, line);
  onEdits(render);
  onChange((patch, snapshot) => {
    if (snapshot || "tuning" in patch || "tunables" in patch) render();
  });
  onConnection(render);
}

/** Says which view is shown: the bar shows only in those with values to stage. */
export function showIn(id) {
  view = id;
  render();
}

function render() {
  if (!bar || !robot.doc) return;
  const { changes, invalid, outside, big, count } = pending();
  const refusal = outcome.refusal;
  bar.hidden = !VIEWS.has(view) || (count === 0 && !refusal);
  if (bar.hidden) return;
  refusalBox.hidden = !refusal;
  if (refusal) replace(refusalBox, refused(refusal));
  line.hidden = count === 0;
  const ready = Object.keys(changes).length;
  const session = robot.doc.tuning;
  // Why nothing can be applied now, said where the edits wait, with the way out when there's one.
  const lock = whyLocked();
  const blocked = invalid.length + outside.length;
  replace(what, h("b", null, count + (count === 1 ? " edit" : " edits")),
    " staged, applied together or not at all.",
    lock && h("span", { class: "lockword" }, " 🔒 " + lock),
    !lock && blocked > 0 && h("span", { class: "bad" }, " ✗ " + blocked
        + (blocked === 1 ? " is" : " are") + " outside " + (blocked === 1 ? "its" : "their")
        + " limits or not a number: fix " + (blocked === 1 ? "it" : "them") + " to apply."),
    !lock && blocked === 0 && h("span", { class: "keys" }, " Enter applies all · Esc drops the "
        + "focused edit"));
  // The big steps, named, the bar in a warning style while there are any.
  bar.classList.toggle("big", big.length > 0 && !lock);
  bigSteps.hidden = big.length === 0 || Boolean(lock);
  replace(bigSteps, big.length === 0 ? [] : [
    h("b", null, "▲ " + (big.length === 1 ? "A big step" : big.length + " big steps") + ": "),
    big.map((step, index) => {
      const value = robot.doc.tunables[step.path];
      return [index > 0 ? "; " : "", h("span", { class: "mono" }, value.config + " " + value.field),
        " " + shown(step.from) + " → " + shown(step.to) + ": " + Math.round(step.share * 100)
          + "% of its range"];
    }),
    ". More than a quarter of a value's tuning range: check it before applying."]);
  switchOn.hidden = !(robot.connected && !session.enabled && !session.fieldConnected);
  applyButton.hidden = !switchOn.hidden;
  applyButton.disabled = Boolean(lock) || blocked > 0 || outcome.applying;
  applyButton.textContent = outcome.applying ? "Applying…" : "Apply " + ready
      + (big.length > 0 ? ", including " + big.length + (big.length === 1 ? " big step"
        : " big steps") : "");
}

/**
 * What the robot said when it refused: each reason by the check that gave it, in the order the
 * checks run, and which checks ran, since the robot stops at the first that fails.
 */
function refused(refusal) {
  const close = h("button", { type: "button", class: "small", onclick: closeRefusal }, "Close");
  const errors = refusal.errors || [];
  if (refusal.status !== 422 || errors.length === 0) {
    return [h("h2", null, (refusal.status === 409 ? "🔒 " : "✗ ") + "Nothing changed: "
        + (refusal.detail || refusal.title)), close];
  }
  const order = Object.keys(LAYERS);
  const sorted = errors.slice().sort((a, b) => order.indexOf(a.layer) - order.indexOf(b.layer));
  return [
    h("h2", null, "✗ Refused: nothing changed. " + errors.length
        + (errors.length === 1 ? " reason:" : " reasons:")),
    h("ol", null, sorted.map((error) => h("li", null,
      h("span", { class: "layer" }, (LAYERS[error.layer] || error.layer)
          + (error.by ? " (" + error.by + ")" : "") + ": "),
      where(error), " ", error.text))),
    h("p", { class: "checked" }, checked(errors) + " The robot stops at the first check that "
        + "fails, and the edits stay staged: fix them and apply again."),
    h("details", null, h("summary", null, "As the robot sent it"), h("pre", null, refusal.detail)),
    close,
  ];
}

/** The values a reason concerns, each a way to it in Tune. */
function where(error) {
  return (error.paths || []).flatMap((path, index) => {
    const value = robot.doc.tunables[path];
    if (!value) return [index > 0 ? ", " : "", h("span", { class: "mono" }, path)];
    const hash = "#tune/" + value.config + "/" + value.field;
    return [index > 0 ? ", " : "", h("a", { class: "mono", href: hash, onclick: (event) => {
      event.preventDefault();
      go(hash);
    } }, value.config + " " + value.field)];
  });
}

/** Which checks ran, and which found something wrong. */
function checked(errors) {
  const layers = new Set(errors.map((error) => error.layer));
  if (layers.has("value") || layers.has("limit")) {
    return "Checked: tuning limits ✗ · records not run.";
  }
  if (layers.has("record")) return "Checked: tuning limits ✓ · records ✗.";
  return "Checked: tuning limits ✓ · records ✓; applying it failed.";
}
