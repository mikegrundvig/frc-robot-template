// The apply bar, at the bottom of Tune and Experiment while anything is staged: how many edits,
// and Apply and Discard. Applying sends them all at once, and the robot applies all or none. When
// it refuses, every reason is listed here, grouped by the check that gave it, each a way to its
// value; the same reasons show beside the values.

import { robot, onChange, onConnection, h, byId, go, replace } from "./core.js";
import { onEdits, pending, apply, discard, closeRefusal, outcome } from "./edits.js";

/** The views it shows in: those with values to stage. */
const VIEWS = new Set(["tune", "experiment"]);

/** The checks, in the order the robot runs them, as the refusal list names them. */
const LAYERS = {
  value: "The values",
  limit: "Tuning limits",
  record: "The records' own checks",
  rule: "Robot rules",
  tunable: "A tunable's own setter",
};

let bar;
let view = null;
let refusalBox;
let line;
let applyButton;
let discardButton;
let what;

export function mountApplyBar() {
  bar = byId("applybar");
  refusalBox = h("div", { class: "refusal", hidden: true });
  what = h("span", { class: "what" });
  discardButton = h("button", { type: "button", onclick: () => discard() }, "Discard");
  applyButton = h("button", { type: "button", class: "primary", onclick: () => apply() });
  line = h("div", { class: "ab-line" }, what, discardButton, applyButton);
  bar.append(refusalBox, line);
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
  const { changes, invalid, count } = pending();
  const refusal = outcome.refusal;
  bar.hidden = !VIEWS.has(view) || (count === 0 && !refusal);
  if (bar.hidden) return;
  refusalBox.hidden = !refusal;
  if (refusal) replace(refusalBox, refused(refusal));
  line.hidden = count === 0;
  const ready = Object.keys(changes).length;
  const locked = !robot.doc.tuning.enabled || !robot.connected;
  replace(what, h("b", null, count + (count === 1 ? " edit" : " edits")),
    " staged, applied together or not at all.",
    invalid.length > 0 && h("span", { class: "bad" },
      " ✗ " + invalid.length + " isn't a number: fix it to apply."));
  applyButton.disabled = locked || invalid.length > 0 || outcome.applying;
  applyButton.textContent = outcome.applying ? "Applying…" : "Apply " + ready;
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

/** The values a reason concerns, each a way to it in Tune; its record, when it names none. */
function where(error) {
  const paths = error.paths || [];
  if (paths.length === 0) {
    return h("span", { class: "mono" }, error.config + (error.group ? " " + error.group : ""));
  }
  return paths.flatMap((path, index) => {
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
    return "Checked: tuning limits ✗ · records not run · robot rules not run.";
  }
  if (layers.has("record")) return "Checked: tuning limits ✓ · records ✗ · robot rules not run.";
  if (layers.has("rule")) return "Checked: tuning limits ✓ · records ✓ · robot rules ✗.";
  return "Checked: tuning limits ✓ · records ✓ · robot rules ✓; applying it failed.";
}
