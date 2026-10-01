// The apply bar, at the bottom of Tune and Experiment while anything is staged: how many edits,
// and Apply and Discard. Applying sends them all at once, and the robot applies all or none.

import { robot, onChange, onConnection, h, byId } from "./core.js";
import { onEdits, pending, apply, discard, outcome } from "./edits.js";
import { replace } from "./rows.js";

/** The views it shows in: those with values to stage. */
const VIEWS = new Set(["tune", "experiment"]);
let bar;
let view = null;
let applyButton;
let discardButton;
let what;

export function mountApplyBar() {
  bar = byId("applybar");
  what = h("span", { class: "what" });
  discardButton = h("button", { type: "button", onclick: discard }, "Discard");
  applyButton = h("button", { type: "button", class: "primary", onclick: apply });
  bar.append(h("div", { class: "ab-line" }, what, discardButton, applyButton));
  onEdits(render);
  onChange((patch, snapshot) => {
    if (snapshot || "tuning" in patch) render();
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
  bar.hidden = !VIEWS.has(view) || count === 0;
  if (bar.hidden) return;
  const ready = Object.keys(changes).length;
  const locked = !robot.doc.tuning.enabled || !robot.connected;
  replace(what, h("b", null, count + (count === 1 ? " edit" : " edits")),
    " staged, applied together or not at all.",
    invalid.length > 0 && h("span", { class: "bad" },
      " ✗ " + invalid.length + " isn't a number: fix it to apply."));
  applyButton.disabled = locked || invalid.length > 0 || outcome.applying;
  applyButton.textContent = outcome.applying ? "Applying…" : "Apply " + ready;
}
