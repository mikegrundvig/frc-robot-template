// Changes: what differs from the committed configurations, as the builder calls that commit it.

import { robot, onChange, send, h } from "./core.js";
import { builderLines, sayResult, sayIn } from "./edits.js";

let none;
let report;
let copyButton;
let resetButton;
let say;

function mount(view) {
  none = h("p", { class: "muted" }, "Everything matches the committed configuration.");
  report = h("pre", { hidden: true });
  copyButton = h("button", { disabled: true, onclick: copy }, "Copy");
  resetButton = h("button", { disabled: true, onclick: async () =>
    sayResult(await send("DELETE", "/api/changes"), say) }, "Reset all");
  const message = h("div", { class: "message", hidden: true });
  say = sayIn(message);
  view.append(
    h("h1", null, "Changes ", copyButton, " ", resetButton),
    none,
    report,
    message,
    h("p", { class: "muted" }, "Nothing is saved on the robot, and a restart returns every value "
        + "to its committed one. To keep a change, write its builder call into the configuration "
        + "and commit it."));
  onChange(render);
}

function render(patch, snapshot) {
  if (!snapshot && !("changes" in patch) && !("tuning" in patch) && !("tunables" in patch)) return;
  const lines = builderLines(Object.keys(robot.doc.changes));
  report.textContent = lines.join("\n");
  report.hidden = lines.length === 0;
  none.hidden = lines.length > 0;
  copyButton.disabled = lines.length === 0;
  resetButton.disabled = lines.length === 0 || !robot.doc.tuning.enabled;
}

/** Copies the robot's report: every change with the builder call that commits it. */
async function copy() {
  const text = await fetch("/api/changes", { headers: { Accept: "text/plain" } })
      .then((response) => response.text());
  try {
    await navigator.clipboard.writeText(text); // needs a secure page, such as localhost
  } catch (error) {
    const area = h("textarea"); // the fallback for http://robot.local
    area.value = text;
    document.body.append(area);
    area.select();
    document.execCommand("copy");
    area.remove();
  }
  copyButton.textContent = "Copied";
  setTimeout(() => (copyButton.textContent = "Copy"), 1500);
}

export default { id: "changes", title: "Changes", mount };
