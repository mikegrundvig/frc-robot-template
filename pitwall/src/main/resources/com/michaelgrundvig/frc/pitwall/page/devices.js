// Pit's devices: a card for each device the robot reports on (a vision coprocessor, say), saying
// whether it's there and how it is, and ending in what can be done about it: its own page, a file
// to download from it, or its log to read. The robot program asks the device; this page asks only
// the robot (/api/devices/...), never a device itself.

import { robot, onChange, api, h, replace, say } from "./core.js";

const MARKS = { ok: ["✓", "ok"], note: ["•", "none"], warning: ["▲", "warn"], problem: ["✗", "bad"] };
const PRIORITIES = ["emerg", "alert", "crit", "err", "warning", "notice", "info", "debug"];

let section;
let grid;
let built = "";
const cards = new Map(); // device name -> its card's parts

/** Mounts the devices' section in a view: hidden while the robot reports no devices. */
export function mountDevices(parent) {
  grid = h("div", { class: "devices" });
  section = h("section", { class: "devices-section", hidden: true, "aria-label": "Devices" },
    h("h2", { class: "devices-title" }, "Devices"),
    h("p", { class: "meta" }, "Each device the robot program asks after, as of its last answer. "
        + "The robot fetches downloads and logs from the device; this page never reaches it."),
    grid);
  parent.append(section);
  onChange((patch, snapshot) => {
    if (snapshot || "devices" in patch) render(snapshot);
  });
}

function render(snapshot) {
  const devices = (robot.doc && robot.doc.devices) || {};
  const names = Object.keys(devices);
  section.hidden = names.length === 0;
  // Rebuilt only when the devices or what can be done from them change; updated in place after.
  const key = names.map((name) => name + ":" + JSON.stringify([devices[name].kind,
    devices[name].links, devices[name].downloads, devices[name].log])).join("\n");
  if (snapshot || key !== built) {
    built = key;
    cards.clear();
    grid.replaceChildren(...names.map((name) => makeCard(name, devices[name])));
  }
  for (const [name, device] of Object.entries(devices)) update(cards.get(name), device);
}

function makeCard(name, device) {
  const parts = {
    state: h("span", { class: "dstate" }),
    lines: h("ul", { class: "slist" }),
  };
  const actions = [];
  for (const link of device.links || []) {
    if (!/^https?:\/\//i.test(link.url)) continue; // a page's link, never a script
    actions.push(h("a", { class: "button", href: link.url, target: "_blank",
                          rel: "noopener noreferrer" }, link.label + " ↗"));
  }
  for (const download of device.downloads || []) {
    const button = h("button", { type: "button" }, download.label);
    button.addEventListener("click", () => fetchFile(button,
      "/api/devices/" + name + "/downloads/" + download.id, name + " " + download.label));
    actions.push(button);
  }
  if (device.log) {
    actions.push(h("button", { type: "button", onclick: () => openLog(name) }, "Download logs"));
  }
  parts.el = h("section", { class: "card device", "aria-label": name },
    h("h3", { class: "dhead" }, h("span", { class: "mono" }, name),
      h("span", { class: "tag plain" }, device.kind), parts.state),
    parts.lines,
    h("div", { class: "acts" }, actions));
  cards.set(name, parts);
  return parts.el;
}

function update(parts, device) {
  if (!parts) return;
  const [mark, tone] = MARKS[device.level] || MARKS.note;
  replace(parts.state, h("span", { class: "sev " + tone, "aria-hidden": "true" }, mark),
    " " + device.state);
  parts.el.classList.toggle("problem", device.level === "problem");
  replace(parts.lines, (device.lines || []).map((line) => {
    const [lineMark, lineTone] = MARKS[line.level] || MARKS.note;
    return h("li", null, h("span", { class: "sev " + lineTone, "aria-hidden": "true" }, lineMark),
      h("span", { class: "k" }, line.label), h("span", null, line.value));
  }));
}

/**
 * Fetches a file through the robot and saves it. A device that doesn't answer is said so here,
 * rather than saved as a file holding the robot's reason.
 */
async function fetchFile(button, path, what) {
  button.disabled = true;
  const was = button.textContent;
  button.textContent = "Fetching…";
  try {
    const response = await api("GET", path);
    if (!response.ok) {
      const problem = await response.json().catch(() => null);
      say(what + ": " + (problem && problem.detail ? problem.detail : response.statusText), true);
      return;
    }
    const blob = await response.blob();
    const disposition = response.headers.get("Content-Disposition") || "";
    const named = /filename="([^"]+)"/.exec(disposition);
    save(blob, named ? named[1] : "download");
    say(what + ": saved.");
  } catch (error) {
    say(what + ": the robot isn't answering.", true);
  } finally {
    button.disabled = false;
    button.textContent = was;
  }
}

function save(blob, name) {
  const url = URL.createObjectURL(blob);
  const link = h("a", { href: url, download: name, hidden: true });
  document.body.append(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 10000);
}

// ---- The log viewer ----

let viewer = null;

/** The log viewer: a dialog with the filters, a page of entries, More, and Download. */
function openLog(name) {
  if (!viewer) viewer = makeViewer();
  viewer.name = name;
  viewer.title.textContent = name + "'s log";
  viewer.dialog.showModal();
  load(false);
}

function makeViewer() {
  const v = { loads: 0, next: "", name: "" };
  v.priority = h("select", { "aria-label": "Least important shown" },
    PRIORITIES.map((word, number) => h("option", { value: String(number) },
      number + " " + word + (number < 7 ? " and worse" : ": everything"))));
  v.priority.value = "6";
  v.unit = h("input", { type: "text", placeholder: "every unit", spellcheck: "false",
                        autocomplete: "off", "aria-label": "Unit, such as photonvision.service" });
  v.list = h("ol", { class: "logview", "aria-live": "polite" });
  v.status = h("p", { class: "meta" });
  // The agent pages forward only: from the latest entries, the next page is the newer ones.
  v.more = h("button", { type: "button", onclick: () => load(true) }, "Newer");
  v.download = h("button", { type: "button" }, "Download as text");
  v.download.addEventListener("click", () => fetchFile(v.download,
    "/api/devices/" + v.name + "/log.txt?" + filters(""), v.name + "'s log"));
  v.title = h("h2");
  v.dialog = h("dialog", { class: "logdialog" },
    h("div", { class: "dlg" }, v.title,
      h("div", { class: "toolbar" },
        h("label", null, "Priority ", v.priority),
        h("label", null, "Unit ", v.unit),
        h("button", { type: "button", onclick: () => load(false) }, "Show")),
      v.list, v.status,
      h("div", { class: "dlg-actions" }, v.more, v.download,
        h("button", { type: "button", class: "primary", onclick: () => v.dialog.close() },
          "Close"))));
  v.unit.addEventListener("keydown", (event) => {
    if (event.key === "Enter") load(false);
  });
  v.priority.addEventListener("change", () => load(false));
  document.body.append(v.dialog);
  return v;
}

function filters(cursor) {
  const query = new URLSearchParams({ priority: viewer.priority.value });
  const unit = viewer.unit.value.trim();
  if (unit) query.set("unit", unit);
  if (cursor) query.set("cursor", cursor);
  return query.toString();
}

/**
 * Loads the latest page for the filters, or the page of newer entries after what's shown. Only the
 * latest load counts: one asked before it (another filter, another device) is dropped when it
 * answers.
 */
async function load(more) {
  const v = viewer;
  const asked = ++v.loads;
  const cursor = more ? v.next : "";
  if (!more) {
    v.next = "";
    v.list.replaceChildren();
  }
  v.more.disabled = true;
  v.status.textContent = "Asking " + v.name + "…";
  const response = await api("GET", "/api/devices/" + v.name + "/log?" + filters(cursor))
    .catch(() => null);
  if (asked !== v.loads) return;
  if (!response) {
    v.status.textContent = "✗ The robot isn't answering.";
    return;
  }
  const body = await response.json().catch(() => null);
  if (asked !== v.loads) return;
  if (!response.ok || !body) {
    v.status.textContent = "✗ " + (body && body.detail ? body.detail : response.statusText);
    return;
  }
  for (const entry of body.entries) {
    const tone = entry.priority <= 3 ? "bad" : entry.priority === 4 ? "warn" : "";
    v.list.append(h("li", { class: tone },
      h("span", { class: "t" }, entry.time), " ",
      h("span", { class: "p" }, PRIORITIES[entry.priority] || String(entry.priority)), " ",
      h("span", { class: "u" }, entry.unit + ":"), " ",
      h("span", null, entry.message)));
  }
  v.next = body.next || "";
  v.more.disabled = !v.next;
  const shown = v.list.childElementCount;
  v.status.textContent = shown === 0 ? "Nothing matches." : shown + " entries, the latest"
    + (v.next ? "; Newer fetches those since." : ".");
}
