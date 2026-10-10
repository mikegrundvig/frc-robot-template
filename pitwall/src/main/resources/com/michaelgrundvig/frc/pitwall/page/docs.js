// What a value, a record, or a mechanism means, from the records' own Javadoc, which Pitwall's
// documentation processor reads as the robot program compiles. Shown beside the values on a laptop
// and under the row on a phone. The text is set as text, through a small formatter for its
// paragraphs, "- " list items, and `code`: never as HTML.
//
// Missing text is a prompt, not a gap: the page says where to write it.

import { robot, shown, h, go, withUnit } from "./core.js";
import { described, edit } from "./edits.js";

/** Text as the documentation lays it out: paragraphs, "- " list items, and `code` spans. */
function prose(text) {
  const blocks = [];
  for (const paragraph of String(text).split(/\n\s*\n/)) {
    const lines = paragraph.split("\n");
    let list = null;
    let words = [];
    const flush = () => {
      if (words.length > 0) blocks.push(h("p", null, inline(words.join(" "))));
      words = [];
    };
    for (const line of lines) {
      if (line.startsWith("- ")) {
        flush();
        if (!list) {
          list = h("ul");
          blocks.push(list);
        }
        list.append(h("li", null, inline(line.slice(2))));
      } else if (line.trim() !== "") {
        list = null;
        words.push(line.trim());
      }
    }
    flush();
  }
  return blocks;
}

/** A line's `code` spans as code, the rest as text. */
function inline(text) {
  return text.split(/(`[^`]+`)/).filter(Boolean).map((part) =>
    part.length > 1 && part.startsWith("`") && part.endsWith("`")
      ? h("code", null, part.slice(1, -1)) : part);
}

/** The first sentence of a text, as the list shows under each value. */
export function firstSentence(text) {
  const found = String(text).match(/^(.+?[.!?])(\s|$)/s);
  return (found ? found[1] : String(text)).split("\n")[0];
}

/** A meaning as a sentence: "volts per radian of error" becomes "Volts per radian of error." */
function sentence(text) {
  const trimmed = text.trim();
  const capital = trimmed.charAt(0).toUpperCase() + trimmed.slice(1);
  return /[.!?]$/.test(capital) ? capital : capital + ".";
}

/** Whether the robot sends documentation at all (a Pitwall built before it doesn't). */
function documented(value) {
  return typeof value.meaning === "string";
}

/** A configuration's documentation, and one of its records': {record, doc, meaning}. */
function recordDocs(config, group) {
  const docs = robot.doc.configs && robot.doc.configs[config];
  if (!docs) return null;
  if (!group) return { record: docs.record, doc: docs.doc, meaning: "" };
  return (docs.groups || []).find((candidate) => candidate.path === group) || null;
}

/** The record a value is in, by its path in its configuration. */
export function groupOf(value) {
  if (typeof value.group === "string") return value.group;
  const cut = value.field.lastIndexOf(".");
  return cut < 0 ? "" : value.field.slice(0, cut);
}

/** The name a value has in code: the last part of its path, steerKp for gains.steerKp. */
export function nameOf(value) {
  return value.field.slice(value.field.lastIndexOf(".") + 1);
}

/**
 * A value's documentation: what it means, where it's set, what it is now and committed, its
 * limits, which rigs tune it, and the builder call that commits it.
 *
 * @param compact just the meaning and the facts, as Experiment shows under a gain
 */
export function valueDocs(path, compact = false) {
  const doc = robot.doc;
  const value = described(path);
  if (!value) return [];
  const name = nameOf(value);
  const group = groupOf(value);
  const now = doc.values[path];
  const tuned = path in doc.changes;
  const state = edit(path);
  const record = value.record || (recordDocs(value.config, group) || {}).record || "";
  const parts = [
    h("h2", null, h("span", { class: "mono" }, name),
      tuned ? h("span", { class: "tag tuned" }, "● tuned") : h("span", { class: "tag plain" }, "committed")),
    h("p", { class: "crumb" }, value.config + (group ? " › " + group : ""),
      record && [" · ", h("span", { class: "mono" }, record)]),
  ];
  if (documented(value) && value.meaning) {
    parts.push(h("p", { class: "meaning-lg" }, inline(sentence(value.meaning))));
  } else if (documented(value)) {
    parts.push(h("p", { class: "prompt" }, "No description yet. Add ",
      h("code", null, "@param " + name), " to " + (record || "its record") + "'s Javadoc, saying "
        + "what it does and in what unit."));
  }
  parts.push(h("dl", { class: "facts" },
    h("dt", null, "Now"), h("dd", null, h("b", null, withUnit(now, value)),
      state.isEdit && state.valid && [" ", h("span", { class: "tag edit" },
        "✎ " + shown(state.number) + " staged")]),
    h("dt", null, "Committed"), h("dd", null, withUnit(value.committed, value)),
    h("dt", null, "Tuning limits"), h("dd", null, value.limits ? value.limits
        + (value.unit ? " " + value.unit : "") : "none: any number"),
    h("dt", null, "Unit"), h("dd", null, value.unit || "none: a plain number, in what its "
        + "meaning says")));
  if (compact) return parts;
  const about = recordDocs(value.config, group);
  if (about && about.doc && about.doc.text) {
    parts.push(h("h3", null, "About " + (about.record || group || value.config)),
      prose(about.doc.text));
  }
  // Whether an experiment (a rig, in code) runs a test motion with this value as a gain: where to
  // tune it by comparing runs, or, with none, where its effect shows instead.
  const rigs = Object.entries(doc.rigs || {}).filter(([, rig]) => (rig.gains || []).includes(path));
  const others = rigs.flatMap(([, rig]) => rig.gains).filter((gain) => gain !== path && described(gain))
    .map((gain) => nameOf(described(gain)));
  parts.push(h("h3", null, "Experiment"), rigs.length === 0
    ? h("p", { class: "muted" }, "No experiment tests this value. Change it, then judge the effect "
        + "from the robot's logs, in AdvantageScope.")
    : h("p", null, "Test it in Experiment: ",
      rigs.map(([rig], index) => [index > 0 ? ", " : "", h("a", { href: "#experiment/" + rig }, rig)]),
      others.length > 0 ? ", along with " + others.join(", ") : "", "."));
  const change = doc.changes[path];
  if (change) {
    parts.push(h("h3", null, "Commit it"),
      h("pre", null, change.builderCall + "   // was " + change.committed),
      h("p", { class: "meta" }, "In the robot's configuration: the " + value.config + "'s "
          + (group ? "." + group.slice(group.lastIndexOf(".") + 1) + "(…) " : "") + "builder."));
  }
  if (documented(value)) {
    parts.push(h("p", { class: "meta" }, value.meaning
      ? ["From ", h("code", null, "@param " + name), " in " + (record || "its record")
          + "'s Javadoc: edit it there, and the page says so after the next build."]
      : "The record's own checks run on every change, and a refusal names them."));
  }
  return parts;
}

/** A record's documentation: what it is, then each of its values with what it means. */
export function groupDocs(config, group) {
  const doc = robot.doc;
  const about = recordDocs(config, group);
  const paths = Object.keys(doc.tunables).filter((path) =>
    doc.tunables[path].config === config && groupOf(doc.tunables[path]) === group);
  const parts = [
    h("h2", null, h("span", { class: "mono" }, group || config),
      about && about.record && h("span", { class: "tag plain mono" }, about.record)),
    h("p", { class: "crumb" }, config + " · " + paths.length
        + (paths.length === 1 ? " tunable value" : " tunable values")),
  ];
  if (about && about.meaning) parts.push(h("p", { class: "meaning-lg" }, inline(sentence(about.meaning))));
  if (about && about.doc && about.doc.text) parts.push(prose(about.doc.text));
  else if (about) {
    parts.push(h("p", { class: "prompt" }, "No description yet: add Javadoc to "
        + (about.record || "the record") + "."));
  }
  parts.push(h("h3", null, "Its values"), valueTable(paths));
  return parts;
}

/** A configuration's documentation: its root record's text, and its records. */
export function configDocs(config) {
  const docs = robot.doc.configs && robot.doc.configs[config];
  const parts = [h("h2", null, config,
    docs && docs.record && h("span", { class: "tag plain mono" }, docs.record))];
  if (docs && docs.doc && docs.doc.text) parts.push(prose(docs.doc.text));
  else if (docs) {
    parts.push(h("p", { class: "prompt" }, "No description yet: add Javadoc to "
        + (docs.record || "its record") + "."));
  }
  const groups = docs ? docs.groups || [] : [];
  if (groups.length > 0) {
    parts.push(h("h3", null, "Its records"), h("ul", null, groups.map((group) => h("li", null,
      h("span", { class: "mono" }, group.path), " (" + group.record + ")",
      group.meaning ? ": " + group.meaning : ""))));
  }
  const paths = Object.keys(robot.doc.tunables).filter((path) =>
    robot.doc.tunables[path].config === config && groupOf(robot.doc.tunables[path]) === "");
  if (paths.length > 0) parts.push(h("h3", null, "Its values"), valueTable(paths));
  return parts;
}

/** Values in a table: each with what it means, now, and committed; each a way to its row. */
function valueTable(paths) {
  const doc = robot.doc;
  return h("table", { class: "settable" },
    h("thead", null, h("tr", null, h("th", { scope: "col" }, "Value"),
      h("th", { scope: "col" }, "Meaning"), h("th", { scope: "col" }, "Now"),
      h("th", { scope: "col" }, "Committed"))),
    h("tbody", null, paths.map((path) => {
      const value = doc.tunables[path];
      const hash = "#tune/" + value.config + "/" + value.field;
      return h("tr", null,
        h("td", null, h("a", { class: "mono", href: hash, onclick: (event) => {
          event.preventDefault();
          go(hash);
        } }, nameOf(value)), path in doc.changes && [" ", h("span", { class: "tag tuned" }, "●")]),
        h("td", null, value.meaning ? firstSentence(value.meaning) : ""),
        h("td", { class: "num" }, withUnit(doc.values[path], value)),
        h("td", { class: "num" }, shown(value.committed)));
    })));
}
