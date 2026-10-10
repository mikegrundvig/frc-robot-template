// Sections that fold away under their headings, each view keeping which ones are folded in this
// browser, so a reload leaves them as they were. Open until someone folds one: nothing goes missing
// by default.

import { h } from "./core.js";

/**
 * A view's folds, kept under its own key.
 *
 * @param key where this browser keeps them, such as "pitwall.pit.folded"
 */
export function folds(key) {
  const folded = new Set(read(key));
  const save = () => {
    try {
      localStorage.setItem(key, JSON.stringify([...folded]));
    } catch (error) {
      // Kept for this page only: storage is off in this browser.
    }
  };
  return {
    /**
     * A section that folds under its summary.
     *
     * @param id what it's kept by: its title, or a name that stays the same across reloads
     * @param summary what shows while it's folded: its title, and anything beside it
     * @param options {className}: its classes, "card fold" by default; {open}: held open (a
     *     filter showing what it matched), which isn't the reader's choice and so isn't kept
     */
    section(id, summary, options, ...content) {
      const held = Boolean(options && options.open);
      const section = h("details", {
        class: (options && options.className) || "card fold",
        open: held || !folded.has(id),
      }, h("summary", null, summary), ...content);
      section.addEventListener("toggle", () => {
        if (held) return;
        if (section.open) folded.delete(id); else folded.add(id);
        save();
      });
      return section;
    },
  };
}

function read(key) {
  try {
    const kept = JSON.parse(localStorage.getItem(key) || "[]");
    return Array.isArray(kept) ? kept : [];
  } catch (error) {
    return [];
  }
}
