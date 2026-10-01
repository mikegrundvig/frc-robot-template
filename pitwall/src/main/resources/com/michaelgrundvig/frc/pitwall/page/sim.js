// Sim: drive and stage the simulation from the page. Shown only in the simulator: a robot program
// that isn't simulated never offers it. Empty for now; the simulator panel fills it.
//
// A view is a module whose default export is {id, title, mount(element), show(), hide(), route()}:
// mount builds it once, into its own section, and it then follows the robot through core.js's
// onChange; show and hide (optional) say when it's on screen; route (optional) takes the rest of
// the address, after "#sim/".

import { h } from "./core.js";

function mount(view) {
  view.append(
    h("h1", null, "Sim"),
    h("p", { class: "muted" }, "Nothing here yet."));
}

export default { id: "sim", title: "Sim", simulatorOnly: true, mount };
