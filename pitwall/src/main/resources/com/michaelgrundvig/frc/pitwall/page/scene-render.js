// Draws a scene with three.js: its frames as groups, its shapes in them (scene-shapes.js), moved by
// the robot's patches. Loaded only once the World view is first shown, with three.js itself
// (/three/, served from the robot like uPlot: no internet at an event).
//
// One renderer, two cameras: from above, orthographic (the 2D map), and around, in perspective. It
// draws only when something changed (a patch, a drag, a resize), never in a loop, so a page left on
// a still robot costs the laptop nothing. Colors are the page's theme tokens, read through computed
// style, light or dark. The canvas is sized by the page's stylesheet: nothing here writes a style.

import * as THREE from "/three/three.module.min.js";
import { OrbitControls } from "/three/OrbitControls.js";
import { Kit, TOKENS, build, measureDashes, restyle, setPose } from "./scene-shapes.js";

/** How far from a thin line a click still picks it, in CSS pixels. */
const PICK_PIXELS = 12;

/** A label's height on screen, in CSS pixels, at any zoom. */
const LABEL_PIXELS = 15;

/** Whether this browser can draw 3D at all: three.js needs WebGL 2. */
export function supported() {
  try {
    return Boolean(document.createElement("canvas").getContext("webgl2"));
  } catch (error) {
    return false;
  }
}

export class SceneRenderer {
  /**
   * @param canvas where it draws
   * @param onLost told when the browser takes the drawing away (a lost GPU context)
   */
  constructor(canvas, onLost) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    canvas.addEventListener("webglcontextlost", (event) => {
      event.preventDefault();
      onLost();
    });
    this.scene = new THREE.Scene();
    this.kit = new Kit();
    this.root = new THREE.Group();
    this.scene.add(this.root);
    this.grid = null;
    this.frames = new Map(); // name -> group; "field" is the root
    this.described = new Map(); // name -> the pose it was described at, or null
    this.shapes = new Map(); // id -> built shape
    this.layers = new Map(); // layer id -> {shown, ids: Set, moment: [built]}
    this.values = new Map();
    this.labels = [];
    this.bounds = new THREE.Box3(new THREE.Vector3(0, 0, 0), new THREE.Vector3(10, 10, 1));
    this.selected = null;
    this.revealed = new Set(); // the shape clicked and those it reveals: shown whatever their layer
    this.followed = null;
    this.followedAt = null;
    this.fitted = new Set();
    this.moved = new Set(); // the views turned, panned or zoomed by hand since Fit framed them
    this.width = 0; // the canvas's width as last sized, in CSS pixels
    this.pending = false;

    // From above: x to the right, y up the screen, as a field's drawing has it.
    this.top = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.1, 400);
    this.top.up.set(0, 1, 0);
    // Around: z is up, as on the field.
    this.around = new THREE.PerspectiveCamera(45, 1, 0.05, 500);
    this.around.up.set(0, 0, 1);
    this.topControls = new OrbitControls(this.top, canvas);
    this.topControls.enableRotate = false;
    this.topControls.screenSpacePanning = true;
    this.topControls.mouseButtons = { LEFT: THREE.MOUSE.PAN, MIDDLE: THREE.MOUSE.DOLLY,
                                      RIGHT: THREE.MOUSE.PAN };
    this.topControls.touches = { ONE: THREE.TOUCH.PAN, TWO: THREE.TOUCH.DOLLY_PAN };
    this.aroundControls = new OrbitControls(this.around, canvas);
    this.aroundControls.maxPolarAngle = Math.PI / 2 - 0.02; // never under the floor
    for (const controls of [this.topControls, this.aroundControls]) {
      let dragging = false;
      controls.addEventListener("start", () => (dragging = true));
      controls.addEventListener("end", () => (dragging = false));
      controls.addEventListener("change", () => {
        if (dragging) this.moved.add(this.view); // by hand, not by Fit or Follow
        this.draw();
      });
    }
    this.use("top");
    this.recolor();
  }

  // ---- What's drawn ----

  /**
   * Builds the scene as described: {"layers": {id: {"title", "shown", "frames", "shapes"}}}. What
   * the robot sent of its state is applied after, by {@link apply}. A layer switched on or off
   * here stays so when the scene is described again.
   */
  describe(description) {
    this.clear();
    const layers = (description && description.layers) || {};
    // Every layer's frames first, since a shape may be in a frame another layer adds.
    this.frames.set("field", this.root);
    this.described.set("field", null);
    const parents = new Map();
    for (const layer of Object.values(layers)) {
      for (const [name, frame] of Object.entries(layer.frames || {})) {
        const group = new THREE.Group();
        group.name = name;
        this.frames.set(name, group);
        this.described.set(name, frame.pose || null);
        parents.set(name, frame.parent);
        if (frame.pose) setPose(group, frame.pose);
      }
    }
    for (const [name, parent] of parents) {
      (this.frames.get(parent) || this.root).add(this.frames.get(name));
    }
    for (const [id, layer] of Object.entries(layers)) {
      const before = this.layers.get(id);
      const chosen = Boolean(before && before.chosen);
      const entry = { shown: chosen ? before.shown : layer.shown !== false, chosen,
                      ids: new Set(), moment: [] };
      this.layers.set(id, entry);
      for (const def of layer.shapes || []) entry.ids.add(this.add(def, id).def.id);
    }
    this.bounds = this.staticBounds();
    this.makeGrid();
    this.place();
    this.fitted = new Set();
    this.moved = new Set();
    this.fit();
    this.draw();
  }

  /**
   * What stays put, which the view frames: every shape but those in frames the robot moves (a
   * frame described with no pose), which stand at the origin until it does.
   */
  staticBounds() {
    this.scene.updateMatrixWorld(true);
    const bounds = new THREE.Box3();
    for (const built of this.shapes.values()) {
      let moves = false;
      for (let object = built.object.parent; object && object !== this.root;
        object = object.parent) {
        if (this.frames.get(object.name) === object && this.described.get(object.name) === null) {
          moves = true;
        }
      }
      if (!moves && !built.between) bounds.expandByObject(built.object, true);
    }
    if (bounds.isEmpty()) bounds.set(new THREE.Vector3(0, 0, 0), new THREE.Vector3(10, 10, 1));
    return bounds;
  }

  /**
   * Applies the robot's state: {"frames", "values", "styles", "shapes"}. With {@code changed}, only
   * what changed (a patch); without, all of it, as when the page starts showing the scene or Hold
   * lets go: then whatever the state doesn't say is as described.
   */
  apply(state, changed) {
    const whole = !changed;
    const part = changed || state || {};
    if (whole) this.reset();
    for (const [name, pose] of Object.entries(part.frames || {})) {
      const frame = this.frames.get(name);
      if (frame && pose) setPose(frame, pose);
    }
    const resized = new Set();
    for (const [name, value] of Object.entries(part.values || {})) {
      this.values.set(name, typeof value === "number" ? value : 0);
      for (const built of this.shapes.values()) {
        if (built.follows && built.follows.includes(name)) resized.add(built);
      }
    }
    for (const built of resized) built.resize(this.values);
    for (const [id, style] of Object.entries(part.styles || {})) {
      const built = this.shapes.get(id);
      if (built) restyle(built, style || built.def.style, this.kit, this.selected === id);
    }
    for (const [layer, defs] of Object.entries(part.shapes || {})) this.moment(layer, defs || []);
    this.place();
    this.draw();
  }

  /** Back to the description: frames where described, sizes at zero, styles and shapes as built. */
  reset() {
    for (const [name, pose] of this.described) {
      const frame = this.frames.get(name);
      if (name === "field" || !frame) continue;
      if (pose) setPose(frame, pose);
      else {
        frame.position.set(0, 0, 0);
        frame.rotation.set(0, 0, 0);
      }
    }
    this.values.clear();
    for (const built of this.shapes.values()) {
      if (built.resize) built.resize(this.values);
      if (built.style !== built.def.style) {
        restyle(built, built.def.style, this.kit, this.selected === built.def.id);
      }
    }
    for (const layer of this.layers.keys()) this.moment(layer, []);
  }

  /** A layer's shapes of the moment, replacing the last of them whole. */
  moment(layerId, defs) {
    const layer = this.layers.get(layerId);
    if (!layer) return;
    for (const built of layer.moment) this.remove(built);
    layer.moment = defs.map((def) => this.add(def, layerId));
    for (const built of layer.moment) built.resize && built.resize(this.values);
  }

  add(def, layerId) {
    const built = build(def, this.kit, (text) => this.label(text));
    built.layer = layerId;
    const old = this.shapes.get(def.id);
    if (old) this.remove(old);
    this.shapes.set(def.id, built);
    // A line between two frames lives in the root, and is kept between them (place).
    const parent = built.between ? this.root : this.frames.get(def.frame) || this.root;
    parent.add(built.object);
    built.object.visible = this.layerShown(layerId) || this.revealed.has(def.id);
    if (this.selected === def.id) restyle(built, built.style, this.kit, true);
    return built;
  }

  /** Whether a layer is switched on: a shape in no layer always is. */
  layerShown(layerId) {
    const layer = this.layers.get(layerId);
    return !layer || layer.shown;
  }

  remove(built) {
    if (this.shapes.get(built.def.id) === built) this.shapes.delete(built.def.id);
    built.object.removeFromParent();
    built.object.traverse((object) => {
      if (object.geometry) object.geometry.dispose();
      if (object.isSprite) {
        object.material.map.dispose();
        object.material.dispose();
        this.labels = this.labels.filter((label) => label !== object);
      }
    });
  }

  clear() {
    for (const built of [...this.shapes.values()]) this.remove(built);
    for (const frame of this.frames.values()) if (frame !== this.root) frame.removeFromParent();
    this.frames.clear();
    this.described.clear();
    this.values.clear();
  }

  /** Puts each line between two frames where those frames are now. */
  place() {
    this.scene.updateMatrixWorld(true);
    const a = new THREE.Vector3();
    const b = new THREE.Vector3();
    for (const built of this.shapes.values()) {
      if (!built.between) continue;
      const from = this.frames.get(built.between[0]);
      const to = this.frames.get(built.between[1]);
      if (!from || !to) continue;
      from.getWorldPosition(a);
      to.getWorldPosition(b);
      const line = built.lines[0];
      line.geometry.setFromPoints([a, b]);
      line.geometry.computeBoundingSphere();
      if (line.material.isLineDashedMaterial) measureDashes(line);
    }
  }

  // ---- Layers, cameras, following ----

  /** Shows or hides a layer, as chosen on the page: kept when the scene is described again. */
  showLayer(id, shown) {
    const layer = this.layers.get(id);
    if (!layer) return;
    layer.shown = shown;
    layer.chosen = true;
    for (const built of this.shapes.values()) {
      if (built.layer === id) built.object.visible = shown || this.revealed.has(built.def.id);
    }
    this.draw();
  }

  /** Uses a camera: "top", from above, or "around", in perspective. */
  use(view) {
    this.view = view;
    this.camera = view === "top" ? this.top : this.around;
    this.controls = view === "top" ? this.topControls : this.aroundControls;
    this.topControls.enabled = view === "top";
    this.aroundControls.enabled = view !== "top";
    this.followedAt = null;
    this.resize();
    if (this.fitted && !this.fitted.has(view)) this.fit();
  }

  /** Frames the whole scene in the camera in use. */
  fit() {
    const center = this.bounds.getCenter(new THREE.Vector3());
    const size = this.bounds.getSize(new THREE.Vector3());
    if (this.view === "top") {
      this.top.zoom = 1;
      this.topControls.target.set(center.x, center.y, 0);
      this.top.position.set(center.x, center.y, 100);
      this.fitTop = Math.max(size.y, size.x / this.aspect()) / 2 * 1.08;
      this.resize();
    } else {
      // From behind the blue end, a little to the side, looking down the field.
      const reach = Math.max(size.x, size.y);
      this.aroundControls.target.set(center.x, center.y, 0);
      this.around.position.set(center.x - reach * 0.75, center.y - reach * 0.45, reach * 0.55);
    }
    this.controls.update();
    this.fitted.add(this.view);
    this.moved.delete(this.view);
    this.followedAt = null;
    this.draw();
  }

  /**
   * Fits the drawing to the canvas after the page changed its width (the details panel opening or
   * closing beside it), so what was clicked stays on screen. A view as Fit framed it is framed
   * again; one moved by hand keeps its left edge, where the panel isn't, and pans just enough to
   * keep the point clicked in sight. Following, the robot is put back in the middle.
   *
   * @param keepX where the point to keep was, in CSS pixels from the canvas's left; null for none
   */
  refit(keepX = null) {
    const before = this.width;
    this.resize();
    const after = this.width;
    if (!before || !after || before === after) return;
    if (this.followed) {
      this.followedAt = null;
      this.draw();
      return;
    }
    if (!this.moved.has(this.view)) {
      this.fit();
      return;
    }
    // A narrower canvas shows less on both sides alike: shifting it back by half the change keeps
    // the left edge; then, if the point kept would land off the right, by as much more.
    const margin = Math.min(40, after / 4);
    let shift = (before - after) / 2;
    if (keepX !== null && keepX > after - margin) shift -= keepX - (after - margin);
    this.pan(shift);
  }

  /** Moves the view so the drawing shifts right on screen by some CSS pixels: left if negative. */
  pan(pixels) {
    this.camera.updateMatrixWorld();
    const right = new THREE.Vector3().setFromMatrixColumn(this.camera.matrixWorld, 0);
    const move = right.multiplyScalar(-pixels * this.metersPerPixel(this.controls.target));
    this.camera.position.add(move);
    this.controls.target.add(move);
    this.controls.update();
    this.draw();
  }

  /** Keeps a frame (the robot) in the middle of the view as it moves; null lets it go. */
  follow(frame) {
    this.followed = frame && this.frames.has(frame) ? frame : null;
    this.followedAt = null;
    this.draw();
  }

  // ---- Drawing ----

  /** Asks for a drawing at the next frame: one, however many things changed before it. */
  draw() {
    if (this.pending) return;
    this.pending = true;
    requestAnimationFrame(() => {
      this.pending = false;
      this.render();
    });
  }

  render() {
    if (this.followed) this.keepFollowing();
    this.sizeLabels();
    this.renderer.render(this.scene, this.camera);
  }

  keepFollowing() {
    const frame = this.frames.get(this.followed);
    if (!frame) return;
    const now = frame.getWorldPosition(new THREE.Vector3());
    now.z = 0;
    if (this.followedAt) {
      const moved = now.clone().sub(this.followedAt);
      for (const [camera, controls] of [[this.top, this.topControls],
        [this.around, this.aroundControls]]) {
        camera.position.add(moved);
        controls.target.add(moved);
      }
    } else {
      const moved = now.clone().sub(this.controls.target);
      moved.z = 0;
      this.camera.position.add(moved);
      this.controls.target.add(moved);
    }
    this.followedAt = now;
    this.controls.update();
  }

  aspect() {
    const width = this.canvas.clientWidth || 1;
    const height = this.canvas.clientHeight || 1;
    return width / height;
  }

  /** Takes the canvas's size from the page's layout. */
  resize() {
    const width = this.canvas.clientWidth;
    const height = this.canvas.clientHeight;
    if (!width || !height) return;
    this.width = width;
    this.renderer.setSize(width, height, false); // false: the stylesheet sizes the canvas
    const half = this.fitTop || 5;
    const aspect = width / height;
    this.top.left = -half * aspect;
    this.top.right = half * aspect;
    this.top.top = half;
    this.top.bottom = -half;
    this.top.updateProjectionMatrix();
    this.around.aspect = aspect;
    this.around.updateProjectionMatrix();
    this.draw();
  }

  /** How many meters a CSS pixel spans at a point, in the camera in use. */
  metersPerPixel(point) {
    const height = this.canvas.clientHeight || 1;
    if (this.camera.isOrthographicCamera) {
      return (this.top.top - this.top.bottom) / this.top.zoom / height;
    }
    const distance = this.camera.position.distanceTo(point);
    return (2 * distance * Math.tan((this.around.fov * Math.PI) / 360)) / height;
  }

  /** Keeps every label the same size on screen at any zoom. */
  sizeLabels() {
    const at = new THREE.Vector3();
    for (const sprite of this.labels) {
      sprite.getWorldPosition(at);
      const meters = LABEL_PIXELS * this.metersPerPixel(at);
      sprite.scale.set(meters * sprite.userData.aspect, meters, 1);
    }
  }

  /** A label's sprite: its text drawn in the page's text color, with an outline to stand out. */
  label(text) {
    const material = new THREE.SpriteMaterial({ depthTest: false, transparent: true });
    const sprite = new THREE.Sprite(material);
    sprite.renderOrder = 10;
    sprite.userData.text = text;
    this.paintLabel(sprite);
    this.labels.push(sprite);
    return sprite;
  }

  paintLabel(sprite) {
    const scale = 2 * Math.min(window.devicePixelRatio || 1, 2);
    const font = `600 ${16 * scale}px ` + (this.colors ? this.colors.font : "sans-serif");
    const canvas = document.createElement("canvas");
    const context = canvas.getContext("2d");
    context.font = font;
    const width = Math.ceil(context.measureText(sprite.userData.text).width) + 8 * scale;
    const height = 22 * scale;
    canvas.width = width;
    canvas.height = height;
    context.font = font;
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.lineJoin = "round";
    context.lineWidth = 4 * scale;
    context.strokeStyle = this.colors ? this.colors.backdrop : "#ffffff";
    context.strokeText(sprite.userData.text, width / 2, height / 2);
    context.fillStyle = this.colors ? this.colors.text : "#000000";
    context.fillText(sprite.userData.text, width / 2, height / 2);
    if (sprite.material.map) sprite.material.map.dispose();
    sprite.material.map = new THREE.CanvasTexture(canvas);
    sprite.material.map.colorSpace = THREE.SRGBColorSpace;
    sprite.material.needsUpdate = true;
    sprite.userData.aspect = width / height;
  }

  /** A faint meter grid under the scene, so distance and depth read in 3D. */
  makeGrid() {
    if (this.grid) {
      this.grid.removeFromParent();
      this.grid.geometry.dispose();
    }
    const min = this.bounds.min;
    const max = this.bounds.max;
    const points = [];
    for (let x = Math.floor(min.x); x <= Math.ceil(max.x); x++) {
      points.push(x, Math.floor(min.y), 0, x, Math.ceil(max.y), 0);
    }
    for (let y = Math.floor(min.y); y <= Math.ceil(max.y); y++) {
      points.push(Math.floor(min.x), y, 0, Math.ceil(max.x), y, 0);
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(points, 3));
    this.grid = new THREE.LineSegments(geometry, this.gridMaterial
      || (this.gridMaterial = new THREE.LineBasicMaterial({ transparent: true, opacity: 0.6 })));
    this.grid.renderOrder = -1;
    this.scene.add(this.grid);
    if (this.colors) this.gridMaterial.color.copy(this.colors.grid);
  }

  /** Takes the page's colors, as its theme has them now: after a change of theme, say. */
  recolor() {
    const style = getComputedStyle(document.documentElement);
    const color = (name, fallback) => {
      const value = style.getPropertyValue(name).trim();
      try {
        return new THREE.Color(value || fallback);
      } catch (error) {
        return new THREE.Color(fallback);
      }
    };
    const colors = { focus: color("--focus", "#2a78d6") };
    for (const [token, name] of Object.entries(TOKENS)) colors[token] = color(name, "#777777");
    colors.grid = color("--line", "#dddddd");
    this.colors = {
      ...colors,
      text: style.getPropertyValue("--text").trim() || "#000000",
      backdrop: style.getPropertyValue("--surface").trim() || "#ffffff",
      font: style.getPropertyValue("--sans").trim() || "sans-serif",
    };
    this.kit.recolor(colors);
    this.renderer.setClearColor(color("--surface", "#ffffff"));
    if (this.gridMaterial) this.gridMaterial.color.copy(colors.grid);
    for (const sprite of this.labels) this.paintLabel(sprite);
    this.draw();
  }

  // ---- Clicking ----

  /**
   * What a click at a point picks: the nearest inspectable line within a few pixels (a thin ray's
   * hit area is wide), else the nearest inspectable solid under the pointer. Null for nothing.
   */
  pick(clientX, clientY) {
    const rect = this.canvas.getBoundingClientRect();
    const x = clientX - rect.left;
    const y = clientY - rect.top;
    const pointer = new THREE.Vector2((x / rect.width) * 2 - 1, -(y / rect.height) * 2 + 1);
    const ray = new THREE.Raycaster();
    ray.setFromCamera(pointer, this.camera);
    let nearestLine = null;
    let lineDistance = PICK_PIXELS;
    let nearestSolid = null;
    let solidDistance = Infinity;
    for (const built of this.shapes.values()) {
      if (!built.def.inspect || !this.isShown(built)) continue;
      for (const line of built.lines) {
        const distance = this.pixelsTo(line, x, y, rect);
        if (distance < lineDistance) {
          lineDistance = distance;
          nearestLine = built;
        }
      }
      for (const fill of built.fills) {
        const hit = ray.intersectObject(fill, false)[0];
        if (hit && hit.distance < solidDistance) {
          solidDistance = hit.distance;
          nearestSolid = built;
        }
      }
    }
    const picked = nearestLine || nearestSolid;
    return picked ? { id: picked.def.id, layer: picked.layer } : null;
  }

  isShown(built) {
    for (let object = built.object; object; object = object.parent) {
      if (!object.visible) return false;
    }
    return true;
  }

  /** How far a point on the canvas is from a line's nearest segment on screen, in CSS pixels. */
  pixelsTo(line, x, y, rect) {
    const position = line.geometry.getAttribute("position");
    if (!position) return Infinity;
    const points = [];
    const v = new THREE.Vector3();
    for (let i = 0; i < position.count; i++) {
      v.fromBufferAttribute(position, i).applyMatrix4(line.matrixWorld).project(this.camera);
      points.push(v.z > 1 || v.z < -1 ? null
        : [(v.x + 1) / 2 * rect.width, (1 - v.y) / 2 * rect.height]);
    }
    const pairs = [];
    if (line.isLineSegments) {
      for (let i = 0; i + 1 < points.length; i += 2) pairs.push([points[i], points[i + 1]]);
    } else {
      for (let i = 0; i + 1 < points.length; i++) pairs.push([points[i], points[i + 1]]);
      if (line.isLineLoop && points.length > 2) pairs.push([points[points.length - 1], points[0]]);
    }
    let best = Infinity;
    for (const [a, b] of pairs) {
      if (a && b) best = Math.min(best, segmentDistance(x, y, a, b));
    }
    return best;
  }

  /**
   * Marks a shape as the one inspected (in the focus color), or none: it, and the shapes it reveals
   * (a camera's full view), are drawn whatever their layers until another is, or none.
   */
  select(id) {
    const before = this.shapes.get(this.selected);
    this.selected = id;
    if (before) restyle(before, before.style, this.kit, false);
    const now = this.shapes.get(id);
    if (now) restyle(now, now.style, this.kit, true);
    const was = this.revealed;
    this.revealed = new Set(now ? [id, ...(now.def.reveals || [])] : []);
    for (const shown of new Set([...was, ...this.revealed])) {
      const built = this.shapes.get(shown);
      if (built) built.object.visible = this.revealed.has(shown) || this.layerShown(built.layer);
    }
    this.draw();
  }

  dispose() {
    this.clear();
    this.topControls.dispose();
    this.aroundControls.dispose();
    this.renderer.dispose();
  }
}

/** How far a point is from a segment, in the same units. */
function segmentDistance(x, y, a, b) {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const length = dx * dx + dy * dy;
  const t = length === 0 ? 0 : Math.max(0, Math.min(1, ((x - a[0]) * dx + (y - a[1]) * dy) / length));
  return Math.hypot(x - (a[0] + t * dx), y - (a[1] + t * dy));
}
