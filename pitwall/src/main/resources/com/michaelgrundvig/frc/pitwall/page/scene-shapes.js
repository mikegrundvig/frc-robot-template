// A scene's shapes as three.js objects: each kind the robot describes (box, cylinder, line,
// polyline, polygon, mesh, arrow, frustum, label), drawn in the page's colors, solid or dashed.
// Loaded with three.js, only once the World view is first shown (scene-render.js).
//
// Every shape is a group in its frame: its fills (faint, so what's behind shows through), its lines
// (where the style's meaning is, solid or dashed), and for a label, a sprite. Lengths are meters and
// angles degrees, as the robot sends them.

import * as THREE from "/three/three.module.min.js";

/** How strongly a shape is filled: faint, so lines and what's behind stay readable. */
const FILL_OPACITY = { muted: 0.1, accent: 0.28, ok: 0.2, bad: 0.2, truth: 0.12 };

/** The tokens a style can name, and the page's color token each is drawn in. */
export const TOKENS = { ok: "--ok", bad: "--bad", muted: "--muted", accent: "--accent",
                        truth: "--truth" };

/** A dash and its gap, in meters: visible across a field and on a robot alike. */
const DASH = 0.12;
const GAP = 0.08;

/** A turn in degrees as three.js takes it, in radians. */
const RAD = Math.PI / 180;

/** Scratch for measuring dashes. */
const FROM = new THREE.Vector3();
const TO = new THREE.Vector3();

/**
 * The materials every shape shares, one per color and kind, so a change of theme recolors every
 * shape at once without rebuilding any.
 */
export class Kit {
  constructor() {
    this.lines = new Map(); // "ok" / "ok dashed" -> material
    this.fills = new Map();
    this.solids = new Map(); // opaque, for arrowheads
    this.faint = new Map(); // barely there, for what spans much of the view: a camera's wedge
    this.selected = new THREE.LineBasicMaterial({ color: 0x2a78d6 });
    this.selectedDashed = new THREE.LineDashedMaterial({ color: 0x2a78d6, dashSize: DASH,
                                                         gapSize: GAP });
    for (const token of Object.keys(TOKENS)) {
      this.lines.set(token, new THREE.LineBasicMaterial());
      this.lines.set(token + " dashed",
        new THREE.LineDashedMaterial({ dashSize: DASH, gapSize: GAP }));
      this.fills.set(token, new THREE.MeshBasicMaterial({
        transparent: true, opacity: FILL_OPACITY[token], depthWrite: false,
        side: THREE.DoubleSide }));
      this.solids.set(token, new THREE.MeshBasicMaterial());
      this.faint.set(token, new THREE.MeshBasicMaterial({
        transparent: true, opacity: 0.035, depthWrite: false, side: THREE.DoubleSide }));
    }
  }

  /** Takes the theme's colors: {token: THREE.Color, focus}. */
  recolor(colors) {
    for (const token of Object.keys(TOKENS)) {
      this.lines.get(token).color.copy(colors[token]);
      this.lines.get(token + " dashed").color.copy(colors[token]);
      this.fills.get(token).color.copy(colors[token]);
      this.solids.get(token).color.copy(colors[token]);
      this.faint.get(token).color.copy(colors[token]);
    }
    this.selected.color.copy(colors.focus);
    this.selectedDashed.color.copy(colors.focus);
  }

  line(style) {
    return this.lines.get(style) || this.lines.get(tokenOf(style));
  }

  solid(style) {
    return this.solids.get(tokenOf(style));
  }
}

/** A style's color token: "bad dashed" is "bad"; anything unknown is muted. */
export function tokenOf(style) {
  const token = String(style || "").split(" ")[0];
  return token in TOKENS ? token : "muted";
}

export function isDashed(style) {
  return String(style || "").split(" ").includes("dashed");
}

/** A pose as the robot sends it ([x, y, yaw] or [x, y, z, roll, pitch, yaw]) put on an object. */
export function setPose(object, pose) {
  if (!Array.isArray(pose)) return;
  const [x, y, z, roll, pitch, yaw] = pose.length === 3
    ? [pose[0], pose[1], 0, 0, 0, pose[2]] : pose;
  object.position.set(x, y, z);
  // WPILib turns by roll about x, then pitch about y, then yaw about z, each about the fixed axes:
  // three.js calls that order ZYX.
  object.rotation.set(roll * RAD, pitch * RAD, yaw * RAD, "ZYX");
}

/**
 * Builds a shape's objects. Returns {def, object, lines, fills, values(values), label}: the group
 * to put in the shape's frame, its lines and fills (for picking and restyling), and for a shape
 * whose size follows the scene's values, how to resize it.
 */
export function build(def, kit, label) {
  const object = new THREE.Group();
  object.name = def.id;
  if (def.at) setPose(object, def.at);
  const built = { def, object, lines: [], fills: [], heads: [], resize: null, style: def.style };
  const builder = BUILDERS[def.kind];
  if (builder) builder(def, built, kit, label);
  restyle(built, def.style, kit);
  return built;
}

/**
 * Measures how far along a dashed line each of its points is, which places its dashes: what
 * three.js's computeLineDistances does, but written into the line's own attribute once it has one,
 * rather than a new one each time (a line kept between two frames is measured at every patch).
 */
export function measureDashes(line) {
  const geometry = line.geometry;
  const position = geometry.getAttribute("position");
  if (!position) return;
  let along = geometry.getAttribute("lineDistance");
  if (!along || along.count !== position.count) {
    along = new THREE.Float32BufferAttribute(new Float32Array(position.count), 1);
    geometry.setAttribute("lineDistance", along);
  }
  let distance = 0;
  if (line.isLineSegments) {
    // Each segment on its own, starting where the last ended, as three.js measures them.
    for (let i = 0; i + 1 < position.count; i += 2) {
      along.setX(i, distance);
      FROM.fromBufferAttribute(position, i);
      distance += FROM.distanceTo(TO.fromBufferAttribute(position, i + 1));
      along.setX(i + 1, distance);
    }
  } else {
    along.setX(0, 0);
    for (let i = 1; i < position.count; i++) {
      FROM.fromBufferAttribute(position, i - 1);
      distance += FROM.distanceTo(TO.fromBufferAttribute(position, i));
      along.setX(i, distance);
    }
  }
  along.needsUpdate = true;
}

/** Draws a built shape in a style; dashed lines need their lengths measured. */
export function restyle(built, style, kit, selected = false) {
  built.style = style;
  for (const line of built.lines) {
    line.material = selected ? (isDashed(style) ? kit.selectedDashed : kit.selected)
      : kit.line(style);
    if (line.material.isLineDashedMaterial) measureDashes(line);
  }
  for (const fill of built.fills) {
    fill.material = (fill.userData.faint ? kit.faint : kit.fills).get(tokenOf(style));
  }
  for (const head of built.heads) head.material = kit.solid(style);
}

/** Lines through points: a polyline, closed or not, or pairs of points as separate segments. */
function lineOf(points, how = "strip") {
  const geometry = new THREE.BufferGeometry().setFromPoints(
    points.map((p) => new THREE.Vector3(p[0], p[1], p[2])));
  if (how === "pairs") return new THREE.LineSegments(geometry);
  if (how === "loop") return new THREE.LineLoop(geometry);
  return new THREE.Line(geometry);
}

/** A solid's fill, and its edges as lines. */
function solid(built, geometry, outline) {
  if (!outline) {
    const fill = new THREE.Mesh(geometry);
    built.fills.push(fill);
    built.object.add(fill);
  }
  const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geometry, 20));
  built.lines.push(edges);
  built.object.add(edges);
}

const BUILDERS = {
  box(def, built) {
    const [l, w, h] = def.size || [1, 1, 1];
    solid(built, new THREE.BoxGeometry(l, w, h), def.outline);
  },

  cylinder(def, built) {
    // A cylinder of radius 1 along z, scaled to its radii: an ellipse's radii can follow values.
    const sized = new THREE.Group();
    built.object.add(sized);
    const length = def.length || 0;
    if (length > 0) {
      const geometry = new THREE.CylinderGeometry(1, 1, length, 40);
      geometry.rotateX(Math.PI / 2);
      const holder = { object: sized, lines: built.lines, fills: built.fills };
      solid(holder, geometry, def.outline);
    } else {
      if (!def.outline) {
        const disc = new THREE.Mesh(new THREE.CircleGeometry(1, 48));
        built.fills.push(disc);
        sized.add(disc);
      }
      const ring = [];
      for (let i = 0; i < 64; i++) {
        const a = (i / 64) * 2 * Math.PI;
        ring.push([Math.cos(a), Math.sin(a), 0]);
      }
      const loop = lineOf(ring, "loop");
      built.lines.push(loop);
      sized.add(loop);
    }
    const radii = def.radii || [def.radius || 1, def.radius || 1];
    built.resize = (values) => {
      const rx = typeof radii[0] === "string" ? values.get(radii[0]) || 0 : radii[0];
      const ry = typeof radii[1] === "string" ? values.get(radii[1]) || 0 : radii[1];
      sized.visible = rx > 0 && ry > 0;
      sized.scale.set(Math.max(rx, 1e-6), Math.max(ry, 1e-6), 1);
    };
    built.follows = radii.filter((r) => typeof r === "string");
    built.resize(new Map());
  },

  line(def, built) {
    // A line between two frames is placed in the scene's root and kept between them as they move.
    const line = lineOf([def.from || [0, 0, 0], def.to || [0, 0, 0]]);
    built.lines.push(line);
    built.object.add(line);
    if (def.between) built.between = def.between;
  },

  polyline(def, built) {
    const line = lineOf(def.points || [], def.closed ? "loop" : "strip");
    built.lines.push(line);
    built.object.add(line);
  },

  polygon(def, built) {
    const points = def.points || [];
    if (!def.outline && points.length >= 3) {
      const shape = new THREE.Shape(points.map((p) => new THREE.Vector2(p[0], p[1])));
      const geometry = new THREE.ShapeGeometry(shape);
      geometry.translate(0, 0, points[0][2]);
      const fill = new THREE.Mesh(geometry);
      built.fills.push(fill);
      built.object.add(fill);
    }
    const edge = lineOf(points, "loop");
    built.lines.push(edge);
    built.object.add(edge);
  },

  mesh(def, built) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position",
      new THREE.Float32BufferAttribute((def.vertices || []).flat(), 3));
    geometry.setIndex((def.triangles || []).flat());
    geometry.computeVertexNormals();
    solid(built, geometry, def.outline);
  },

  arrow(def, built) {
    const from = new THREE.Vector3(...(def.from || [0, 0, 0]));
    const to = new THREE.Vector3(...(def.to || [1, 0, 0]));
    const direction = to.clone().sub(from).normalize();
    const shaft = lineOf([from.toArray(), to.toArray()]);
    const head = new THREE.Mesh(new THREE.ConeGeometry(1, 1, 16));
    built.lines.push(shaft);
    built.heads.push(head);
    built.object.add(shaft, head);
    const place = (length) => {
      const visible = Math.abs(length) > 1e-3;
      shaft.visible = head.visible = visible;
      if (!visible) return;
      const way = direction.clone().multiplyScalar(Math.sign(length));
      const size = Math.min(0.12, Math.abs(length) * 0.35);
      const tip = from.clone().addScaledVector(way, Math.abs(length));
      const base = tip.clone().addScaledVector(way, -size);
      shaft.geometry.setFromPoints([from, base]);
      if (shaft.material && shaft.material.isLineDashedMaterial) measureDashes(shaft);
      head.scale.set(size * 0.35, size, size * 0.35);
      head.position.copy(base).addScaledVector(way, size / 2);
      head.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), way);
    };
    if (typeof def.length === "string") {
      built.follows = [def.length];
      built.resize = (values) => place(values.get(def.length) || 0);
      place(0);
    } else {
      place(to.distanceTo(from));
    }
  },

  frustum(def, built) {
    const [h, v] = (def.fov || [70, 50]).map((d) => d * RAD);
    const [near, far] = def.range || [0, 1];
    const corners = (d) => {
      const y = d * Math.tan(h / 2);
      const z = d * Math.tan(v / 2);
      return [[d, y, z], [d, -y, z], [d, -y, -z], [d, y, -z]];
    };
    const farCorners = corners(far);
    const nearCorners = near > 0 ? corners(near) : [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]];
    const segments = [];
    for (let i = 0; i < 4; i++) {
      segments.push(nearCorners[i], farCorners[i]);
      segments.push(farCorners[i], farCorners[(i + 1) % 4]);
      if (near > 0) segments.push(nearCorners[i], nearCorners[(i + 1) % 4]);
    }
    const lines = lineOf(segments, "pairs");
    built.lines.push(lines);
    built.object.add(lines);
    // Its sides, faintly: from above, a camera's view is a wedge on the floor.
    const sides = [];
    for (let i = 0; i < 4; i++) {
      const a = nearCorners[i];
      const b = nearCorners[(i + 1) % 4];
      const c = farCorners[(i + 1) % 4];
      const d = farCorners[i];
      sides.push(...a, ...d, ...c, ...a, ...c, ...b);
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(sides, 3));
    const fill = new THREE.Mesh(geometry);
    fill.userData.faint = true; // a wedge may reach across the field: barely tinted
    built.fills.push(fill);
    built.object.add(fill);
  },

  label(def, built, kit, label) {
    const sprite = label(def.text || "");
    built.label = sprite;
    built.object.add(sprite);
  },
};
