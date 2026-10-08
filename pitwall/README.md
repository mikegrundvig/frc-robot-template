# Pitwall

Pitwall is the robot program's own web page: status, actions, live tuning, experiments, devices,
and the simulator's controls, served by the program itself on port 5820 (http://localhost:5820 in
the simulator). It's a library of its own (`com.michaelgrundvig.frc.pitwall`), free of robot code:
the robot program contributes to it, as data. The template's [README](../README.md) describes each
part; this file is for contributors, and covers its scenes: the World view.

**Another port:** `Pitwall.start(name)` serves on 5820 unless the `pitwall.port` system property
names another (`-Dpitwall.port=5920`; 0 picks a free one, and Pitwall prints the one it took). In
the template, `./gradlew run -Ppitwall.port=5920` passes it to the simulator. Run a second simulator
beside yours (an agent's, another branch's) on another port: a page left open on 5820 reconnects to
whatever answers there, and drives it.

## Scenes: the World view

A **scene** is the world drawn in 3D on the page, from what the robot program says is there: a
field, the robot on it, what its cameras see. Pitwall knows nothing of fields or robots: a scene
is named frames and shapes in them, grouped into layers the page can switch on and off,
and the page draws every shape itself. With no scene added, the page has no World view. A robot
program adds its scenes to `pitwall.scenes()`, as Pitwall's own `ScenesTest` does:

```java
scenes.add(
    Scene.named("world", "World")
        .layer(field) // describes once: walls, tags
        .layer(robotLayer) // describes, then updates while a page shows the scene
        .layer(rays, Scene.Visibility.OFF) // off until switched on, on the page
        .focus("robot")); // the frame the page's Follow keeps in view
```

### A layer

```java fragment
interface Layer {
  String id(); String title();
  void describe(SceneBuilder scene);          // what stays: frames, values, shapes
  default void update(SceneState state) {}    // what moves: in the robot loop, while a page shows it
  default Optional<Inspection> inspect(String shapeId) { ... }  // a click, on a server thread
  default Object configuration() { ... }      // describe again when this changes (by equals)
}
```

`Layer.of(id, title, describe)` makes one that only describes.

- **`describe`** runs when the scene is added, and again only when the layer's `configuration()`
  changes. The whole scene is checked then: a frame whose parent isn't a frame, two shapes with one
  id, a shape in a frame the scene doesn't have, or a value it doesn't declare, stops the program
  with what's wrong, as a configuration mistake does.
- **`update`** runs every loop while a page shows the scene, and never while none does. It says
  what moved, and keeps it to a handful of numbers: everything said is kept until it's said again.
  A layer that throws costs only itself, and what it threw is printed once.
- **`inspect`** is for a layer that answers clicks itself. Most don't need it: a shape made with
  `.inspect(() -> Inspection.of(...))` answers for itself.

### Frames, values, and shapes

**Frames** are named, each with a parent and a pose in it; the root is `field`. A layer may draw in
any frame another layer of its scene adds. Lengths are meters and angles radians, turned as WPILib
turns (roll about x, then pitch about y, then yaw about z):

```java fragment
scene
    .frame("robot", "field") // moved by update
    .frame(
        "camera",
        "robot",
        new Transform3d(0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-10), 0))) // fixed, here
    .value("speed", 0.05); // a size that follows a number
...
scene.frame("estimate", "field").deadband("estimate", 0.01, Math.toRadians(0.5)); // "Quiet when still"
```

**Shapes**, each with an id unique in its scene, in a frame, set up by name:

| Shape | What it is |
|---|---|
| `box` | a box centered where it's put: `size(x, y, z)`; filled faintly, `outline()` for edges only |
| `cylinder` | along its z: `radius`, or `radii(x, y)`, or radii that follow two values (an ellipse); `length` 0 makes a disc or, with `outline()`, a ring |
| `line` | `from(...).to(...)` in its frame, or `between(frameA, frameB)`, kept between two frames' origins as they move |
| `polyline` | points, `closed()` or not |
| `polygon` | a flat polygon in its frame's xy plane, filled faintly |
| `mesh` | vertices and triangles: anything else, such as a ramp |
| `arrow` | `from(...).to(...)`, or a `length` that follows a value (a wheel's speed; negative points back) |
| `frustum` | what a camera sees, along its frame's x: `fov(horizontal, vertical)`, `range(near, far)` |
| `label` | a few words, the same size on screen at any zoom: a tag's number, a camera's name |

Each takes `at(...)` (where in its frame, turned or not), `style(...)`, `inspect(...)`, and
`reveal(ids...)`.

**Styles** are the page's color tokens by meaning, never colors: `Style.OK`, `BAD`, `MUTED`,
`ACCENT`, `TRUTH`, each `.dashed()` or not. Color never carries meaning alone: a dashed line is
something not to trust as drawn (a rejected measurement, a bound, the simulator's truth). The page
takes each token from its theme, light or dark.

**In `update`** a layer says, through `SceneState`: a frame's pose (`pose(name, x, y, yaw)`, or a
`Pose2d` or `Pose3d`), a value, a shape's style (`style(id, Style.BAD)`, or `null` to restore the
described one), and its shapes of the moment: `state.shapes()` starts them over, and the shapes
added replace the last ones whole. A layer that doesn't call it keeps what it has, so call it only
when something changed.

**A click** shows the shape's `Inspection` beside the drawing: a title, labeled details, and the
names of page actions (`pitwall.actions()`) it ends in, run exactly as from Pit, gate and all. The
supplier runs on a web server thread, so it reads only what can't change under it, such as a record
the loop has finished with. A shape of the moment stays inspectable for two seconds after it's
gone, so with the page's *Hold* (which freezes the drawing) a sight line that came and went can
still be clicked. While its details are open, the shape clicked and the shapes it `reveal`s are
drawn whatever their layers: a camera reveals its full view, kept in a layer that starts off. What
it reveals must be in the scene's description, or of the same moment.

### What it costs, and the wire

- **The description** goes in the page's first snapshot, as the `scenes` section (`{id: {"title",
  "focus", "layers": {id: {"title", "shown", "frames", "shapes"}}}}`), and again only when a layer is
  described again. Numbers go to the millimeter and the tenth of a degree.
- **What moves** goes only to the pages that show the scene. The page names it on its event stream
  (`/api/events?views=world`, and `PUT /api/views {"views": [...]}` as it changes or its tab is
  hidden). A page that starts showing it first gets a `replace` event with all of it, then JSON
  merge patches at `scenes.<id>.state`: `{"frames": {name: pose}, "values": {name: number},
  "styles": {shape: style}, "shapes": {layer: [shape, ...]}}`. A pose is `[x, y, yaw]` when level on
  the floor, else `[x, y, z, roll, pitch, yaw]`, in meters and degrees.
- **Quiet when still:** a frame is sent only once it has moved its deadband from what was last sent
  (by default the resolution, a millimeter and a tenth of a degree), a value its resolution. An
  estimate that wanders a few millimeters while the robot stands still takes a wider deadband, so
  a still robot sends nothing.
- **At most 20 patches a second,** each holding only what changed, serialized off the robot loop
  and shared by every page showing the scene.
- **Click-to-inspect** is `GET /api/scenes/<id>/inspect/<shape>`, answered from what the loop
  published; 404 once nothing is known of the shape.
- **With no page open it costs nothing:** the loop doesn't call `update`, or anything else of a
  scene.

Measured by `SceneStreamTest` (a robot driving on a field with 32 tags) and, for the template's
own robot, its `WorldViewStreamSimTest`; the design's limit is 250 kbps.

### The page

`page/world.js` is the view; `page/scene-render.js` and `page/scene-shapes.js` draw with three.js,
which loads the first time the view is shown. One renderer, two cameras: *Top*, orthographic (the
2D map), and *3D*, orbiting. *Fit* frames what stays put, *Follow* keeps the scene's focus frame
in view, *Hold* freezes the drawing. A click picks the nearest inspectable line within 12 px, so a
thin ray is easy to hit, else the solid under the pointer. It draws only when a patch arrives or
something is dragged, never in a loop. A browser without WebGL 2 is told so, and the rest of the
page works.

**three.js** comes from its WebJar on Maven Central (`gradle/libs.versions.toml`). The build
(`vendorThree` in `build.gradle`) takes three files out of it, its own minified build and the
orbit controls, into the jar, served at `/three/`. The orbit controls import three.js by the bare
name `'three'`, which a browser resolves only through an import map, and an import map is inline
script, which the page's Content-Security-Policy (`default-src 'self'`) refuses: so the copy
imports `./three.module.min.js` instead, and the build fails if that line stops matching, or if
any file it copies imports anything by a bare name (an update adding `'three/addons/...'`, say). Nothing
here writes a `style` attribute: the stylesheet sizes the canvas, and labels are textures drawn on
a canvas. three.js's license is in [`licenses/`](../licenses/three.js-License.md).

### Not building

The World view draws the code's beliefs, to act on: no timeline, scrubbing, log loading, CAD, game
pieces, or charts. AdvantageScope does those.

## The template's World

The template contributes its world from `frc.robot.views.world` (`WorldView.scene`, added in
`RobotContainer.addTo`), and `./gradlew run` shows it at http://localhost:5820/#world:

- **Field:** the field model's walls and carpet, its raised ground (bumps, ramps, platforms), and
  its AprilTags where the layout puts them, each with its number.
- **Robot:** at the localization estimate, on the ground under it and tilted as its IMU says, as
  its `RobotDefinition` describes it: its frame and bumpers from `Body` with an arrow out the
  front; each wheel at its module, turned to its measured angle, with an arrow as far as it will go
  in 0.2 s; each camera at its configured mount, an arrow along its optical axis and the first
  meter of its view, its name beside it, off the robot; and the gyro's IMU, a small block where its
  `ImuMount` puts it, turned as it's mounted, with an arrow along its own x axis. A click on a
  camera says how it's mounted, offers its calibration, and shows its full view until the details
  close; a click on the IMU says how it's mounted and whether its accelerometer has been checked. A
  wedge pointing the wrong way is a mistake in `Robots.java`, found at a glance.
- **Camera views,** off at first: every camera's full view, out as far as it reads tags (its
  sensor's `sightRange`, 7 m on the template's robot).
- **Uncertainty (2σ):** the estimate's position uncertainty as a dashed ellipse, two standard
  deviations across; in the color of trouble while the estimate is lost or the gyro is.
- **Sight lines:** from each camera to the tags its measurements used, solid where the estimator
  accepted the measurement and dashed where it refused it. A click says why: the verdict, the
  innovation, χ², the distance, the measured pose, and the solve, and offers the camera's
  calibration.
- **Truth:** in the simulator only, where the robot really is, as a dashed ghost beside the
  estimate.
