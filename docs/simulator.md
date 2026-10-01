# The simulator

`./gradlew run` runs the robot program on your laptop, in WPILib's simulation, with this template's
simulated world (`frc.robot.sim`): a drivetrain that really moves, cameras that render what they'd
see, and sensors that err as a season-worn robot's do. AdvantageScope shows the field.

Pitwall serves the simulator's controls (http://localhost:5820/api/simulator): the Driver Station,
joysticks, pausing and stepping, and failing the robot's parts live, which WPILib's simulator GUI
can't do. So far that's the server side, the API below; the Pitwall page's *Sim* view, which will
drive it in place of the GUI, is coming. Until then, drive the simulator from the GUI, and use the
API by hand (`curl`) for what the GUI can't do.

| Job | Command |
|---|---|
| The simulator, enabled on a start and drivable from the GUI's keyboard | `./gradlew run` |
| Without WPILib's simulator GUI, disabled: drive it through the API | `./gradlew run -Pgui=false` |
| With the GUI, whatever the default | `./gradlew run -Pgui` |
| Disabled, with no alliance station, like a real robot | `./gradlew run -PstartDisabled` |
| In a scenario: a fault, or something that happens to the robot | `./gradlew run -Pscenario=collision` |
| List the scenarios | `./gradlew scenarios` |

**The GUI is on by default for now**, and turns off by default once the *Sim* view replaces it;
`-Pgui` then brings it back. A real Driver Station can connect to the simulator either way
(`halsim_ds_socket` stays loaded). Without the GUI the robot starts disabled, since nothing on
screen could disable it yet; enable it through the API or a real Driver Station. Without the GUI
you also lose its HAL device views, its NetworkTables browser and plots, its `Field2d` window
(AdvantageScope and Elastic show the same), its notifier table, keyboard remapping, and its
joystick reading, which works whatever window has focus.

**The GUI and the API's joysticks don't mix.** Read in alpha-7's `DriverStationGui.cpp`
(`DriverStationExecute`): while the Driver Station is attached and not in autonomous, the GUI
writes all six joysticks every frame, zeros for a slot it has no device for, and announces new
data every 20 ms. Its *Zero disconnected joysticks* setting doesn't stop that: the check tests the
setting's pointer, not its value, so it always zeros. The GUI's own match time counts down while
enabled too. So with the GUI on, the API's joystick input is overwritten within a frame, and its
match time fights the GUI's. Drive through the API with `-Pgui=false`, or turn the GUI's Driver
Station off (its *Disable DS* setting), which makes it only read.

## Pitwall's simulator controls

Everything the *Sim* view will do to the simulator goes through an API Pitwall serves only in
WPILib's simulation, asked of the HAL itself (its runtime type is simulation; anything else counts
as a robot), and not while replaying a log. On a robot none of it exists: every route answers 404
and the event stream has none of its sections, so nothing can write the Driver Station's state
there (`PitwallRobotTest` checks this with WPILib told it's on a Systemcore). The controls are
exempt from the field lock: in the simulator they *are* the field, so attaching the field system
locks tuning and actions exactly as a real field does, and the controls stay usable to detach it
again.

### In the event stream

Three sections, beside the page's others, sent as JSON merge patches when they change:

- **`simulator`**: `enabled`, `mode` (`AUTONOMOUS`, `TELEOPERATED`, `UTILITY`, or `UNKNOWN` before
  a Driver Station set one), `allianceStation` (`RED_1` to `BLUE_3`, or `UNKNOWN`), `fmsAttached`,
  `dsAttached`, `matchTime` (whole seconds left, counting down while enabled; -1 for none),
  `paused`, and `joysticks`: for `"0"` and `"1"`, `fromPage` (the page drives it) and
  `centeredByDeadman` (the page went quiet, and it was centered).
- **`components`**: each part that can fail, by name: `{"kind", "faults": {name: {"does",
  "lasting", "undo", "parameters": [{"name", "unit", "min", "max", "initial"}], "on", "values"}}}`.
  A lasting fault has `on`, and while on, the `values` it was applied with; an event has neither.
  `undo` says what undoing a lasting fault does, or for an event, how the robot recovers.
- **`scenarios`**: each by name: `{"description", "faults": [{"component", "fault", "values"}]}`.

### Requests

Every request names the page's event stream in the `Pitwall-Stream` header, as the page's other
requests do; a joystick's input without one is refused (400), since the joystick is let go when
that stream closes. Errors are problem details, as elsewhere: 400 for a body of the wrong JSON
type or shape, 422 for one that names what isn't there or gives a value out of range (with the
reason), 409 for one that can't be done now.

| Request | Body | Answer |
|---|---|---|
| `GET /api/simulator` | | `{"simulator", "components", "scenarios"}`, read by the next loop; while paused, at once, as the last loop left it |
| `PATCH /api/simulator` | any of `enabled`, `mode`, `allianceStation`, `fmsAttached`, `dsAttached`, `matchTime` (0 to 600 s, or `null` for none), `paused` | the `simulator` section, once the loop has applied it all; 202 if it didn't within a second (while paused, it waits for a Step) |
| `POST /api/simulator/step` | | one loop of a paused simulator, then the `simulator` section; 409 while it runs |
| `POST /api/simulator/stop` | | 202, then the program ends: the stream closes |
| `PUT /api/simulator/joysticks` | `{"0": {"axes": [...], "buttons": [...], "povs": [...]}, "1": null}` | 204 at once |
| `POST /api/simulator/faults` | `{"component", "fault", "values": {name: number}}` to apply; `{"component", "fault", "undo": true}` to undo | the `components` section |
| `DELETE /api/simulator/faults` | | every lasting fault undone: the robot as built |
| `POST /api/simulator/scenarios` | `{"apply": name}` | the `components` section |

A `PATCH` applies its changes in one loop, in the order given. Enabling also attaches the Driver
Station, as a real one would be, and detaching it disables the robot. Pausing while paused answers
at once; a resume drops any pause asked before it, and a Stop any pause at all. A parameter a fault
request leaves out takes its `initial` value; a lasting fault whose values leave the part as it
was built changes nothing, and is refused (422, "changes nothing: as built"), never shown as on.

**Joysticks.** Each joystick's whole input, by WPILib's indices (for a gamepad,
`org.wpilib.driverstation.Gamepad`'s): axes from -1 to 1, a stick negative pushed up or left, at
most 12; buttons pressed or not, at most 64; POVs in degrees clockwise from up, in steps of 45, or
-1 centered, at most 8. A joystick is declared a gamepad, with at least a gamepad's 6 axes, 26
buttons, and 1 POV, the first time the page drives it. `null` lets it go: centered, and no longer
the page's. A joystick the page never drove is never touched, so the GUI's keyboard or a real
Driver Station still can. The page sends a joystick's input when it changes, at most once a frame,
and again every 200 ms while anything is pressed; after 500 ms of silence on the robot's clock the
robot centers it (`centeredByDeadman`), and when the page's event stream closes it lets it go.

### How it works, and why

Like the rest of Pitwall, only the robot loop changes anything. A request waits in a queue, and
Pitwall's `periodic()`, first thing in `robotPeriodic()`, applies it through WPILib's own
simulation (`DriverStationSim`) and announces it at the HAL alone, as a real Driver Station's
packet arrives. WPILib reads it as the next loop begins, so that loop's mode methods
(`utilityInit()`), the robot's code, and AdvantageKit's log, which captures the Driver Station as
it was read, all see it in the same loop, as they do a real Driver Station's. (WPILib's
`DriverStationSim.notifyNewData()` would refresh WPILib's copy partway through a loop: the rest of
that loop would see a change its mode methods hadn't, and the replay, which hands each loop its
logged Driver Station as the loop begins, would see it a loop early.)

Nothing new needs logging: AdvantageKit logs the Driver Station's state and the joysticks on every
robot and replays them, and a fault changes the simulated hardware, whose readings the IO layer
logs. `ReplayEndToEndTest` sets the Driver Station and drives a joystick through these controls,
and the replay matches loop for loop, `utilityInit()` included. A replay has no simulator controls
(its Driver Station and hardware come from its log), so what they record to say why is never
replayed: the faults on (`Pitwall/Simulator/Faults`) and every sentence each loop said
(`Pitwall/Simulator/Said`) are outputs of the simulator's own run alone.

Three things can't wait for the loop, because a paused simulator runs no loops: resuming and
stepping happen at once on the server's thread (`SimHooks`), and so does asking to stop, which
resumes a paused clock so a loop runs to end the program. Pausing goes through the loop, so its
last patch tells every page. A step moves the clock to the next alarm of any notifier, as the GUI's
Step does: between loops that's the robot loop's own, so one loop runs and applies whatever was
asked while paused, but a team's own `Notifier` due sooner would fire instead, and Step would need
pressing again. Stop ends AdvantageKit's log, so its last loops are kept, then the program; it
refuses anywhere but WPILib's simulation.

## What can fail, live

The simulated world declares its parts to Pitwall (`frc.robot.sim.Faults`); Pitwall knows nothing
of any robot's parts. A fault that lasts says whether it's on by asking the world, and undoing it
puts the part back as the world was built: a season-worn robot's sensors (`SensorErrors.TYPICAL`),
not perfect ones. An event happens once, and nothing undoes it. The robot's code is never told:
it finds out from its sensors, as on a robot.

| Part | Fault | Lasts or happens | Takes |
|---|---|---|---|
| Each camera (`Camera front-left`) | Unplugged: no results, and it reads disconnected | lasts | |
| | Frozen: its last image's tags again and again, stamped as new | lasts | |
| | Moved on its mount, from where it was mounted | lasts | forward, left, up (in) |
| | Turned on its mount | lasts | roll, pitch, yaw (°) |
| | Late: images arrive later than its configuration says, still stamped when taken | lasts | extra latency (ms) |
| Gyro, and the backup IMU | Unplugged | lasts | |
| | Frozen: the same yaw every time | lasts | |
| | Errors: its scale, drift, and noise from now on, carrying on from what it reads | lasts | scale, drift (°/min), noise (°) |
| | Reset: its yaw jumps to zero, as a gyro that restarts does | happens | |
| Each module (`Front left module`) | Stale encoder: its encoders' readings stop changing while the wheel turns and steers, then catch up | lasts | |
| | Zeroed off: its absolute encoder's zero | lasts | offset (°) |
| | Worn tread: its wheel's true radius over the configured one | lasts | scale |
| | Slips: spins without moving the robot | happens | turns |
| Robot | Shoved, without its wheels turning; only its gyro feels a turn | happens | forward, left (m), turn (°) |
| | Hit, with a 4 g jolt its IMU feels | happens | forward, left (m) |
| | Tilted by hand, whatever the ground | lasts | pitch, roll (°) |
| | Slipping: only a share of its wheels' rolling moves it | lasts | grip |
| | Placed, as the drive team carries it | happens | x, y (m), heading (°) |
| | Carried to the nearest bump, facing across it | happens | |
| Field | Tag moved: one tag, from where the field was built; moving another puts it back. The robot's code and the cameras' multi-tag solves still believe the layout | lasts | tag (its ID), x, y, up (in), in the field's frame |

Carrying the robot to a bump is declared only on a field with bumps. A world can also be built
with its tags a little off the layout (`SensorErrors.tagErrors`), as the consistency study builds
it; undoing a moved tag puts it back there.

### What the simulator can't make fail yet

The one list of what's not possible yet; the stand-ins and the code point here. Each waits on the
simulator, not on Pitwall. Add each to `Faults`, and a scenario for it, when the simulator can
make it.

- One wheel off the floor: `wheel-lifted` tilts the robot and scales every wheel's grip alike, so
  its wheels never disagree as a lifted one's would, and the robot isn't raised.
- A wheel slipping on its own under acceleration: traction isn't modeled (maple-sim,
  [stand-ins](stand-ins.md)).
- A CAN device dropping off the bus, or browning out: the simulated modules have no bus.
- The battery sagging: the simulated world models no battery.
- A camera's axes swapped in the configuration, the wrong field variant, or one camera with the
  wrong tag layout.
- Ramp crossings: the terrain has bumps only.

## Scenarios

A scenario is a named set of these faults (`frc.robot.sim.Scenarios`). `./gradlew run
-Pscenario=<name>` applies its sensor faults before the robot's first loop and makes what happens
in it happen once the robot is enabled; the API applies all of it at once, to a running
simulation (`POST /api/simulator/scenarios`), and shows its lasting faults as on, each undone on its
own or all together. A fault in a scenario that changes nothing (the part is that way as built) is
passed over. A robot without cameras isn't offered the camera scenarios, nor a field without bumps
the bump one. `-Pscenario` in a program without the simulator's controls stops it, saying so.

## What the *Sim* view needs

The server side is done; the page's *Sim* view is built on it next.

- **The minimal panel first:** enabled or disabled; teleop, autonomous, or utility; Driver Station
  attached; the alliance station; FMS attached; keyboard and gamepad driving of joystick 0 (and 1);
  Stop. Then pause, resume, and step; match time; the faults and scenarios.
- **Keyboard driving is switched on explicitly** ("Drive with keyboard"), shows in the state bar
  while on, and pauses whenever focus is in a text field, so typing `w` into a value never drives
  the robot. One fixed layout for this template's driving: W/S and A/D the left stick (W is -1 on
  axis 1, A is -1 on axis 0), J/L or the arrows the right stick's X (axis 2), and a key for each
  button the template binds (Back, button 4, resets the heading).
- **A gamepad** through the browser's Gamepad API, standard mapping to WPILib's indices: face
  buttons 0-3 to buttons 0-3; bumpers 4-5 to buttons 9-10; triggers 6-7 to axes 4-5; back and start
  8-9 to buttons 4 and 6; stick clicks 10-11 to buttons 7-8; the D-pad 12-15 to buttons 11-14; home
  16 to button 5; axes 0-3 to axes 0-3, same sign. A slot picker (joystick 0 or 1) remembered per
  browser.
- **Send on change, at most once a frame, and every 200 ms while anything is pressed**, naming the
  page's stream (required); send the joystick centered (or `null`) on blur, when the tab is hidden, and when
  driving is switched off. The robot's deadman covers a page that can't.
- **An unfocused page.** Keys reach only a focused page, as they reach only a focused GUI window
  today. Whether a gamepad keeps reaching a page that's unfocused or covered by AdvantageScope is
  unverified: the W3C spec hides gamepads until a button is pressed, and browsers throttle hidden
  tabs. So the page polls the gamepad each animation frame only while `document.visibilityState`
  is visible, centers the joystick when the page is hidden, says so ("Not driving: this tab is
  hidden"), and suggests side-by-side windows. Test it in Chrome and Firefox before calling the
  panel a strict improvement for gamepad users.
- **Paused, the stream goes quiet:** a paused simulator runs no loops, so no patches arrive, only
  heartbeats; the page's own pause answer says it's paused. A page that opens while the simulator
  is paused gets no snapshot until a loop runs: when none arrives within a second or two, it asks
  `GET /api/simulator` (answered at once while paused), and offers Resume and Step.
- **Changes show a loop later:** the robot program sees a Driver Station change as its next loop
  begins, as with a real Driver Station; the page's answer already shows it.
- **Without the GUI the robot starts disabled**; once the *Sim* view is the way to enable it, the
  build can start it enabled again.
- **Stop** ends the program, and the stream with it: show "stopped", not "lost connection".
- The match time arrives in whole seconds; count down between them while enabled, if smoothness
  matters.
