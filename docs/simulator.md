# The simulator

`./gradlew run` runs the robot program on your laptop, in WPILib's simulation, with this template's
simulated world (`frc.robot.sim`): a drivetrain that really moves, as heavy as the robot's
configuration says, cameras that render what they'd see, and sensors that err as a season-worn
robot's do. AdvantageScope shows the field.

Pitwall's page drives the simulator from its *Sim* view (http://localhost:5820/#sim): the Driver
Station, joysticks from the keyboard or a gamepad, pausing and stepping, Stop, and failing the
robot's parts live, which WPILib's simulator GUI can't do. The view is a client of the simulator's
API (http://localhost:5820/api/simulator), below, which anything else that speaks HTTP can use too.

| Job | Command |
|---|---|
| The simulator, driven from *Sim*: in teleop on a start, disabled until you press Enable | `./gradlew run` |
| With WPILib's simulator GUI too, enabled and drivable from its keyboard | `./gradlew run -Pgui` |
| Disabled, with no alliance station, like a real robot | `./gradlew run -PstartDisabled` |
| In a scenario: a fault, or something that happens to the robot | `./gradlew run -Pscenario=collision` |
| List the scenarios | `./gradlew scenarios` |

**WPILib's simulator GUI is off unless you ask for it** with `-Pgui` (and `-Pgui=false` is the
default spelled out). *Sim* does its job. A real Driver Station can connect to the simulator
either way (`halsim_ds_socket` stays loaded). The GUI's default settings in `sim/` are copied
into the project folder only when it runs, and only when missing. Without it you lose its HAL
device views, its NetworkTables browser and plots, its `Field2d` window (AdvantageScope and
Elastic show the same), its notifier table, keyboard remapping, and its joystick reading, which
works whatever window has focus.

**The robot starts ready to drive, but disabled:** in teleop, on the blue alliance's station 2 (the
center start) unless one was chosen, with a Driver Station attached, so it stands on its start and
*Sim*'s Enable is one press from driving. It doesn't enable itself. To drive you come to *Sim*
anyway, to switch the keyboard or a gamepad on, and Enable is there beside them; a robot that
enabled itself would be enabled before anyone was watching, and a scenario's events, which happen
once it's enabled, would be over before the page or AdvantageScope was open. With the GUI it starts
enabled, as it always has: the GUI's keyboard drives it at once, and the GUI is on screen to
disable it. `-PstartDisabled` starts it as a real robot starts, before its Driver Station connects:
disabled, with no station or mode.

**The GUI and the API's joysticks don't mix.** Read in alpha-7's `DriverStationGui.cpp`
(`DriverStationExecute`): while the Driver Station is attached and not in autonomous, the GUI
writes all six joysticks every frame, zeros for a slot it has no device for, and announces new
data every 20 ms. Its *Zero disconnected joysticks* setting doesn't stop that: the check tests the
setting's pointer, not its value, so it always zeros. The GUI's own match time counts down while
enabled too. So with the GUI on, the API's joystick input is overwritten within a frame, and its
match time fights the GUI's. So drive from *Sim* without the GUI, the default, or with `-Pgui` turn
the GUI's Driver Station off (its *Disable DS* setting), which makes it only read.

## Pitwall's simulator controls

Everything the *Sim* view does to the simulator goes through an API Pitwall serves only in
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
  `paused`, and `joysticks`: for `"0"` and `"1"`, `fromPage` (a page drives it),
  `centeredByDeadman` (the page went quiet, and it was centered), and `stream` (the event stream
  of the page whose input it took last, or empty).
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
the page's. Only the page that drives it now can: when two pages drive one joystick, the latest
input wins, and `null` from the other is ignored, so a page that stops driving never lets go of a
joystick another page drives. A joystick the page never drove is never touched, so the GUI's
keyboard or a real Driver Station still can. The page sends a joystick's input when it changes, at most once a frame,
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
| | Spinning: its wheel keeps spinning faster than the floor moves it, as one losing its grip does | lasts | speed (m/s) |
| Robot | Shoved, without its wheels turning; only its gyro feels a turn | happens | forward, left (m), turn (°) |
| | Pushed, its wheels skidding: they never see it, and its IMU feels the push get going and stop (3 m/s²) | lasts | forward, left (m/s) |
| | Hit, with a 4 g jolt its IMU feels | happens | forward, left (m) |
| | Tilted by hand, whatever the ground | lasts | pitch, roll (°) |
| | Slipping: only a share of its wheels' rolling moves it | lasts | grip |
| | Placed, as the drive team carries it | happens | x, y (m), heading (°) |
| | Carried to the nearest bump, facing across it | happens | |
| Field | Tag moved: one tag, from where the field was built; moving another puts it back. The robot's code and the cameras' multi-tag solves still believe the layout | lasts | tag (its ID), x, y, up (in), in the field's frame |
| Each coprocessor (`Coprocessor vision-front`), its simulated health agent | Unreachable: its agent stops answering, as one off or off the network | lasts | |
| | Wrong version: its image carries another PhotonVision (`v2026.3.4`), which it publishes too | lasts | |
| | Hot: its big cores this far below their trip point (85 °C); at it or past it, their clock is capped | lasts | below its trip point (°C) |
| | Camera unplugged: one of its cameras, by its number in the table | lasts | camera (from 1) |
| | PhotonVision restarts: systemd restarts it, back after a few seconds; the count stays until it reboots | happens | |
| | Duplicated address: a second board with its image answers too, by turns | lasts | |
| | Settings changed: its PhotonVision settings differ from those it was stamped with, as tuning in its UI does | lasts | |
| | Ignores power-down: its image predates powering down from the robot, so its agent refuses (404) and it keeps running | lasts | |

Carrying the robot to a bump is declared only on a field with bumps, and a coprocessor for each
computer in the table compiled into the program (`coprocessors/coprocessors.yaml`); a computer
with no cameras in the table has no camera to unplug. A simulated coprocessor answers as a healthy
board running this build's image would, once a second on the robot's clock, through the same
records a real agent's answer is read into, so its card and alerts on the robot are the real ones
([the coprocessors](coprocessors.md)) Asked to power down (Pit's *Power down coprocessors*), it halts three seconds
later and stays halted, unreachable on both its ports, until the simulator starts again, as a real
board stays off until its power is cycled. A world can also be built
with its tags a little off the layout (`SensorErrors.tagErrors`), as the consistency study builds
it; undoing a moved tag puts it back there.

### What the simulator can't make fail yet

The one list of what's not possible yet; the stand-ins and the code point here. Each waits on the
simulator, not on Pitwall. Add each to `Faults`, and a scenario for it, when the simulator can
make it.

- One wheel off the floor: `wheel-lifted` tilts the robot and scales every wheel's grip alike, so
  its wheels never disagree as a lifted one's would, and the robot isn't raised.
- One wheel slipping on its own under acceleration: the robot keeps to its tread's grip as a whole,
  and every wheel carries the same share, so they slip all alike or not at all (maple-sim,
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

## The *Sim* view

The page's *Sim* view, offered only in the simulator, does what the GUI did and what it couldn't,
through the API above (`page/sim.js`, with `sim-drive.js` for driving and `sim-faults.js` for what
fails).

**The Driver Station** is at the top, and stays there as the view scrolls: whether the robot is
enabled (filled in) or disabled (an outline), its mode, station, and match time, with Enable and a
large Disable. Enable needs a mode, as a real Driver Station always has one. Below are the mode,
the alliance station, Driver Station attached, FMS attached, and the match time (set, cleared, and
counting down while enabled, in whole seconds as a Driver Station shows it). The state bar's mode
is filled in while the simulated robot is enabled, and leads here. Attaching the field asks first
when values are tuned, listing them, since the field returns them to committed; the field's band
then points back to the switch, its way out in the simulator.

**Esc disables the robot** in *Sim* always, even with the focus in a field; and from any other view
while this page drives it, with the keyboard or a gamepad, unless the focus is in a field, where
Esc drops what was typed. Elsewhere Esc keeps its meaning: it stops a run or an action. Disabling
there would also end the enable an experiment's next run needs, turning "stop this run" into "stop
it, and enable again". When a disable fails, the page says so at once, wherever Esc was pressed.

**Keyboard driving is switched on explicitly** ("Drive with the keyboard"), and shows in the state
bar while on: "⌨ Driving", or why not now. One fixed layout for this template's driving, by
physical key, so it's where it is on any keyboard layout:

| Keys | Joystick 0 |
|---|---|
| W, S | the left stick's Y, axis 1: W is -1 |
| A, D | the left stick's X, axis 0: A is -1 |
| J, L, or ← → | the right stick's X, axis 2: turning |
| R | Back, button 4, which resets the heading |

The keys pause, and the joystick is centered, while the focus is in a field (typing `w` into a
value never drives), when the window loses the keyboard, and when the tab is hidden; a key held
then counts as released, since its release can't be heard. So do Cmd, the Windows key, and Alt,
pressed or let go, and a context menu: a key let go while one is down may never say so (on macOS,
never while Cmd is down), and a key held that the page thinks is held forever would keep the
robot's deadman from ever stopping it. Hold it again to drive on. Switching driving off lets the
joystick go.

**Gamepads** drive through the browser's Gamepad API, switched on in *Sim* too, with the standard
mapping to WPILib's `Gamepad`:

| Standard gamepad | WPILib |
|---|---|
| axes 0-3 | axes 0-3, the same sign |
| face buttons 0-3 | buttons 0-3 |
| bumpers 4-5 | buttons 9-10 |
| triggers 6-7 | axes 4-5, from 0 to 1 |
| back 8, start 9 | buttons 4 and 6 |
| stick clicks 10-11 | buttons 7-8 |
| the D-pad 12-15 | buttons 11-14 |
| home 16 | button 5 |

The first gamepad drives the joystick chosen in *Sim* (0 or 1, remembered in this browser), and a
second one the other. A gamepad this browser doesn't know the standard layout of can't drive, and
*Sim* says so. Axes go in a Driver Station's steps of 1/127.

**What's sent:** each joystick's whole input when it changes, at most once a frame, and again every
200 ms while anything on it is pressed, each joystick by its own clock, naming the page's stream.
The keyboard's joystick is centered on blur (a gamepad keeps driving a visible page); every
joystick the page drives is centered when the tab is hidden; and `null` lets one go when driving
is switched off. Input the robot didn't answer (it timed out, or the robot was busy) is sent again
shortly, a let-go included, and *Sim* says so meanwhile. *Sim* shows each joystick the page drives,
its axes and the buttons held, and says when the robot's deadman centered it, or when another page
drives it too.

**An unfocused or hidden page.** Keys reach only a focused page, as they reached only a focused GUI
window. The page polls gamepads each frame only while the tab is visible (the W3C spec also hides a
gamepad until a button is pressed, and browsers throttle hidden tabs), centers its joysticks when
the tab is hidden, and says so in the tab's title ("⏸ Not driving"), then in *Sim* on return,
suggesting side-by-side windows. Whether a gamepad keeps reaching a page that's visible but
unfocused, beside AdvantageScope, is unverified: test it in Chrome and Firefox before calling the
view a strict improvement for gamepad users.

**Time:** Pause and Resume; Step, one loop, only while paused; and Stop, after asking (it lists
the tuned values not committed, which end with the program). A paused simulator runs no loops, so
the stream goes quiet but for heartbeats: a request made while paused says it waits for a Step or
Resume (one the loop was slow to answer, that it waits for the next loop), and that note goes once
a loop has run. A page that opens while the simulator is paused gets no snapshot; after a second
and a half it asks `GET /api/simulator`, answered at once while paused, and if the simulator says
it's paused, offers *Sim*, says why the rest is empty, and offers Resume and Step. The rest fills
in when a loop runs.

**Stop** ends the program, and the stream with it: the page that stopped it says "Stopped", in the
state bar, its band, and *Sim*, not that the link was lost, and reconnects by itself when the
simulator starts again. Another page open on the simulator sees only the link lost.

**What fails:** a card for each part (each camera, the gyro, the backup IMU, each module, the
robot, and each coprocessor), each fault with what it does and what undoing it does, as `frc.robot.sim.Faults` declares
them; each parameter with its unit and limits, checked on the page before it's sent; Apply, and
Undo for a fault that's on. A lasting fault that's on is marked as a tuned value is, an amber bar,
●, and "on", with the values it was applied with; an event is a button that makes it happen once.
What's on is listed above the cards, with Undo all. The scenarios are listed there too, each with
what it does and its faults, those it put on marked as on while they are; Apply says which faults
it put on, which happened, and which it passed over, the part being that way as built. A refusal
(400, 409, or 422) is shown where it was asked, in the simulator's words, and read out when the
reader is free.

**Changes show a loop later:** the robot program sees a Driver Station change as its next loop
begins, as with a real Driver Station; *Sim* shows the simulator's answer.
