# The simulator

`./gradlew run` runs the robot program on your laptop, in WPILib's simulation, with this template's
simulated world (`frc.robot.sim`): a drivetrain that really moves, cameras that render what they'd
see, and sensors that err as a season-worn robot's do. Pitwall's page drives it
(http://localhost:5820): the Driver Station, the keyboard or a gamepad, and pausing and stepping,
in place of WPILib's simulator GUI. AdvantageScope shows the field.

## Pitwall's simulator controls

Everything the page does to the simulator goes through an API it serves only in the simulator. On
a robot none of it exists: every route answers 404 and the event stream has none of its sections,
so nothing can write the Driver Station's state there (`PitwallRobotTest` checks this with WPILib
told it's on a robot). The controls are exempt from the field lock: in the simulator they *are* the
field, so attaching the field system locks tuning and actions exactly as a real field does, and the
panel stays usable to detach it again.

### In the event stream

A section beside the page's others, sent as JSON merge patches when it changes:

- **`simulator`**: `enabled`, `mode` (`AUTONOMOUS`, `TELEOPERATED`, `UTILITY`, or `UNKNOWN` before
  a Driver Station set one), `allianceStation` (`RED_1` to `BLUE_3`, or `UNKNOWN`), `fmsAttached`,
  `dsAttached`, `matchTime` (whole seconds left, counting down while enabled; -1 for none),
  `paused`, and `joysticks`: for `"0"` and `"1"`, `fromPage` (the page drives it) and
  `centeredByDeadman` (the page went quiet, and it was centered).
### Requests

Every request names the page's event stream in the `Pitwall-Stream` header, as the page's other
requests do. Errors are problem details, as elsewhere: 400 for a request that isn't well formed,
422 for one that names what isn't there or a value out of range (with the reason), 409 for one
that can't be done now.

| Request | Body | Answer |
|---|---|---|
| `GET /api/simulator` | | `{"simulator"}`, read by the next loop; while paused, as the last loop left it |
| `PATCH /api/simulator` | any of `enabled`, `mode`, `allianceStation`, `fmsAttached`, `dsAttached`, `matchTime` (0 to 600 s, or `null` for none), `paused` | the `simulator` section, once the loop has applied it all; 202 if it didn't within a second |
| `POST /api/simulator/step` | | one loop of a paused simulator, then the `simulator` section; 409 while it runs |
| `POST /api/simulator/stop` | | 202, then the program ends: the stream closes |
| `PUT /api/simulator/joysticks` | `{"0": {"axes": [...], "buttons": [...], "povs": [...]}, "1": null}` | 204 at once |

A `PATCH` applies its changes in one loop, in the order given. Enabling also attaches the Driver
Station, as a real one would be, and detaching it disables the robot.

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
simulation (`DriverStationSim`) and announces the new data, so the rest of the loop sees the same
Driver Station, and AdvantageKit, which captures the Driver Station at the end of the loop, logs
what the loop saw. Nothing new needs logging: AdvantageKit logs the Driver Station's state and the
joysticks on every robot and replays them. `ReplayEndToEndTest` sets the Driver Station and drives a joystick through these controls,
and the replay matches.

Three things can't wait for the loop, because a paused simulator runs no loops: resuming and
stepping happen at once on the server's thread (`SimHooks`), and so does asking to stop, which
resumes a paused clock so a loop runs to end the program. Pausing goes through the loop, so its
last patch tells every page. A step moves the clock to the next alarm any notifier waits for,
which between loops is the robot loop's own, as the GUI's Step does; it applies whatever was asked
while paused. Stop ends AdvantageKit's log, so its last loops are kept, then the program.

## What the *Sim* view needs

The server side is done; the page builds on it.

- **The minimal panel first:** enabled or disabled; teleop, autonomous, or utility; Driver Station
  attached; the alliance station; FMS attached; keyboard and gamepad driving of joystick 0 (and 1);
  Stop. Then pause, resume, and step; match time.
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
  page's stream; send the joystick centered (or `null`) on blur, when the tab is hidden, and when
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
  `GET /api/simulator`, and offers Resume and Step.
- **Stop** ends the program, and the stream with it: show "stopped", not "lost connection".
- The match time arrives in whole seconds; count down between them while enabled, if smoothness
  matters.
