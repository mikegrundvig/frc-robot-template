# Dashboard layouts

`elastic-layout.json` is the drive team's dashboard for [Elastic](https://github.com/Gold872/elastic_dashboard):
a **Pre-Match** tab (on its start, placement, station, battery, localization, cameras, loop, alerts)
and a **Match** tab (match time, ready to score and why not, localization, alerts).

It lives next to the code because it reads what the code publishes. Every widget reads a value
under `/Telemetry/Dashboard/`, published every loop by `dashboard/Dashboard.java`.

**To change it:** open it in Elastic (File > Open Layout), connect to the simulator (`./gradlew run`)
or the robot, arrange it, and save it back over this file (File > Save Layout). Commit it with the
code it goes with.

**`DashboardLayoutTest` checks it on every build.** It runs the robot in the simulator and fails if
a widget reads a value nothing publishes, or a value of the wrong kind: a true-or-false box on text,
a match timer on anything but a number. The failure names the tab and the widget. To fix one, point
the widget at a value `dashboard/Dashboard.java` publishes, or publish the value there.

## AdvantageScope

`advantagescope.json` is AdvantageScope's layout for the simulator and its logs: the robot where
it believes it is, beside where it really is, in 3D and 2D, with the cameras' tags and the
estimate's uncertainty; the swerve modules; and graphs of the estimate's error and the slip
check. [runtime/docs/simulator.md](../runtime/docs/simulator.md#see-it-in-advantagescope) says how to open it and
what each tab shows.

It's AdvantageScope v27.0.0-alpha-6's format (File > Export Layout...), the release that pairs
with the AdvantageKit in `vendordeps/`. Importing it into another version, AdvantageScope warns
that compatibility isn't guaranteed.

**To change it:** import it (File > Import Layout...), arrange it, export it over this file
(File > Export Layout...), and commit it with the code it goes with.

**Two tests check it on every build.** `AdvantageScopeLayoutTest` checks that each source is one
its tab can show and that each robot model it names is one the build generates
(`ascope_assets/`). `ReplayEndToEndTest` runs the robot program in the simulator and fails if a
source reads a value the robot never logs, or logs as another type, naming the tab and the source.
To fix one, point the source at a value the robot logs, or log the value it needs.
