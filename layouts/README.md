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
