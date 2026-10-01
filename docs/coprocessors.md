# The coprocessors, from the robot

Each vision coprocessor runs a small health agent (`coprocessor/README.md` says what it answers,
and how the table of coprocessors, `coprocessors/coprocessors.yaml`, describes them). This page is
the robot's side: how the robot program asks each agent, what it alerts on, the coprocessors'
cards on Pitwall's page, and the check `deploy` makes first. The code is
`frc.robot.subsystems.coprocessors`.

## Asking without waiting

Each coprocessor is asked for its health (`/v1/health`, which carries its stamp) once a second, on a
thread of its own, within 250 ms to connect and 500 ms to answer (`Coprocessors.Settings`, in
[the values register](values.md#coprocessorssettings)). The robot loop never waits on the network:
it reads only each coprocessor's latest answer and how old it is. An answer is placed on the
robot's clock halfway between asking and hearing.

What each answer says is logged under `Coprocessors/<name>` as AdvantageKit inputs (the stamp, load
per core, the big cores' clock against their most, every thermal zone with its trip point,
PhotonVision's service and restarts, the cameras plugged in, the journal's errors, the drive), so
the coprocessors' health sits in the match log and replays with it. A replay judges each
coprocessor from its log exactly as the robot did. `Coprocessors/<name>/BootedAtRobotNanos` is the
robot's time when the coprocessor booted: an entry in its journal, which counts from its boot, is
on the robot's clock at that plus the entry's monotonic time. That places a coprocessor's journal
on the match's timeline, though the boards keep no reliable wall time.

## Alerts

Each coprocessor is judged against what this build expects of it: the table compiled into the
program, with this build's PhotonVision version, image recipe, and each computer's committed
settings.

| Alert | Level | When |
|---|---|---|
| Missing | high while enabled, medium while disabled | No answer for 3 s, or none since the robot program started, with why (no connection, no answer in time) |
| The wrong image | high | Its stamp names another PhotonVision, recipe, name, team, or address than this build's: both are named |
| Two boards answer | high | MACs take turns answering at its address: a spare drive with its image in another board, say. A board replaced by another isn't two boards, since the first never answers again |
| A camera missing | medium | A camera the table gives it isn't plugged in where its settings expect it |
| Hot | medium | Its hottest zone within 5 °C of its first passive trip point, or its big cores held below their most |
| PhotonVision restarted | medium | systemd restarted PhotonVision since the coprocessor booted |
| Errors in its journal | medium | USB, UVC, filesystem, memory, heat, or other errors since boot, with the latest |
| Settings differ from Git | medium | Its PhotonVision settings now differ from those committed for it: normal while calibrating, so not red |
| PhotonVision's version | high | The version PhotonVision publishes (`/photonvision/version`) isn't this program's PhotonLib (`PhotonVersion.versionString`), as exact strings |

The last catches what PhotonLib lets through: its own check compares only the message format, so
two versions that share one pass it. Every coprocessor publishes its version to that one topic, so
it holds whichever started last; the alert names the coprocessors whose stamp carries the version
heard. A program built without its table (anything but Gradle) watches no coprocessors, and says
so in a high alert.

## Pitwall's cards

Pit has a card for each coprocessor: present, missing, or two boards answering; its PhotonVision
version and stamp against this build's, and its settings against Git's; the hottest zone against
its trip point; the big cores' load, and whether they're throttled; the cameras present against the
table; and since boot, PhotonVision's restarts, whether the last shutdown was clean, and the
journal's errors. Each card ends in what can be done:

- **Open PhotonVision**: its own page, by address (`http://10.TE.AM.x:5800`).
- **Download settings backup**: the agent's `/v1/settings.zip`, laid out like the repository's
  folder for the computer: unzip it into the repository, review the diff, and commit it.
- **Download logs**: its journal, a page at a time, filtered by priority and unit, each entry on
  the robot's clock when it's from the coprocessor's current boot; or saved as text.

Pitwall asks the agent for these on its own server threads, when the page asks, never in the robot
loop; the browser never talks to a coprocessor.

## Powering down before switching off

A power cut can lose what a coprocessor was writing (its root is read-only; its settings aren't),
so Pit has an action, **Power down coprocessors**, to run before someone flips the breaker. It runs
only while the robot is disabled and the field system isn't connected, never in a match; enabling
the robot stops it waiting.

1. It asks every computer in the table to power down (`POST /v1/shutdown`, which the agent takes
   only from the robot's own address): PhotonVision stops, then the board halts.
2. It waits until each is gone on both its agent's port and PhotonVision's (5800), each tried once
   a second on its IO's own thread.
3. Then Pit and the state bar say **Safe to switch off**.

A computer still answering 30 s after it was asked raises a high alert naming it, with what its
agent answered (an image without the request answers 404): don't switch off yet. A halted board
stays off while it has power, so a robot left on after powering down says, after three minutes,
*Coprocessors halted: cycle the robot's power to restart them*, and the missing-coprocessor alerts
say each was powered down from Pitwall, not that it failed.

A button can start the same sequence: wire a push button between one of Systemcore's digital
inputs and ground, and set `powerDownButton` in `Coprocessors.Settings` to that input (it's -1, no
button, by default). Holding it a second asks Pitwall to run the action, under the same rule:
disabled, and off the field.

## The check before a deploy

`./gradlew deploy` first runs `coprocessorCheck`, which asks each coprocessor in the table for its
stamp, within a second:

- a coprocessor whose stamp names another PhotonVision, recipe, name, team, or address than this
  build's **fails** the deploy, naming both;
- so does one that answers on PhotonVision's port (5800) but not the agent's: it isn't running this
  repository's image;
- one that doesn't answer at all only **warns**, so the robot can be deployed with its vision off.

`./gradlew coprocessorCheck` runs it alone. `-PskipCoprocessorCheck` skips it, and says so: for a
deploy that knowingly leaves a coprocessor as it is.

## In the simulator

Each computer in the table gets a simulated agent, which answers as a healthy board running this
build's image would, once a second on the robot's clock, through the same records a real answer is
read into. Its faults (unreachable, the wrong version, hot, a camera unplugged, PhotonVision
restarting, a duplicated address, settings changed, ignoring power-down) are in Pitwall's *Sim*
view, as [the simulator](simulator.md#what-can-fail-live) lists them, and show on its card and in
its alerts as a real coprocessor's would. Powered down, it halts three seconds later, and stays
halted until the simulator starts again. *Open PhotonVision* has no page behind it in the simulator.

## Unverified

- The vendor kernel's thermal trip points: the agent reads each board's at run time.
- Whether Systemcore's SmartIO inputs are pulled up, as a roboRIO's are, for the button.
- How quickly two boards at one address take turns answering: it depends on how often the robot
  renews its address table, so the window (a minute) may need tuning against a real pair.
