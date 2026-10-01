# Vision coprocessors

Each vision coprocessor (an Orange Pi 5-family board running PhotonVision) is built from this
repository: one table says which computers the robot has, and each one runs a small health agent
the robot polls. This folder holds what they share with the robot program.

| Path | What |
|---|---|
| `coprocessors/coprocessors.yaml` (at the repository's root) | The table: the team's coprocessors, the one place they're described |
| `coprocessors/<computer>/settings/` | Each computer's PhotonVision settings and calibrations, one file per row |
| `coprocessor/photonvision.lock` | The PhotonVision version the coprocessors run, with the checksums an image is built from |
| `coprocessor/common/` (Gradle `:coprocessor-common`) | What the robot program, the agent, and the build share: the table, the agent's API records, the settings' canonical form and hash, and the JSON they're written in |
| `coprocessor/agent/` (Gradle `:coprocessor-agent`) | The health agent each coprocessor runs, and its systemd unit |

## The table: `coprocessors/coprocessors.yaml`

```yaml
team: 0                # the team number; addresses are 10.TE.AM.x
agentPort: 5808        # the health agent's port, unless a computer says otherwise
computers:
  - name: vision-front # its hostname: lowercase letters, digits, hyphens
    address: 11        # the last number of its address: 10.TE.AM.11 (.6 to .19)
    board: orangepi-5  # orangepi-5 | orangepi-5b | orangepi-5-plus | orangepi-5-pro | orangepi-5-max
    cameras: [front-left, front-right]   # the PhotonVision camera names it runs (its role)
```

| Key | Required | What it may be |
|---|---|---|
| `team` | yes | 0 to 25599. A computer's address is 10.TE.AM.x: team 2611's `.11` is 10.26.11.11 |
| `agentPort` | no, 5808 | 1024 to 65535, except 5800 (PhotonVision's page) and 5810 (NetworkTables). 5808 sits in FRC's team range, 5800-5810 |
| `computers` | yes (`[]` for none) | A list of computers |
| `computers[].name` | yes | A hostname: lowercase letters, digits, and hyphens, starting with a letter and not ending with a hyphen, at most 63 characters. Unique |
| `computers[].address` | yes | 6 to 19: FRC's static range for devices on the robot (WPILib's "IP Configurations"; .20-.199 is the field's DHCP pool, .200-.219 the radio's). 11 and up by convention. Unique |
| `computers[].board` | yes | One of the five boards above |
| `computers[].cameras` | no, none | PhotonVision camera names: the names robot code gives `PhotonCamera`. Not empty, no `/` or control characters, at most 64 characters. Unique across the whole table, since PhotonVision publishes each one at `/photonvision/<camera>` |
| `computers[].agentPort` | no, the table's | As `agentPort` |

The build reads the table and refuses one with a problem, listing every problem with its line. A
key it doesn't know is a problem too, so a misspelling can't pass silently.

The template's table is an example: team 0, one computer, no settings. The robot's build accepts
team 0 and an empty list, but the image workflow builds nothing until a team sets its number and
lists at least one computer.

**The YAML it reads.** The table is read without a YAML library, so nothing extra reaches the robot
program or the agent. It reads the YAML above and anything shaped like it: one `key: value` per
line, nested by indenting with spaces; lists as `- item` lines (at the key's indent or deeper) or
on one line in `[...]`; plain, `'single'`, and `"double"` quoted values; and `#` comments.
Everything else YAML allows (tabs, `{...}` mappings, anchors, tags, multi-line values, more than
one document) is refused with its line number, rather than read in a way you didn't mean. Values
are never guessed at: a camera named `no` is the text "no", not false.

In Java: `Table.readYaml(path)` (or `Table.parseYaml(text, name)`) gives a checked `Table`, or a
`TableException` whose `problems()` are the messages. A `Table` is checked however it's made, from
JSON too.

## The agent's API, version 1

Each coprocessor's agent answers on its `agentPort`: HTTP, JSON, no authentication. It runs on a
closed robot network, runs nothing a request says, and bounds every answer. It's read-only but for
one action, a soft power-off, which only the robot controller may ask for.

| Request | Answer |
|---|---|
| `GET /v1/stamp` | `Stamp`: which image this is, and as which computer |
| `GET /v1/health` | `Health`: the computer's health, about 2 KB |
| `GET /v1/journal?cursor=&priority=&unit=&limit=` | `JournalPage`: a page of the journal |
| `GET /v1/settings` | PhotonVision's settings as canonical rows, with their hash |
| `GET /v1/settings.zip` | The same as files, laid out like the repository's folder for the computer |
| `POST /v1/shutdown` | `ShutdownAnswer` (202): stops PhotonVision, then powers the computer off. From the robot controller (10.TE.AM.2) only |

A refusal or a failure answers `{"error": "..."}` with its status: 400 for a bad query, 403 for a
shutdown from anyone but the robot controller, 404 for an unknown path (or no settings database
yet), 405 for the wrong method (with `Allow`), and 500 when the agent failed.

The answers are Java records in `com.michaelgrundvig.frc.coprocessor.api`; the paths and limits are
`AgentApi`'s constants. Each record has `toJson()` and `fromJson(...)`, and the top-level ones
`parse(String)`: the robot program reads an answer with `Health.parse(body)`. Every record's
Javadoc says what each value means and in what unit.

**Reading is tolerant one way.** A value that's missing (or `null`) reads as its default (empty,
zero, none), and a value the reader doesn't know is ignored, so a robot and an agent built a
version apart still understand each other. A value of the wrong kind (text where a number
belongs) is an error.

### `Stamp`

```json
{"name": "vision-front", "team": 2611, "address": "10.26.11.11", "board": "orangepi-5",
 "cameras": ["front-left", "front-right"], "release": "coprocessors-2027.1",
 "recipeHash": "4f2c…", "photonvisionVersion": "v2027.1.0", "settingsHash": "9a8b…",
 "bootId": "3c1e6a2e-…", "mac": "c0:74:2b:fe:12:34"}
```

`cameras` is the computer's role. `settingsHash` is the hash of the settings stamped into the
image, empty when it was built with none.

**The stamp file.** The image's stamping writes `/etc/coprocessor/stamp.json` as this JSON without
`bootId` and `mac`, which the agent adds as it answers (from
`/proc/sys/kernel/random/boot_id`, and the network interface), plus `agentPort`, the port it
serves on. Other members are ignored.

### `Health`

| Member | Record | What |
|---|---|---|
| `stamp` | `Stamp` | As `/v1/stamp` |
| `boot` | `Boot` | Boot ID, uptime, the agent's monotonic clock (microseconds since boot, the journal's clock), whether the boot before shut down cleanly (`null`: unknown), the root device and whether it's read-only, the SPI bootloader's version |
| `cpu` | `Cpu`, `CpuCluster` | Each core's busy percentage over `windowSeconds`; each cluster's cores, current, limit, and maximum MHz. `bigClusters()` and `bigCoresCapped()` answer "are PhotonVision's cores held back" |
| `thermal` | `ThermalZone`, `TripPoint` | Every thermal zone's temperature, with its trip points as the running kernel has them. `firstPassive()` is where throttling starts |
| `photonvision` | `Service` | PhotonVision's service as systemd sees it: active state, sub-state, result, restarts this boot, and when it last started |
| `cameras` | `Cameras`, `ExpectedCamera`, `UsbCamera` | The cameras in the role, each with the USB path PhotonVision's settings match it by and whether something is plugged in there; and every USB camera plugged in, with its port and link speed |
| `journal` | `JournalSummary`, `JournalEntry` | This boot's count of USB, UVC, filesystem, out-of-memory, thermal, and other errors, with the latest three (messages cut to 200 characters) |
| `drive` | `Drive` | The NVMe drive's temperature, wear, unsafe shutdowns (each power cut counts one), power cycles, hours, media errors, and critical warnings; `null` without an NVMe drive |
| `settings` | `SettingsState` | The stamped settings hash against the live one; `matches()` |
| `problems` | text | What the agent couldn't read, each saying what and why (at most 10); empty when it read everything. `shutting down: asked for by the robot` comes first once a shutdown is under way |

A health answer is 3 to 4 KB for an RK3588 with two cameras: more than the design's guess of 2 KB,
mostly the trip points and the journal's latest entries. Health asked for again within half a
second is the same answer, so two pollers cost no more than one.

### `JournalPage`

`entries` (oldest first), `cursor` (ask again with it for the entries after these), and `more`
(whether more were waiting). Without a `cursor`, the page is the latest entries; `priority` (0-7,
or `emerg` to `debug`) gives that priority and worse, `unit` one systemd unit, and `limit` the page
size (100 unless asked, at most 500). A message longer than 2048 characters is cut. Entries of
every boot the journal keeps are there; each says its boot.

### `POST /v1/shutdown`

A soft power-off, for a coprocessor kept powered after the robot is switched off (by a battery
pack, say), so it stops cleanly rather than losing power mid-write. Only the robot controller,
10.TE.AM.2 by the stamp's team, may ask; anyone else gets 403. The agent logs the request to the
journal, answers 202 at once with `{"shuttingDown": true, "alreadyRequested": false}`, then runs
`systemctl stop photonvision.service` (so PhotonVision saves its settings; up to 2 minutes) and
`systemctl poweroff`, which it runs even if the stop failed. Asking again while it's under way
answers 202 with `alreadyRequested: true` and does nothing more. The agent answers until the
computer goes down; health says `shutting down` meanwhile.

## Settings in Git

PhotonVision keeps all its settings, calibrations included, in one SQLite database,
`photon.sqlite`: a `global` table (network, hardware, field layout, ...) and a `cameras` table
(each camera's configuration, pipelines, and calibrations), with JSON text in their columns. In
Git, they're one file per row, under `coprocessors/<computer>/settings/`:

```
database.json                  {"userVersion": 2}: the database's schema version
global/networkConfig.json      {"contents": {...}}
cameras/<unique name>.json     {"config_json": {...}, "drivermode_json": null, ...}
```

Each file is the row's columns, each column's JSON as JSON (so a diff shows a calibration's
changed member, not a changed line of escaped text), sorted and indented by `Json.pretty`. A key
becomes a file name with everything but letters, digits, `.`, `_`, and `-` written as `%XX`, as
are a leading dot and the names Windows reserves, so every key makes a file on every system; two
keys that differ only in case would be one file on Windows, and are refused. A column whose text
isn't JSON is kept as text, under its name plus `.text`.

Sorted members are safe for PhotonVision to read back: its JSON library (avaje-jsonb) generates
readers that take members in any order, the type member of a polymorphic value included (checked
in avaje-jsonb's generator source). It matters, because PhotonVision's own order isn't stable: some
of its maps are keyed by enums, whose order changes from run to run.

In Java (`com.michaelgrundvig.frc.coprocessor.settings`): `Settings` and `SettingsRow` are the
rows; `SettingsFiles` reads and writes the folder (`read`, `write`, and `render`/`parse` for the
files as text); `SettingsDatabase` reads and writes the database through JDBC's own API, so this
project needs no driver (the agent brings xerial's sqlite-jdbc, and the tests use it too). It reads
every table, with its primary key as the key and every other column as JSON, so a column a new
PhotonVision adds comes along unasked. Writing replaces every row in one transaction, and refuses a
database at another schema version, or without a table or column the settings name.
`SettingsText` is the rows as the database's text, parsed a row at a time: a calibration is about a
megabyte of JSON, many times that parsed, and the agent has 64 MB.

**The tests' database** is made from PhotonVision's schema (its `DatabaseSchema` migrations,
transcribed in `src/testFixtures/resources/photonvision/`) with example rows written from its
configuration classes' fields; no PhotonVision jar is downloaded. A round trip against a database
the pinned version's own jar made (its `--smoketest` makes an empty one) is a follow-up for CI, once
the jar is archived.

### The settings hash

`SettingsHash.of(settings)` (or `settings.hash()`) is the SHA-256, in lowercase hex, of
`Json.hashable` of `{"tables": {<table>: {<key>: <columns>}}, "userVersion": <n>}`, after the rules
below. So rewriting doesn't change it (members reordered, `70.0` written `70`), files and the
database give the same hash, and a changed setting changes it. The robot's build hashes each
computer's committed settings into the program; each agent hashes the live database; empty means
none are committed.

Some values PhotonVision changes by itself, as it runs. The hash leaves them out, or reads them in
a fixed order (`SettingsHash.RULES`, each with its reason; `SettingsHashTest` pins the list):

| Table, column | Value | Rule | Why |
|---|---|---|---|
| `cameras`, `config_json` | `currentPipelineIndex` | left out | The robot program switches pipelines and driver mode, and PhotonVision saves the choice |
| `cameras`, `config_json` | `streamIndex` | left out | PhotonVision assigns each camera's stream ports as it starts |
| `cameras`, `config_json` | `matchedCameraInfo.dev` | left out | The `/dev/videoN` number the kernel gave the camera this boot |
| `cameras`, `config_json` | `matchedCameraInfo.path` | left out | The `/dev/videoN` device; PhotonVision matches cameras by the by-path entry in `otherPaths` |
| `cameras`, `config_json` | `matchedCameraInfo.otherPaths` | any order | The camera's other paths, in an order PhotonVision notes can change; which ones (the USB port) still counts |

**Unverified:** the list comes from reading PhotonVision's source. A bench test on a real board
confirms it: the hash holding steady across reboots, replugs, and pipeline switches. Changing the
list changes every hash, so every image needs stamping again.

## The version lock: `coprocessor/photonvision.lock`

PhotonLib's vendordep (`vendordeps/photonlib.json`) says which PhotonVision the robot program is
built for; the lock says the same version, with what pins an image's inputs:

```json
{
  "version": "dev-v2027.0.0-alpha-2-69-g71416112",
  "jar": {"url": "...", "sha256": "..."},
  "images": {"orangepi-5": {"url": ".../photonvision_opi5.img.xz", "sha256": "..."}, ...}
}
```

`version` must equal the vendordep's. `jar` is the PhotonVision jar for 64-bit ARM Linux, and
`images` each board's base image: PhotonVision's own image for that board, from the
[photon-image-modifier](https://github.com/PhotonVision/photon-image-modifier) release the pinned
PhotonVision version builds on, whose file name it keeps. A value not known yet is a placeholder,
text starting `PLACEHOLDER`: the robot's build allows it and names it, and the image build refuses
it. The template's lock has the base images of release v2027.2.3, with the SHA-256 digests the
release publishes, and placeholders for the jar: PhotonVision's rolling development release no
longer holds this version's, so it has to be archived (built from its commit, if need be) before
images can be built. The lock is JSON, which YAML tools read too (the image scripts use yq).

**Every build checks it:** `./gradlew checkPhotonVisionLock` (part of `check`, so of `build`,
`ci`, and `deploy`) fails when the lock's version isn't the vendordep's, saying both. So when
PhotonLib is updated, the lock moves with it, in the same commit, with the new jar's and images'
addresses and checksums (or placeholders until they're known).

## The compiled table

Every build compiles the table into the robot program as `/coprocessor/table.json` (the
`coprocessorTable` task, from `gradle/coprocessors.gradle`; a generated resource, not source), with
what the robot checks each coprocessor against: PhotonLib's PhotonVision version, the recipe hash,
and each computer's committed settings hash (empty for a computer with none). A table with a
problem, or settings that can't be read, fail the build there.

`CompiledTable.load()` reads it, and `mismatches(computer, stamp)` says, as sentences naming both
values, how a coprocessor's stamp differs from this build in name, team, address, PhotonVision
version, or recipe hash.

**The recipe hash** says how the common image is built. `coprocessor/image/recipe.paths` lists what
it covers (the image's scripts, the agent's and the shared code, the lock), and the hash is the
SHA-256 of what `git ls-tree -r --full-tree HEAD -- <those paths>` prints: committed files only, so
it's the same on every machine and OS, and the same the image workflow computes
(`coprocessor/image/recipe-hash.sh`). Uncommitted changes don't count: an image is built from a
commit. Without Git (a downloaded copy, or no commit yet) the compiled table has no recipe hash,
and the build says so. `CoprocessorBuild` runs these tasks, so the build checks with exactly the
code the robot and the agents run.

## Tests

`./gradlew :coprocessor-common:test :coprocessor-agent:test` runs them; `./gradlew ci` runs them
with every other check. They need nothing of the computer they run on: the agent's read a fixture
tree, and PhotonVision's database is made from its schema (`src/testFixtures` in
`coprocessor/common`, which the agent's tests use too).

## JSON, without a library

The robot program, the agent, and the build all read and write JSON through one small class,
`com.michaelgrundvig.frc.coprocessor.json.Json`, with nothing but the JDK. Pitwall uses avaje-jsonb,
which WPILib brings the robot program, but the coprocessors don't run WPILib, and settings hashes
must be computed identically by the robot's build and by each agent for years: a library's output
can change under a version bump, and a hash with it. So:

- A parsed number keeps the text it was written as, so a value read and written again is unchanged
  digit for digit.
- `Json.compact` writes members in their order with no spaces: what the agent sends.
- `Json.pretty` writes members sorted by name, two-space indents, and a newline at the end, with a
  container of plain values short enough (80 characters) on one line: the form files take in Git.
- `Json.hashable` writes sorted, with no spaces and every number in one form (`1`, `1.0`, and
  `1e0` alike), for hashes.

Names sort by UTF-16 code units, as RFC 8785 does; strings escape only what JSON requires.

## The agent

`coprocessor/agent/` (Gradle `:coprocessor-agent`) is the health agent each coprocessor runs:
plain Java on the JDK's own web server, with xerial's sqlite-jdbc to read PhotonVision's database.

**What it reads**, each from the computer itself:

| Part | From |
|---|---|
| The stamp | `/etc/coprocessor/stamp.json`; the boot from `/proc/sys/kernel/random/boot_id`; the MAC from the first wired interface in `/sys/class/net` (one with a device behind it, preferring one that's up) |
| The boot | `/proc/uptime`; the root filesystem from `/proc/self/mountinfo` and its device from `/sys/dev/block/<major:minor>/uevent`; whether the boot before ended cleanly from its journal (`journalctl -b -1`: journald stopping, or systemd reaching its power-off, reboot, or shutdown target; unverified on these boards); the bootloader from `/run/coprocessor/spi-uboot-version` |
| The CPU | `/proc/stat`, between answers; each cluster from `/sys/devices/system/cpu/cpufreq/policy*` |
| Heat | `/sys/class/thermal/thermal_zone*`, with each zone's trip points |
| PhotonVision | `systemctl show photonvision.service` |
| Cameras | `/dev/v4l/by-path/*-video-index0`, each through its video device to its USB device in `/sys` (port, speed, IDs, name); where each camera of the role is expected, from PhotonVision's settings, as PhotonVision matches it (its by-path entry) |
| The journal | `journalctl -o json`: the kernel's messages and priority 3 or worse, counted a bounded piece at a time, picking up by cursor |
| The drive | `/run/coprocessor/nvme-smart-log.json` and `nvme-id-ctrl.json`, nvme-cli's reports, which a root job on the image writes every minute (the agent can't read the drive itself). nvme-cli's JSON has changed between versions, so a temperature reads as kelvins or with its unit, and a few names have two spellings (unverified against the image's nvme-cli) |
| Settings | PhotonVision's database, `/opt/photonvision/photonvision_config/photon.sqlite`, opened read-only, read again only when the file changes, in one transaction so PhotonVision's writes wait milliseconds at most |

Every path and command goes through one `Host`, so the tests run against a fixture tree of an
RK3588 (`src/test/resources/rk3588`: eight cores in three clusters, seven thermal zones, two
cameras, an NVMe drive) with canned command output, on any system. Colons in sysfs names are
written `%3A` there, since Windows can't check out a file named with one.

**Bounds.** A command has 2 seconds, and is stopped once it has printed what's needed (a journal
page's entries, at most 4 MB); a file is read to 256 KB; an answer is at most 4 MB but the settings
(48 MB of text); the web server allows 10 seconds to read a request and 60 to send an answer. Two
threads serve requests, one times commands, and a shutdown has its own.

**Cost**, measured on a desktop: up and answering in about 160 ms; 75 MB resident after a minute
of health and full journal pages twice a second; a health answer in about 2 ms when it's half a
second old, 250 ms for the first (which counts the boot's journal).

### Built and installed

`./gradlew :coprocessor-agent:build` makes **`coprocessor/agent/build/libs/coprocessor-agent.jar`**,
run with `java -jar`: the agent and everything it needs, with sqlite-jdbc's native libraries for
64-bit ARM and x86 Linux only (1.6 MB). The image installs it as
`/opt/coprocessor/coprocessor-agent.jar`, and **`coprocessor/agent/coprocessor-agent.service`** as
`/etc/systemd/system/coprocessor-agent.service`, enabled. The unit is also the launcher: its
`ExecStart` carries the JVM's settings (a 64 MB heap, the serial collector, the quick compiler
only) and runs `/usr/bin/java`, where PhotonVision's image installs OpenJDK 25. It runs on the
small cores (`AllowedCPUs=0-3`; PhotonVision has 4-7) at `Nice=10`, read-only everywhere but
`/run/coprocessor-agent`, where sqlite-jdbc unpacks its library.

`java -jar coprocessor-agent.jar [serve] [--port=N] [--bind=ADDRESS] [--database=PATH]
[--unit=NAME]` serves on the stamp file's `agentPort` (5808 when it has none) unless `--port`
says. The stamp file is the `Stamp` JSON without `bootId` and `mac`, plus `agentPort`; the image's
stamping writes it.

**What the image provides:**

- **The user** `coprocessor-agent`, a system user (no login, no home) in the `systemd-journal`
  group, so it can read the journal.
- **`/run/coprocessor/`**, world-readable, written by root every minute: `nvme-smart-log.json`
  (`nvme smart-log --output-format=json /dev/nvme0`), `nvme-id-ctrl.json` (`nvme id-ctrl
  --output-format=json /dev/nvme0`), and `spi-uboot-version` (one line). A file that's missing
  reads as unknown.
- **Read access** to PhotonVision's folder and database (its files are 644, its folders 755).
- **A polkit rule** letting the agent's user stop PhotonVision's unit and power off, and nothing
  else, in `/etc/polkit-1/rules.d/` (the actions are systemd's
  `org.freedesktop.systemd1.manage-units`, with the unit and verb it passes, and logind's
  `org.freedesktop.login1.power-off`, and `power-off-multiple-sessions` for when someone is logged
  in over SSH):

  ```js
  polkit.addRule(function (action, subject) {
    if (subject.user !== "coprocessor-agent") return polkit.Result.NOT_HANDLED;
    if (action.id === "org.freedesktop.systemd1.manage-units" &&
        action.lookup("unit") === "photonvision.service" &&
        action.lookup("verb") === "stop") return polkit.Result.YES;
    if (action.id === "org.freedesktop.login1.power-off" ||
        action.id === "org.freedesktop.login1.power-off-multiple-sessions") return polkit.Result.YES;
    return polkit.Result.NOT_HANDLED;
  });
  ```

**For stamping:** `java -jar coprocessor-agent.jar settings-db ROWS_DIR EMPTY_DB OUT_DB` builds
`OUT_DB` from a copy of `EMPTY_DB` (the empty database the pinned PhotonVision made in its smoke
test) and the committed settings in `ROWS_DIR`, reads it back to check nothing was lost, and prints
the settings' hash, the one the stamp carries. It fails, saying why, on settings that aren't there,
a database at another schema version, or anything it can't write.
