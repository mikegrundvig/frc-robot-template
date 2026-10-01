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

Each coprocessor's agent answers on its `agentPort`: read-only HTTP, JSON, no authentication. It
runs on a closed robot network, executes nothing a request asks for, and bounds every answer.

| Request | Answer |
|---|---|
| `GET /v1/stamp` | `Stamp`: which image this is, and as which computer |
| `GET /v1/health` | `Health`: the computer's health, about 2 KB |
| `GET /v1/journal?cursor=&priority=&unit=&limit=` | `JournalPage`: a page of the journal |
| `GET /v1/settings` | PhotonVision's settings as canonical rows, with their hash |
| `GET /v1/settings.zip` | The same as files, laid out like the repository's folder for the computer |

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
`/proc/sys/kernel/random/boot_id`, and the network interface). Extra members are ignored.

### `Health`

| Member | Record | What |
|---|---|---|
| `stamp` | `Stamp` | As `/v1/stamp` |
| `boot` | `Boot` | Boot ID, uptime, the agent's monotonic clock (microseconds since boot, the journal's clock), whether the boot before shut down cleanly (`null`: unknown), the root device and whether it's read-only, the SPI bootloader's version |
| `cpu` | `Cpu`, `CpuCluster` | Each core's busy percentage over `windowSeconds`; each cluster's cores, current, limit, and maximum MHz. `bigClusters()` and `bigCoresCapped()` answer "are PhotonVision's cores held back" |
| `thermal` | `ThermalZone`, `TripPoint` | Every thermal zone's temperature, with its trip points as the running kernel has them. `firstPassive()` is where throttling starts |
| `photonvision` | `Service` | PhotonVision's service as systemd sees it: active state, sub-state, result, restarts this boot, and when it last started |
| `cameras` | `Cameras`, `ExpectedCamera`, `UsbCamera` | The cameras in the role, each with the USB path PhotonVision's settings match it by and whether something is plugged in there; and every USB camera plugged in, with its port and link speed |
| `journal` | `JournalSummary`, `JournalEntry` | This boot's count of USB, UVC, filesystem, out-of-memory, thermal, and other errors, with the latest few |
| `drive` | `Drive` | The NVMe drive's temperature, wear, unsafe shutdowns (each power cut counts one), power cycles, hours, media errors, and critical warnings; `null` without an NVMe drive |
| `settings` | `SettingsState` | The stamped settings hash against the live one; `matches()` |
| `problems` | text | What the agent couldn't read, each saying what and why; empty when it read everything |

### `JournalPage`

`entries` (oldest first), `cursor` (ask again with it for the entries after these), and `more`
(whether more were waiting). Without a `cursor`, the page is the latest entries; `priority` (0-7)
gives that priority and worse, `unit` one systemd unit, and `limit` the page size (100 unless
asked, at most 500). A message longer than 2048 characters is cut.

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
