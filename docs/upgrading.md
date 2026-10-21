# Upgrading WPILib

WPILib and the vendor libraries built for it move together: each WPILib release (every alpha and
beta, then the season's release and its updates) has vendor releases made for it, and mixing
versions fails the build or, worse, the robot. Nothing updates them automatically (Dependabot
leaves WPILib and Gradle alone), so upgrade on purpose: all of it in one change, on a branch,
merged when `./gradlew ci` passes and the robot drives.

## Where the versions are

| What | Where |
|---|---|
| WPILib itself, through GradleRIO (the HAL, NetworkTables, the simulator, and the rest) | `gradle/libs.versions.toml`: `gradlerio`, which must be the version the runtime was built with ([the libraries](#the-libraries)) |
| The FRC libraries: the runtime, with the robot model, localization, the simulation, Pitwall, and the runtime's Gradle plugins | `gradle/libs.versions.toml`: `frc-libs`, one dated version for all of them ([the libraries](#the-libraries)) |
| Gradle | `gradle/wrapper/gradle-wrapper.properties`: `distributionUrl` and `distributionSha256Sum` |
| The JDK Gradle runs on, and where a laptop without it downloads it | `gradle/gradle-daemon-jvm.properties`, written by `./gradlew updateDaemonJvm` |
| The WPILib year that editors' WPILib tools use | `.wpilib/wpilib_preferences.json`: `projectYear` |
| Vendor libraries: AdvantageKit, Commands v3, Phoenix 6, PhotonLib, REVLib | `vendordeps/*.json`, at the versions the runtime was built with |
| The checksum of every library and plugin the build downloads, the FRC libraries' among them | `gradle/verification-metadata.xml`, written by Gradle ([below](#dependency-checksums)) |
| What waits on a library's release | the runtime's [docs/stand-ins.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/stand-ins.md) |

## The libraries

The runtime, the robot model, localization and Pitwall are repositories of their own, published
together as one dated release (such as `2026.10.20`) to their Maven repository,
[mikegrundvig.github.io/frc-maven](https://mikegrundvig.github.io/frc-maven/), which lists every
release. This build names the release in one line, `frc-libs` in `gradle/libs.versions.toml`;
the runtime brings the rest, and its Gradle plugins (AdvantageScope's assets) are published with it
at the same version. Releases are pre-release: anything may change from one to the next, so read
the libraries' commits before moving.

**To move to a new release,** change `frc-libs` to it, then write the checksums again
([below](#dependency-checksums)) and run `./gradlew ci`.

The runtime is built and tested against one GradleRIO version and one set of vendor libraries, and
this build runs it with its own, so the two must be the same versions. The runtime's jar says which
it was built with (`META-INF/frc-robot-runtime/built-with.json`), and the build's
`checkRuntimeVersions` task (part of `check`, so of `ci`) compares GradleRIO's version
(`gradlerio`) and each of those vendor libraries, by its uuid, with `vendordeps/`, and fails, listing
each difference, until they agree. So a WPILib upgrade waits for a release of the libraries built
for it: move `frc-libs` to that release along with the steps below, and `ci` passes once both are
done.

### Developing the libraries

A team never needs this. To change a library and see it in this robot program before it's
published, clone the libraries' repositories into one folder, as `model/`, `localization/`,
`pitwall/` and `runtime/` (any of them), and give the build that folder:

```bash
./gradlew ci -PlocalLibs=../libs        # or put localLibs=/path/to/libs in ~/.gradle/gradle.properties
```

Each clone found there is built from its source in place of the published version (a Gradle
composite build), and `ci` runs its checks too; the runtime's plugins come from its clone as well.
Without `-PlocalLibs`, nothing local is used. Each library's README says how to build, test and
publish it on its own, and frc-maven's README how a release is made. `-PfrcMaven=<url>` reads the
published libraries from another copy of their repository, such as a clone holding a release not
pushed yet (`-PfrcMaven=file:///home/you/src/frc-maven`).

## The steps

1. **Read the release notes**, WPILib's and each vendor's, for what changed and what broke.
2. **GradleRIO:** set `gradlerio` in `gradle/libs.versions.toml` to the new version, and `frc-libs`
   to the libraries' release built for it ([above](#the-libraries)). WPILib's
   "check for updates" in VS Code looks for the version in `build.gradle`, so it won't see this
   one: change it by hand.
3. **Gradle:** use the version the new WPILib installer ships, so a laptop set up with the
   installer still builds offline. Set it in `distributionUrl`, and set `distributionSha256Sum` to
   that version's checksum from gradle.org (it matches the zip the installer carries). Gradle
   refuses a download whose checksum differs.
4. **The JDK Gradle runs on:** `./gradlew updateDaemonJvm --jvm-version=25` (the Java version the
   build's toolchains name) writes `gradle/gradle-daemon-jvm.properties`: for each platform, the
   link a laptop without that JDK downloads it from. Run it after a new Gradle or Java, and
   whenever a fresh laptop can't download its JDK: a link dies when its vendor takes the build
   down. Check that every platform has a line, `LINUX.X86_64` above all, and that no
   `toolchainVendor` line narrows it: with no vendor named, each platform gets Temurin where Foojay
   lists it and another vendor's (Azul's Zulu) where it doesn't. On 2026-10-07 Foojay listed no
   Temurin 25 for Linux, so `--jvm-vendor=adoptium` left Linux out, and a fresh Linux laptop
   couldn't build.
5. **`projectYear`** in `.wpilib/wpilib_preferences.json`: the new installer's name for the
   year, the name of the folder it installs into (such as `2027_alpha7`).
6. **Vendor libraries:** replace each file in `vendordeps/` with the one made
   for the new WPILib.
   - Before the season's release, take them from WPILib's vendor-json-repo
     (github.com/wpilibsuite/vendor-json-repo), from the folder for the new WPILib version. The
     vendors' own "latest" links can point at another year's release there, or at nothing.
   - After it, `./gradlew vendordep --url=<the jsonUrl in the file>` fetches the latest.
   - `CommandsV3.json` ships with WPILib: copy it from the `vendordeps` folder of the new install.
   - A file whose name carries its version (Phoenix 6's): delete the old file. Two versions of one
     library can't both be in `vendordeps/`.
   - PhotonLib: the coprocessors must run the same PhotonVision version as the library.
7. **Build:** `./gradlew ci`, and fix what moved. Prereleases rename and move APIs; the release
   notes say where.
   - **Gradle's configuration cache** (`gradle.properties`) can't hold most of GradleRIO's tasks,
     so `build.gradle` marks them incompatible (and `run`), and a
     build that runs one goes without the cache. Three are kept under it, since `ci` and the tests
     run them: `extractReleaseNative`, `extractDebugNative`, and `downloadDepsPreemptively` (the
     list `cacheable` in `build.gradle`). If a new GradleRIO breaks one of them under the cache
     (`ci` fails storing or loading it, naming the task), take the task off the list. If `ci` ends
     with *Configuration cache disabled because incompatible tasks were found.*, a new GradleRIO
     task is in its graph: when it works under the cache, add it to the list; until then `ci`
     works, just without the cache.
8. **Checksums:** write `gradle/verification-metadata.xml` again, on every platform
   ([below](#dependency-checksums)).
9. **Drive it:** `./gradlew run`, open the Pitwall page's Sim view (http://localhost:5820/#sim),
   switch on driving with the keyboard, press Enable (it starts disabled), and drive around.
10. **Update the docs:** in the runtime's repository, its `docs/stand-ins.md`: a stand-in may now
   have its real library, and a pinned build its release.
11. **On the robot:** image it with the Systemcore image released alongside the new WPILib, and
   update the Driver Station, as WPILib's release notes say. Then deploy and drive.

## Dependency checksums

`gradle/verification-metadata.xml` holds the SHA-256 checksum of every library and plugin the build
downloads, and Gradle checks each download against it
([dependency verification](https://docs.gradle.org/current/userguide/dependency_verification.html)):
a library replaced on its server under the same version, or a download changed on its way, doesn't
match. It covers what Gradle downloads as a library or plugin, not the JDK, Gradle itself
(`distributionSha256Sum` checks that), or the Eclipse bundles Spotless's Groovy formatter fetches
from Eclipse's own update sites.

For now it's lenient (`org.gradle.dependency.verification=lenient` in `gradle.properties`): a
library the file lacks, or whose checksum differs, is reported, as a warning that names it and a
report under `build/reports/dependency-verification/`, and the build goes on. A missing one is
expected after a version changes. A checksum that differs is not: find out why before going on.

Each platform downloads its own native libraries (WPILib's and the vendors'), so the file needs a
run on each. It was written on Linux, from what `ci` had already downloaded there, so it lacks
Windows' and macOS's native libraries, and what `ci` never downloads: the native libraries' debug
builds, and WPILib's tools (Glass, SysId and the rest). Until it's written with them, a build that
needs one of them warns, as every build on Windows or macOS does. To write it:

| Where | Run |
|---|---|
| Linux, macOS, Git Bash on Windows | `./gradlew --write-verification-metadata sha256 ci` |
| Windows, PowerShell | `.\gradlew.bat --write-verification-metadata sha256 ci` |
| Windows, Command Prompt | `gradlew.bat --write-verification-metadata sha256 ci` |

Each run downloads everything any part of the build names, not only what `ci` uses, so the first
takes a while. It keeps every entry the file has and adds what it downloaded, which is how the
platforms' results merge: run it on one, commit the file, and run it on the next with that commit
checked out (two runs from the same commit may conflict when Git merges them). After a build
tool's version changes, such as in a Dependabot pull request, one run on any platform adds it.
After a WPILib or vendor upgrade, which changes every platform's native libraries, run it on all
three. Gradle never removes an entry, so to drop the old versions', delete the file first and run
all three.

**At release, make it strict:** once the file has every platform's entries and `ci` runs on Linux
and Windows without a verification warning, delete the `lenient` line from `gradle.properties`
(strict is Gradle's default). From then on a library that doesn't match the file fails the build,
and so does one it lacks: write it again on a Dependabot pull request's branch before merging.
