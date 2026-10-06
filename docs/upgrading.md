# Upgrading WPILib

WPILib and the vendor libraries built for it move together: each WPILib release (every alpha and
beta, then the season's release and its updates) has vendor releases made for it, and mixing
versions fails the build or, worse, the robot. Nothing updates them automatically (Dependabot
leaves WPILib and Gradle alone), so upgrade on purpose: all of it in one change, on a branch,
merged when `./gradlew ci` passes and the robot drives.

## Where the versions are

| What | Where |
|---|---|
| WPILib itself, through GradleRIO (the HAL, NetworkTables, the simulator, and the rest) | `gradle/libs.versions.toml`: `gradlerio` |
| Gradle | `gradle/wrapper/gradle-wrapper.properties`: `distributionUrl` and `distributionSha256Sum` |
| The WPILib year that editors' WPILib tools use | `.wpilib/wpilib_preferences.json`: `projectYear` |
| Vendor libraries: AdvantageKit, Commands v3, Phoenix 6, PhotonLib, REVLib | `vendordeps/*.json` |
| What waits on a library's release | `docs/stand-ins.md` |
| The version the README names | `README.md`, its status note |

## The steps

1. **Read the release notes**, WPILib's and each vendor's, for what changed and what broke.
2. **GradleRIO:** set `gradlerio` in `gradle/libs.versions.toml` to the new version. WPILib's
   "check for updates" in VS Code looks for the version in `build.gradle`, so it won't see this
   one: change it by hand.
3. **Gradle:** use the version the new WPILib installer ships, so a laptop set up with the
   installer still builds offline. Set it in `distributionUrl`, and set `distributionSha256Sum` to
   that version's checksum from gradle.org (it matches the zip the installer carries). Gradle
   refuses a download whose checksum differs.
4. **`projectYear`** in `.wpilib/wpilib_preferences.json`: the new installer's name for the
   year, the name of the folder it installs into (such as `2027_alpha7`).
5. **Vendor libraries:** replace each file in `vendordeps/` with the one made for the new WPILib.
   - Before the season's release, take them from WPILib's vendor-json-repo
     (github.com/wpilibsuite/vendor-json-repo), from the folder for the new WPILib version. The
     vendors' own "latest" links can point at another year's release there, or at nothing.
   - After it, `./gradlew vendordep --url=<the jsonUrl in the file>` fetches the latest.
   - `CommandsV3.json` ships with WPILib: copy it from the `vendordeps` folder of the new install.
   - A file whose name carries its version (Phoenix 6's): delete the old file. Two versions of one
     library can't both be in `vendordeps/`.
   - PhotonLib: the coprocessors must run the same PhotonVision version as the library.
6. **Build:** `./gradlew ci`, and fix what moved. Prereleases rename and move APIs; the release
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
7. **Drive it:** `./gradlew run`, open the Pitwall page's Sim view (http://localhost:5820/#sim),
   switch on driving with the keyboard, press Enable (it starts disabled), and drive around.
8. **Update the docs:** `docs/stand-ins.md` (a stand-in may now have its real library, and a
   pinned build its release) and the README's status note.
9. **On the robot:** image it with the Systemcore image released alongside the new WPILib, and
   update the Driver Station, as WPILib's release notes say. Then deploy and drive.
