# Editors and commands

Gradle is the tool. Every job, from testing to simulating, deploying, and debugging, is a `./gradlew`
command that works the same on every operating system, in every editor, and in CI. An editor is a
view onto the Gradle project: it reads the build to understand the code, and it runs everything
through Gradle. Use whichever editor you like, or none. The repository commits no editor settings,
and nothing depends on a particular editor.

## The commands

Run these from the repository folder. They work as written in PowerShell, Git Bash, macOS, and
Linux; in the Windows Command Prompt, leave off the `./` (`gradlew ci`).

| Job | Command |
|---|---|
| Every check CI runs; run it before pushing | `./gradlew ci` |
| Rerun the tests on every save | `./gradlew test -Pquick --continuous` |
| Run one test class ([more on tests](testing.md)) | `./gradlew :test --tests frc.robot.ArchitectureTest` |
| Start the simulator, driven from Pitwall's Sim view at http://localhost:5820/#sim ([more](simulator.md)) | `./gradlew run` |
| Start it with WPILib's simulator GUI too, enabled and drivable from its keyboard | `./gradlew run -Pgui` |
| Start it disabled, like a real robot | `./gradlew run -PstartDisabled` |
| Start it in a scenario: a fault, or something that happens to the robot | `./gradlew run -Pscenario=collision` |
| List the simulator's scenarios | `./gradlew scenarios` |
| Start it waiting for a debugger on port 5005 | `./gradlew run --debug-jvm` |
| Fix formatting (every local build does this too) | `./gradlew spotlessApply` |
| Redraw AdvantageScope's robot models from `Robots.java` (every build does this too; [more](simulator.md#see-it-in-advantagescope)) | `./gradlew advantageScopeAssets` |
| Store your team number, once, before deploying | `./gradlew setTeamNumber -PteamNumber=NNNN` |
| Deploy to the robot, after every check passes | `./gradlew deploy` |
| Deploy, with the robot waiting for a debugger on port 8349 | `./gradlew deploy -PdebugMode` |
| Find the robot on the network | `./gradlew discoversystemcore` |
| Add a vendor library | `./gradlew vendordep --url=<the vendor's JSON URL>` |
| List every task | `./gradlew tasks` |

**A debug deploy is for the pit only.** `deploy -PdebugMode` deploys a robot program that waits
for a debugger before it starts, and takes one from any device on the robot's network. Until a
debugger attaches, the robot does nothing, and after a brownout or a power cycle it waits again.
When you're done, deploy again without `-PdebugMode`, and never take a debug deploy onto the field.

Gradle's configuration cache is off until GradleRIO supports it. Turn it on for yourself with
`--configuration-cache` (`./gradlew ci --configuration-cache`), for anything but GradleRIO's own
tasks (`run`, `deploy`, `vendordep`), which fail with it.

## IntelliJ IDEA

1. **File > Open**, choose the repository folder, and trust the project when asked. IntelliJ reads
   the Gradle build. The first time, Gradle downloads its JDK and the libraries, which takes a few
   minutes.
2. Build once (**Build > Build Project**). Some code is generated when the project compiles
   (build information, the configuration builders, AdvantageKit's logged inputs); until the first
   build, IntelliJ shows the references to it as errors.
3. There is nothing to configure. IntelliJ builds, runs, and tests through Gradle by default, so
   everything behaves as it does on the command line: the checks, the generated code, and the native
   libraries the simulator and tests load. (If someone changed that: Settings > Build, Execution,
   Deployment > Build Tools > Gradle, with *Build and run using* and *Run tests using* both set to
   *Gradle*.)

Day to day:

- **Run anything** from the Gradle tool window, which lists every task (`verification > ci`,
  `application > run`, `DeployUtils > deploy`, and so on), or from the terminal.
- **Run or debug tests** with the icons beside each test class and method.
- **Debug the simulator:** in the Gradle tool window, right-click `application > run` and choose
  *Debug*. IntelliJ attaches by itself.
- **Debug the robot:** run `./gradlew deploy -PdebugMode`, then attach with a *Remote JVM Debug*
  run configuration (host `robot.local`, port 8349).
- **Formatting:** `.editorconfig` gives IntelliJ the project's indentation, and every build formats
  the code anyway. The google-java-format plugin makes IntelliJ's own formatter match the build's.

## VS Code

1. Install Microsoft's **Extension Pack for Java** from the Extensions view.
2. **File > Open Folder**, choose the repository folder, and trust it. The Java extension imports
   the Gradle build; progress shows in the status bar.

Day to day:

- **Run anything** from the terminal, or from the Gradle view in the side bar, which lists every
  task.
- **Run tests** with the Gradle commands above. <!-- VERIFY: the Testing view, and native libraries -->
- **Debug the simulator:** run `./gradlew run --debug-jvm`, then start a Java *Attach* debug
  configuration on port 5005. <!-- VERIFY: exact steps -->
- **Debug the robot:** run `./gradlew deploy -PdebugMode`, then attach the same way to host
  `robot.local`, port 8349.

### The WPILib installer's VS Code

It works as-is. This is the standard WPILib project layout, and its Build, Test, and Deploy commands
run the same Gradle tasks; *Set Team Number* writes the same file as `setTeamNumber`. One difference:
*Simulate Robot Code* uses Gradle only to prepare, then starts the robot itself under the debugger,
with whichever simulation extensions it offers you (WPILib's simulator GUI among them). The robot
then starts as a real robot does, disabled with no station or mode, and without the GUI's
first-run layout. Pitwall's Sim view's Driver Station works either way, but when you choose the GUI
it overwrites Sim's joysticks every frame, unless you turn on the GUI's *Disable DS*
([why](simulator.md)). `./gradlew run` starts it ready to drive, in teleop on a start, without the
GUI.

## Any other editor

Open the folder, let the editor import the Gradle build if it can, and use the commands above.
