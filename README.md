# Swerve test chassis

This branch is the robot program for the swerve test chassis and nothing else: a square frame on
four swerve modules, with a Pigeon 2, every device on one CANivore. You can drive it two ways:

- **In the simulator**, on any laptop, with no robot at all.
- **For real**, from a laptop, with the chassis's CANivore plugged into the laptop's USB port. The
  laptop runs the same program the robot will run, and drives the real motors.

The code is WPILib 2027 (Java), Commands v3, AdvantageKit and CTRE's Phoenix 6.

## Get it

You need git and Java (any version; the build downloads the one it needs). No WPILib installer is
needed to build, test, or run the simulator.

```bash
git clone -b swerve-bench https://github.com/mikegrundvig/frc-robot-template.git
cd frc-robot-template
```

The commands below are for Windows (`gradlew.bat`, in Command Prompt or PowerShell). On macOS or
Linux, type `./gradlew` instead of `gradlew.bat`. The first build downloads a lot, so give it a few
minutes.

## Drive it in the simulator

```bash
gradlew.bat run
```

Open http://localhost:5820/#sim in a browser. That's Pitwall's Sim view: click **Enable**, then
drive with the keyboard or a gamepad. Stop the program with Ctrl+C in the terminal.

## Drive the real chassis

```bash
gradlew.bat run -PhwSim
```

This drives real motors. A mentor runs this, and goes through the runtime's
[bench guide](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/bench.md) first:
what the laptop needs, the firmware, the safety checklist (chassis on blocks, a hand on the
breaker), how to enable from WPILib's simulator GUI, and what to do when something's wrong. The
chassis starts disabled.

## Where things are

| File | What's in it |
|---|---|
| `src/main/java/frc/robot/ChassisRobot.java` | The chassis's facts: its CAN IDs, module zeros, gear ratios, size, current limits and gains. Read it from the top. The first block holds every value still to check on the real chassis, each marked "Unconfirmed": check it in Phoenix Tuner X, fix it if it's wrong, and mark it "Checked" with the date. |
| `src/main/java/frc/robot/Controls.java` | What the sticks and buttons do. The driver's sticks already drive the chassis and the back button resets its heading; add your own buttons here. |
| `src/main/java/frc/robot/Robots.java` | The list of robots. This branch has one, the chassis. |
| `src/main/java/frc/robot/Main.java` | Where the program starts. You won't need to change it. |
| `src/test/java/frc/robot/` | The tests. |

The program itself (the drivetrain, the simulator, Pitwall) comes from the
[runtime](https://github.com/mikegrundvig/frc-robot-runtime), a library the build downloads. Its
[docs](https://github.com/mikegrundvig/frc-robot-runtime/tree/main/docs) explain driving, the
simulator and testing in more depth. [docs/editors.md](docs/editors.md) sets up IntelliJ IDEA or
VS Code.

## Run the tests

```bash
gradlew.bat test -Pquick
gradlew.bat ci
```

`test -Pquick` runs the fast tests while you work. `ci` runs every check: the formatter, the static
analysis, every test (the simulator's too) and the coverage floor. Run it before you push. When a
check fails, its message says what to fix.
