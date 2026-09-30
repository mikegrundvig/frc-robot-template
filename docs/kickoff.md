# Kickoff: a new game's field

The robot's code refers to the field by name: its starts, targets, and zones, its AprilTags, and
its terrain. All of that is one class in `src/main/java/frc/robot/game/`, written for the blue
alliance; the field model works out the red alliance's. At kickoff, the new game gets its own class,
and last season's becomes a test fixture.

## Steps

1. **Keep last season's field as a fixture.** Move its class to the test tree, in the same package,
   so the tests that use it keep compiling:

   ```bash
   git mv src/main/java/frc/robot/game/Rebuilt.java src/test/java/frc/robot/game/Rebuilt.java
   ```

   Many simulation tests check fundamentals (zones that look ahead, the placement check, riding
   over terrain, alliance symmetry) against last season's field. They stay: those fundamentals don't
   change with the game, and a known field with bumps and trenches exercises them well.

2. **Write the new game** in `src/main/java/frc/robot/game/`, following last season's class:
   - its AprilTag layout, from WPILib's `Fields` for the new year;
   - its symmetry (mirrored or rotated): the test in step 4 checks it against the tags;
   - its starts, places (targets near tags are derived from each alliance's own tags), and zones;
   - its terrain: bumps, ramps, and platforms, each written for blue's half.

3. **Use it:** change the one line in `Robot.java` that builds the field.

4. **Test it,** as `RebuiltTest` tests last season's: the symmetry against the tags, the places
   where the manual puts them, the zones where the field's elements are.

5. **Run `./gradlew ci`.**
