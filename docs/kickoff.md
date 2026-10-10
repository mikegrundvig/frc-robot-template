# Kickoff: a new game's field

The robot's code refers to the field by name: its starts, targets, and zones, its AprilTags, and
its terrain. A season's field is facts about a published game, written once when the game is
revealed, so it lives in the robot model (`model/`, `com.michaelgrundvig.frc.world.game`), one class
per season (`Rebuilt` for 2026), written for the blue alliance; the field model works out the red
alliance's. On kickoff day a new version of the model is cut with the new season's field. `Main`
hands the runtime that class (`Rebuilt::game`, the season's field and its venues).

## Steps

1. **Update the model.** Move `model/` to the version with the new season's field. Last season's
   class stays in it, so the 2026 robots and the runtime's tests, which are on 2026's field, keep
   compiling. Many simulation tests check fundamentals (zones that look ahead, the placement check,
   riding over terrain, alliance symmetry) against that field. They stay: those fundamentals don't
   change with the game, and a known field with bumps and trenches exercises them well.

2. **Use it:** change the one line in `Main.java` that names the field, `Rebuilt::game`, to the
   new season's class's `game` method, and the imports that name `Rebuilt` to match.

3. **If the model doesn't have it yet,** define a `Game` in your own code and pass it to
   `RobotProgram.start` until it does. A `Game` is plain data (`com.michaelgrundvig.frc.world.game`):
   - its official layout and every layout a venue may be measured against, from WPILib's `Fields`
     for the new year;
   - its symmetry (mirrored or rotated), which the model's tests check against the tags;
   - a function from the field's tags to the field model: its starts, places (targets near tags
     are derived from each alliance's own tags), zones, and terrain (bumps, ramps, and platforms,
     each written for blue's half).

   Follow `Rebuilt` in the model, and test your class as `RebuiltTest` tests it: the symmetry
   against the tags, the places where the manual puts them, the zones where the manual draws them.
   When the model has the season, delete yours and use the model's.

4. **Check the limits.** Point `SeasonLimitsTest` at the new season's limits (`LIMITS` in its
   class), so every robot's definition is checked against the new manual.

5. **Recheck the rules' numbers.** Every row of [runtime/docs/values.md](../runtime/docs/values.md) whose source is
   "rules" (the frame, its height, the bumpers, the mass) comes from last season's manual. Check
   each against the new one, and update `Robots.java` and the row.

6. **Run `./gradlew ci`.**

The camera placement study needs only the new field's tag layout, not its class: run it the day the
field is published, so CAD has camera requirements early ([camera-placement.md](../runtime/docs/camera-placement.md)).
