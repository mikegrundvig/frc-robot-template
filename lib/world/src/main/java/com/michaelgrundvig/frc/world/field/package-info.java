/**
 * The field model: the field's size, AprilTags, and symmetry, the places and zones robot code
 * refers to by name, the raised ground robots drive over, and the one conversion between the
 * alliances.
 *
 * <p>Frames, written down once:
 *
 * <ul>
 *   <li><b>Field:</b> WPILib's. The origin is on the floor at the blue alliance wall, in the corner
 *       on the blue drivers' right; x runs toward the red alliance wall, y to the blue drivers'
 *       left, z up. Angles are counterclockwise from +x. Poses, tags, and logs are always in this
 *       frame, whichever alliance the robot is on.
 *   <li><b>Robot:</b> the origin is the center of the frame, on the floor; x forward, y left, z up.
 *   <li><b>Alliance intent:</b> start positions, targets, and zones are written as if the robot
 *       were on the blue alliance. {@link
 *       com.michaelgrundvig.frc.world.field.FieldModel#forAlliance} turns them into the red
 *       alliance's, and is the only place that knows how: no other code flips coordinates.
 * </ul>
 *
 * <p>Places and targets near AprilTags are derived from the tags of the alliance they are for (the
 * hub between two of its tags, a spot facing a tag, a speaker's mouth above its tag), so a layout
 * measured at an event corrects both alliances' at once.
 *
 * <p>A tag is known twice: where the official layout puts it, and where a venue's survey found it
 * ({@link com.michaelgrundvig.frc.world.field.Tag}). The symmetry turns only the first, with the
 * rest of the nominal geometry; a survey is a measurement, taken where each tag stands.
 *
 * <p>The ground is the floor, raised where a {@link com.michaelgrundvig.frc.world.field.Surface}
 * is: a bump, a ramp, a platform, or a mesh from CAD, each described exactly twice over, as heights
 * and as triangles.
 */
@NullMarked
package com.michaelgrundvig.frc.world.field;

import org.jspecify.annotations.NullMarked;
