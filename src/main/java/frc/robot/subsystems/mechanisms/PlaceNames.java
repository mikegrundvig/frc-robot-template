package frc.robot.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Position;
import java.util.Comparator;
import java.util.List;

/**
 * Where a joint is, in the names of its places, as its {@code At} logs it: "at out" within its band
 * of one, "between run and out", "below in", "above out"; where it is, as a number, for a joint
 * with no places. Its names are worked out once, so a loop builds none.
 */
final class PlaceNames {
  private final Joint joint;

  /** How near a place it must be to be at it, in its units: its band to arrive. */
  private final double enter;

  /** Its named positions, lowest first, and how each reads, between and beyond them. */
  private final List<Position> places;

  private final String[] atPlace;
  private final String[] betweenPlaces;
  private final String belowPlaces;
  private final String abovePlaces;

  /**
   * @param joint the joint, its places its named positions
   * @param enter how near one it must be to be at it, in its units
   */
  PlaceNames(Joint joint, double enter) {
    this.joint = joint;
    this.enter = enter;
    places =
        joint.positions().stream().sorted(Comparator.comparingDouble(Position::value)).toList();
    atPlace = new String[places.size()];
    betweenPlaces = new String[Math.max(0, places.size() - 1)];
    for (int i = 0; i < places.size(); i++) {
      atPlace[i] = "at " + places.get(i).name();
      if (i > 0) {
        betweenPlaces[i - 1] =
            "between " + places.get(i - 1).name() + " and " + places.get(i).name();
      }
    }
    belowPlaces = places.isEmpty() ? "" : "below " + places.getFirst().name();
    abovePlaces = places.isEmpty() ? "" : "above " + places.getLast().name();
  }

  /** Where the joint is, at a position, in its places' names. */
  String at(double position) {
    if (places.isEmpty()) {
      return joint.shown(position);
    }
    for (int i = 0; i < places.size(); i++) {
      if (Math.abs(position - places.get(i).value()) <= enter) {
        return atPlace[i];
      }
    }
    if (position < places.getFirst().value()) {
      return belowPlaces;
    }
    for (int i = 1; i < places.size(); i++) {
      if (position < places.get(i).value()) {
        return betweenPlaces[i - 1];
      }
    }
    return abovePlaces;
  }
}
