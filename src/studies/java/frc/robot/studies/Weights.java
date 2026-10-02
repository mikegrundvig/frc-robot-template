package frc.robot.studies;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.shown;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigException;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Translation2d;

/**
 * How much each spot on the field counts, as a team decides once it knows the game: regions, each
 * written for the blue alliance and applied to its image on red's half too, by the field's
 * symmetry. A spot counts as the product of the weights of the regions it's in, and 1 in none.
 */
public final class Weights {
  /** Every spot counts the same. */
  public static final Weights NONE = new Weights(List.of());

  private final List<Region> regions;

  /**
   * A rectangle of the field, and how much a spot in it counts.
   *
   * @param name what it is, for the report and for errors: "Hub", "Scoring"
   * @param xMin its near edge, from the blue alliance's wall, in meters (the field's frame)
   * @param yMin its right edge, as blue's drivers see it
   * @param xMax its far edge
   * @param yMax its left edge
   * @param weight how much a spot in it counts: 0 leaves it out (inside a field element), 3 counts
   *     it three times
   */
  public record Region(
      String name, double xMin, double yMin, double xMax, double yMax, double weight) {
    public Region {
      Checks.on(Region.class, name)
          .field("name", name, notBlank())
          .field("xMin", xMin, finite())
          .field("yMin", yMin, finite())
          .field("xMax", xMax, finite())
          .field("yMax", yMax, finite())
          .field("weight", weight, nonNegative())
          .check("xMax", () -> xMax > xMin ? null : shown(xMax) + " must be more than xMin")
          .check("yMax", () -> yMax > yMin ? null : shown(yMax) + " must be more than yMin")
          .orThrow();
    }

    boolean contains(Translation2d point) {
      return point.getX() >= xMin
          && point.getX() <= xMax
          && point.getY() >= yMin
          && point.getY() <= yMax;
    }
  }

  public Weights(List<Region> regions) {
    this.regions = List.copyOf(regions);
  }

  public List<Region> regions() {
    return regions;
  }

  /** The columns of a weights file, in order. */
  static final List<String> COLUMNS = List.of("name", "x min", "y min", "x max", "y max", "weight");

  /**
   * Reads regions from a CSV file, one per line: {@code name, x min, y min, x max, y max, weight},
   * in meters. Blank lines and lines starting with {@code #} are skipped.
   *
   * @throws IllegalArgumentException naming the line, and the column, of any region that isn't one
   */
  public static Weights read(Path file) throws IOException {
    List<Region> regions = new ArrayList<>();
    for (CsvFile.Line line : CsvFile.read(file, COLUMNS)) {
      if (regions.isEmpty() && line.namesColumns(1, 2, 3, 4, 5)) {
        throw line.wrong("looks like the columns' names: start it with # to make it a comment");
      }
      try {
        regions.add(
            new Region(
                line.values().get(0),
                line.number(1, COLUMNS.get(1)),
                line.number(2, COLUMNS.get(2)),
                line.number(3, COLUMNS.get(3)),
                line.number(4, COLUMNS.get(4)),
                line.number(5, COLUMNS.get(5))));
      } catch (ConfigException e) {
        throw line.wrong(String.valueOf(e.getMessage()));
      }
    }
    return new Weights(regions);
  }

  /** How much a spot counts: the product of the weights of the regions it's in, on either half. */
  public double at(FieldModel field, Translation2d point) {
    // A spot is in a region's image on red's half when the spot's own image is in the region.
    Translation2d image = field.forAlliance(Alliance.RED, point);
    double weight = 1;
    for (Region region : regions) {
      if (region.contains(point) || region.contains(image)) {
        weight *= region.weight();
      }
    }
    return weight;
  }
}
