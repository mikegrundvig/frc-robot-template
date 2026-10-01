package frc.robot.studies;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * A study's input file, written by hand or saved from a spreadsheet: one record a line, its values
 * separated by commas. Blank lines and lines starting with {@code #} are skipped, and the mark some
 * spreadsheets put before the first character (a UTF-8 byte-order mark) is ignored. Every error
 * names the file and the line, and the column when it's one value that's wrong.
 */
final class CsvFile {
  private CsvFile() {}

  /**
   * One line's values, and where it came from.
   *
   * @param file the file's name
   * @param number the line's number in the file, from 1
   * @param values its values, each with the spaces around it taken off
   */
  record Line(String file, int number, List<String> values) {
    Line {
      values = List.copyOf(values);
    }

    /** A problem with this line, naming it: "weights.csv line 3: ..." */
    IllegalArgumentException wrong(String why) {
      return new IllegalArgumentException(file + " line " + number + ": " + why);
    }

    /** A problem with one of its values, naming the line and the column. */
    IllegalArgumentException wrong(String column, String why) {
      return wrong("column " + column + ": " + why);
    }

    /** The value at a place on the line, as a number. */
    double number(int at, String column) {
      String text = values.get(at);
      try {
        return Double.parseDouble(text);
      } catch (NumberFormatException e) {
        throw wrong(column, "\"" + text + "\" isn't a number");
      }
    }

    /** The value at a place, as numbers separated by spaces: "0 30 45". */
    List<Double> numbers(int at, String column) {
      List<Double> numbers = new ArrayList<>();
      for (String text : values.get(at).split("\\s+")) {
        if (text.isEmpty()) {
          continue;
        }
        try {
          numbers.add(Double.parseDouble(text));
        } catch (NumberFormatException e) {
          throw wrong(column, "\"" + text + "\" isn't a number");
        }
      }
      return numbers;
    }

    /** Whether none of the values at these places is a number: column names, most likely. */
    boolean namesColumns(int... at) {
      for (int place : at) {
        try {
          Double.parseDouble(values.get(place));
          return false;
        } catch (NumberFormatException e) {
          // Not a number: so far, a name.
        }
      }
      return true;
    }
  }

  /**
   * The file's lines with something in them, each with the number of values it must have.
   *
   * @param columns what each value is, in order, for the message when a line has too few or too
   *     many
   * @throws IllegalArgumentException naming a line without one value per column
   */
  static List<Line> read(Path file, List<String> columns) throws IOException {
    String name = file.getFileName().toString();
    List<String> text = Files.readAllLines(file);
    List<Line> lines = new ArrayList<>();
    for (int i = 0; i < text.size(); i++) {
      String line = text.get(i);
      if (i == 0 && line.startsWith("﻿")) {
        line = line.substring(1);
      }
      line = line.strip();
      if (line.isEmpty() || line.startsWith("#")) {
        continue;
      }
      List<String> values = new ArrayList<>();
      for (String value : line.split(",", -1)) {
        values.add(value.strip());
      }
      Line read = new Line(name, i + 1, values);
      if (values.size() != columns.size()) {
        throw read.wrong(
            "a line is " + columns.size() + " values, " + String.join(", ", columns) + ": " + line);
      }
      lines.add(read);
    }
    return lines;
  }
}
