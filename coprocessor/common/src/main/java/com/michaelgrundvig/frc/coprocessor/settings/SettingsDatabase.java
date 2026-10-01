package com.michaelgrundvig.frc.coprocessor.settings;

import java.sql.Connection;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

/**
 * Settings read from and written to PhotonVision's SQLite database, through JDBC's own API: the
 * driver comes from whoever calls (the agent brings xerial's sqlite-jdbc), so the robot program,
 * which only hashes files, needs none.
 *
 * <p>Nothing here knows PhotonVision's tables: it reads every table, with its primary key as the
 * row's key and every other column as JSON, so a column a new version adds comes along unasked.
 */
public final class SettingsDatabase {
  private SettingsDatabase() {}

  /** Every row of every table, and the schema version, parsed. */
  public static Settings read(Connection connection) throws SQLException {
    return readText(connection).parse();
  }

  /**
   * Every row of every table, and the schema version, as the database's text: one quick pass, which
   * holds PhotonVision's writes off for as short a time as it can.
   */
  public static SettingsText readText(Connection connection) throws SQLException {
    // One transaction, so every table is read as of one moment, between PhotonVision's saves.
    boolean autoCommit = connection.getAutoCommit();
    connection.setAutoCommit(false);
    try {
      return readTextInTransaction(connection);
    } finally {
      connection.rollback();
      connection.setAutoCommit(autoCommit);
    }
  }

  private static SettingsText readTextInTransaction(Connection connection) throws SQLException {
    List<SettingsText.Row> rows = new ArrayList<>();
    for (String table : tables(connection)) {
      TableShape shape = shape(connection, table);
      String sql =
          "SELECT * FROM " + quote(table) + " ORDER BY " + quote(shape.key()) + " COLLATE BINARY";
      try (Statement statement = connection.createStatement();
          ResultSet result = statement.executeQuery(sql)) {
        while (result.next()) {
          String key = result.getString(shape.key());
          if (key == null) {
            throw new SettingsException("table " + table + " has a row with no key");
          }
          Map<String, String> columns = new LinkedHashMap<>();
          for (String column : shape.columns()) {
            String text = result.getString(column);
            columns.put(column, text == null ? "null" : text);
          }
          rows.add(new SettingsText.Row(table, key, columns));
        }
      }
    }
    return new SettingsText(userVersion(connection), rows);
  }

  /**
   * Replaces every row of every table with the settings' rows, in one transaction. The database
   * must be at the settings' schema version, and have each table and column they name: that is, be
   * made by the PhotonVision version the settings came from.
   */
  public static void write(Connection connection, Settings settings) throws SQLException {
    int version = userVersion(connection);
    if (version != settings.userVersion()) {
      throw new SettingsException(
          "the settings are for schema version "
              + settings.userVersion()
              + " and the database is at "
              + version
              + ": save the settings again from the PhotonVision version the database is for");
    }
    Set<String> tables = tables(connection);
    for (SettingsRow row : settings.rows()) {
      if (!tables.contains(row.table())) {
        throw new SettingsException(
            "the database has no table " + row.table() + "; it's for another PhotonVision version");
      }
    }
    boolean autoCommit = connection.getAutoCommit();
    connection.setAutoCommit(false);
    try {
      for (String table : tables) {
        TableShape shape = shape(connection, table);
        try (Statement statement = connection.createStatement()) {
          statement.executeUpdate("DELETE FROM " + quote(table));
        }
        for (SettingsRow row : settings.rows(table)) {
          insert(connection, shape, row);
        }
      }
      connection.commit();
    } catch (SQLException | RuntimeException e) {
      connection.rollback();
      throw e;
    } finally {
      connection.setAutoCommit(autoCommit);
    }
  }

  private static void insert(Connection connection, TableShape shape, SettingsRow row)
      throws SQLException {
    Map<String, String> text = row.columnText();
    for (String column : text.keySet()) {
      if (!shape.columns().contains(column)) {
        throw new SettingsException(
            "table "
                + shape.table()
                + " has no column "
                + column
                + " (row \""
                + row.key()
                + "\"); the settings are for another PhotonVision version");
      }
    }
    List<String> names = new ArrayList<>();
    names.add(shape.key());
    names.addAll(text.keySet());
    String sql =
        "INSERT INTO "
            + quote(shape.table())
            + " ("
            + String.join(", ", names.stream().map(SettingsDatabase::quote).toList())
            + ") VALUES ("
            + String.join(", ", names.stream().map(name -> "?").toList())
            + ")";
    try (PreparedStatement statement = connection.prepareStatement(sql)) {
      statement.setString(1, row.key());
      int index = 2;
      for (String value : text.values()) {
        statement.setString(index++, value);
      }
      statement.executeUpdate();
    }
  }

  private record TableShape(String table, String key, List<String> columns) {}

  private static TableShape shape(Connection connection, String table) throws SQLException {
    String key = null;
    int keys = 0;
    List<String> columns = new ArrayList<>();
    try (Statement statement = connection.createStatement();
        ResultSet result = statement.executeQuery("PRAGMA table_info(" + quote(table) + ")")) {
      while (result.next()) {
        String name = result.getString("name");
        if (result.getInt("pk") > 0) {
          key = name;
          keys++;
        } else {
          columns.add(name);
        }
      }
    }
    if (key == null || keys != 1) {
      throw new SettingsException("table " + table + " has no single primary key");
    }
    return new TableShape(table, key, columns);
  }

  private static Set<String> tables(Connection connection) throws SQLException {
    Set<String> tables = new TreeSet<>();
    try (Statement statement = connection.createStatement();
        ResultSet result =
            statement.executeQuery(
                "SELECT name FROM sqlite_master WHERE type = 'table'"
                    + " AND name NOT LIKE 'sqlite\\_%' ESCAPE '\\'")) {
      while (result.next()) {
        tables.add(result.getString(1));
      }
    }
    return tables;
  }

  /** The database's schema version: {@code PRAGMA user_version}. */
  public static int userVersion(Connection connection) throws SQLException {
    try (Statement statement = connection.createStatement();
        ResultSet result = statement.executeQuery("PRAGMA user_version")) {
      return result.next() ? result.getInt(1) : 0;
    }
  }

  /** An identifier, quoted for SQLite. */
  static String quote(String identifier) {
    return "\"" + identifier.replace("\"", "\"\"") + "\"";
  }
}
