package com.michaelgrundvig.frc.coprocessor.agent;

import java.nio.file.Path;
import java.sql.Connection;
import java.sql.SQLException;
import org.sqlite.SQLiteConfig;

/** SQLite connections, through xerial's sqlite-jdbc: the agent's only use of the driver. */
final class Sqlite {
  /** How long to wait for PhotonVision to finish a write before giving up, milliseconds. */
  static final int BUSY_TIMEOUT = 2000;

  private Sqlite() {}

  /** A read-only connection: the agent never writes PhotonVision's live database. */
  static Connection openReadOnly(Path file) throws SQLException {
    SQLiteConfig config = new SQLiteConfig();
    config.setReadOnly(true);
    config.setBusyTimeout(BUSY_TIMEOUT);
    return config.createConnection("jdbc:sqlite:" + file.toAbsolutePath());
  }

  /** A connection that may write: for building a database at stamping, never a live one. */
  static Connection open(Path file) throws SQLException {
    SQLiteConfig config = new SQLiteConfig();
    config.setBusyTimeout(BUSY_TIMEOUT);
    return config.createConnection("jdbc:sqlite:" + file.toAbsolutePath());
  }
}
