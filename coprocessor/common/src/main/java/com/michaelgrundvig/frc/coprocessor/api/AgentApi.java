package com.michaelgrundvig.frc.coprocessor.api;

/**
 * The coprocessor agent's HTTP API, version 1: read-only, JSON, on the computer's {@code agentPort}
 * (5808 unless the table says otherwise). It executes nothing a request asks for, and bounds every
 * answer.
 */
public final class AgentApi {
  /** Which image this is, and as which computer: a {@link Stamp}. */
  public static final String STAMP = "/v1/stamp";

  /** The computer's health: a {@link Health}. */
  public static final String HEALTH = "/v1/health";

  /**
   * A page of the journal: a {@link JournalPage}. Asks with {@code cursor} (the entries after it;
   * without one, the latest), {@code priority} (0-7: that and worse), {@code unit} (one systemd
   * unit), and {@code limit} (at most {@link #MAX_JOURNAL_PAGE}).
   */
  public static final String JOURNAL = "/v1/journal";

  /** PhotonVision's settings as canonical rows, with their hash. */
  public static final String SETTINGS = "/v1/settings";

  /** PhotonVision's settings as a zip laid out like the repository's folder for the computer. */
  public static final String SETTINGS_ZIP = "/v1/settings.zip";

  /** Where the image keeps its stamp. */
  public static final String STAMP_FILE = "/etc/coprocessor/stamp.json";

  /** How many journal entries a page has unless the request says. */
  public static final int DEFAULT_JOURNAL_PAGE = 100;

  /** The most journal entries a page may have. */
  public static final int MAX_JOURNAL_PAGE = 500;

  /** The longest a journal message is sent; longer ones are cut. */
  public static final int MAX_MESSAGE = 2048;

  private AgentApi() {}
}
