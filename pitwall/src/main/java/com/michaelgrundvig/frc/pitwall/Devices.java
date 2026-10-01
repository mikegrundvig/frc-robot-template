package com.michaelgrundvig.frc.pitwall;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URI;
import java.net.URISyntaxException;
import java.time.Duration;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import java.util.regex.Pattern;
import org.jspecify.annotations.Nullable;

/**
 * The robot's devices as the page's Pit view shows them: a card for each, saying whether it's there
 * and how it is, ending in what can be done about it: a link to the device's own page, a file to
 * download from it, or its log to read. A device is whatever the robot program knows how to ask: a
 * vision coprocessor, say. Pitwall knows nothing of any device; the robot describes each.
 *
 * <pre>{@code
 * pitwall.devices().add(
 *     Devices.Device.named("vision-front", "Coprocessor", coprocessor::card)
 *         .link("Open PhotonVision", "http://10.0.0.11:5800")
 *         .download("settings", "Download settings backup", coprocessor::settingsBackup)
 *         .log(coprocessor::journal));
 * }</pre>
 *
 * <p>A card is read in the robot loop, like a status item: only while a page watches, and at most
 * once an interval, so keep it cheap (what the loop already read). Downloads and log pages are
 * fetched on the page's own request, on Pitwall's server threads and never in the robot loop, so
 * they may wait on the network; the page never talks to a device itself. None of them changes the
 * robot, so nothing is logged for replay.
 */
public final class Devices {
  /**
   * A device's name, and a download's id: lowercase letters, digits, and hyphens, as they appear in
   * the page's addresses ({@code /api/devices/vision-front/downloads/settings}).
   */
  public static final Pattern NAME = Pattern.compile("[a-z0-9][a-z0-9-]{0,62}");

  /** The most often a card is read: once a second, as the status lines that follow sensors. */
  public static final Duration INTERVAL = Duration.ofSeconds(1);

  /** The schemes a card may link to: a page's, never a script's. */
  private static final Set<String> LINKED_SCHEMES = Set.of("http", "https");

  /** How a line, or a whole card, stands: the page marks each its own way, never by color alone. */
  public enum Level {
    /** As it should be: ✓. */
    OK,
    /** Worth knowing, neither good nor bad: •. */
    NOTE,
    /** Worth a look before a match: ▲. */
    WARNING,
    /** Wrong: ✗. */
    PROBLEM
  }

  /**
   * One line of a card.
   *
   * @param label what it is, such as "Version"
   * @param value what it says, as text
   * @param level how it stands
   */
  public record Line(String label, String value, Level level) {}

  /**
   * A device's card as of the last read.
   *
   * @param state a word or two for whether it's there: "Present", "Missing", "Two answer"
   * @param level how the device stands as a whole
   * @param lines what the card says, in order
   */
  public record Card(String state, Level level, List<Line> lines) {
    public Card {
      lines = List.copyOf(lines);
    }
  }

  /**
   * A file a device gives, streamed to the page as it arrives: Pitwall never holds a file of known
   * length whole. Pitwall closes it once sent.
   *
   * @param name the file's name as the browser saves it: letters, digits, dots, hyphens, and
   *     underscores
   * @param contentType its media type, such as {@code application/zip}
   * @param length how many bytes it holds, or -1 when the device doesn't say: one that ends short
   *     of its length reaches the page cut off, as an error, never as a whole file
   * @param body what it holds
   */
  public record File(String name, String contentType, long length, InputStream body)
      implements AutoCloseable {
    private static final Pattern FILE_NAME = Pattern.compile("[A-Za-z0-9._-]{1,100}");
    private static final Pattern MEDIA_TYPE =
        Pattern.compile("[a-z]+/[a-z0-9.+-]+(; ?charset=[a-z0-9-]+)?");

    public File {
      if (!FILE_NAME.matcher(name).matches() || name.startsWith(".")) {
        throw new IllegalArgumentException("not a plain file name: " + name);
      }
      if (!MEDIA_TYPE.matcher(contentType).matches()) {
        throw new IllegalArgumentException("not a media type: " + contentType);
      }
      if (length < -1) {
        throw new IllegalArgumentException("a file's length is -1 (unknown) or more: " + length);
      }
    }

    /** A file held whole. */
    public static File of(String name, String contentType, byte[] bytes) {
      return new File(name, contentType, bytes.length, new ByteArrayInputStream(bytes));
    }

    @Override
    public void close() throws IOException {
      body.close();
    }
  }

  /** Fetches a file from a device, on Pitwall's server threads. */
  @FunctionalInterface
  public interface Fetch {
    /**
     * @throws IOException saying why there's no file: the device didn't answer, say
     */
    File fetch() throws IOException;
  }

  /**
   * What the page asks of a device's log: one page of entries.
   *
   * @param cursor where the last page ended, empty for the latest entries
   * @param priority the least important entry wanted, as syslog numbers them: 0, emergencies only,
   *     to 7, everything down to debugging
   * @param unit only this unit's entries, such as {@code photonvision.service}; empty for all
   */
  public record LogQuery(String cursor, int priority, String unit) {
    /** A unit's name, as systemd writes them. */
    public static final Pattern UNIT = Pattern.compile("[A-Za-z0-9@._:-]{0,100}");

    /** A cursor: opaque, but printable and short. */
    public static final Pattern CURSOR = Pattern.compile("[\\x21-\\x7e]{0,512}");

    public LogQuery {
      if (priority < 0 || priority > 7) {
        throw new IllegalArgumentException("a priority is from 0 to 7: " + priority);
      }
      if (!UNIT.matcher(unit).matches()) {
        throw new IllegalArgumentException("not a unit's name: " + unit);
      }
      if (!CURSOR.matcher(cursor).matches()) {
        throw new IllegalArgumentException("not a cursor");
      }
    }
  }

  /**
   * One entry of a device's log.
   *
   * @param time when it was written, as the device's robot program can place it: on the robot's
   *     clock where it can
   * @param priority 0 (emergency) to 7 (debug), as syslog numbers them
   * @param unit what wrote it, such as {@code kernel} or {@code photonvision.service}
   * @param message what it says
   */
  public record LogEntry(String time, int priority, String unit, String message) {}

  /**
   * One page of a device's log, newest last.
   *
   * @param entries the entries
   * @param next the cursor for the page after this one; empty when there's no more
   */
  public record LogPage(List<LogEntry> entries, String next) {
    public LogPage {
      entries = List.copyOf(entries);
    }
  }

  /** Reads a device's log, a page at a time, on Pitwall's server threads. */
  @FunctionalInterface
  public interface Log {
    /**
     * @throws IOException saying why there's no page: the device didn't answer, say
     */
    LogPage read(LogQuery query) throws IOException;
  }

  /**
   * A link on a card.
   *
   * @param label what the button says
   * @param url where it goes: http or https only
   */
  public record Link(String label, String url) {
    public Link {
      String scheme;
      try {
        scheme = new URI(url).getScheme();
      } catch (URISyntaxException e) {
        throw new IllegalArgumentException("not a link: " + url, e);
      }
      if (scheme == null || !LINKED_SCHEMES.contains(scheme.toLowerCase(Locale.ROOT))) {
        throw new IllegalArgumentException("a card links only to http or https: " + url);
      }
    }
  }

  /**
   * A download on a card.
   *
   * @param id its name in the page's address
   * @param label what the button says
   * @param fetch fetches the file
   */
  public record Download(String id, String label, Fetch fetch) {
    public Download {
      if (!NAME.matcher(id).matches()) {
        throw new IllegalArgumentException(
            "a download's id is lowercase letters, digits, and hyphens: " + id);
      }
    }
  }

  /**
   * One device: its name, what kind it is, its card, and what can be done from it. Described whole
   * before it's added: once added, the server's threads read it, so it can't change.
   */
  public static final class Device {
    private final String name;
    private final String kind;
    private final Supplier<Card> card;
    private final List<Link> links = new ArrayList<>();
    private final List<Download> downloads = new ArrayList<>();
    private @Nullable Log log;
    private boolean added;

    private Device(String name, String kind, Supplier<Card> card) {
      if (!NAME.matcher(name).matches()) {
        throw new IllegalArgumentException(
            "a device's name is lowercase letters, digits, and hyphens: " + name);
      }
      this.name = name;
      this.kind = kind;
      this.card = card;
    }

    /**
     * A device.
     *
     * @param name its name: lowercase letters, digits, and hyphens
     * @param kind what it is, such as "Coprocessor"
     * @param card its card now, read in the robot loop
     */
    public static Device named(String name, String kind, Supplier<Card> card) {
      return new Device(name, kind, card);
    }

    /** Adds a link, such as to the device's own page. */
    public Device link(String label, String url) {
      unchanged();
      links.add(new Link(label, url));
      return this;
    }

    /** Adds a file to download from it. */
    public Device download(String id, String label, Fetch fetch) {
      unchanged();
      Download download = new Download(id, label, fetch);
      if (downloads.stream().anyMatch(d -> d.id().equals(id))) {
        throw new IllegalArgumentException(name + " has two downloads named " + id);
      }
      downloads.add(download);
      return this;
    }

    /** Gives it a log the page can read, a page at a time, and download. */
    public Device log(Log log) {
      unchanged();
      this.log = log;
      return this;
    }

    /** Refuses a change once it's on the page, where the server's threads read it. */
    private void unchanged() {
      if (added) {
        throw new IllegalStateException(
            name + " is on the page already: describe a device whole before adding it");
      }
    }

    public String name() {
      return name;
    }

    public String kind() {
      return kind;
    }

    public List<Link> links() {
      return List.copyOf(links);
    }

    public List<Download> downloads() {
      return List.copyOf(downloads);
    }

    /** Its download with this id, if it has one. */
    public Optional<Download> download(String id) {
      return downloads.stream().filter(d -> d.id().equals(id)).findFirst();
    }

    /** Its log, if it has one. */
    public Optional<Log> log() {
      return Optional.ofNullable(log);
    }
  }

  private final LongSupplier clock;
  private final long intervalNanos;
  private volatile List<Device> devices = List.of();
  private volatile Map<String, Object> section = Map.of();
  private long readNanos;
  private boolean everRead;

  /**
   * @param clock the time in nanoseconds, for how often cards are read: the robot's clock
   */
  public Devices(LongSupplier clock) {
    this(clock, INTERVAL);
  }

  Devices(LongSupplier clock, Duration interval) {
    this.clock = clock;
    intervalNanos = interval.toNanos();
  }

  /**
   * Adds a device, shown in the order added.
   *
   * @return this, to add the next
   * @throws IllegalArgumentException if another device has its name
   */
  public synchronized Devices add(Device device) {
    if (named(device.name()).isPresent()) {
      throw new IllegalArgumentException("two devices are named " + device.name());
    }
    device.added = true;
    List<Device> more = new ArrayList<>(devices);
    more.add(device);
    devices = List.copyOf(more);
    section = Map.of(); // read the new one at the next chance
    everRead = false;
    return this;
  }

  /** The device with this name, if there is one. Safe from any thread. */
  public Optional<Device> named(String name) {
    return devices.stream().filter(d -> d.name().equals(name)).findFirst();
  }

  /** Every device, in the order added. Safe from any thread. */
  public List<Device> all() {
    return devices;
  }

  /**
   * Reads every card, if a page watches and an interval has passed since the last read. Call it
   * once per loop, in the robot loop.
   */
  public void periodic(boolean watched) {
    if (!watched || devices.isEmpty()) {
      return;
    }
    long now = clock.getAsLong();
    if (everRead && now - readNanos < intervalNanos) {
      return;
    }
    readNanos = now;
    everRead = true;
    Map<String, Object> shown = new LinkedHashMap<>();
    for (Device device : devices) {
      shown.put(device.name(), shown(device, read(device)));
    }
    section = shown;
  }

  /**
   * The devices as of the last read, for the event stream: {@code {name: {"kind", "state", "level",
   * "lines": [{"label", "value", "level"}], "links": [{"label", "url"}], "downloads": [{"id",
   * "label"}], "log"}}}, levels in lowercase.
   */
  Map<String, Object> view() {
    return section;
  }

  /** A card that couldn't be read says why, rather than stopping the robot. */
  private static Card read(Device device) {
    try {
      return device.card.get();
    } catch (RuntimeException e) {
      String why = e.getMessage();
      return new Card(
          "Unknown",
          Level.WARNING,
          List.of(new Line("Card", "Error: " + (why != null ? why : e.toString()), Level.WARNING)));
    }
  }

  private static Map<String, Object> shown(Device device, Card card) {
    Map<String, Object> shown = new LinkedHashMap<>();
    shown.put("kind", device.kind());
    shown.put("state", card.state());
    shown.put("level", level(card.level()));
    List<Object> lines = new ArrayList<>();
    for (Line line : card.lines()) {
      Map<String, Object> each = new LinkedHashMap<>();
      each.put("label", line.label());
      each.put("value", line.value());
      each.put("level", level(line.level()));
      lines.add(each);
    }
    shown.put("lines", lines);
    List<Object> links = new ArrayList<>();
    for (Link link : device.links) {
      Map<String, Object> each = new LinkedHashMap<>();
      each.put("label", link.label());
      each.put("url", link.url());
      links.add(each);
    }
    shown.put("links", links);
    List<Object> downloads = new ArrayList<>();
    for (Download download : device.downloads) {
      Map<String, Object> each = new LinkedHashMap<>();
      each.put("id", download.id());
      each.put("label", download.label());
      downloads.add(each);
    }
    shown.put("downloads", downloads);
    shown.put("log", device.log != null);
    return shown;
  }

  private static String level(Level level) {
    return level.name().toLowerCase(Locale.ROOT);
  }
}
