package com.michaelgrundvig.frc.pitwall;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.time.Duration;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import java.util.regex.Pattern;

/**
 * The robot's devices as the page's Pit view shows them: a card for each, saying whether it's there
 * and how it is, ending in what can be done about it: a file to download from it or send to it, or
 * a choice to make. A device is whatever the robot program knows how to ask, or keep: the venue the
 * robot plays on, say. Pitwall knows nothing of any device; the robot describes each, as the
 * template's venue card does:
 *
 * {@snippet :
 * return Devices.Device.named("venue", "Field", pit::card)
 *     .upload("survey", "Upload venue", pit::upload)
 *     .choose("venue", pit::choose)
 *     .download("file", "Download venue file", pit::download);
 * }
 *
 * <p>A card is read in the robot loop, like a status item: only while a page watches, and at most
 * once an interval, so keep it cheap (what the loop already read). Downloads, uploads and choices
 * are made on the page's own request, on Pitwall's server threads and never in the robot loop, so
 * they may wait on the network or the disk; the page never talks to a device itself. None of them
 * changes the robot loop's state, so nothing is logged for replay.
 */
public final class Devices {
  /**
   * A device's name, and a download's id: lowercase letters, digits, and hyphens, as they appear in
   * the page's addresses ({@code /api/devices/vision-front/downloads/settings}).
   */
  public static final Pattern NAME = Pattern.compile("[a-z0-9][a-z0-9-]{0,62}");

  /** The most often a card is read: once a second, as the status lines that follow sensors. */
  public static final Duration INTERVAL = Duration.ofSeconds(1);

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
   * @param choices what the card offers to choose among, as of now, each answered by the device's
   *     {@link Device#choose} of the same id
   */
  public record Card(String state, Level level, List<Line> lines, List<Choice> choices) {
    public Card {
      lines = List.copyOf(lines);
      choices = List.copyOf(choices);
    }

    /** A card with nothing to choose. */
    public Card(String state, Level level, List<Line> lines) {
      this(state, level, lines, List.of());
    }
  }

  /**
   * Something a card offers to choose, with what's chosen now: the venue the robot plays on, say.
   *
   * @param id which of the device's choosers answers it: lowercase letters, digits, and hyphens
   * @param label what the page calls it, such as "Venue"
   * @param options what may be chosen, in order
   * @param current the option chosen now, by its value
   */
  public record Choice(String id, String label, List<Option> options, String current) {
    public Choice {
      if (!NAME.matcher(id).matches()) {
        throw new IllegalArgumentException(
            "a choice's id is lowercase letters, digits, and hyphens: " + id);
      }
      options = List.copyOf(options);
    }
  }

  /**
   * One option of a choice.
   *
   * @param value what's sent back when it's chosen
   * @param label what the page shows
   */
  public record Option(String value, String label) {}

  /**
   * A file the page sends a device: a venue's survey, say, checked by the robot before it keeps it.
   *
   * @param id its id in the page's addresses: lowercase letters, digits, and hyphens
   * @param label the button's words, such as "Upload venue"
   * @param accept takes the file
   */
  public record Upload(String id, String label, Accept accept) {
    public Upload {
      if (!NAME.matcher(id).matches()) {
        throw new IllegalArgumentException(
            "an upload's id is lowercase letters, digits, and hyphens: " + id);
      }
    }
  }

  /** Takes a file the page sends, on Pitwall's server threads, never in the robot loop. */
  @FunctionalInterface
  public interface Accept {
    /**
     * @param body the file, at most 64 KB
     * @return what the page says it did, such as "Saved Practice field, October"
     * @throws Refused saying why the file isn't taken: every problem, in words for the pit
     * @throws IOException if taking it failed
     */
    String accept(byte[] body) throws IOException;
  }

  /** Takes a choice the page makes, on Pitwall's server threads, never in the robot loop. */
  @FunctionalInterface
  public interface Choose {
    /**
     * @param value the chosen option's value
     * @return what the page says it did
     * @throws Refused saying why not
     * @throws IOException if choosing failed
     */
    String choose(String value) throws IOException;
  }

  /** A file or a choice the device won't take, and why: the page is answered 422 with the why. */
  public static final class Refused extends IOException {
    private static final long serialVersionUID = 1;

    public Refused(String why) {
      super(why);
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

  /**
   * A device busy with another request of the same kind, asking to be asked again shortly: the page
   * is answered 503, with {@code Retry-After}.
   */
  public static final class Busy extends IOException {
    private static final long serialVersionUID = 1;

    public Busy(String message) {
      super(message);
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
    private final List<Download> downloads = new ArrayList<>();
    private final List<Upload> uploads = new ArrayList<>();
    private final Map<String, Choose> choosers = new LinkedHashMap<>();
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

    /** Adds a file the page can send it: the page's button, and what takes the file. */
    public Device upload(String id, String label, Accept accept) {
      unchanged();
      Upload upload = new Upload(id, label, accept);
      if (uploads.stream().anyMatch(u -> u.id().equals(id))) {
        throw new IllegalArgumentException(name + " has two uploads named " + id);
      }
      uploads.add(upload);
      return this;
    }

    /**
     * Answers the card's choice with this id: its options and what's chosen come from the card, as
     * of each read, and the page's choice comes here.
     */
    public Device choose(String id, Choose choose) {
      unchanged();
      if (!NAME.matcher(id).matches()) {
        throw new IllegalArgumentException(
            "a choice's id is lowercase letters, digits, and hyphens: " + id);
      }
      if (choosers.putIfAbsent(id, choose) != null) {
        throw new IllegalArgumentException(name + " has two choosers named " + id);
      }
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

    /** Its download with this id, if it has one. */
    public Optional<Download> download(String id) {
      return downloads.stream().filter(d -> d.id().equals(id)).findFirst();
    }

    /** Its upload with this id, if it has one. */
    public Optional<Upload> upload(String id) {
      return uploads.stream().filter(u -> u.id().equals(id)).findFirst();
    }

    /** What answers its card's choice with this id, if anything does. */
    public Optional<Choose> chooser(String id) {
      return Optional.ofNullable(choosers.get(id));
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
   * "lines": [{"label", "value", "level"}], "downloads": [{"id", "label"}], "uploads": [{"id",
   * "label"}], "choices": [{"id", "label", "options": [{"value", "label"}], "current"}]}}}, levels
   * in lowercase.
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
    List<Object> downloads = new ArrayList<>();
    for (Download download : device.downloads) {
      Map<String, Object> each = new LinkedHashMap<>();
      each.put("id", download.id());
      each.put("label", download.label());
      downloads.add(each);
    }
    shown.put("downloads", downloads);
    List<Object> uploads = new ArrayList<>();
    for (Upload upload : device.uploads) {
      Map<String, Object> each = new LinkedHashMap<>();
      each.put("id", upload.id());
      each.put("label", upload.label());
      uploads.add(each);
    }
    shown.put("uploads", uploads);
    List<Object> choices = new ArrayList<>();
    for (Choice choice : card.choices()) {
      if (!device.choosers.containsKey(choice.id())) {
        continue; // nothing would answer it
      }
      Map<String, Object> each = new LinkedHashMap<>();
      each.put("id", choice.id());
      each.put("label", choice.label());
      List<Object> options = new ArrayList<>();
      for (Option option : choice.options()) {
        Map<String, Object> shownOption = new LinkedHashMap<>();
        shownOption.put("value", option.value());
        shownOption.put("label", option.label());
        options.add(shownOption);
      }
      each.put("options", options);
      each.put("current", choice.current());
      choices.add(each);
    }
    shown.put("choices", choices);
    return shown;
  }

  private static String level(Level level) {
    return level.name().toLowerCase(Locale.ROOT);
  }
}
