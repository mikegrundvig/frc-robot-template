package com.michaelgrundvig.frc.pitwall.docs;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.jspecify.annotations.Nullable;

/**
 * A doc comment, read for display on a page that shows plain text: its main text, the main text's
 * first sentence, and each {@code @param}.
 *
 * <p>The markup becomes plain text. {@code {@code x}} becomes {@code `x`} (more backticks around
 * code that holds one); {@code {@link A#b}} becomes {@code b}, or the link's label when it has one.
 * {@code {@value A#B}} becomes {@code B}, the constant's name, deliberately: its value may live in
 * another class, and each record's documentation is read from that record alone, which keeps the
 * processor isolating. {@code <p>} starts a paragraph, after a blank line, as do lists and the
 * other HTML blocks ({@code <ul>}, {@code <pre>}...); {@code <li>} starts a list item, on a line of
 * its own beginning {@code "- "}. Other HTML is dropped, its text kept. Within a paragraph or an
 * item, line breaks and runs of spaces become one space, in {@code <pre>} too.
 *
 * <p>Entities become their characters: HTML's own ({@code &lt; &gt; &amp; &quot; &apos; &nbsp;}),
 * any by number ({@code &#176;}), and the units and symbols a configuration writes ({@code &deg;
 * &times; &plusmn; &micro; &Omega; &mdash; &ndash; &rarr; &le; &ge;}). A tab, line feed, or
 * carriage return becomes a space. Other control characters are dropped, written as entities or
 * not; a name or number that isn't a character stays as written.
 *
 * <p>A Markdown comment ({@code ///}) reads the same way, and so does this much of its own markup:
 * a blank line starts a paragraph; a line starting {@code "- "}, {@code "* "}, {@code "+ "}, or a
 * number and {@code "."} or {@code ")"} starts a list item; a code span stays code, paired within
 * its paragraph, and a fenced code block becomes one, a paragraph of its own; a backslash before
 * punctuation keeps it as written; {@code [label][ref]} and {@code [label](url)} become their
 * label, and {@code [ref]} becomes what {@code ref} names when it's a Java reference (otherwise the
 * brackets stay: {@code [0, 1]}). Emphasis, headings, and indented code stay as written.
 *
 * @param summary the main text's first sentence: up to the first period followed by a space,
 *     outside code, as the javadoc tool finds it, within the first paragraph
 * @param text the main text, the summary included
 * @param params each {@code @param}'s text by the name it documents, in the order written
 */
record DocComment(String summary, String text, Map<String, String> params) {
  // Marks, until the text is laid out, of a paragraph break, a list item's start, and a code span's
  // ends. A comment's own text never holds them: its control characters are dropped.
  private static final char PARAGRAPH = '\u0001';
  private static final char ITEM = '\u0002';
  private static final char CODE = '\u0003';
  private static final char END_CODE = '\u0004';

  /** HTML elements that start a paragraph, opening or closing. */
  private static final Set<String> BLOCKS =
      Set.of("p", "ul", "ol", "dl", "pre", "blockquote", "table", "div");

  private static final Map<String, String> ENTITIES =
      Map.ofEntries(
          Map.entry("lt", "<"),
          Map.entry("gt", ">"),
          Map.entry("amp", "&"),
          Map.entry("quot", "\""),
          Map.entry("apos", "'"),
          Map.entry("nbsp", " "),
          Map.entry("deg", "°"),
          Map.entry("times", "×"),
          Map.entry("plusmn", "±"),
          Map.entry("micro", "µ"),
          Map.entry("Omega", "Ω"),
          Map.entry("mdash", "—"),
          Map.entry("ndash", "–"),
          Map.entry("rarr", "→"),
          Map.entry("le", "≤"),
          Map.entry("ge", "≥"));

  /** The control characters that are whitespace, laid out as spaces. */
  private static final String WHITESPACE = "\t\n\u000B\f\r";

  /** What Markdown lets a backslash keep as written: ASCII punctuation. */
  private static final String PUNCTUATION = "!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~";

  /**
   * A Java reference, as a Markdown link names one: {@code SwerveConfig}, {@code #steerKp}, {@code
   * Limits#between(double, double)}.
   */
  private static final Pattern REFERENCE =
      Pattern.compile("#?[A-Za-z_$][\\w.$]*(#[\\w$]+(\\([^)]*\\))?)?");

  /** A Markdown list item's start: {@code "- "}, {@code "* "}, {@code "+ "}, or {@code "2. "}. */
  private static final Pattern BULLET = Pattern.compile("[ \t]*(?:[-*+]|(\\d{1,9})[.)])[ \t]+");

  /** A Markdown code fence, opening or closing a block: three or more backticks or tildes. */
  private static final Pattern FENCE = Pattern.compile("[ \t]{0,3}(`{3,}|~{3,})");

  /** A blank line, which ends a paragraph. */
  private static final Pattern BLANK_LINE = Pattern.compile("\n[ \t]*(?:\n|$)");

  /**
   * Markup that holds text as it is: a code span or a fenced block.
   *
   * @param from where its text starts
   * @param to where its text stops
   * @param end where the markup ends
   */
  private record Verbatim(int from, int to, int end) {}

  /**
   * Reads a doc comment, as the compiler gives it: without the comment's own stars or slashes.
   *
   * @param markdown whether it's a Markdown comment, written with {@code ///}
   */
  static DocComment parse(String comment, boolean markdown) {
    List<String> sections = sections(comment, markdown);
    String text = laidOut(marked(sections.get(0), markdown, true));
    Map<String, String> params = new LinkedHashMap<>();
    for (String tag : sections.subList(1, sections.size())) {
      String[] words = tag.strip().split("\\s+", 3);
      // @param <T> documents a type parameter, not a value.
      if (words[0].equals("@param") && words.length > 1 && !words[1].startsWith("<")) {
        String param = words.length > 2 ? laidOut(marked(words[2], markdown, false)) : "";
        params.putIfAbsent(words[1], shown(param));
      }
    }
    return new DocComment(shown(summary(text)), shown(text), params);
  }

  /** Text with its markup made plain, and its whitespace laid out. */
  static String plain(String text, boolean markdown) {
    return shown(laidOut(marked(text, markdown, true)));
  }

  /**
   * The main text, then each block tag ({@code @param}, {@code @return}...): a block tag starts
   * with {@code @} at the start of a line, unless it's inside an inline tag such as {@code {@code
   * ...}}, or in Markdown, inside code or after a backslash.
   */
  private static List<String> sections(String comment, boolean markdown) {
    List<String> sections = new ArrayList<>();
    int start = 0;
    boolean lineStart = true;
    int i = 0;
    while (i < comment.length()) {
      char c = comment.charAt(i);
      int past = past(comment, i, markdown);
      if (past > i) {
        i = past;
        lineStart = comment.charAt(i - 1) == '\n';
        continue;
      }
      if (c == '@' && lineStart) {
        sections.add(comment.substring(start, i));
        start = i;
      }
      lineStart = c == '\n' || (lineStart && Character.isWhitespace(c));
      i++;
    }
    sections.add(comment.substring(start));
    return sections;
  }

  /**
   * Where markup starting at {@code i} that can hold an {@code @} ends: an inline tag, or in
   * Markdown a fenced block, a code span (or backticks nothing closes), or an escape. {@code i}
   * itself when none starts there.
   */
  private static int past(String text, int i, boolean markdown) {
    char c = text.charAt(i);
    if (c == '{' && text.startsWith("{@", i)) {
      return Math.min(closingBrace(text, i) + 1, text.length());
    }
    if (!markdown) {
      return i;
    }
    Verbatim fence = i == 0 || text.charAt(i - 1) == '\n' ? fence(text, i) : null;
    if (fence != null) {
      return fence.end();
    }
    if (c == '`') {
      Verbatim code = codeSpan(text, i);
      return code != null ? code.end() : i + run(text, i, '`');
    }
    return isEscape(text, i) ? i + 2 : i;
  }

  /**
   * Text with its markup turned into plain text and marks, for {@link #laidOut}.
   *
   * @param startsLine whether the text starts a line, where Markdown's lists and blocks can start
   */
  private static String marked(String text, boolean markdown, boolean startsLine) {
    StringBuilder out = new StringBuilder();
    boolean afterBlank = startsLine; // Markdown: any number then starts a list; else 1, or in one
    boolean inList = false;
    int i = 0;
    while (i < text.length()) {
      boolean atLine = i == 0 ? startsLine : text.charAt(i - 1) == '\n';
      if (markdown && atLine) {
        int lineEnd = text.indexOf('\n', i);
        String line = text.substring(i, lineEnd < 0 ? text.length() : lineEnd);
        Verbatim fence = fence(text, i);
        Matcher item = BULLET.matcher(line);
        if (line.isBlank()) {
          out.append(PARAGRAPH); // its own whitespace is laid out away
          afterBlank = true;
          inList = false;
        } else if (fence != null) {
          out.append(PARAGRAPH).append(code(text.substring(fence.from(), fence.to())));
          out.append(PARAGRAPH);
          i = fence.end();
          continue;
        } else if (item.lookingAt()
            && (item.group(1) == null || item.group(1).equals("1") || afterBlank || inList)) {
          out.append(ITEM);
          i += item.end();
          afterBlank = false;
          inList = true;
          continue;
        } else {
          afterBlank = false;
        }
      }
      i = markOne(text, i, markdown, out);
    }
    return out.toString();
  }

  /** Marks what starts at {@code i}: an inline tag, a code span, a tag, a character... */
  private static int markOne(String text, int i, boolean markdown, StringBuilder out) {
    char c = text.charAt(i);
    Verbatim code = markdown && c == '`' ? codeSpan(text, i) : null;
    String tag = c == '<' ? htmlTag(text, i) : null;
    String entity = c == '&' ? entity(text, i) : null;
    if (markdown && isEscape(text, i)) {
      out.append(text.charAt(i + 1));
      return i + 2;
    }
    if (code != null) {
      out.append(code(text.substring(code.from(), code.to())));
      return code.end();
    }
    if (markdown && c == '`') {
      int run = run(text, i, '`'); // nothing closes it, so it's text
      out.append(text, i, i + run);
      return i + run;
    }
    if (markdown && c == '[') {
      return markdownLink(text, i, out);
    }
    if (c == '{' && text.startsWith("{@", i)) {
      int end = closingBrace(text, i);
      out.append(inlineTag(text.substring(i + 2, end)));
      return Math.min(end + 1, text.length());
    }
    if (text.startsWith("<!--", i)) {
      int end = text.indexOf("-->", i);
      return end < 0 ? text.length() : end + 3;
    }
    if (tag != null) {
      out.append(whatTagLeaves(tag));
      return text.indexOf('>', i) + 1;
    }
    if (entity != null) {
      out.append(entity);
      return text.indexOf(';', i) + 1;
    }
    if (!isControl(c)) {
      out.append(c);
    }
    return i + 1;
  }

  /**
   * Writes the Markdown link starting at {@code start} as its text, and returns where it ends:
   * {@code [label][ref]} and {@code [label](url)} as their label, and {@code [ref]} as {@code
   * {@link ref}} names it, when the reference is a Java one. Anything else is no link: its bracket
   * is written as it is.
   */
  private static int markdownLink(String text, int start, StringBuilder out) {
    int close = closingBracket(text, start);
    if (close > start) {
      String label = text.substring(start + 1, close);
      int next = close + 1;
      int refClose = text.startsWith("[", next) ? text.indexOf(']', next) : -1;
      int urlClose = text.startsWith("(", next) ? text.indexOf(')', next) : -1;
      if (refClose > 0) {
        String ref = text.substring(next + 1, refClose);
        if (isReference(ref.isEmpty() ? label : ref)) {
          out.append(marked(label, true, false));
          return refClose + 1;
        }
      } else if (urlClose > 0 && text.substring(next, urlClose).indexOf('\n') < 0) {
        out.append(marked(label, true, false));
        return urlClose + 1;
      }
      if (isReference(label)) {
        out.append(link(label));
        return next;
      }
    }
    out.append('[');
    return start + 1;
  }

  private static boolean isReference(String text) {
    return REFERENCE.matcher(text).matches();
  }

  /** The {@code ]} that closes the {@code [} at {@code start}, within its paragraph; -1 if none. */
  private static int closingBracket(String text, int start) {
    int depth = 0;
    int limit = paragraphEnd(text, start);
    for (int i = start; i < limit; i++) {
      char c = text.charAt(i);
      if (c == '[') {
        depth++;
      } else if (c == ']' && --depth == 0) {
        return i;
      }
    }
    return -1;
  }

  /**
   * The code span opening at {@code start}: a run of backticks, closed by the next run of as many
   * within the paragraph. Null when none closes it.
   */
  private static @Nullable Verbatim codeSpan(String text, int start) {
    int opening = run(text, start, '`');
    int limit = paragraphEnd(text, start);
    int i = start + opening;
    while (i < limit) {
      if (text.charAt(i) != '`') {
        i++;
        continue;
      }
      int closing = run(text, i, '`');
      if (closing == opening) {
        return new Verbatim(start + opening, i, i + closing);
      }
      i += closing;
    }
    return null;
  }

  /**
   * The fenced code block whose opening fence starts the line at {@code start}, closed by a line of
   * the same fence, at least as long, or by the text's end. Null when the line opens none.
   */
  private static @Nullable Verbatim fence(String text, int start) {
    Matcher opening = FENCE.matcher(text).region(start, text.length());
    if (!opening.lookingAt()) {
      return null;
    }
    String fence = opening.group(1);
    int from = nextLine(text, start);
    for (int line = from; line < text.length(); line = nextLine(text, line)) {
      Matcher closing = FENCE.matcher(text).region(line, text.length());
      if (closing.lookingAt()
          && closing.group(1).charAt(0) == fence.charAt(0)
          && closing.group(1).length() >= fence.length()
          && text.substring(closing.end(), lineEnd(text, line)).isBlank()) {
        return new Verbatim(from, line, nextLine(text, line));
      }
    }
    return new Verbatim(from, text.length(), text.length());
  }

  private static int lineEnd(String text, int from) {
    int end = text.indexOf('\n', from);
    return end < 0 ? text.length() : end;
  }

  private static int nextLine(String text, int from) {
    return Math.min(lineEnd(text, from) + 1, text.length());
  }

  /** Where the paragraph holding {@code from} ends: at the next blank line, or the text's end. */
  private static int paragraphEnd(String text, int from) {
    Matcher blank = BLANK_LINE.matcher(text);
    return blank.find(from) ? blank.start() : text.length();
  }

  /** How many of {@code c} there are in a row from {@code start}. */
  private static int run(String text, int start, char c) {
    int end = start;
    while (end < text.length() && text.charAt(end) == c) {
      end++;
    }
    return end - start;
  }

  /** Whether a Markdown backslash at {@code i} keeps the punctuation after it as written. */
  private static boolean isEscape(String text, int i) {
    return text.charAt(i) == '\\'
        && i + 1 < text.length()
        && PUNCTUATION.indexOf(text.charAt(i + 1)) >= 0;
  }

  /** Code, marked, without its control characters. */
  private static String code(String code) {
    return CODE + withoutControls(code) + END_CODE;
  }

  /** Where the inline tag starting at {@code start} ends: its closing brace, or the text's end. */
  private static int closingBrace(String text, int start) {
    int depth = 0;
    for (int i = start; i < text.length(); i++) {
      char c = text.charAt(i);
      if (c == '{') {
        depth++;
      } else if (c == '}' && --depth == 0) {
        return i;
      }
    }
    return text.length();
  }

  /** An inline tag, marked, from what's inside its braces: {@code "code x"}. */
  private static String inlineTag(String inside) {
    String[] parts = inside.split("\\s", 2);
    String content = parts.length > 1 ? parts[1].strip() : "";
    return switch (parts[0]) {
      case "code" -> code(content);
      case "link", "linkplain", "value" -> withoutControls(link(content));
      case "inheritDoc", "docRoot" -> "";
      default -> withoutControls(content); // {@literal x}, {@index x}...: their text
    };
  }

  /** A link's label, or what it names when it has none: {@code b} for {@code A#b}. */
  private static String link(String content) {
    // The reference ends at the first space outside a method's parentheses.
    int depth = 0;
    int end = 0;
    while (end < content.length() && (depth > 0 || !Character.isWhitespace(content.charAt(end)))) {
      char c = content.charAt(end++);
      if (c == '(') {
        depth++;
      } else if (c == ')') {
        depth--;
      }
    }
    String label = content.substring(end).strip();
    if (!label.isEmpty()) {
      return label;
    }
    String reference = content.substring(0, end);
    int member = reference.indexOf('#');
    if (member >= 0) {
      return reference.substring(member + 1);
    }
    // A type: drop its package, the lowercase names in front
    // (com.michaelgrundvig.frc.world.robot.DrivetrainGeometry).
    String[] names = reference.split("\\.");
    int first = 0;
    while (first < names.length - 1
        && !names[first].isEmpty()
        && Character.isLowerCase(names[first].charAt(0))) {
      first++;
    }
    return String.join(".", List.of(names).subList(first, names.length));
  }

  /** What an HTML tag leaves in the text: a paragraph break, an item's start, or nothing. */
  private static String whatTagLeaves(String tag) {
    if (BLOCKS.contains(tag)) {
      return String.valueOf(PARAGRAPH);
    }
    if (tag.equals("li")) {
      return String.valueOf(ITEM);
    }
    return tag.equals("br") ? " " : "";
  }

  /**
   * The lowercase name of the HTML tag starting at {@code start} ({@code "p"} for {@code <p>} and
   * {@code </p>}), or null if a {@code <} there doesn't start one: {@code a < b}.
   */
  private static @Nullable String htmlTag(String text, int start) {
    int end = text.indexOf('>', start);
    if (end < 0) {
      return null;
    }
    String inside = text.substring(start + 1, end);
    int from = inside.startsWith("/") ? 1 : 0;
    int to = from;
    while (to < inside.length() && Character.isLetterOrDigit(inside.charAt(to))) {
      to++;
    }
    if (to == from || !Character.isLetter(inside.charAt(from))) {
      return null;
    }
    if (to < inside.length()
        && !Character.isWhitespace(inside.charAt(to))
        && inside.charAt(to) != '/') {
      return null;
    }
    return inside.substring(from, to).toLowerCase(Locale.ROOT);
  }

  /**
   * What the HTML entity starting at {@code start} stands for ({@code <} for {@code &lt;}, a space
   * for a tab, nothing for another control character), or null if a {@code &} there doesn't start
   * one this decodes.
   */
  private static @Nullable String entity(String text, int start) {
    int end = text.indexOf(';', start);
    if (end < 0 || end - start > 10) {
      return null;
    }
    String name = text.substring(start + 1, end);
    if (!name.startsWith("#")) {
      return ENTITIES.get(name);
    }
    int codePoint;
    try {
      codePoint =
          name.startsWith("#x") || name.startsWith("#X")
              ? Integer.parseInt(name.substring(2), 16)
              : Integer.parseInt(name.substring(1));
    } catch (NumberFormatException notANumber) {
      return null;
    }
    if (!Character.isValidCodePoint(codePoint)
        || (codePoint >= Character.MIN_SURROGATE && codePoint <= Character.MAX_SURROGATE)) {
      return null; // not a character
    }
    if (WHITESPACE.indexOf(codePoint) >= 0) {
      return " ";
    }
    return isControl(codePoint) ? "" : Character.toString(codePoint);
  }

  /** Whether a character is a control character other than whitespace: dropped. */
  private static boolean isControl(int c) {
    return Character.isISOControl(c) && WHITESPACE.indexOf(c) < 0;
  }

  private static String withoutControls(String text) {
    StringBuilder kept = new StringBuilder(text.length());
    text.chars().filter(c -> !isControl(c)).forEach(c -> kept.append((char) c));
    return kept.toString();
  }

  /**
   * The text with each paragraph's and item's whitespace collapsed, a blank line before each
   * paragraph, and a line of its own for each item. Code stays marked.
   */
  private static String laidOut(String marked) {
    StringBuilder text = new StringBuilder();
    StringBuilder words = new StringBuilder();
    char block = PARAGRAPH;
    for (char c : (marked + PARAGRAPH).toCharArray()) {
      if (c != PARAGRAPH && c != ITEM) {
        words.append(c);
        continue;
      }
      String flat = words.toString().strip().replaceAll("\\s+", " ");
      if (!flat.isEmpty()) {
        if (!text.isEmpty()) {
          text.append(block == ITEM ? "\n" : "\n\n");
        }
        text.append(block == ITEM ? "- " : "").append(flat);
      }
      words.setLength(0);
      block = c;
    }
    return text.toString();
  }

  /**
   * The first sentence of the laid-out text's first paragraph, as the javadoc tool finds it: up to
   * the first period followed by a space, outside code.
   */
  private static String summary(String laidOut) {
    String first = laidOut.lines().findFirst().orElse("");
    boolean inCode = false;
    for (int i = 0; i + 1 < first.length(); i++) {
      char c = first.charAt(i);
      if (c == CODE || c == END_CODE) {
        inCode = c == CODE;
      } else if (c == '.' && !inCode && first.charAt(i + 1) == ' ') {
        return first.substring(0, i + 1);
      }
    }
    return first;
  }

  /**
   * The laid-out text as the page reads it: each code span in backticks, more of them around code
   * that holds one.
   */
  private static String shown(String laidOut) {
    StringBuilder out = new StringBuilder();
    int i = 0;
    while (i < laidOut.length()) {
      int open = laidOut.indexOf(CODE, i);
      if (open < 0) {
        out.append(laidOut, i, laidOut.length());
        break;
      }
      int close = laidOut.indexOf(END_CODE, open);
      if (close < 0) {
        close = laidOut.length();
      }
      out.append(laidOut, i, open);
      String code = laidOut.substring(open + 1, close).strip();
      if (!code.isEmpty()) {
        int longest = 0;
        for (int at = code.indexOf('`'); at >= 0; at = code.indexOf('`', at + 1)) {
          longest = Math.max(longest, run(code, at, '`'));
        }
        String ticks = "`".repeat(longest + 1);
        String pad = code.startsWith("`") || code.endsWith("`") ? " " : "";
        out.append(ticks).append(pad).append(code).append(pad).append(ticks);
      }
      i = close + 1;
    }
    return out.toString();
  }
}
