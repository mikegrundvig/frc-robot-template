package com.michaelgrundvig.frc.pitwall.docs;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import org.jspecify.annotations.Nullable;

/**
 * A doc comment, read for display on a page that shows plain text: its main text, the main text's
 * first sentence, and each {@code @param}.
 *
 * <p>The markup becomes plain text. {@code {@code x}} becomes {@code `x`}; {@code {@link A#b}}
 * becomes {@code b}, or the link's label when it has one. {@code <p>} starts a paragraph, after a
 * blank line, as do lists and the other HTML blocks ({@code <ul>}, {@code <pre>}...); {@code <li>}
 * starts a list item, on a line of its own beginning {@code "- "}. Other HTML is dropped, its text
 * kept, and entities such as {@code &lt;} become their characters. Within a paragraph or an item,
 * line breaks and runs of spaces become one space, as in a browser.
 *
 * @param summary the main text's first sentence: up to the first period followed by a space, as the
 *     javadoc tool finds it, within the first paragraph
 * @param text the main text, the summary included
 * @param params each {@code @param}'s text by the name it documents, in the order written
 */
record DocComment(String summary, String text, Map<String, String> params) {
  // Mark a paragraph break and a list item's start until the whitespace is laid out.
  private static final char PARAGRAPH = '\u0001';
  private static final char ITEM = '\u0002';

  /** HTML elements that start a paragraph, opening or closing. */
  private static final Set<String> BLOCKS =
      Set.of("p", "ul", "ol", "dl", "pre", "blockquote", "table", "div");

  private static final Map<String, String> ENTITIES =
      Map.of("lt", "<", "gt", ">", "amp", "&", "quot", "\"", "apos", "'", "nbsp", " ");

  /** Reads a doc comment, as the compiler gives it: without the comment's own stars. */
  static DocComment parse(String comment) {
    List<String> sections = sections(comment);
    String text = plain(sections.get(0));
    Map<String, String> params = new LinkedHashMap<>();
    for (String tag : sections.subList(1, sections.size())) {
      String[] words = tag.strip().split("\\s+", 3);
      // @param <T> documents a type parameter, not a value.
      if (words[0].equals("@param") && words.length > 1 && !words[1].startsWith("<")) {
        params.putIfAbsent(words[1], words.length > 2 ? plain(words[2]) : "");
      }
    }
    return new DocComment(summary(text), text, params);
  }

  /**
   * The main text, then each block tag ({@code @param}, {@code @return}...): a block tag starts
   * with {@code @} at the start of a line, unless it's inside an inline tag such as {@code {@code
   * ...}}.
   */
  private static List<String> sections(String comment) {
    List<String> sections = new ArrayList<>();
    int start = 0;
    int depth = 0; // braces open inside an inline tag
    boolean lineStart = true;
    for (int i = 0; i < comment.length(); i++) {
      char c = comment.charAt(i);
      if (depth > 0) {
        depth += c == '{' ? 1 : c == '}' ? -1 : 0;
      } else if (c == '{' && comment.startsWith("{@", i)) {
        depth = 1;
      } else if (c == '@' && lineStart) {
        sections.add(comment.substring(start, i));
        start = i;
      }
      lineStart = c == '\n' || (lineStart && Character.isWhitespace(c));
    }
    sections.add(comment.substring(start));
    return sections;
  }

  /** Text with its markup made plain, and its whitespace laid out. */
  static String plain(String marked) {
    StringBuilder out = new StringBuilder();
    int i = 0;
    while (i < marked.length()) {
      char c = marked.charAt(i);
      String tag = c == '<' ? htmlTag(marked, i) : null;
      String entity = c == '&' ? entity(marked, i) : null;
      if (c == '{' && marked.startsWith("{@", i)) {
        int end = closingBrace(marked, i);
        out.append(inlineTag(marked.substring(i + 2, end)));
        i = Math.min(end + 1, marked.length());
      } else if (marked.startsWith("<!--", i)) {
        int end = marked.indexOf("-->", i);
        i = end < 0 ? marked.length() : end + 3;
      } else if (tag != null) {
        out.append(left(tag));
        i = marked.indexOf('>', i) + 1;
      } else if (entity != null) {
        out.append(entity);
        i = marked.indexOf(';', i) + 1;
      } else {
        out.append(c);
        i++;
      }
    }
    return laidOut(out.toString());
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

  /** An inline tag as plain text, from what's inside its braces: {@code "code x"}. */
  private static String inlineTag(String inside) {
    String[] parts = inside.split("\\s", 2);
    String content = parts.length > 1 ? parts[1].strip() : "";
    return switch (parts[0]) {
      case "code" -> content.isEmpty() ? "" : "`" + content + "`";
      case "link", "linkplain" -> link(content);
      case "inheritDoc", "docRoot" -> "";
      default -> content; // {@literal x}, {@value X}, {@index x}...: their text
    };
  }

  /** A link's label, or what it names when it has none: {@code b} for {@code A#b}. */
  private static String link(String content) {
    // The reference ends at the first space outside a method's parentheses.
    int depth = 0;
    int end = 0;
    while (end < content.length() && (depth > 0 || !Character.isWhitespace(content.charAt(end)))) {
      char c = content.charAt(end++);
      depth += c == '(' ? 1 : c == ')' ? -1 : 0;
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
    // A type: drop its package, the lowercase names in front (frc.robot.lib.config.SwerveConfig).
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
  private static String left(String tag) {
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
   * The character an HTML entity starting at {@code start} stands for ({@code <} for {@code &lt;}),
   * or null if a {@code &} there doesn't start one.
   */
  private static @Nullable String entity(String text, int start) {
    int end = text.indexOf(';', start);
    if (end < 0 || end - start > 10) {
      return null;
    }
    String name = text.substring(start + 1, end);
    try {
      if (name.startsWith("#x") || name.startsWith("#X")) {
        return Character.toString(Integer.parseInt(name.substring(2), 16));
      }
      if (name.startsWith("#")) {
        return Character.toString(Integer.parseInt(name.substring(1)));
      }
    } catch (IllegalArgumentException notANumber) {
      return null;
    }
    return ENTITIES.get(name);
  }

  /**
   * The text with each paragraph's and item's whitespace collapsed, a blank line before each
   * paragraph, and a line of its own for each item.
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

  /** The first sentence of the first paragraph, as the javadoc tool finds it. */
  private static String summary(String text) {
    String first = text.lines().findFirst().orElse("");
    int end = first.indexOf(". ");
    return end < 0 ? first : first.substring(0, end + 1);
  }
}
