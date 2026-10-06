package com.michaelgrundvig.frc.world.field;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Reads JSON strictly, into maps (in the file's order), lists, text, numbers as {@code Double},
 * {@code Boolean}, and {@link #NULL}: the little the venue reader needs, with no library to add. A
 * duplicate key, trailing text, or nesting deeper than {@link #MAX_DEPTH} is refused.
 */
final class JsonText {
  /** JSON's null, which a map or list can't hold as Java's. */
  static final Object NULL = new Object();

  /** Deeper nesting than this is refused: a field layout is five levels deep. */
  static final int MAX_DEPTH = 64;

  private final String text;
  private int at;

  private JsonText(String text) {
    this.text = text;
  }

  /**
   * The value the text holds.
   *
   * @throws IllegalArgumentException if it isn't one JSON value, saying where
   */
  static Object parse(String text) {
    JsonText parser = new JsonText(text);
    parser.skipSpace();
    Object value = parser.value(0);
    parser.skipSpace();
    if (parser.at < text.length()) {
      throw parser.error("more after the value");
    }
    return value;
  }

  private Object value(int depth) {
    if (depth > MAX_DEPTH) {
      throw error("nested deeper than " + MAX_DEPTH);
    }
    if (at >= text.length()) {
      throw error("the text ends where a value should be");
    }
    char c = text.charAt(at);
    return switch (c) {
      case '{' -> object(depth);
      case '[' -> list(depth);
      case '"' -> string();
      case 't' -> word("true", Boolean.TRUE);
      case 'f' -> word("false", Boolean.FALSE);
      case 'n' -> word("null", NULL);
      default -> {
        if (c == '-' || (c >= '0' && c <= '9')) {
          yield number();
        }
        throw error("unexpected '" + c + "'");
      }
    };
  }

  private Map<String, Object> object(int depth) {
    Map<String, Object> map = new LinkedHashMap<>();
    at++;
    skipSpace();
    if (peek('}')) {
      at++;
      return map;
    }
    while (true) {
      skipSpace();
      if (!peek('"')) {
        throw error("expected a key in quotes");
      }
      int keyAt = at;
      String key = string();
      if (map.containsKey(key)) {
        at = keyAt;
        throw error("the key \"" + key + "\" appears twice");
      }
      skipSpace();
      expect(':');
      skipSpace();
      map.put(key, value(depth + 1));
      skipSpace();
      if (peek(',')) {
        at++;
        continue;
      }
      expect('}');
      return map;
    }
  }

  private List<Object> list(int depth) {
    List<Object> list = new ArrayList<>();
    at++;
    skipSpace();
    if (peek(']')) {
      at++;
      return list;
    }
    while (true) {
      skipSpace();
      list.add(value(depth + 1));
      skipSpace();
      if (peek(',')) {
        at++;
        continue;
      }
      expect(']');
      return list;
    }
  }

  private String string() {
    at++;
    StringBuilder out = new StringBuilder();
    while (at < text.length()) {
      char c = text.charAt(at++);
      if (c == '"') {
        return out.toString();
      }
      if (c < 0x20) {
        at--;
        throw error("a control character inside a string");
      }
      if (c != '\\') {
        out.append(c);
        continue;
      }
      if (at >= text.length()) {
        break;
      }
      char escaped = text.charAt(at++);
      switch (escaped) {
        case '"', '\\', '/' -> out.append(escaped);
        case 'b' -> out.append('\b');
        case 'f' -> out.append('\f');
        case 'n' -> out.append('\n');
        case 'r' -> out.append('\r');
        case 't' -> out.append('\t');
        case 'u' -> {
          if (at + 4 > text.length()) {
            throw error("a \\u escape cut short");
          }
          try {
            out.append((char) Integer.parseInt(text.substring(at, at + 4), 16));
          } catch (NumberFormatException e) {
            throw error("a \\u escape that isn't four hex digits");
          }
          at += 4;
        }
        default -> {
          at--;
          throw error("an unknown escape \\" + escaped);
        }
      }
    }
    throw error("a string with no closing quote");
  }

  private Double number() {
    int start = at;
    if (peek('-')) {
      at++;
    }
    if (peek('0')) {
      at++;
    } else {
      digits("a digit");
    }
    if (peek('.')) {
      at++;
      digits("a digit after the decimal point");
    }
    if (peek('e') || peek('E')) {
      at++;
      if (peek('+') || peek('-')) {
        at++;
      }
      digits("a digit in the exponent");
    }
    double value = Double.parseDouble(text.substring(start, at));
    if (!Double.isFinite(value)) {
      at = start;
      throw error("a number too large for a double");
    }
    return value;
  }

  private void digits(String what) {
    int start = at;
    while (at < text.length() && Character.isDigit(text.charAt(at))) {
      at++;
    }
    if (at == start) {
      throw error("expected " + what);
    }
  }

  private Object word(String word, Object value) {
    if (!text.startsWith(word, at)) {
      throw error("unexpected '" + text.charAt(at) + "'");
    }
    at += word.length();
    return value;
  }

  private boolean peek(char c) {
    return at < text.length() && text.charAt(at) == c;
  }

  private void expect(char c) {
    if (!peek(c)) {
      throw error(at < text.length() ? "expected '" + c + "'" : "the text ends before '" + c + "'");
    }
    at++;
  }

  private void skipSpace() {
    while (at < text.length()) {
      char c = text.charAt(at);
      if (c != ' ' && c != '\t' && c != '\n' && c != '\r') {
        return;
      }
      at++;
    }
  }

  private IllegalArgumentException error(String what) {
    int line = 1;
    int column = 1;
    for (int i = 0; i < Math.min(at, text.length()); i++) {
      if (text.charAt(i) == '\n') {
        line++;
        column = 1;
      } else {
        column++;
      }
    }
    return new IllegalArgumentException(
        "not JSON: " + what + " (line " + line + ", column " + column + ")");
  }
}
