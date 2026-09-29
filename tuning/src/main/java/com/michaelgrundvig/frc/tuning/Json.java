package com.michaelgrundvig.frc.tuning;

/** The tuning page's JSON, written by hand: its shape is small and fixed. */
final class Json {
  private Json() {}

  static String view(Tuning.View view) {
    StringBuilder json = new StringBuilder(4096);
    json.append("{\"name\":")
        .append(string(view.name()))
        .append(",\"enabled\":")
        .append(view.enabled())
        .append(",\"fieldConnected\":")
        .append(view.fieldConnected())
        .append(",\"values\":[");
    for (int i = 0; i < view.values().size(); i++) {
      Tuning.Value value = view.values().get(i);
      String refused = value.refused();
      json.append(i == 0 ? "" : ",")
          .append("{\"path\":")
          .append(string(value.path()))
          .append(",\"unit\":")
          .append(string(value.unit()))
          .append(",\"committed\":")
          .append(number(value.committed()))
          .append(",\"current\":")
          .append(number(value.current()))
          .append(",\"refused\":")
          .append(refused == null ? "null" : string(refused))
          .append('}');
    }
    json.append("],\"changes\":[");
    for (int i = 0; i < view.changes().size(); i++) {
      ConfigTuner.Change change = view.changes().get(i);
      json.append(i == 0 ? "" : ",")
          .append("{\"path\":")
          .append(string(change.path()))
          .append(",\"committed\":")
          .append(string(change.committed()))
          .append(",\"tuned\":")
          .append(string(change.tuned()))
          .append(",\"builderCall\":")
          .append(string(change.builderCall()))
          .append('}');
    }
    return json.append("]}").toString();
  }

  /** A JSON string: quoted, with quotes, backslashes, and control characters escaped. */
  static String string(String text) {
    StringBuilder json = new StringBuilder(text.length() + 2).append('"');
    for (int i = 0; i < text.length(); i++) {
      char c = text.charAt(i);
      switch (c) {
        case '"' -> json.append("\\\"");
        case '\\' -> json.append("\\\\");
        case '\n' -> json.append("\\n");
        default -> {
          if (c < 0x20) {
            json.append(String.format("\\u%04x", (int) c));
          } else {
            json.append(c);
          }
        }
      }
    }
    return json.append('"').toString();
  }

  /** JSON has no NaN or infinity; configuration values are always finite, but stay safe. */
  private static String number(double value) {
    return Double.isFinite(value) ? Double.toString(value) : "null";
  }
}
