package frc.robot;

import io.avaje.jsonb.Jsonb;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.telemetry.MockTelemetryBackend;

/**
 * Checks an Elastic layout against what the robot publishes: every widget must read a value the
 * robot code really publishes, of a kind the widget can show. A widget that fails shows up blank in
 * the queue line, which looks the same as "not ready yet".
 */
final class ElasticLayoutCheck {
  /** Where the robot's telemetry appears in NetworkTables: WPILib's RobotBase puts it there. */
  static final String TELEMETRY_ROOT = "/Telemetry";

  private static final Set<String> SHOWS_TRUE_OR_FALSE =
      Set.of("Boolean Box", "Toggle Switch", "Toggle Button");
  private static final Set<String> SHOWS_A_NUMBER =
      Set.of("Match Time", "Voltage View", "Number Slider", "Number Bar", "Radial Gauge", "Graph");

  /** The values Elastic's Alerts widget reads under its topic. */
  private static final List<String> ALERT_LISTS = List.of("errors", "warnings", "infos");

  private ElasticLayoutCheck() {}

  /** One widget: its tab, title, kind, and the topic it reads. */
  record Widget(String tab, String title, String type, String topic) {
    @Override
    public String toString() {
      return tab + ", \"" + title + "\" (" + type + ")";
    }
  }

  /** Every widget in a layout, on every tab, including those inside list layouts. */
  @SuppressWarnings("unchecked")
  static List<Widget> widgets(String layoutJson) {
    Map<String, Object> layout =
        (Map<String, Object>) Jsonb.instance().type(Object.class).fromJson(layoutJson);
    List<Widget> widgets = new ArrayList<>();
    Object tabs = Objects.requireNonNull(layout.get("tabs"), "an Elastic layout lists its tabs");
    for (Object tabValue : (List<Object>) tabs) {
      Map<String, Object> tab = (Map<String, Object>) tabValue;
      Map<String, Object> grid = (Map<String, Object>) tab.get("grid_layout");
      if (grid == null) {
        continue;
      }
      for (String group : List.of("containers", "layouts")) {
        Object items = grid.get(group);
        if (items != null) {
          collect(String.valueOf(tab.get("name")), (List<Object>) items, widgets);
        }
      }
    }
    return widgets;
  }

  @SuppressWarnings("unchecked")
  private static void collect(String tab, List<Object> items, List<Widget> widgets) {
    for (Object itemValue : items) {
      Map<String, Object> item = (Map<String, Object>) itemValue;
      Object children = item.get("children");
      if (children != null) {
        collect(tab, (List<Object>) children, widgets);
        continue;
      }
      Map<String, Object> properties = (Map<String, Object>) item.get("properties");
      Object topic = properties == null ? null : properties.get("topic");
      widgets.add(
          new Widget(
              tab,
              String.valueOf(item.get("title")),
              String.valueOf(item.get("type")),
              topic == null ? "" : String.valueOf(topic)));
    }
  }

  /** The last value published to each telemetry path, as the mock backend recorded them. */
  static Map<String, Object> published(MockTelemetryBackend telemetry) {
    Map<String, Object> last = new HashMap<>();
    for (MockTelemetryBackend.Action action : telemetry.getActions()) {
      last.put(action.path(), action.value());
    }
    return last;
  }

  /**
   * What's wrong with the widgets, given the last value published to each telemetry path: one line
   * per problem, naming the tab and the widget.
   */
  static List<String> problems(List<Widget> widgets, Map<String, Object> published) {
    List<String> problems = new ArrayList<>();
    for (Widget widget : widgets) {
      String topic = widget.topic();
      if (!topic.startsWith(TELEMETRY_ROOT + "/")) {
        problems.add(
            widget
                + ": "
                + (topic.isEmpty() ? "reads no topic" : topic + " is outside the robot's telemetry")
                + " (publish it through Dashboard, under "
                + TELEMETRY_ROOT
                + ")");
        continue;
      }
      String path = topic.substring(TELEMETRY_ROOT.length());
      if (widget.type().equals("Alerts")) {
        for (String list : ALERT_LISTS) {
          check(
              widget,
              topic + "/" + list,
              published.get(path + "/" + list),
              String[].class,
              problems);
        }
      } else if (SHOWS_TRUE_OR_FALSE.contains(widget.type())) {
        check(widget, topic, published.get(path), Boolean.class, problems);
      } else if (SHOWS_A_NUMBER.contains(widget.type())) {
        check(widget, topic, published.get(path), Number.class, problems);
      } else {
        check(widget, topic, published.get(path), Object.class, problems);
      }
    }
    return problems;
  }

  private static void check(
      Widget widget,
      String topic,
      @Nullable Object value,
      Class<?> expected,
      List<String> problems) {
    if (value == null) {
      problems.add(widget + ": " + topic + " is never published");
    } else if (!expected.isInstance(value)) {
      problems.add(
          widget
              + ": "
              + topic
              + " is "
              + kind(value)
              + ", not "
              + kind(expected)
              + " as it shows");
    }
  }

  private static String kind(Object value) {
    if (value instanceof Boolean) {
      return "true or false";
    }
    if (value instanceof Number) {
      return "a number";
    }
    if (value instanceof String[]) {
      return "a list of text";
    }
    if (value instanceof MockTelemetryBackend.LogStringValue) {
      return "text";
    }
    return value.getClass().getSimpleName();
  }

  private static String kind(Class<?> expected) {
    if (expected == Boolean.class) {
      return "true or false";
    }
    if (expected == Number.class) {
      return "a number";
    }
    return "a list of text";
  }
}
