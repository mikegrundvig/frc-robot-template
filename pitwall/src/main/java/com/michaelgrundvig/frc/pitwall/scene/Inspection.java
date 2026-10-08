package com.michaelgrundvig.frc.pitwall.scene;

import java.util.ArrayList;
import java.util.List;

/**
 * What a shape on the page says when it's clicked: a title, its details as labeled lines, and the
 * actions that can be taken about it, which end the panel as buttons:
 *
 * {@snippet :
 * Inspection said =
 *     Inspection.of(line.camera + " → tag " + line.tag)
 *         .detail("Verdict", verdict.accepted() ? "accepted" : "rejected: " + verdict.reason())
 *         ...
 * return calibrate.isPresent() ? said.action(calibrate.get()) : said;
 * }
 *
 * <p>An action is one of the page's actions, by name ({@code pitwall.actions()}): it runs as it
 * would from Pit, with the same gate, and a name that isn't one isn't shown. A panel that ends in
 * something to do is what Pitwall is for.
 *
 * @param title what was clicked, in a few words
 * @param details what there is to know about it, in order
 * @param actions the names of the actions that can be taken about it
 */
public record Inspection(String title, List<Detail> details, List<String> actions) {
  /**
   * One line of what there is to know.
   *
   * @param label what it is, such as "Verdict"
   * @param value what it says, such as "rejected: gate 45.2 > 13.8"
   */
  public record Detail(String label, String value) {}

  public Inspection {
    details = List.copyOf(details);
    actions = List.copyOf(actions);
  }

  /** An inspection with a title, and nothing else yet. */
  public static Inspection of(String title) {
    return new Inspection(title, List.of(), List.of());
  }

  /** This, with a line of detail more. */
  public Inspection detail(String label, String value) {
    List<Detail> more = new ArrayList<>(details);
    more.add(new Detail(label, value));
    return new Inspection(title, more, actions);
  }

  /** This, ending in one more action, by its name on the page. */
  public Inspection action(String name) {
    List<String> more = new ArrayList<>(actions);
    more.add(name);
    return new Inspection(title, details, more);
  }
}
