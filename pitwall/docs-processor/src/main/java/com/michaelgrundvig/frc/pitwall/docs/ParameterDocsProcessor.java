package com.michaelgrundvig.frc.pitwall.docs;

import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.RoundEnvironment;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.RecordComponentElement;
import javax.lang.model.element.TypeElement;
import javax.lang.model.util.Elements;
import javax.tools.Diagnostic;
import javax.tools.FileObject;
import javax.tools.StandardLocation;

/**
 * Writes each configuration record's Javadoc into the compiled program, for Pitwall's page to show
 * beside its values: the record's main text, its first sentence, and what each component means (its
 * {@code @param}), with the markup made plain ({@link DocComment}). Each record gets {@code
 * META-INF/pitwall/docs/<binary name>.json}, which Pitwall's {@code ParameterDocs} reads:
 *
 * <pre>
 * {
 *   "summary": "Control gains.",
 *   "text": "Control gains.\n\nAll are tunable on a running robot.",
 *   "params": {
 *     "steerKp": "volts per radian of error",
 *     "steerKd": "volts per radian per second"
 *   }
 * }
 * </pre>
 *
 * <p>Which records: those marked by the annotation the compiler option {@code
 * -Apitwall.docs.records} names ({@code com.michaelgrundvig.frc.world.config.ConfigRecord}), nested
 * records included; without the option, every record. The annotation is named as text, so this
 * never depends on robot code; a name the compiler can't find fails the compile, saying so. A
 * record without a doc comment gets no file, and a component without a {@code @param} no entry:
 * Pitwall shows those as missing.
 *
 * <p>Each file comes from one record and nothing else, so Gradle runs this as an isolating
 * processor (META-INF/gradle/incremental.annotation.processors): changing one record rewrites only
 * that record's files.
 */
public final class ParameterDocsProcessor extends AbstractProcessor {
  /** The compiler option naming the annotation that marks the records to document. */
  static final String RECORDS_OPTION = "pitwall.docs.records";

  /** Where each record's documentation goes, as its binary name and ".json"; Pitwall reads it. */
  static final String FOLDER = "META-INF/pitwall/docs/";

  private final Set<String> written = new HashSet<>();

  /** Whether the option was found to name no annotation, and said so, already. */
  private boolean saidWrongName;

  @Override
  public Set<String> getSupportedOptions() {
    return Set.of(RECORDS_OPTION);
  }

  /**
   * Every annotation, whatever the option names, so the compiler runs this on every compile. Run
   * only for the marking annotation, it wouldn't run on a compile with nothing marked (Gradle
   * recompiling one changed file, say), and the compiler would then warn that no processor knows
   * the option, which {@code -Werror} turns into a failure that points the wrong way.
   */
  @Override
  public Set<String> getSupportedAnnotationTypes() {
    return Set.of("*");
  }

  @Override
  public SourceVersion getSupportedSourceVersion() {
    return SourceVersion.latestSupported();
  }

  @Override
  public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment round) {
    List<TypeElement> records = new ArrayList<>();
    String option = processingEnv.getOptions().get(RECORDS_OPTION);
    if (option == null || option.isBlank()) {
      for (Element element : round.getRootElements()) {
        addRecords(element, records);
      }
    } else {
      TypeElement marker = processingEnv.getElementUtils().getTypeElement(option.strip());
      if (marker == null || marker.getKind() != ElementKind.ANNOTATION_TYPE) {
        sayWrongName(
            option.strip(),
            marker == null
                ? "names nothing the compiler can find"
                : "names a "
                    + marker.getKind().toString().toLowerCase(Locale.ROOT)
                    + ", not an annotation");
        return false;
      }
      for (Element marked : round.getElementsAnnotatedWith(marker)) {
        if (marked instanceof TypeElement type && type.getKind() == ElementKind.RECORD) {
          records.add(type);
        }
      }
    }
    for (TypeElement record : records) {
      write(record);
    }
    return false; // every annotation is still the other processors' (RecordBuilder's, for one)
  }

  /** Fails the compile once, saying what's wrong with the option and how to fix it. */
  private void sayWrongName(String marker, String wrong) {
    if (!saidWrongName) {
      saidWrongName = true;
      processingEnv
          .getMessager()
          .printMessage(
              Diagnostic.Kind.ERROR,
              "-A"
                  + RECORDS_OPTION
                  + "="
                  + marker
                  + " "
                  + wrong
                  + ". Fix the name in build.gradle: it's the annotation that marks configuration"
                  + " records, with its package (com.michaelgrundvig.frc.world.config.ConfigRecord"
                  + " in the template).");
    }
  }

  /** The records in a type, itself and its nested types included. */
  private static void addRecords(Element element, List<TypeElement> records) {
    if (element instanceof TypeElement type) {
      if (type.getKind() == ElementKind.RECORD) {
        records.add(type);
      }
      for (Element member : type.getEnclosedElements()) {
        addRecords(member, records);
      }
    }
  }

  private void write(TypeElement record) {
    Elements elements = processingEnv.getElementUtils();
    String name = elements.getBinaryName(record).toString();
    String comment = elements.getDocComment(record);
    if (comment == null || comment.isBlank() || !written.add(name)) {
      return;
    }
    boolean markdown = elements.getDocCommentKind(record) == Elements.DocCommentKind.END_OF_LINE;
    DocComment doc = DocComment.parse(comment, markdown);
    // Each component's, in the record's order; a @param that names no component is left out.
    Map<String, String> params = new LinkedHashMap<>();
    for (RecordComponentElement component : record.getRecordComponents()) {
      String param = doc.params().get(component.getSimpleName().toString());
      if (param != null && !param.isEmpty()) {
        params.put(component.getSimpleName().toString(), param);
      }
    }
    try {
      FileObject file =
          processingEnv
              .getFiler()
              .createResource(StandardLocation.CLASS_OUTPUT, "", FOLDER + name + ".json", record);
      try (Writer out = new OutputStreamWriter(file.openOutputStream(), StandardCharsets.UTF_8)) {
        out.write(json(doc.summary(), doc.text(), params));
      }
    } catch (IOException e) {
      processingEnv
          .getMessager()
          .printMessage(
              Diagnostic.Kind.ERROR,
              "couldn't write its documentation for Pitwall: " + e.getMessage(),
              record);
    }
  }

  /** The documentation as JSON, one value to a line so it reads in the jar too. */
  private static String json(String summary, String text, Map<String, String> params) {
    StringBuilder json = new StringBuilder("{\n");
    json.append("  \"summary\": ").append(quoted(summary)).append(",\n");
    json.append("  \"text\": ").append(quoted(text)).append(",\n");
    json.append("  \"params\": {");
    String separator = "\n";
    for (Map.Entry<String, String> param : params.entrySet()) {
      json.append(separator)
          .append("    ")
          .append(quoted(param.getKey()))
          .append(": ")
          .append(quoted(param.getValue()));
      separator = ",\n";
    }
    json.append(params.isEmpty() ? "}\n" : "\n  }\n").append("}\n");
    return json.toString();
  }

  /** A JSON string: quotes, backslashes, and control characters escaped. */
  static String quoted(String text) {
    StringBuilder quoted = new StringBuilder("\"");
    for (char c : text.toCharArray()) {
      switch (c) {
        case '"' -> quoted.append("\\\"");
        case '\\' -> quoted.append("\\\\");
        case '\n' -> quoted.append("\\n");
        default -> {
          if (c < 0x20) {
            quoted.append(String.format(Locale.ROOT, "\\u%04x", (int) c));
          } else {
            quoted.append(c);
          }
        }
      }
    }
    return quoted.append('"').toString();
  }
}
