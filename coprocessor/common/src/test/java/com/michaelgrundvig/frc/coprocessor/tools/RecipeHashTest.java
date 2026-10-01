package com.michaelgrundvig.frc.coprocessor.tools;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** The recipe hash is the SHA-256 of git ls-tree's listing of what recipe.paths names. */
class RecipeHashTest {
  @TempDir Path root;

  private void write(String path, String text) throws IOException {
    Path file = root.resolve(path);
    Files.createDirectories(file.getParent());
    Files.writeString(file, text, StandardCharsets.UTF_8);
  }

  /** Runs git in the repository; false when there's no git to run. */
  private boolean git(String... args) throws IOException, InterruptedException {
    List<String> command =
        new ArrayList<>(
            List.of(
                "git",
                "-c",
                "user.name=Test",
                "-c",
                "user.email=test@example.org",
                "-c",
                "commit.gpgsign=false",
                "-c",
                "core.autocrlf=false"));
    command.addAll(List.of(args));
    Process process;
    try {
      process =
          new ProcessBuilder(command)
              .directory(root.toFile())
              .redirectErrorStream(true)
              .redirectOutput(ProcessBuilder.Redirect.DISCARD)
              .start();
    } catch (IOException e) {
      return false;
    }
    return process.waitFor(60, TimeUnit.SECONDS) && process.exitValue() == 0;
  }

  /** The object ID git gives a file's content. */
  private static String blob(String content) throws NoSuchAlgorithmException {
    byte[] bytes = content.getBytes(StandardCharsets.UTF_8);
    MessageDigest sha1 = MessageDigest.getInstance("SHA-1");
    sha1.update(("blob " + bytes.length + "\0").getBytes(StandardCharsets.UTF_8));
    return HexFormat.of().formatHex(sha1.digest(bytes));
  }

  private static String sha256(String text) throws NoSuchAlgorithmException {
    return HexFormat.of()
        .formatHex(
            MessageDigest.getInstance("SHA-256").digest(text.getBytes(StandardCharsets.UTF_8)));
  }

  @Test
  void theHashIsOfTheCommittedFilesRecipePathsNames() throws Exception {
    write(
        RecipeHash.PATHS,
        "# what the recipe covers\n\ncoprocessor/image/provision.sh\ncoprocessor/agent/src/main\n");
    write("coprocessor/image/provision.sh", "#!/bin/sh\n");
    write("coprocessor/agent/src/main/java/A.java", "class A {}\n");
    write("coprocessor/agent/src/test/java/ATest.java", "class ATest {}\n");
    assumeTrue(git("init", "-q"), "needs git");
    assumeTrue(git("add", "."));
    assumeTrue(git("commit", "-q", "-m", "recipe"));

    String listing =
        "100644 blob "
            + blob("class A {}\n")
            + "\tcoprocessor/agent/src/main/java/A.java\n"
            + "100644 blob "
            + blob("#!/bin/sh\n")
            + "\tcoprocessor/image/provision.sh\n";
    RecipeHash.Result result = RecipeHash.of(root);
    assertThat(result.hash()).isEqualTo(sha256(listing));
    assertThat(result.why()).isEmpty();

    // Uncommitted changes don't count: an image is built from a commit.
    write("coprocessor/image/provision.sh", "#!/bin/sh\necho changed\n");
    assertThat(RecipeHash.of(root).hash()).isEqualTo(sha256(listing));
    assumeTrue(git("commit", "-q", "-a", "-m", "changed"));
    assertThat(RecipeHash.of(root).hash()).isNotEqualTo(sha256(listing));
  }

  @Test
  void withoutARecipeOrGitThereIsNoHashAndItSaysWhy() throws IOException {
    assertThat(RecipeHash.of(root))
        .isEqualTo(
            new RecipeHash.Result("", "no coprocessor/image/recipe.paths, so no recipe to hash"));
    write(RecipeHash.PATHS, "# nothing\n");
    assertThat(RecipeHash.of(root).why())
        .isEqualTo("coprocessor/image/recipe.paths lists no paths");
    write(RecipeHash.PATHS, "coprocessor/image\n");
    RecipeHash.Result notACheckout = RecipeHash.of(root);
    assertThat(notACheckout.hash()).isEmpty();
    assertThat(notACheckout.why()).isNotEmpty();
  }

  @Test
  void recipePathsListsEveryLineButBlanksAndComments() {
    assertThat(RecipeHash.paths("# comment\n\n  # indented comment\na/b\r\nc d\n   \n"))
        .containsExactly("a/b", "c d");
  }

  @Test
  void anEmptyListingHashesAsNothing() {
    // What `printf '' | sha256sum` prints.
    assertThat(RecipeHash.of(new byte[0]))
        .isEqualTo("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  }
}
