package com.michaelgrundvig.frc.coprocessor.tools;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.jspecify.annotations.Nullable;

/**
 * The hash of the recipe a coprocessor image is built from, which the robot checks each
 * coprocessor's stamp against. {@code coprocessor/image/recipe.paths} lists what it covers (the
 * image's scripts, the agent's and the shared code, the lock); the hash is the SHA-256, in
 * lowercase hex, of what
 *
 * <pre>git ls-tree -r --full-tree HEAD -- &lt;each path in recipe.paths&gt;</pre>
 *
 * prints: the committed files only, so it's the same on every machine and every OS, and the same
 * the image workflow computes ({@code coprocessor/image/recipe-hash.sh}). Uncommitted changes don't
 * count: an image is built from a commit.
 */
public final class RecipeHash {
  /** The list of what the recipe covers, from the repository's root. */
  public static final String PATHS = "coprocessor/image/recipe.paths";

  private RecipeHash() {}

  /**
   * The recipe hash of the repository at {@code root}; empty, with the reason, when there's no
   * recipe to hash (no recipe.paths) or no Git to ask (not a checkout, or no commit yet).
   */
  public static Result of(Path root) throws IOException {
    Path list = root.resolve(PATHS);
    if (!Files.isRegularFile(list)) {
      return new Result("", "no " + PATHS + ", so no recipe to hash");
    }
    List<String> paths = paths(Files.readString(list, StandardCharsets.UTF_8));
    if (paths.isEmpty()) {
      return new Result("", PATHS + " lists no paths");
    }
    byte[] prefix = git(root, List.of("git", "rev-parse", "--show-prefix"));
    if (prefix == null) {
      return new Result("", "git rev-parse failed: not a Git checkout, or no git to run");
    }
    if (!new String(prefix, StandardCharsets.UTF_8).isBlank()) {
      return new Result(
          "",
          "the project is in a folder of its Git repository, and recipe.paths are from the"
              + " repository's root");
    }
    List<String> command =
        new ArrayList<>(List.of("git", "ls-tree", "-r", "--full-tree", "HEAD", "--"));
    command.addAll(paths);
    byte[] listing = git(root, command);
    if (listing == null) {
      return new Result("", "git ls-tree failed: no commit yet");
    }
    return new Result(of(listing), "");
  }

  /** What a git command prints, or null when it can't run or fails. */
  private static byte @Nullable [] git(Path root, List<String> command) throws IOException {
    Process git;
    try {
      git =
          new ProcessBuilder(command)
              .directory(root.toFile())
              .redirectError(ProcessBuilder.Redirect.DISCARD)
              .start();
    } catch (IOException e) {
      return null;
    }
    ByteArrayOutputStream out = new ByteArrayOutputStream();
    try (InputStream in = git.getInputStream()) {
      in.transferTo(out);
      if (!git.waitFor(60, TimeUnit.SECONDS)) {
        git.destroyForcibly();
        return null;
      }
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      git.destroyForcibly();
      return null;
    }
    return git.exitValue() == 0 ? out.toByteArray() : null;
  }

  /** The hash of a listing, as {@code git ls-tree} printed it. */
  static String of(byte[] listing) {
    try {
      return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(listing));
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException("every Java has SHA-256", e);
    }
  }

  /** The paths a recipe.paths lists: every line but blank ones and comments ({@code #}). */
  static List<String> paths(String list) {
    List<String> paths = new ArrayList<>();
    for (String line : list.split("\n", -1)) {
      String path = line.endsWith("\r") ? line.substring(0, line.length() - 1) : line;
      if (!path.isBlank() && !path.strip().startsWith("#")) {
        paths.add(path);
      }
    }
    return paths;
  }

  /**
   * A recipe hash, or why there's none.
   *
   * @param hash 64 lowercase hex digits; empty when there's none
   * @param why why there's none; empty when there is one
   */
  public record Result(String hash, String why) {}
}
