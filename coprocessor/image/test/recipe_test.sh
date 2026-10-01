# shellcheck shell=bash
# recipe-hash.sh: its definition (which the robot build reproduces), and what changes it.

# shellcheck disable=SC2153 # IMAGE comes from run.sh
test_recipe_hash_is_the_sha256_of_git_ls_tree_over_the_listed_paths() {
  need git
  make_repo
  local paths
  paths=$(grep -Ev '^[[:space:]]*(#|$)' "$TMP/repo/coprocessor/image/recipe.paths")
  local expected
  # shellcheck disable=SC2086 # one path per word, as listed
  expected=$(git -C "$TMP/repo" ls-tree -r --full-tree HEAD -- $paths | sha256sum | cut -c1-64)
  assert_eq "$("$IMAGE/recipe-hash.sh" "$TMP/repo" 2>/dev/null)" "$expected"
}

test_recipe_hash_changes_with_the_recipe_not_its_documentation() {
  need git
  make_repo
  local first
  first=$("$IMAGE/recipe-hash.sh" "$TMP/repo" 2>/dev/null)
  echo 'more on how to build' >>"$TMP/repo/coprocessor/image/README.md"
  git_commit "$TMP/repo" "documentation"
  assert_eq "$("$IMAGE/recipe-hash.sh" "$TMP/repo" 2>/dev/null)" "$first" "after a README change"
  sed -i 's/alpha-2-69/alpha-2-70/' "$TMP/repo/coprocessor/photonvision.lock"
  git_commit "$TMP/repo" "a new PhotonVision"
  [[ $("$IMAGE/recipe-hash.sh" "$TMP/repo" 2>/dev/null) != "$first" ]] || fail "a lock change didn't change the hash"
}

test_recipe_hash_counts_committed_content_only() {
  need git
  make_repo
  local first
  first=$("$IMAGE/recipe-hash.sh" "$TMP/repo" 2>/dev/null)
  echo 'echo uncommitted' >>"$TMP/repo/coprocessor/image/provision.sh"
  assert_eq "$("$IMAGE/recipe-hash.sh" "$TMP/repo" 2>/dev/null)" "$first" "with an uncommitted change"
}

# What provision.sh reads from the repository by default makes the common image, so a change to
# any of it must change the recipe hash: the scripts and their folders, the board files, the
# installed files, the agent's unit, the lock, and what builds the agent's jar.
test_recipe_covers_everything_provision_reads() {
  # shellcheck source=../lib/common.sh
  . "$IMAGE/lib/common.sh"
  local entries=() line path missing=()
  while IFS= read -r line; do
    [[ $line =~ ^[[:space:]]*(#|$) ]] || entries+=("$line")
  done <"$IMAGE/recipe.paths"
  covered() {
    local entry
    for entry in "${entries[@]}"; do
      [[ $1 == "$entry" || $1 == "$entry"/* ]] && return 0
    done
    return 1
  }
  local image=coprocessor/image
  local consumed=("$image/provision.sh" "$image/lib/common.sh" "$COPROC_AGENT_UNIT" "$COPROC_LOCK")
  for path in "$IMAGE"/boards/* "$IMAGE"/files/*; do
    consumed+=("$image/${path#"$IMAGE"/}")
  done
  # The agent's jar is built, not committed: what builds it is.
  consumed+=(coprocessor/agent/src/main/x coprocessor/agent/build.gradle coprocessor/common/src/main/x
    coprocessor/common/build.gradle build.gradle settings.gradle gradle/libs.versions.toml)
  # The paths provision.sh names itself, from the repository, are all among those.
  grep -qF "repo/\$COPROC_AGENT_JAR" "$IMAGE/provision.sh" || fail "provision.sh reads the agent's jar differently now"
  grep -qF "repo/\$COPROC_AGENT_UNIT" "$IMAGE/provision.sh" || fail "provision.sh reads the agent's unit differently now"
  for path in "${consumed[@]}"; do
    covered "$path" || missing+=("$path")
  done
  ((${#missing[@]} == 0)) || fail "recipe.paths doesn't cover: ${missing[*]}"
}
