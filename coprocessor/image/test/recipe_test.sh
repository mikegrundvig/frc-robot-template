# shellcheck shell=bash
# recipe-hash.sh: its definition (which the robot build reproduces), and what changes it.

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
