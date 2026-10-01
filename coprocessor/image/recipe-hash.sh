#!/usr/bin/env bash
# recipe-hash.sh: prints the recipe hash, which identifies how the common image is built.
#
#   coprocessor/image/recipe-hash.sh [REPO_DIR]
#
# It's the sha256 of `git ls-tree -r --full-tree HEAD -- <paths>`, for the paths recipe.paths
# lists, in its order: the committed content only, so it's the same on every machine and every OS,
# and the same as the robot build's own computation of it. Uncommitted changes don't count: an
# image is built from a commit.
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib/common.sh
. "$here/lib/common.sh"

repo=${1:-$(cd "$here/../.." && pwd)}
list="$repo/$COPROC_RECIPE_PATHS"
[[ -f $list ]] || die "no $COPROC_RECIPE_PATHS in $repo"
paths=()
while IFS= read -r line; do
  [[ $line =~ ^[[:space:]]*(#|$) ]] && continue
  paths+=("$line")
done <"$list"
((${#paths[@]})) || die "$COPROC_RECIPE_PATHS lists no paths"
tree=$(git -C "$repo" ls-tree -r --full-tree HEAD -- "${paths[@]}") ||
  die "git ls-tree failed in $repo (is it a git checkout?)"
tab=$'\t'
for path in "${paths[@]}"; do
  grep -qF "$tab$path" <<<"$tree" || say "note: $path in $COPROC_RECIPE_PATHS matches no committed file"
done
# Exactly git's output: one line per file, each ending in a newline (none at all for no files).
if [[ -n $tree ]]; then
  printf '%s\n' "$tree"
fi | sha256sum | cut -c1-64
