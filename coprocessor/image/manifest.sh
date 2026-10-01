#!/usr/bin/env bash
# manifest.sh: writes a release's SHA256SUMS and manifest.json, from its images and their stamps.
#
#   coprocessor/image/manifest.sh DIR
#
# DIR holds each computer's image, NAME.img.xz, beside its stamp, NAME.stamp.json (what stamp.sh
# wrote with --stamp-out). Writes into DIR:
#   SHA256SUMS     one line per image, as sha256sum writes them, so `sha256sum -c SHA256SUMS`
#                  (or PowerShell's Get-FileHash, by eye) checks a download
#   manifest.json  the release (team, release name, PhotonVision version, recipe hash) and each
#                  computer: name, address, board, cameras (its role), image file, image sha256,
#                  PhotonVision version, recipe hash, and settings hash
# Fails if the images don't all come from one release, version, and recipe, or an image has no
# stamp. Needs yq (mikefarah's, version 4).
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib/common.sh
. "$here/lib/common.sh"

dir=${1:?usage: manifest.sh DIR}
[[ -d $dir ]] || die "no directory '$dir'"
require_yq

entries=()
sums=""
shopt -s nullglob
for image in "$dir"/*.img.xz; do
  [[ -f ${image%.img.xz}.stamp.json ]] || die "${image##*/} has no stamp (${image%.img.xz}.stamp.json)"
done
for stamp in "$dir"/*.stamp.json; do
  image=${stamp%.stamp.json}.img.xz
  [[ -f $image ]] || die "${stamp##*/} has no image (${image##*/})"
  sum=$(sha256sum <"$image" | cut -c1-64)
  sums+="$sum  ${image##*/}"$'\n'
  entries+=("$(IMAGE=${image##*/} SUM=$sum yq -p json -o=json -I=0 \
    '. + {"image": strenv(IMAGE), "imageSha256": strenv(SUM)}' "$stamp")")
done
((${#entries[@]})) || die "no images in $dir"
all="[$(IFS=,; echo "${entries[*]}")]"
for field in team release photonvisionVersion recipeHash; do
  [[ $(FIELD=$field yq -p json -r '[.[] | .[strenv(FIELD)]] | unique | length' <<<"$all") == 1 ]] ||
    die "the images don't share one $field"
done

sort -k2 <<<"${sums%$'\n'}" >"$dir/SHA256SUMS"
yq -p json -o=json -I=2 '{
  "schema": 1,
  "team": .[0].team,
  "release": .[0].release,
  "photonvisionVersion": .[0].photonvisionVersion,
  "recipeHash": .[0].recipeHash,
  "computers": (sort_by(.name) | map({
    "name": .name,
    "address": .address,
    "board": .board,
    "cameras": .cameras,
    "image": .image,
    "imageSha256": .imageSha256,
    "photonvisionVersion": .photonvisionVersion,
    "recipeHash": .recipeHash,
    "settingsHash": .settingsHash
  }))
}' <<<"$all" >"$dir/manifest.json"
say "wrote SHA256SUMS and manifest.json for ${#entries[@]} computers"
