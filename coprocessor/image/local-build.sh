#!/usr/bin/env bash
# local-build.sh: builds the release's images on a mentor's Linux machine, the way CI does, in a
# privileged container (README.md, "Building on your own Linux machine").
#
#   coprocessor/image/local-build.sh --agent-jar PATH --agent-unit PATH \
#       --settings-tool COMMAND [--release NAME] [--out DIR]
#
# From the repository's root, as root, with: bash, curl, xz, sfdisk, losetup, mount, e2fsck,
# resize2fs, mkfs.ext4, mkfs.fat, yq (mikefarah's, version 4), git, and the settings tool's Java.
# On a machine that isn't arm64, the chroot's ARM programs also need qemu-user-static registered
# in the kernel's binfmt_misc with its "F" flag (README.md says how).
#
# The same steps as the workflow: plan.sh's checks; per board, the base image and the jar checked
# against the lock, the root grown, provision.sh run in a chroot of it; per computer, layout.sh
# and stamp-image.sh; then manifest.sh. Downloads are kept in OUT/downloads, by checksum.
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo=$(cd "$here/../.." && pwd)
# shellcheck source=lib/common.sh
. "$here/lib/common.sh"

agent_jar="" agent_unit="" settings_tool="" release="" out="$repo/build/coprocessor-image/local"
while (($#)); do
  case $1 in
    --agent-jar) agent_jar=$(realpath "${2:?}"); shift 2 ;;
    --agent-unit) agent_unit=$(realpath "${2:?}"); shift 2 ;;
    --settings-tool) settings_tool=${2:?}; shift 2 ;;
    --release) release=${2:?}; shift 2 ;;
    --out) out=${2:?}; shift 2 ;;
    *) die "unknown option: $1" ;;
  esac
done
((EUID == 0)) || die "needs root (in a privileged container): it mounts images"
[[ -f $agent_jar && -f $agent_unit ]] || die "--agent-jar and --agent-unit: the built agent and its unit"
[[ -n $settings_tool ]] || die "--settings-tool is required"
for tool in curl xz sfdisk losetup mount e2fsck resize2fs mkfs.ext4 git; do
  command -v "$tool" >/dev/null || die "needs $tool"
done
require_yq
if [[ $(uname -m) != aarch64 ]] && ! grep -qs '^flags:.*F' /proc/sys/fs/binfmt_misc/qemu-aarch64; then
  die "this machine isn't arm64, and qemu-aarch64 isn't registered in binfmt_misc with the F flag"
fi
mkdir -p "$out/downloads"
out=$(cd "$out" && pwd)
cd "$repo"

# The workflow's plan, as shell variables.
plan=$(GITHUB_OUTPUT='' "$here/plan.sh" --release "$release" --sha "$(git rev-parse HEAD)")
get() {
  sed -n "s/^$1=//p" <<<"$plan"
}
team=$(get team) version=$(get version) recipe_hash=$(get recipe-hash) release=$(get release)
jar_url=$(get jar-url) jar_sha=$(get jar-sha256) boards=$(get boards) computers=$(get computers)

# Downloads a URL once, into OUT/downloads, and checks it.
fetch() {
  local url=$1 sum=$2 file="$out/downloads/$2-${1##*/}"
  if [[ ! -f $file ]]; then
    curl --fail --location --silent --show-error --retry 3 --output "$file.part" "$url"
    mv "$file.part" "$file"
  fi
  echo "$sum  $file" | sha256sum --check --strict --quiet || die "$url doesn't match the lock's sha256"
  printf '%s' "$file"
}

mnt="" loop=""
cleanup() {
  if [[ -n $mnt ]]; then
    if [[ -e $mnt/etc/resolv.conf.coproc-saved ]]; then
      mv -f "$mnt/etc/resolv.conf.coproc-saved" "$mnt/etc/resolv.conf"
    fi
    umount --recursive "$mnt" 2>/dev/null || true
    rmdir "$mnt" 2>/dev/null || true
  fi
  if [[ -n $loop ]]; then
    losetup --detach "$loop" 2>/dev/null || true
  fi
  mnt="" loop=""
}
trap cleanup EXIT

pv_jar=$(fetch "$jar_url" "$jar_sha")
while IFS= read -r row <&3; do
  board=$(yq -p json -r '.board' <<<"$row")
  # shellcheck source=boards/orangepi-5.env
  . "$here/boards/$board.env"
  common="$out/common-$board.img"
  say "$board: the common image"
  base=$(fetch "$(yq -p json -r '.url' <<<"$row")" "$(yq -p json -r '.sha256' <<<"$row")")
  xz -dc "$base" >"$common"
  # The root grown by the headroom PhotonVision's own build gives it.
  truncate -s "+$(yq -p json -r '.minimumFreeMb' <<<"$row")M" "$common"
  if [[ $(sfdisk --dump "$common" | sed -n 's/^label: *//p') == gpt ]]; then
    sfdisk --quiet --relocate gpt-bak-std "$common"
  fi
  echo ', +' | sfdisk --quiet --no-reread --no-tell-kernel -N "$BOARD_ROOT_PARTITION" "$common"
  loop=$(losetup --find --show --partscan "$common")
  root_dev="${loop}p$BOARD_ROOT_PARTITION"
  e2fsck -pf "$root_dev"
  resize2fs "$root_dev"
  # A chroot of the image's root, as photon-image-runner makes one: the repository at /tmp/build.
  mnt=$(mktemp -d)
  mount "$root_dev" "$mnt"
  mount -t proc proc "$mnt/proc"
  mount -t sysfs sys "$mnt/sys"
  mount --rbind /dev "$mnt/dev"
  mount -t tmpfs tmpfs "$mnt/run"
  mkdir -p "$mnt/tmp/build"
  mount --bind "$repo" "$mnt/tmp/build"
  mv -f "$mnt/etc/resolv.conf" "$mnt/etc/resolv.conf.coproc-saved"
  cp /etc/resolv.conf "$mnt/etc/resolv.conf"
  mkdir -p "$repo/build/coprocessor-image/local-inputs"
  cp "$pv_jar" "$repo/build/coprocessor-image/local-inputs/photonvision.jar"
  cp "$agent_jar" "$repo/build/coprocessor-image/local-inputs/coprocessor-agent.jar"
  cp "$agent_unit" "$repo/build/coprocessor-image/local-inputs/coprocessor-agent.service"
  chroot "$mnt" bash -c "cd /tmp/build && bash coprocessor/image/provision.sh --board $board \
    --photonvision-jar build/coprocessor-image/local-inputs/photonvision.jar \
    --photonvision-sha256 $jar_sha \
    --agent-jar build/coprocessor-image/local-inputs/coprocessor-agent.jar \
    --agent-unit build/coprocessor-image/local-inputs/coprocessor-agent.service \
    --smoketest-dir build/coprocessor-image/local-smoketest"
  cp "$repo/build/coprocessor-image/local-smoketest/photonvision_config/photon.sqlite" \
    "$out/empty-photon-$board.sqlite"
  # Zeros where nothing is, so the image compresses as CI's does.
  cat /dev/zero >"$mnt/coproc-zeros" 2>/dev/null || true
  rm -f "$mnt/coproc-zeros"
  cleanup
done 3< <(yq -p json -o=json -I=0 '.[]' <<<"$boards")

mkdir -p "$out/release"
while IFS= read -r row <&3; do
  name=$(yq -p json -r '.name' <<<"$row")
  board=$(yq -p json -r '.board' <<<"$row")
  image="$out/$team-$name-pv$version-$release.img"
  say "$name: stamping"
  cp --sparse=always "$out/common-$board.img" "$image"
  "$here/layout.sh" --board "$board" "$image"
  "$here/stamp-image.sh" --board "$board" "$image" -- \
    --computer "$name" --release "$release" --recipe-hash "$recipe_hash" \
    --photonvision-version "$version" --empty-db "$out/empty-photon-$board.sqlite" \
    --settings-tool "$settings_tool" --stamp-out "$out/release/$(basename "$image" .img).stamp.json"
  xz -T0 -c "$image" >"$out/release/${image##*/}.xz"
  rm -f "$image"
done 3< <(yq -p json -o=json -I=0 '.[]' <<<"$computers")
"$here/manifest.sh" "$out/release"
say "the release's files are in $out/release"
