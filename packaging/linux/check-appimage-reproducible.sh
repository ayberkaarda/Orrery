#!/usr/bin/env bash
# Checks that build-appimage.sh produces the same AppImage, byte for byte,
# from the same installed tree (docs/packaging.md, "Reproducible AppImage").
#
#   packaging/linux/check-appimage-reproducible.sh <install-dir> <version> <work-dir>
#
# Builds the AppImage twice, one build after the other (so at different
# times), from two copies of <install-dir>: every file of the first is dated
# 2001, the second carries the time of the copy and is built under a stricter
# umask. Both builds use the same SOURCE_DATE_EPOCH (taken from the
# environment, else the time of the last commit, as build-appimage.sh does).
# Exits 0 when the two SHA-256 checksums are equal; otherwise prints both,
# and the differing file listings when unsquashfs is installed, and exits 1.
set -euo pipefail

usage() {
    echo "usage: $0 <install-dir> <version> <work-dir>" >&2
    exit 2
}

[ "$#" -eq 3 ] || usage
here=$(dirname "$(realpath "$0")")
install_dir=$(realpath "$1")
version=$2
work=$(realpath -m "$3")
export ORRERY_APPIMAGE_TOOLS=${ORRERY_APPIMAGE_TOOLS:-$work/tools}

rm -rf "$work/install-a" "$work/install-b" "$work/a" "$work/b"
mkdir -p "$work" "$ORRERY_APPIMAGE_TOOLS"
cp -a "$install_dir" "$work/install-a"
find "$work/install-a" -exec touch -h -d "2001-02-03 04:05:06" {} +
cp -R --no-preserve=timestamps "$install_dir" "$work/install-b"

"$here/build-appimage.sh" "$work/install-a" "$version" "$work/a"
(umask 0077 && "$here/build-appimage.sh" "$work/install-b" "$version" "$work/b")

name=orrery-$version-linux-x86_64.AppImage
sum_a=$(sha256sum "$work/a/$name" | cut -d' ' -f1)
sum_b=$(sha256sum "$work/b/$name" | cut -d' ' -f1)
echo "first build:  $sum_a"
echo "second build: $sum_b"
if [ "$sum_a" = "$sum_b" ]; then
    echo "reproducible: the two AppImages are identical"
    exit 0
fi
echo "error: the two AppImages differ" >&2
if command -v unsquashfs > /dev/null; then
    for build in a b; do
        offset=$(APPIMAGE_EXTRACT_AND_RUN=1 "$work/$build/$name" --appimage-offset)
        unsquashfs -o "$offset" -lln "$work/$build/$name" > "$work/$build.listing" 2>&1 || true
    done
    diff "$work/a.listing" "$work/b.listing" >&2 || true
fi
exit 1
