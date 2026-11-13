#!/usr/bin/env bash
# Builds the Orrery AppImage from an installed tree (docs/packaging.md).
#
#   packaging/linux/build-appimage.sh <install-dir> <version> <output-dir>
#
# <install-dir> is the result of `cmake --install <build> --prefix <dir> --strip`
# for a linux-package build: bin/orrery_app, share/orrery/assets, the desktop
# entry and the icon. The script copies it to <output-dir>/Orrery.AppDir/usr,
# lets linuxdeploy add the shared libraries the executable needs (the AppImage
# exclude list keeps libc, libGL, X11 and the other host libraries out), packs
# the AppDir with appimagetool and runs the result once (--version).
#
# The tools are downloaded once into $ORRERY_APPIMAGE_TOOLS (default
# <output-dir>/tools), each pinned to a release and checked against its
# SHA-256 before use. They run with APPIMAGE_EXTRACT_AND_RUN=1, so FUSE is not
# needed (containers, CI runners).
#
# The output is reproducible: the same installed tree gives the same AppImage
# byte for byte, whenever and by whoever it is built. SOURCE_DATE_EPOCH (from
# the environment, else the commit time of the checkout this script belongs
# to) dates every file of the AppDir and the SquashFS image; permissions are
# normalised (0755 for directories and executables, 0644 otherwise); every
# file is root-owned in the image; mksquashfs 4.6 (bundled with appimagetool)
# writes directory entries sorted and the same image on any number of
# threads. packaging/linux/check-appimage-reproducible.sh checks this.
set -euo pipefail
# Files the tools write (appimagetool rewrites the desktop entry) get the
# same modes under any umask of the caller.
umask 022

LINUXDEPLOY_URL="https://github.com/linuxdeploy/linuxdeploy/releases/download/1-alpha-20251107-1/linuxdeploy-x86_64.AppImage"
LINUXDEPLOY_SHA256="c20cd71e3a4e3b80c3483cef793cda3f4e990aca14014d23c544ca3ce1270b4d"
APPIMAGETOOL_URL="https://github.com/AppImage/appimagetool/releases/download/1.9.1/appimagetool-x86_64.AppImage"
APPIMAGETOOL_SHA256="ed4ce84f0d9caff66f50bcca6ff6f35aae54ce8135408b3fa33abfc3cb384eb0"
# The AppImage runtime is given explicitly; appimagetool would otherwise
# download whatever its moving "continuous" release holds.
RUNTIME_URL="https://github.com/AppImage/type2-runtime/releases/download/20251108/runtime-x86_64"
RUNTIME_SHA256="2fca8b443c92510f1483a883f60061ad09b46b978b2631c807cd873a47ec260d"

usage() {
    echo "usage: $0 <install-dir> <version> <output-dir>" >&2
    exit 2
}

[ "$#" -eq 3 ] || usage
[ "$(uname -m)" = "x86_64" ] || { echo "error: x86_64 hosts only" >&2; exit 1; }
install_dir=$(realpath "$1")
version=$2
out_dir=$(realpath -m "$3")
case "$version" in
    [0-9]*.[0-9]*.[0-9]*) ;;
    *) echo "error: version '$version' is not MAJOR.MINOR.PATCH" >&2; exit 2 ;;
esac
tools=${ORRERY_APPIMAGE_TOOLS:-$out_dir/tools}
appdir=$out_dir/Orrery.AppDir
appimage=$out_dir/orrery-$version-linux-x86_64.AppImage
export APPIMAGE_EXTRACT_AND_RUN=1

if [ -z "${SOURCE_DATE_EPOCH:-}" ]; then
    SOURCE_DATE_EPOCH=$(git -C "$(dirname "$(realpath "$0")")" log -1 --format=%ct 2> /dev/null || true)
fi
case "${SOURCE_DATE_EPOCH:-}" in
    '' | *[!0-9]*)
        echo "error: SOURCE_DATE_EPOCH is not set and the commit time is not available" \
            "(not a git checkout?); set SOURCE_DATE_EPOCH to a Unix time" >&2
        exit 2
        ;;
esac
export SOURCE_DATE_EPOCH

# fetch <file> <url> <sha256>: downloads into $tools unless a verified copy is there.
fetch() {
    local file=$tools/$1
    if [ -f "$file" ] && echo "$3  $file" | sha256sum --check --status; then
        return 0
    fi
    rm -f "$file" "$file.part"
    curl --fail --silent --show-error --location --retry 3 --connect-timeout 20 \
        --max-time 300 --output "$file.part" "$2"
    if ! echo "$3  $file.part" | sha256sum --check --status; then
        echo "error: checksum mismatch for $2" >&2
        rm -f "$file.part"
        exit 1
    fi
    mv "$file.part" "$file"
    chmod +x "$file"
}

for required in bin/orrery_app share/orrery/assets/experiments share/applications/orrery.desktop \
    share/icons/hicolor/256x256/apps/orrery.png; do
    if [ ! -e "$install_dir/$required" ]; then
        echo "error: $install_dir/$required is missing; install a linux-package build first" >&2
        exit 1
    fi
done

mkdir -p "$tools" "$out_dir"
fetch linuxdeploy "$LINUXDEPLOY_URL" "$LINUXDEPLOY_SHA256"
fetch appimagetool "$APPIMAGETOOL_URL" "$APPIMAGETOOL_SHA256"
fetch runtime-x86_64 "$RUNTIME_URL" "$RUNTIME_SHA256"

rm -rf "$appdir" "$appimage"
mkdir -p "$appdir/usr"
cp -a "$install_dir/." "$appdir/usr/"

# The installed files are stripped already (cmake --install --strip); the
# strip linuxdeploy bundles may not understand newer ELF sections.
NO_STRIP=1 "$tools/linuxdeploy" \
    --appdir "$appdir" \
    --executable "$appdir/usr/bin/orrery_app" \
    --desktop-file "$appdir/usr/share/applications/orrery.desktop" \
    --icon-file "$appdir/usr/share/icons/hicolor/256x256/apps/orrery.png"

if find "$appdir" -name '*.debug' -print -quit | grep -q .; then
    echo "error: debug information in the AppDir" >&2
    exit 1
fi

# Normalised metadata: permissions independent of the umask and the
# installed modes, every timestamp at SOURCE_DATE_EPOCH (symbolic links
# included), every file owned by root in the image.
find "$appdir" ! -type l -exec chmod u+rwX,go=rX {} +
find "$appdir" -exec touch -h -d "@$SOURCE_DATE_EPOCH" {} +

ARCH=x86_64 VERSION=$version "$tools/appimagetool" --no-appstream \
    --mksquashfs-opt -all-root \
    --runtime-file "$tools/runtime-x86_64" "$appdir" "$appimage"

reported=$("$appimage" --version)
case "$reported" in
    *"Orrery $version"*) ;;
    *) echo "error: the AppImage reports '$reported', expected Orrery $version" >&2; exit 1 ;;
esac
(cd "$out_dir" && sha256sum "$(basename "$appimage")" > "$(basename "$appimage").sha256")
echo "$appimage ($(du -k "$appimage" | cut -f1) KiB): $reported"
