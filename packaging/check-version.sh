#!/usr/bin/env bash
# Reads the project version that configuring wrote to <build>/orrery_version.txt
# (cmake/Packaging.cmake; the source is project(VERSION) in CMakeLists.txt).
# On a v* tag the tag must name that version (v1.2.3, or v1.2.3-suffix for a
# pre-release). In GitHub Actions the version is exported as ORRERY_VERSION
# for the later steps.
#
#   packaging/check-version.sh <build>/orrery_version.txt
set -euo pipefail

[ "$#" -eq 1 ] || { echo "usage: $0 <build>/orrery_version.txt" >&2; exit 2; }
version=$(tr -d '\r\n' < "$1")
case "$version" in
    [0-9]*.[0-9]*.[0-9]*) ;;
    *) echo "error: '$version' in $1 is not MAJOR.MINOR.PATCH" >&2; exit 1 ;;
esac
case "${GITHUB_REF:-}" in
    refs/tags/v*)
        tag=${GITHUB_REF_NAME:-${GITHUB_REF#refs/tags/}}
        case "$tag" in
            "v$version" | "v$version"-*) ;;
            *)
                echo "error: tag $tag does not match the project version $version" >&2
                exit 1
                ;;
        esac
        ;;
esac
if [ -n "${GITHUB_ENV:-}" ]; then
    echo "ORRERY_VERSION=$version" >> "$GITHUB_ENV"
fi
echo "version $version"
