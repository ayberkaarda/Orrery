# Packaging

How Orrery is installed and packaged (Phase 11, spec section 8: binary and
assets under 80 MB, cold start to the first frame under 3 s). The install
rules and the CPack configuration live in `cmake/Packaging.cmake`, the
installer resources and scripts in `packaging/`, the release workflow in
`.github/workflows/release.yml`.

## 1. Packages

| Platform | File | Contents |
|---|---|---|
| Windows x64 | `orrery-<version>-windows-x64.msi` | per-user installer (no elevation), Start menu shortcut, uninstaller entry |
| Windows x64 | `orrery-<version>-windows-x64.zip` | portable copy of the same tree |
| Linux x86_64 | `orrery-<version>-linux-x86_64.tar.gz` | the installed tree (`bin/`, `share/`) |
| Linux x86_64 | `orrery-<version>-linux-x86_64.AppImage` | single file, runs on glibc 2.35 or newer |

Every package comes with a `<file>.sha256` checksum. All of them hold a
Release build of the windowed application (`orrery_app`, which also runs
`--headless`); none contains debug information.

**Why WiX for the installer.** The CPack WIX generator is driven by the WiX
.NET tool, which installs from NuGet with an exact version
(`dotnet tool install wix --version 5.0.2`) on any machine with the .NET SDK,
including the GitHub Windows runners (the .NET SDK is part of their image).
Nothing depends on a toolset preinstalled on the runner image, where the WiX
and NSIS installations differ between images and change over time; NSIS
would have to be downloaded and pinned separately. WiX 5.0.2 builds the
WiX 4 schema that CPack writes (`CPACK_WIX_VERSION 4`) and is MS-RL licensed;
moving to a newer major version means reviewing its licence terms first. An
MSI also gives a standard per-user install, upgrade and uninstall without
installer code of our own, and `wix msi validate` checks it with the Windows
Installer consistency rules (ICE).

## 2. Installed layout

The same relative layout on every platform:

```
bin/orrery_app[.exe]             the application (Windows: and the DLLs it loads)
share/orrery/assets/             bundled experiments/ and scenarios/
share/doc/orrery/                README.md, README.tr.md, docs/, notice.txt, third_party/<library>/
share/applications/orrery.desktop                Linux only
share/icons/hicolor/{256x256,scalable}/apps/     Linux only
```

Shaders are compiled into the executable (`rendering.md`), so the bundled
assets are the only data files. The application finds them from its own
location (`app::asset_directories`, docs/app.md section 4): `assets/` beside
the executable, then `../share/orrery/assets`, then, in development builds
only, the source tree. An installed tree can therefore be moved anywhere,
and a tree with an `assets/` folder beside the executable keeps working as a
portable copy. Package builds set `ORRERY_SOURCE_ASSET_FALLBACK=OFF`, so no
build machine path is compiled into a shipped executable (the install test
checks this).

### Third-party notices

`share/doc/orrery/third_party` holds the licence notice of every vcpkg
library of the build (vcpkg's `copyright` file per port; build helpers and
the test framework are left out). Where a library's licence asks for more
than that file, the extra notice is kept in `packaging/third_party/<port>/`,
copied verbatim from the upstream release the port builds, and listed in
`packaging/third_party/notices.cmake` with that release's version, its
SHA-256 and its source URL. Configuring fails when vcpkg moves the port to
another version (the notice must be taken again from the new release) or
when the file's checksum differs.

| Port | Bundled as | Extra notice | Why |
|---|---|---|---|
| `tbb` (oneTBB 2023.2.0) | `tbb12.dll`; statically linked on Linux | `third-party-programs.txt` from the upstream `v2023.2.0` tag | the library contains the ITT Notify API, whose BSD licence requires its notice in binary distributions; vcpkg's `copyright` holds only Apache-2.0 |

The other ports' `copyright` files already carry everything their licences
ask for (for example, assimp's includes the licences of the code it bundles:
earcut, openddlparser and Open3DGC). The MSVC runtime DLLs are distributed
under the distributable-code terms of the Visual Studio licence.
`install.relocated` checks the installed tree against this
(`packaging/check_notices.cmake`): every DLL beside the Windows executable
must map to a port whose notice is installed (an unknown DLL fails the test
until its licence has been looked at), and every port with an extra notice
must have it, with the pinned checksum.

## 3. Building the packages locally

The package presets inherit the release presets (`release`,
`linux-release`, which also carry the release compiler and link-time
optimisation settings) and add:

| Cache variable | Value | Why |
|---|---|---|
| `ORRERY_BUILD_TESTS` | `OFF` | the test suite is CI's job; packages build only the application |
| `ORRERY_INSTALL_TEST` | `ON` | registers `install.relocated` (section 4) |
| `ORRERY_SOURCE_ASSET_FALLBACK` | `OFF` | no source tree path in the executable |
| `CMAKE_EXE_LINKER_FLAGS` (Linux) | `-static-libstdc++ -static-libgcc` | the binary needs only glibc from the host's C++ runtime side |

### Windows

Prerequisites: Visual Studio 2022 (MSVC x64), CMake 4.1 or newer, Ninja,
vcpkg (`VCPKG_ROOT`), the .NET SDK, and the WiX tool with its UI extension:

```bat
dotnet tool install --tool-path C:\tools\wix wix --version 5.0.2
set WIX=C:\tools\wix
C:\tools\wix\wix.exe extension add --global WixToolset.UI.wixext/5.0.2
```

(`--global` extensions go to `%USERPROFILE%\.wix`, or under
`%WIX_EXTENSIONS%` when that variable is set; CPack finds `wix.exe` through
`WIX` or `PATH`.) Then, from a developer command prompt:

```bat
cmake --preset package
cmake --build --preset package
ctest --preset package
cpack --preset package
```

The `.msi`, `.zip` and checksums are written to `build/package/`. Without
the WiX tool, `cpack --preset package -G ZIP` still builds the zip.

To look inside the installer without installing it, extract it
administratively: `msiexec /a build\package\orrery-<version>-windows-x64.msi /qn TARGETDIR=C:\temp\orrery-msi`
(the tree appears under `LocalApp\Programs\Orrery`). `wix msi validate
<file>.msi` runs the ICE checks; the expected warnings are ICE61 (CPack's
template allows same-version upgrades), ICE69 (CPack's shortcut component)
and ICE91 (files in a per-user directory), and there must be no error.

The MSI installs per user, without elevation, to
`%LOCALAPPDATA%\Programs\Orrery` (the user may choose another folder), adds
an "Orrery" Start menu shortcut and an entry in Installed apps.
`packaging/windows/wix_patch.xml` adds the removal entry for the
`Programs` folder that a per-user installation needs (it is removed on
uninstall only when empty).

### Linux

Prerequisites: the packages of the CI build (`LINUX_DEPS` in
`.github/workflows/ci.yml`: X11 and GL headers, autotools for vcpkg), clang
17 or GCC 13, CMake 4.1 or newer, Ninja, vcpkg, `curl` and `file`.

```sh
cmake --preset linux-package
cmake --build --preset linux-package
ctest --preset linux-package
cpack --preset linux-package                       # build/linux-package/orrery-<version>-linux-x86_64.tar.gz
cmake --install build/linux-package --prefix /tmp/orrery-install --strip
packaging/linux/build-appimage.sh /tmp/orrery-install "$(cat build/linux-package/orrery_version.txt)" \
    build/linux-package/appimage
```

`build-appimage.sh` copies the installed tree into `Orrery.AppDir/usr`,
lets linuxdeploy add the shared libraries the executable needs (the AppImage
exclude list keeps libc, libGL, the X11 libraries and other host libraries
out, as they must match the host's driver), packs the AppDir with
appimagetool and runs the result once with `--version`. linuxdeploy
(1-alpha-20251107-1), appimagetool (1.9.1) and the AppImage runtime
(type2-runtime 20251108) are downloaded into `<output>/tools` (or
`$ORRERY_APPIMAGE_TOOLS`) and checked against the SHA-256 pinned in the
script before use. The tools run with `APPIMAGE_EXTRACT_AND_RUN=1`, so FUSE
is not needed to build the AppImage. To run an AppImage without FUSE, use
`./orrery-<version>-linux-x86_64.AppImage --appimage-extract-and-run`.

#### Reproducible AppImage

The same installed tree gives the same AppImage, byte for byte, whenever
and by whoever it is built:

- `SOURCE_DATE_EPOCH` comes from the environment, else from the commit time
  of the checkout (`git log -1 --format=%ct`); without either the script
  stops. Every file of the AppDir is dated to it (symbolic links included),
  and the SquashFS creation time follows it (mksquashfs reads the variable).
- Permissions are normalised (`umask 022`, then `u+rwX,go=rX`: 0755 for
  directories and executables, 0644 otherwise), so neither the installed
  modes nor the caller's umask reach the image.
- Every file is owned by root in the image (`-all-root`), and mksquashfs 4.6
  (bundled with the pinned appimagetool) writes directory entries sorted by
  name and the same image on any number of threads.

`packaging/linux/check-appimage-reproducible.sh <install-dir> <version>
<work-dir>` builds the AppImage twice from two copies of the installed tree,
one with every file dated 2001 and one with fresh timestamps built under
umask 077, and compares the SHA-256 of the results. It was run under WSL
(Ubuntu) with a stand-in executable: identical checksums, where the script
without these steps gave different ones. The release workflow does not run
it (it builds once); the real application's AppImage was not built twice.

Host requirements of the Linux packages: x86_64, glibc 2.35 or newer
(Ubuntu 22.04 or later), an OpenGL 4.6 driver and an X11 session (Wayland
through XWayland). The vcpkg libraries are linked statically.

## 4. The relocated install test

`install.relocated` (`packaging/install_smoke.cmake`, registered when
`ORRERY_INSTALL_TEST=ON`):

1. `cmake --install <build> --prefix <staging> --strip`, then moves the
   tree to a path with a space, so nothing may refer back to the staging
   prefix or the build tree;
2. checks the layout, that no `.pdb`/`.ilk`/`.debug` file is installed, that
   the Linux executable is stripped, and the total size against 80,000,000
   bytes; checks every relative link and image of the installed
   documentation with `tools/check_docs.py --tree share/doc/orrery`, which
   also rejects a link that leaves that folder (the READMEs link to `docs/`,
   to each other and to the screenshots, so `docs/` is installed beside
   them; Python 3 is therefore needed to configure a package build);
3. runs the moved `orrery_app` with only the system directories on `PATH`
   (Windows) or without `LD_LIBRARY_PATH` (Linux), so every DLL has to come
   from the installed tree: `--version`, then `--headless` on an experiment
   written only into the moved tree (found by name from another working
   directory, its scenario resolved relative to it) and on the bundled
   `solar_system`, timing each run;
4. with `ORRERY_SOURCE_ASSET_FALLBACK=OFF`, requires that the executable
   holds no source tree path.

Without an OpenGL 4.6 context the application exits with code 3 after it
has found and loaded the experiment; the test then still passes (the lookup
is proven) unless `ORRERY_REQUIRE_GL=1`. Hosted Linux runners have no such
context, so there the images are not rendered.

## 5. Versioning

The version has one source: `project(Orrery VERSION ...)` in the top-level
`CMakeLists.txt`. From it come `orrery_app --version`, the Windows file
version resource, the MSI product version, the package file names, and
`build/<preset>/orrery_version.txt`. `vcpkg.json` repeats the version
(`version-semver`, required by vcpkg manifests); configuring fails when the
two differ, so they change together.

A release is a tag `v<version>` (or `v<version>-<suffix>` for a
pre-release) on a commit whose project version matches; the release
workflow refuses a tag that names another version. The MSI keeps one
upgrade code for all versions, so installing a newer version replaces the
older one; a lower version is refused while a higher one is installed.

## 6. Size and start-up

Measured on Windows (MSVC 19.44, `package` preset, version 0.0.1):

| Item | Size |
|---|---|
| installed tree (65 files) | 19.2 MiB (20,119,093 bytes in the MSI's extracted tree) |
| `orrery_app.exe` | 3.6 MiB |
| largest DLLs | assimp 5.3 MiB, libcrypto 5.0 MiB |
| `.msi` | 6.8 MiB (7,104,484 bytes) |
| `.zip` | 7.8 MiB (8,228,837 bytes) |

The spec limit for binary and assets is 80 MB: the whole installed tree,
runtime libraries included, uses about a quarter of it. The Linux numbers
are reported by the release workflow in its job summary.

Start-up to the first frame (headless, from the unpacked zip, Radeon RX
9060 XT; process start, OpenGL context, experiment load, one frame and the
PNG written; five runs each):

| Run | Time |
|---|---|
| `--version` | 11 to 60 ms |
| `solar_system`, 1920x1080 | 0.56 to 0.60 s |
| `black_hole`, 1920x1080 | 0.81 to 1.22 s |
| `solar_system`, 160x90 | 0.39 to 2.6 s |

The spec target is under 3 s. The very first runs of a freshly installed
or unpacked tree were slower, 1.5 to 4.5 s (the 4.5 s for a 1080p frame):
the antivirus scans the new binaries and the driver compiles the shaders
into its cache once. So a first launch after installation can exceed 3 s
on this machine; later launches stay well under it. The windowed start
was not timed (it would open a window).

## 7. Signing

The packages are **not signed**. On Windows, SmartScreen shows "Windows
protected your PC" for the MSI and for an `orrery_app.exe` taken from the
zip; "More info", then "Run anyway" starts it. Files extracted from a
downloaded zip carry the mark of the web; right-click the zip, Properties,
Unblock, before extracting avoids the prompt for every file. Signing would
add a `signtool sign` step (code-signing certificate from repository
secrets) for `orrery_app.exe` before `cpack` and for the `.msi` after it;
the workflow has no such secrets. The AppImage is not signed either (no
embedded signature); the `.sha256` files beside every package let a
download be checked.

## 8. Uninstalling

- **MSI:** Settings, Apps, Installed apps, Orrery, Uninstall (or
  `msiexec /x orrery-<version>-windows-x64.msi`). Removes the installed
  files and the Start menu shortcut. The user's own files stay (section 9).
- **zip, tarball, AppImage:** delete the extracted folder or the file.

## 9. Configuration, data and logs after installation

None of these are inside the installed tree; they belong to the user and
survive an uninstall or upgrade (delete them by hand for a clean slate):

| What | Windows | Linux |
|---|---|---|
| configuration (`config.toml`) | `%APPDATA%\orrery\config` | `$XDG_CONFIG_HOME/orrery`, else `~/.config/orrery` |
| saved experiments | `%APPDATA%\orrery\data\experiments` | `$XDG_DATA_HOME/orrery/experiments`, else `~/.local/share/orrery/experiments` |
| crash log (`crash.log`) | `%APPDATA%\orrery\data` | `$XDG_DATA_HOME/orrery`, else `~/.local/share/orrery` |
| downloaded models | `%LOCALAPPDATA%\orrery\cache\models` | `$XDG_CACHE_HOME/orrery/models`, else `~/.cache/orrery/models` |

The window layout is defined in code (no `imgui.ini`), and the application
writes nothing next to its executable, so it runs from a read-only location
(Program Files, a mounted AppImage). Details: docs/app.md sections 10 and
11, url-import-security.md.

## 10. Release workflow

`.github/workflows/release.yml` runs on `workflow_dispatch` and on tags
`v*`, never on pull requests; it is not one of the required checks. Jobs:

- **windows** (windows-2022): MSVC, the WiX tool, the icon check, `cmake
  --preset package`, the version check (`packaging/check-version.sh`), build,
  `ctest --preset package`, `cpack --preset package`; uploads the `.msi`,
  `.zip` and checksums as the `orrery-windows-x64` artifact.
- **linux** (ubuntu-22.04, clang 17): the same steps with `linux-package`,
  then the AppImage; uploads `orrery-linux-x86_64`.
- **publish** (tags only, `contents: write`): downloads both artifacts and
  attaches them to the GitHub release of the tag, creating it (with
  generated notes, marked pre-release for a `v<version>-<suffix>` tag) when
  it does not exist yet.

Actions are pinned by major tag like `ci.yml`; CMake, Ninja and the WiX tool
by exact version; the AppImage tools by SHA-256.

## 11. The application icon

`packaging/icon/make_icon.py` draws the icon (a sun inside a tilted orbit
with one planet, on a dark disc) analytically and writes `orrery.svg`,
`orrery.png` (256 px) and `orrery.ico` (16 to 256 px, PNG entries). It uses
only the Python standard library and no transcendental functions, so the
pixels are the same on every platform; the release workflow runs
`make_icon.py --check`. The ICO is the executable's `GLFW_ICON` resource
(`packaging/windows/orrery_app.rc.in`), so the file, its shortcut, the
taskbar and the window share it, and the MSI's entry in Installed apps uses
it too; Linux uses the PNG and SVG through the desktop entry.

## 12. What is checked where

| Check | Where |
|---|---|
| Windows: configure, build, install test, MSI and zip | locally and in the release workflow |
| MSI contents and the installed application | locally: administrative extract, then the extracted `orrery_app.exe` run headless with a system-only `PATH` |
| Linux: build, install test, tarball, AppImage of the real application | release workflow only (no Linux build environment locally) |
| `build-appimage.sh` end to end | locally under WSL with a stand-in executable: tools downloaded and verified, AppDir built, AppImage packed and run (`--version` found the assets) |
