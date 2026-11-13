# Third-party notices of the packages (docs/packaging.md, "Third-party
# notices"). Used by cmake/Packaging.cmake to install them and by
# packaging/check_notices.cmake to check an installed tree against them.
#
# Every vcpkg port the build uses installs its copyright file, which
# cmake/Packaging.cmake copies to share/doc/orrery/third_party/<port>/.
# Some libraries require more than that file: those notices are kept here,
# copied verbatim from the upstream release the port builds, and pinned by
# SHA-256 and by the port version they belong to. When vcpkg moves the port
# to another version, configuring fails until the notice is refreshed.

# <port>|<file in packaging/third_party/<port>/>|<port version>|<SHA-256>|<source>
set(ORRERY_EXTRA_NOTICES
    # oneTBB: tbb12.dll (Windows) and the static library (Linux) contain the
    # ITT Notify API, whose BSD licence requires its notice in binary
    # distributions; the port's copyright file holds only Apache-2.0.
    "tbb|third-party-programs.txt|2023.2.0|1634857eb058dce5278c57a832b8974190785145f4be9acdbdb511f3a0c4091c|https://github.com/uxlfoundation/oneTBB/blob/v2023.2.0/third-party-programs.txt"
)

# Windows: every DLL installed beside the executable, by file name (regular
# expression), and the vcpkg port whose notices cover it. A DLL that matches
# no entry fails the install test, so a new dependency cannot ship without
# its notice being looked at.
set(ORRERY_BUNDLED_DLLS
    "^assimp-.*\\.dll$|assimp"
    "^glfw3\\.dll$|glfw3"
    "^kubazip\\.dll$|kubazip"
    "^libcrypto-.*\\.dll$|openssl"
    "^libssl-.*\\.dll$|openssl"
    "^libcurl\\.dll$|curl"
    "^miniz\\.dll$|miniz"
    "^minizip\\.dll$|minizip"
    "^poly2tri\\.dll$|jhasse-poly2tri"
    "^pugixml\\.dll$|pugixml"
    "^tbb12\\.dll$|tbb"
    "^tomlplusplus-.*\\.dll$|tomlplusplus"
    "^z\\.dll$|zlib"
)

# The Microsoft Visual C++ runtime, installed app-local by
# InstallRequiredSystemLibraries: distributed under the distributable-code
# terms of the Visual Studio licence, not under a vcpkg port.
set(ORRERY_SYSTEM_RUNTIME_DLLS "^(concrt140|msvcp140(_[a-z0-9_]+)?|vcruntime140(_[0-9]+)?)\\.dll$")
