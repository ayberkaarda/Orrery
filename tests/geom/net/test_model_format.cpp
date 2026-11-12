#include "net_test_support.h"

#include "orrery/geom/net/model_format.h"

#include <catch2/catch_test_macros.hpp>

#include <string>

using orrery::geom::net::content_type_allowed;
using orrery::geom::net::content_type_matches;
using orrery::geom::net::detect_file_format;
using orrery::geom::net::detect_model_format;
using orrery::geom::net::ModelFormat;
using namespace orrery::geom::net::test;

namespace {

ModelFormat detect(const std::string& content) {
    return detect_model_format(bytes_of(content), content.size());
}

std::string glb(std::uint32_t version, std::uint32_t declared_length, std::size_t size) {
    std::string out = "glTF";
    for (const std::uint32_t value : {version, declared_length}) {
        for (unsigned i = 0; i < 4; ++i) {
            out.push_back(static_cast<char>((value >> (8U * i)) & 0xFFU));
        }
    }
    out.resize(size, ' ');
    return out;
}

} // namespace

TEST_CASE("Model formats are detected from magic bytes", "[net][format]") {
    CHECK(detect(binary_stl(1)) == ModelFormat::stl_binary);
    CHECK(detect(binary_stl(3)) == ModelFormat::stl_binary);
    CHECK(detect(ascii_stl()) == ModelFormat::stl_ascii);
    CHECK(detect(obj_text()) == ModelFormat::obj);
    CHECK(detect("\xEF\xBB\xBF" + obj_text()) == ModelFormat::obj);
    CHECK(detect(glb(2, 64, 64)) == ModelFormat::glb);
    CHECK(detect(R"({"asset": {"version": "2.0"}, "meshes": []})") == ModelFormat::gltf);
    CHECK(detect("ply\nformat ascii 1.0\nelement vertex 0\nend_header\n") == ModelFormat::ply);
    CHECK(detect("ply\r\nformat binary_little_endian 1.0\r\nend_header\r\n") == ModelFormat::ply);
    CHECK(detect(std::string{"Kaydara FBX Binary  \0\x1a\0", 23} + std::string(40, '\x01')) ==
          ModelFormat::fbx);
    CHECK(detect("; FBX 7.4.0 project file\nFBXHeaderExtension: {\n}\n") == ModelFormat::fbx);
    CHECK(detect(std::string{"PK\x03\x04", 4} + std::string(40, '\0')) == ModelFormat::zip);
}

TEST_CASE("A binary STL whose header begins with 'solid' is still binary", "[net][format]") {
    std::string stl = binary_stl(2);
    stl.replace(0, 6, "solid ");
    CHECK(detect(stl) == ModelFormat::stl_binary);
}

TEST_CASE("Content that is not a model is not recognised", "[net][format]") {
    CHECK(detect(executable_bytes()) == ModelFormat::unknown);
    CHECK(detect(std::string{"\x7f"
                             "ELF\x02\x01\x01",
                             7} +
                 std::string(100, '\0')) == ModelFormat::unknown);
    CHECK(detect("<!DOCTYPE html><html><body>model.stl</body></html>") == ModelFormat::unknown);
    CHECK(detect("#!/bin/sh\nrm -rf /\n") == ModelFormat::unknown);
    CHECK(detect("{\"name\": \"not a glTF\"}") == ModelFormat::unknown);
    CHECK(detect("solid object, nothing more\n") == ModelFormat::unknown);
    CHECK(detect("v 0 0 0\nnot an obj statement\n") == ModelFormat::unknown);
    CHECK(detect("# only comments\n") == ModelFormat::unknown);
    CHECK(detect("") == ModelFormat::unknown);
    // A binary STL size that does not match its triangle count.
    std::string truncated = binary_stl(2);
    truncated.pop_back();
    CHECK(detect(truncated) == ModelFormat::unknown);
    CHECK(detect(binary_stl(0)) == ModelFormat::unknown);
    // A GLB whose declared length differs from the real size, or version 1.
    CHECK(detect(glb(2, 128, 64)) == ModelFormat::unknown);
    CHECK(detect(glb(1, 64, 64)) == ModelFormat::unknown);
    // NUL bytes inside otherwise OBJ-like text.
    CHECK(detect(std::string{"v 0 0 0\0\nf 1 1 1\n", 17}) == ModelFormat::unknown);
}

TEST_CASE("The file name never decides the format", "[net][format]") {
    const TempDir dir;
    const auto exe_named_stl = dir.path() / "part.stl";
    write_file(exe_named_stl, executable_bytes());
    const auto stl_named_exe = dir.path() / "setup.exe";
    write_file(stl_named_exe, binary_stl(1));

    const auto exe = detect_file_format(exe_named_stl);
    REQUIRE(exe.has_value());
    CHECK(exe.value() == ModelFormat::unknown);
    const auto stl = detect_file_format(stl_named_exe);
    REQUIRE(stl.has_value());
    CHECK(stl.value() == ModelFormat::stl_binary);
}

TEST_CASE("Content-Type allowlist and consistency with the content", "[net][format][mime]") {
    for (const char* allowed :
         {"", "application/octet-stream", "Application/Octet-Stream; charset=binary",
          "text/plain; charset=utf-8", "model/stl", "model/gltf-binary", "model/gltf+json",
          "application/zip", "model/obj"}) {
        INFO(allowed);
        CHECK(content_type_allowed(allowed));
    }
    for (const char* rejected :
         {"text/html", "text/html; charset=utf-8", "application/x-msdownload",
          "application/javascript", "image/png", "application/x-sh", "text/xml",
          "application/x-executable", "application/vnd.microsoft.portable-executable"}) {
        INFO(rejected);
        CHECK_FALSE(content_type_allowed(rejected));
    }
    CHECK(content_type_matches("application/octet-stream", ModelFormat::glb));
    CHECK(content_type_matches("", ModelFormat::zip));
    CHECK(content_type_matches("model/stl", ModelFormat::stl_ascii));
    CHECK(content_type_matches("model/stl", ModelFormat::stl_binary));
    CHECK_FALSE(content_type_matches("model/stl", ModelFormat::obj));
    CHECK_FALSE(content_type_matches("model/gltf-binary", ModelFormat::zip));
    CHECK_FALSE(content_type_matches("application/zip", ModelFormat::stl_binary));
    CHECK_FALSE(content_type_matches("text/html", ModelFormat::obj));
}
