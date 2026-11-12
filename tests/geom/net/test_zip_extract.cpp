#include "net_test_support.h"

#include "orrery/geom/net/zip_extract.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

using orrery::geom::net::ArchiveLimits;
using orrery::geom::net::extract_archive_models;
using orrery::geom::net::FetchErrorKind;
using orrery::geom::net::is_safe_member_name;
using orrery::geom::net::ModelFormat;
using namespace orrery::geom::net::test;

namespace {

struct Extraction {
    TempDir dir;
    std::filesystem::path out;

    explicit Extraction(const std::string& archive) : out(dir.path() / "out") {
        write_file(dir.path() / "archive.zip", archive);
        std::filesystem::create_directories(out);
    }

    auto run(const ArchiveLimits& limits = {}) {
        return extract_archive_models(dir.path() / "archive.zip", out, limits);
    }
};

FetchErrorKind rejection(const std::vector<ZipMember>& members, const ArchiveLimits& limits = {}) {
    Extraction extraction{build_zip(members)};
    const auto result = extraction.run(limits);
    REQUIRE_FALSE(result.has_value());
    INFO(result.error().message);
    // Nothing is left behind by a rejected archive.
    CHECK(count_files(extraction.out) == 0);
    return result.error().kind;
}

} // namespace

TEST_CASE("Archive member names must be plain relative paths", "[net][zip]") {
    for (const char* safe : {"wing.obj", "models/wing.obj", "a..b.stl", "dir/./x.obj",
                             "deep/er/path/model.glb", "models\\wing.obj"}) {
        INFO(safe);
        CHECK(is_safe_member_name(safe));
    }
    for (const char* unsafe :
         {"", "../evil.obj", "models/../../evil.obj", "a\\..\\..\\evil.obj", "..", "x/..",
          "/etc/evil.obj", "\\evil.obj", "C:\\evil.obj", "C:evil.obj", "model.obj:stream",
          "bad\nname.obj", "bad\x01name.obj"}) {
        INFO(unsafe);
        CHECK_FALSE(is_safe_member_name(unsafe));
    }
}

TEST_CASE("A zip extracts only its model members, in archive order", "[net][zip]") {
    const std::string nested = build_zip({{"inner.stl", binary_stl(1)}});
    Extraction extraction{build_zip({
        {"readme.txt", "This archive contains a wing.\n", true},
        {"models/wing.obj", obj_text(), true},
        {"nested.zip", nested},
        {"setup.stl", executable_bytes()},
        {"models/", ""},
        {"parts/hull.bin", binary_stl(2)},
    })};
    const auto result = extraction.run();
    REQUIRE(result.has_value());
    const auto& models = result.value();
    REQUIRE(models.size() == 2);
    CHECK(models[0].format == ModelFormat::obj);
    CHECK(models[0].member_name == "models/wing.obj");
    CHECK(models[0].file.extension() == ".obj");
    CHECK(read_file(models[0].file) == obj_text());
    CHECK(models[1].format == ModelFormat::stl_binary);
    CHECK(models[1].file.extension() == ".stl"); // from the content, not ".bin"
    CHECK(read_file(models[1].file) == binary_stl(2));
    // The text file, the nested archive and the executable never reach the disk.
    CHECK(count_files(extraction.out) == 2);
    for (const auto& model : models) {
        CHECK(model.file.parent_path() == extraction.out);
    }
}

TEST_CASE("Zip bomb: an entry with a compression ratio above 100 is rejected", "[net][zip]") {
    // One megabyte of zeros deflates to about a kilobyte (ratio ~ 1000).
    CHECK(rejection({{"bomb.stl", std::string(1 << 20, '\0'), true}}) ==
          FetchErrorKind::archive_bomb);
    // The same data passes when the limit allows its ratio.
    Extraction lenient{build_zip({{"zeros.obj", "v 0 0 0\n" + std::string(1 << 16, '\n'), true}})};
    ArchiveLimits limits;
    limits.max_compression_ratio = 100'000;
    CHECK(lenient.run(limits).has_value());
    // And a modest ratio is rejected when the limit is lower than it.
    limits.max_compression_ratio = 2;
    CHECK(rejection({{"wing.obj", obj_text() + std::string(4096, '#'), true}}, limits) ==
          FetchErrorKind::archive_bomb);
}

TEST_CASE("Zip bomb: an entry that expands beyond its declared size is rejected", "[net][zip]") {
    ZipMember liar{"wing.obj", "v 0 0 0\n" + std::string(200'000, '\n'), true};
    liar.declared_size = 1000; // honest-looking ratio, false size
    CHECK(rejection({liar}) == FetchErrorKind::archive_bomb);
}

TEST_CASE("Zip bomb: the total uncompressed size is capped", "[net][zip]") {
    ArchiveLimits limits;
    limits.max_total_uncompressed_bytes = 1000;
    CHECK(rejection({{"a.stl", binary_stl(10)}, {"b.stl", binary_stl(10)}}, limits) ==
          FetchErrorKind::archive_bomb);
}

TEST_CASE("Zip with path traversal or absolute names is rejected", "[net][zip]") {
    for (const char* name : {"../evil.obj", "models/../../evil.obj", "/tmp/evil.obj", "\\evil.obj",
                             "C:\\Windows\\evil.obj", "C:evil.obj", "a\\..\\..\\evil.obj"}) {
        INFO(name);
        CHECK(rejection({{"good.obj", obj_text()}, {name, obj_text()}}) ==
              FetchErrorKind::archive_unsafe_entry);
    }
}

TEST_CASE("Zip with a symbolic link or an encrypted entry is rejected", "[net][zip]") {
    ZipMember link{"wing.obj", "/etc/passwd"};
    link.unix_mode = 0120777;
    CHECK(rejection({link}) == FetchErrorKind::archive_unsafe_entry);

    ZipMember regular{"wing.obj", obj_text()};
    regular.unix_mode = 0100644;
    Extraction ok{build_zip({regular})};
    CHECK(ok.run().has_value());

    ZipMember encrypted{"wing.obj", obj_text()};
    encrypted.flags = 1;
    CHECK(rejection({encrypted}) == FetchErrorKind::archive_unsafe_entry);
}

TEST_CASE("Zip with too many entries is rejected", "[net][zip]") {
    std::vector<ZipMember> members;
    for (int i = 0; i < 1001; ++i) {
        members.push_back({"m" + std::to_string(i) + ".txt", "x"});
    }
    CHECK(rejection(members) == FetchErrorKind::archive_too_many_entries);

    ArchiveLimits limits;
    limits.max_entries = 3;
    CHECK(rejection({{"a.obj", obj_text()},
                     {"b.obj", obj_text()},
                     {"c.obj", obj_text()},
                     {"d.obj", obj_text()}},
                    limits) == FetchErrorKind::archive_too_many_entries);
}

TEST_CASE("Zip without a model, or with only a nested archive, is rejected", "[net][zip]") {
    CHECK(rejection({{"readme.txt", "hello\n"}}) == FetchErrorKind::archive_without_model);
    CHECK(rejection({{"inner.zip", build_zip({{"wing.obj", obj_text()}})}}) ==
          FetchErrorKind::archive_without_model);
    CHECK(rejection({}) == FetchErrorKind::archive_without_model);
}

TEST_CASE("Corrupt zip data is rejected", "[net][zip]") {
    std::string archive = build_zip({{"wing.obj", obj_text(), true}});
    Extraction truncated{archive.substr(0, archive.size() / 2)};
    const auto result = truncated.run();
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == FetchErrorKind::archive_invalid);
}
