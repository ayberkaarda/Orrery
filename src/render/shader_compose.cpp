#include "orrery/render/shader_compose.h"

#include <cstddef>
#include <regex>

namespace orrery::render {

core::Result<ComposedShader, std::string> compose_shader(const ShaderSource& main,
                                                         std::span<const ShaderSource> includes) {
    const std::string_view code = main.code;
    const std::size_t first_end = code.find('\n');
    const std::string_view first_line = code.substr(0, first_end);
    if (!first_line.starts_with("#version")) {
        return core::failure(std::string(main.name) +
                             ":1: the first line must be the #version directive");
    }

    ComposedShader composed;
    composed.names.emplace_back(main.name);
    std::string version(first_line);
    if (!version.empty() && version.back() == '\r') {
        version.pop_back();
    }
    composed.strings.push_back(version + "\n");
    for (std::size_t i = 0; i < includes.size(); ++i) {
        std::string piece = "#line 1 " + std::to_string(i + 1) + "\n";
        piece += includes[i].code;
        if (piece.back() != '\n') {
            piece += '\n';
        }
        composed.strings.push_back(std::move(piece));
        composed.names.emplace_back(includes[i].name);
    }
    std::string rest = "#line 2 0\n";
    if (first_end != std::string_view::npos) {
        rest += code.substr(first_end + 1);
    }
    composed.strings.push_back(std::move(rest));
    return composed;
}

std::string annotate_shader_log(std::string_view log, std::span<const std::string> names) {
    // "<string>:<line>" (AMD, Mesa, Intel) or "<string>(<line>)" (NVIDIA),
    // not preceded by an identifier character.
    static const std::regex reference(R"((^|[^0-9A-Za-z_])([0-9]+)(?::([0-9]+)|\(([0-9]+)\)))");
    std::string out;
    std::size_t start = 0;
    while (start < log.size()) {
        std::size_t end = log.find('\n', start);
        if (end == std::string_view::npos) {
            end = log.size();
        }
        const std::string line(log.substr(start, end - start));
        std::smatch match;
        if (std::regex_search(line, match, reference) && match[2].length() <= 6) {
            const unsigned long index = std::stoul(match[2].str());
            if (index < names.size()) {
                const std::string number = match[3].matched ? match[3].str() : match[4].str();
                out += match.prefix().str();
                out += match[1].str();
                out += names[index];
                out += ':';
                out += number;
                out += match.suffix().str();
            } else {
                out += line;
            }
        } else {
            out += line;
        }
        out += '\n';
        start = end + 1;
    }
    return out;
}

} // namespace orrery::render
