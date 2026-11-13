#include "aero_overlay.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <map>

#include <glm/geometric.hpp>

#include "orrery/render/ui_theme.h"

namespace orrery::render::detail {

namespace {

constexpr double kPi = 3.141592653589793;
// Glyph units: cap height 10, x-height 7, descender -3; the stroke is 1.25
// units wide and letters are spaced by kTracking units.
constexpr double kCapUnits = 10.0;
constexpr double kStrokeUnits = 1.25;
constexpr double kTracking = 2.0;

using Stroke = std::vector<glm::dvec2>;

struct Glyph {
    double advance = 6.0;
    std::vector<Stroke> strokes;
};

Stroke polyline(std::initializer_list<double> xy) {
    Stroke s;
    s.reserve(xy.size() / 2);
    for (auto it = xy.begin(); it != xy.end() && std::next(it) != xy.end(); it += 2) {
        s.emplace_back(*it, *std::next(it));
    }
    return s;
}

// Elliptic arc from angle a0 to a1 (degrees, counter-clockwise for a1 > a0).
Stroke arc(double cx, double cy, double rx, double ry, double a0, double a1) {
    const int n = std::max(6, static_cast<int>(std::ceil(std::abs(a1 - a0) / 10.0)));
    Stroke s;
    s.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
        const double a = (a0 + (a1 - a0) * i / n) * kPi / 180.0;
        s.emplace_back(cx + rx * std::cos(a), cy + ry * std::sin(a));
    }
    return s;
}

Stroke dot(double x, double y) {
    return {{x, y}, {x, y + 0.15}};
}

const std::map<char, Glyph>& font() {
    static const std::map<char, Glyph> glyphs = [] {
        std::map<char, Glyph> g;
        g['0'] = {6.0, {arc(3.0, 5.0, 3.0, 5.0, 0.0, 360.0)}};
        g['1'] = {6.0, {polyline({1.0, 7.9, 3.4, 10.0, 3.4, 0.0})}};
        {
            Stroke two = arc(3.0, 7.0, 3.0, 3.0, 160.0, -32.0);
            two.emplace_back(0.0, 0.0);
            two.emplace_back(6.0, 0.0);
            g['2'] = {6.0, {two}};
        }
        g['3'] = {
            6.0,
            {arc(3.0, 7.55, 2.75, 2.45, 150.0, -90.0), arc(3.0, 2.55, 3.0, 2.55, 90.0, -152.0)}};
        g['4'] = {6.4, {polyline({4.6, 0.0, 4.6, 10.0, 0.0, 3.0, 6.4, 3.0})}};
        {
            Stroke top = polyline({5.6, 10.0, 1.0, 10.0, 0.55, 5.4});
            Stroke bowl = arc(3.0, 3.2, 3.0, 3.2, 136.0, -152.0);
            top.insert(top.end(), bowl.begin(), bowl.end());
            g['5'] = {6.0, {top}};
        }
        g['6'] = {6.0,
                  {arc(3.0, 3.1, 3.0, 3.1, 0.0, 360.0), arc(6.0, 3.1, 6.0, 6.9, 180.0, 104.0)}};
        g['7'] = {6.0, {polyline({0.0, 10.0, 6.0, 10.0, 2.0, 0.0})}};
        g['8'] = {6.0, {arc(3.0, 7.6, 2.6, 2.4, 0.0, 360.0), arc(3.0, 2.6, 3.0, 2.6, 0.0, 360.0)}};
        g['9'] = {6.0, {arc(3.0, 6.9, 3.0, 3.1, 0.0, 360.0), arc(0.0, 6.9, 6.0, 6.9, 0.0, -76.0)}};
        g['.'] = {1.6, {dot(0.8, 0.3)}};
        g[','] = {1.6, {polyline({1.0, 0.6, 0.2, -1.6})}};
        g['-'] = {4.6, {polyline({0.4, 4.2, 4.2, 4.2})}};
        g['+'] = {6.0, {polyline({0.4, 4.5, 5.6, 4.5}), polyline({3.0, 1.9, 3.0, 7.1})}};
        g['='] = {6.0, {polyline({0.4, 5.8, 5.6, 5.8}), polyline({0.4, 3.2, 5.6, 3.2})}};
        g[' '] = {3.2, {}};
        g['e'] = {6.0, {[] {
                      Stroke s = polyline({0.1, 3.6, 6.0, 3.6});
                      Stroke a = arc(3.0, 3.5, 2.95, 3.5, 2.0, 318.0);
                      s.insert(s.end(), a.begin(), a.end());
                      return s;
                  }()}};
        g['C'] = {7.0, {arc(3.8, 5.0, 3.8, 5.0, 42.0, 318.0)}};
        g['c'] = {5.6, {arc(3.2, 3.5, 3.0, 3.5, 45.0, 315.0)}};
        g['p'] = {6.0, {polyline({0.0, 7.0, 0.0, -3.0}), arc(3.0, 3.5, 3.0, 3.5, 0.0, 360.0)}};
        g['d'] = {6.0, {polyline({6.0, 10.0, 6.0, 0.0}), arc(3.0, 3.5, 3.0, 3.5, 0.0, 360.0)}};
        g['b'] = {6.0, {polyline({0.0, 10.0, 0.0, 0.0}), arc(3.0, 3.5, 3.0, 3.5, 0.0, 360.0)}};
        g['o'] = {6.0, {arc(3.0, 3.5, 3.0, 3.5, 0.0, 360.0)}};
        g['a'] = {6.0, {arc(3.0, 3.5, 3.0, 3.5, 0.0, 360.0), polyline({6.0, 7.0, 6.0, 0.0})}};
        g['g'] = {6.0,
                  {arc(3.0, 3.5, 3.0, 3.5, 0.0, 360.0), polyline({6.0, 7.0, 6.0, -0.8}),
                   arc(3.0, -0.8, 3.0, 2.2, 0.0, -160.0)}};
        g['u'] = {6.0,
                  {[] {
                       Stroke s = polyline({0.0, 7.0, 0.0, 3.0});
                       Stroke a = arc(3.0, 3.0, 3.0, 3.0, 180.0, 360.0);
                       s.insert(s.end(), a.begin(), a.end());
                       return s;
                   }(),
                   polyline({6.0, 7.0, 6.0, 0.0})}};
        g['n'] = {6.0, {polyline({0.0, 0.0, 0.0, 7.0}), [] {
                            Stroke a = arc(3.0, 4.0, 3.0, 3.0, 180.0, 0.0);
                            a.emplace_back(6.0, 0.0);
                            return a;
                        }()}};
        g['m'] = {9.4,
                  {polyline({0.0, 0.0, 0.0, 7.0}),
                   [] {
                       Stroke a = arc(2.35, 4.9, 2.35, 2.1, 180.0, 0.0);
                       a.emplace_back(4.7, 0.0);
                       return a;
                   }(),
                   [] {
                       Stroke a = arc(7.05, 4.9, 2.35, 2.1, 180.0, 0.0);
                       a.emplace_back(9.4, 0.0);
                       return a;
                   }()}};
        g['r'] = {4.4, {polyline({0.0, 0.0, 0.0, 7.0}), arc(3.6, 4.3, 3.6, 2.7, 180.0, 75.0)}};
        g['s'] = {
            5.6,
            {arc(2.8, 5.25, 2.55, 1.75, 30.0, 270.0), arc(2.8, 1.75, 2.8, 1.75, 90.0, -150.0)}};
        g['t'] = {
            4.0,
            {polyline({1.4, 9.4, 1.4, 1.3, 2.0, 0.1, 3.6, 0.1}), polyline({0.0, 7.0, 3.6, 7.0})}};
        g['i'] = {2.0, {polyline({1.0, 0.0, 1.0, 7.0}), dot(1.0, 9.3)}};
        g['l'] = {2.0, {polyline({1.0, 10.0, 1.0, 0.0})}};
        g['x'] = {6.0, {polyline({0.0, 0.0, 6.0, 7.0}), polyline({0.0, 7.0, 6.0, 0.0})}};
        g['v'] = {6.0, {polyline({0.0, 7.0, 3.0, 0.0, 6.0, 7.0})}};
        g['L'] = {5.6, {polyline({0.0, 10.0, 0.0, 0.0, 5.6, 0.0})}};
        g['D'] = {7.0, {[] {
                      Stroke s = polyline({2.6, 0.0, 0.0, 0.0, 0.0, 10.0, 2.6, 10.0});
                      Stroke a = arc(2.6, 5.0, 4.0, 5.0, 90.0, -90.0);
                      s.insert(s.end(), a.begin(), a.end());
                      return s;
                  }()}};
        g['Q'] = {7.6, {arc(3.8, 5.0, 3.8, 5.0, 0.0, 360.0), polyline({4.8, 2.0, 7.6, -0.8})}};
        g['R'] = {6.4,
                  {[] {
                       Stroke s = polyline({0.0, 0.0, 0.0, 10.0, 3.4, 10.0});
                       Stroke a = arc(3.4, 7.5, 2.6, 2.5, 90.0, -90.0);
                       s.insert(s.end(), a.begin(), a.end());
                       s.emplace_back(0.0, 5.0);
                       return s;
                   }(),
                   polyline({3.0, 5.0, 6.2, 0.0})}};
        g['U'] = {7.0, {[] {
                      Stroke s = polyline({0.0, 10.0, 0.0, 3.4});
                      Stroke a = arc(3.5, 3.4, 3.5, 3.4, 180.0, 360.0);
                      s.insert(s.end(), a.begin(), a.end());
                      s.emplace_back(7.0, 10.0);
                      return s;
                  }()}};
        g['['] = {3.0, {polyline({2.8, 11.0, 0.6, 11.0, 0.6, -2.0, 2.8, -2.0})}};
        g[']'] = {3.0, {polyline({0.2, 11.0, 2.4, 11.0, 2.4, -2.0, 0.2, -2.0})}};
        g['('] = {3.4, {arc(4.6, 4.5, 4.0, 7.0, 118.0, 242.0)}};
        g[')'] = {3.4, {arc(-1.2, 4.5, 4.0, 7.0, 62.0, -62.0)}};
        g['/'] = {4.6, {polyline({0.0, -1.5, 4.6, 11.0})}};
        g['|'] = {2.0, {polyline({1.0, 11.0, 1.0, -2.5})}};
        return g;
    }();
    return glyphs;
}

double segment_distance(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b) {
    const glm::dvec2 ab = b - a;
    const double length2 = glm::dot(ab, ab);
    const double t = length2 > 0.0 ? std::clamp(glm::dot(p - a, ab) / length2, 0.0, 1.0) : 0.0;
    return glm::length(p - (a + ab * t));
}

// Signed distance to a rounded rectangle (negative inside).
double rounded_rect_distance(const glm::dvec2& p, double x, double y, double w, double h,
                             double r) {
    const glm::dvec2 centre(x + 0.5 * w, y + 0.5 * h);
    const glm::dvec2 half(0.5 * w - r, 0.5 * h - r);
    const glm::dvec2 q = glm::abs(p - centre) - half;
    return glm::length(glm::max(q, glm::dvec2(0.0))) + std::min(std::max(q.x, q.y), 0.0) - r;
}

glm::dvec4 to_rgba(const UiColor& c, double alpha_scale = 1.0) {
    return {c.r, c.g, c.b, static_cast<double>(c.a) * alpha_scale};
}

} // namespace

Canvas::Canvas(int w, int h)
    : width(std::max(w, 1)), height(std::max(h, 1)),
      pixels(static_cast<std::size_t>(std::max(w, 1)) * static_cast<std::size_t>(std::max(h, 1)),
             glm::dvec4(0.0)) {}

void Canvas::blend(int x, int y, const glm::dvec4& color, double coverage) {
    if (x < 0 || y < 0 || x >= width || y >= height || !(coverage > 0.0)) {
        return;
    }
    const double a = std::clamp(color.a * coverage, 0.0, 1.0);
    glm::dvec4& d = pixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                           static_cast<std::size_t>(x)];
    d = glm::dvec4(glm::dvec3(color) * a, a) + d * (1.0 - a);
}

void Canvas::fill_rounded_rect(double x, double y, double w, double h, double radius,
                               const glm::dvec4& color) {
    const int x0 = static_cast<int>(std::floor(x)) - 1;
    const int y0 = static_cast<int>(std::floor(y)) - 1;
    const int x1 = static_cast<int>(std::ceil(x + w)) + 1;
    const int y1 = static_cast<int>(std::ceil(y + h)) + 1;
    for (int py = y0; py <= y1; ++py) {
        for (int px = x0; px <= x1; ++px) {
            const double d = rounded_rect_distance({px + 0.5, py + 0.5}, x, y, w, h, radius);
            blend(px, py, color, std::clamp(0.5 - d, 0.0, 1.0));
        }
    }
}

void Canvas::stroke_rounded_rect(double x, double y, double w, double h, double radius,
                                 double thickness, const glm::dvec4& color) {
    const int x0 = static_cast<int>(std::floor(x)) - 2;
    const int y0 = static_cast<int>(std::floor(y)) - 2;
    const int x1 = static_cast<int>(std::ceil(x + w)) + 2;
    const int y1 = static_cast<int>(std::ceil(y + h)) + 2;
    for (int py = y0; py <= y1; ++py) {
        for (int px = x0; px <= x1; ++px) {
            const double d = rounded_rect_distance({px + 0.5, py + 0.5}, x, y, w, h, radius);
            blend(px, py, color, std::clamp(0.5 * thickness + 0.5 - std::abs(d), 0.0, 1.0));
        }
    }
}

void Canvas::fill_colour_bar(double x, double y, double w, double h, Colormap map) {
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = static_cast<int>(std::ceil(x + w));
    const int y1 = static_cast<int>(std::ceil(y + h));
    for (int py = y0; py < y1; ++py) {
        const double t = 1.0 - std::clamp((py + 0.5 - y) / h, 0.0, 1.0);
        const glm::dvec3 c = colormap_srgb(map, t);
        const double cy = std::clamp(std::min(py + 1.0, y + h) - std::max(double(py), y), 0.0, 1.0);
        for (int px = x0; px < x1; ++px) {
            const double cx =
                std::clamp(std::min(px + 1.0, x + w) - std::max(double(px), x), 0.0, 1.0);
            blend(px, py, glm::dvec4(c, 1.0), cx * cy);
        }
    }
}

void Canvas::line(const glm::dvec2& a, const glm::dvec2& b, double w, const glm::dvec4& color) {
    const double pad = 0.5 * w + 1.0;
    const int x0 = static_cast<int>(std::floor(std::min(a.x, b.x) - pad));
    const int y0 = static_cast<int>(std::floor(std::min(a.y, b.y) - pad));
    const int x1 = static_cast<int>(std::ceil(std::max(a.x, b.x) + pad));
    const int y1 = static_cast<int>(std::ceil(std::max(a.y, b.y) + pad));
    for (int py = y0; py <= y1; ++py) {
        for (int px = x0; px <= x1; ++px) {
            const double d = segment_distance({px + 0.5, py + 0.5}, a, b);
            blend(px, py, color, std::clamp(0.5 * w + 0.5 - d, 0.0, 1.0));
        }
    }
}

double Canvas::text_width(std::string_view text, double cap_height) {
    const double scale = cap_height / kCapUnits;
    double units = 0.0;
    const auto& glyphs = font();
    for (std::size_t i = 0; i < text.size(); ++i) {
        const auto found = glyphs.find(text[i]);
        units += found != glyphs.end() ? found->second.advance : 6.0;
        if (i + 1 < text.size()) {
            units += kTracking;
        }
    }
    return units * scale + kStrokeUnits * scale;
}

void Canvas::text(double x, double y, std::string_view text, double cap_height,
                  const glm::dvec4& color, double align) {
    const double scale = cap_height / kCapUnits;
    const double half_width = 0.5 * kStrokeUnits * scale;
    double pen = x - align * text_width(text, cap_height) + half_width;
    const auto& glyphs = font();
    for (const char ch : text) {
        const auto found = glyphs.find(ch);
        if (found == glyphs.end()) {
            pen += (6.0 + kTracking) * scale;
            continue;
        }
        const Glyph& g = found->second;
        // Pixel bounds of the glyph and the segments in pixel space (y down).
        std::vector<std::array<glm::dvec2, 2>> segments;
        for (const Stroke& s : g.strokes) {
            for (std::size_t i = 0; i + 1 < s.size(); ++i) {
                segments.push_back({glm::dvec2(pen + s[i].x * scale, y - s[i].y * scale),
                                    glm::dvec2(pen + s[i + 1].x * scale, y - s[i + 1].y * scale)});
            }
        }
        if (!segments.empty()) {
            const double pad = half_width + 1.0;
            const int x0 = static_cast<int>(std::floor(pen - pad - 2.0 * scale));
            const int x1 = static_cast<int>(std::ceil(pen + (g.advance + 2.0) * scale + pad));
            const int y0 = static_cast<int>(std::floor(y - 12.0 * scale - pad));
            const int y1 = static_cast<int>(std::ceil(y + 3.5 * scale + pad));
            for (int py = y0; py <= y1; ++py) {
                for (int px = x0; px <= x1; ++px) {
                    double d = 1.0e9;
                    const glm::dvec2 p(px + 0.5, py + 0.5);
                    for (const auto& seg : segments) {
                        d = std::min(d, segment_distance(p, seg[0], seg[1]));
                    }
                    blend(px, py, color, std::clamp(half_width + 0.5 - d, 0.0, 1.0));
                }
            }
        }
        pen += (g.advance + kTracking) * scale;
    }
}

std::vector<std::uint8_t> Canvas::to_rgba8_bottom_up() const {
    std::vector<std::uint8_t> bytes(pixels.size() * 4, 0);
    for (int y = 0; y < height; ++y) {
        const auto row = static_cast<std::size_t>(height - 1 - y);
        for (int x = 0; x < width; ++x) {
            const glm::dvec4& p =
                pixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                       static_cast<std::size_t>(x)];
            const std::size_t o =
                (row * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)) * 4;
            for (int c = 0; c < 4; ++c) {
                bytes[o + static_cast<std::size_t>(c)] =
                    static_cast<std::uint8_t>(std::lround(std::clamp(p[c], 0.0, 1.0) * 255.0));
            }
        }
    }
    return bytes;
}

OverlayStyle overlay_style() {
    const UiTheme theme = default_theme();
    OverlayStyle style;
    style.panel = to_rgba(theme.window, 0.88);
    style.border = to_rgba(theme.border);
    style.text = to_rgba(theme.text);
    style.muted = to_rgba(theme.text_muted);
    style.bar_outline = {0.0, 0.0, 0.0, 0.55};
    return style;
}

} // namespace orrery::render::detail
