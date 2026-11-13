#pragma once

#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include <glm/ext/vector_double2.hpp>
#include <glm/ext/vector_double4.hpp>

#include "orrery/render/aero_colormap.h"

// Display-referred overlay of the aero view, rasterised on the CPU: legend
// panels (colour bar, ticks, labels) and coefficient read-outs. Text uses a
// small built-in stroke font (monoline glyphs drawn as anti-aliased
// capsules), so the renderer needs no font library and no user-interface
// code. Colours are sRGB-encoded with straight alpha on input; the canvas
// stores premultiplied sRGB, which the overlay pass blends with ONE,
// ONE_MINUS_SRC_ALPHA (blending in sRGB values, as Dear ImGui does).
namespace orrery::render::detail {

struct Canvas {
    int width = 0;
    int height = 0;
    std::vector<glm::dvec4> pixels; // premultiplied sRGB, rows top to bottom

    Canvas(int w, int h);

    // Source-over of `color` (straight alpha) with coverage in [0, 1].
    void blend(int x, int y, const glm::dvec4& color, double coverage);
    // Anti-aliased rounded rectangle; (x, y) top-left, pixels.
    void fill_rounded_rect(double x, double y, double w, double h, double radius,
                           const glm::dvec4& color);
    // Outline of the same shape, `thickness` pixels wide.
    void stroke_rounded_rect(double x, double y, double w, double h, double radius,
                             double thickness, const glm::dvec4& color);
    // A vertical colour bar from `map`: t = 1 at the top.
    void fill_colour_bar(double x, double y, double w, double h, Colormap map);
    // Anti-aliased line segment (round caps).
    void line(const glm::dvec2& a, const glm::dvec2& b, double width, const glm::dvec4& color);

    // Text with its baseline at y; `cap_height` pixels per capital letter.
    // align: 0 left, 0.5 centred, 1 right at x.
    void text(double x, double y, std::string_view text, double cap_height, const glm::dvec4& color,
              double align = 0.0);
    [[nodiscard]] static double text_width(std::string_view text, double cap_height);

    // 8-bit premultiplied sRGB RGBA, rows bottom to top (OpenGL upload order).
    [[nodiscard]] std::vector<std::uint8_t> to_rgba8_bottom_up() const;
};

// Theme colours of the overlay (sRGB, straight alpha), from ui_theme.
struct OverlayStyle {
    glm::dvec4 panel{0.0};
    glm::dvec4 border{0.0};
    glm::dvec4 text{0.0};
    glm::dvec4 muted{0.0};
    glm::dvec4 bar_outline{0.0};
};
[[nodiscard]] OverlayStyle overlay_style();

} // namespace orrery::render::detail
