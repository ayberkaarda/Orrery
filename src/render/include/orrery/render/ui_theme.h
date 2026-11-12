#pragma once

// Visual theme of the user interface (docs/rendering.md, "User interface").
// Colours are display-referred sRGB values in [0, 1], as Dear ImGui uses
// them; the theme is applied to the ImGui style by UiContext. Plain data so
// that contrast rules are unit-tested without a UI context.
namespace orrery::render {

struct UiColor {
    float r = 0.0F;
    float g = 0.0F;
    float b = 0.0F;
    float a = 1.0F;

    friend constexpr bool operator==(const UiColor&, const UiColor&) = default;
};

struct UiTheme {
    // Surfaces, darkest to lightest.
    UiColor title{};   // title bars, tab bar background
    UiColor window{};  // panel background
    UiColor inset{};   // child regions, plot background
    UiColor control{}; // input frames, buttons at rest
    UiColor control_hover{};
    UiColor control_active{};
    UiColor border{};
    UiColor separator{};
    // Text.
    UiColor text{};
    UiColor text_muted{}; // labels, units
    UiColor text_disabled{};
    // Accent (sunlight amber) and a cool secondary for second data series.
    UiColor accent{};
    UiColor accent_hover{};
    UiColor accent_active{};
    UiColor accent_secondary{};
    UiColor warning{};
    // Metrics [px].
    float font_size = 15.0F;
    float rounding = 4.0F;
    float window_rounding = 6.0F;
    float window_padding_x = 12.0F;
    float window_padding_y = 10.0F;
    float frame_padding_x = 8.0F;
    float frame_padding_y = 4.0F;
    float item_spacing_x = 8.0F;
    float item_spacing_y = 6.0F;
};

// The Orrery dark theme: slate surfaces close to the black of space, light
// grey text, one warm accent for interaction and data.
[[nodiscard]] UiTheme default_theme() noexcept;

// WCAG 2.x relative luminance of an sRGB colour (alpha ignored).
[[nodiscard]] double relative_luminance(const UiColor& color) noexcept;
// WCAG contrast ratio (L1 + 0.05) / (L2 + 0.05) of two opaque colours, >= 1.
[[nodiscard]] double contrast_ratio(const UiColor& a, const UiColor& b) noexcept;
// `over` composited onto an opaque `under` (straight alpha, in sRGB values as
// the GPU blends them).
[[nodiscard]] UiColor composite(const UiColor& over, const UiColor& under) noexcept;

} // namespace orrery::render
