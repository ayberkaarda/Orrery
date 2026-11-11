#include "orrery/render/ui_context.h"

#include <algorithm>
#include <string>
#include <utility>

#include "gl_context_internal.h"
#include "gl_loader.h"
#include "orrery/render/ui_panels.h"

// Dear ImGui after the loader: the OpenGL 3 backend brings its own loader and
// is compiled separately by the imgui port; only its header is used here.
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_internal.h>
#include <implot.h>

namespace orrery::render {

namespace {

ImVec4 im(const UiColor& c, float alpha_scale = 1.0F) {
    return {c.r, c.g, c.b, c.a * alpha_scale};
}

ImVec4 with_alpha(const UiColor& c, float alpha) {
    return {c.r, c.g, c.b, alpha};
}

void apply_theme(const UiTheme& t) {
    ImGuiStyle& style = ImGui::GetStyle();
    style = ImGuiStyle();
    style.FontSizeBase = t.font_size;
    style.WindowPadding = ImVec2(t.window_padding_x, t.window_padding_y);
    style.FramePadding = ImVec2(t.frame_padding_x, t.frame_padding_y);
    style.ItemSpacing = ImVec2(t.item_spacing_x, t.item_spacing_y);
    style.ItemInnerSpacing = ImVec2(6.0F, 4.0F);
    style.CellPadding = ImVec2(6.0F, 3.0F);
    style.IndentSpacing = 14.0F;
    style.ScrollbarSize = 10.0F;
    style.GrabMinSize = 5.0F;
    style.WindowRounding = t.window_rounding;
    style.ChildRounding = t.rounding;
    style.FrameRounding = t.rounding;
    style.PopupRounding = t.rounding;
    style.ScrollbarRounding = 6.0F;
    style.GrabRounding = 2.0F;
    style.TabRounding = t.rounding;
    style.WindowBorderSize = 1.0F;
    style.ChildBorderSize = 1.0F;
    style.FrameBorderSize = 0.0F;
    style.PopupBorderSize = 1.0F;
    style.TabBorderSize = 0.0F;
    style.WindowTitleAlign = ImVec2(0.0F, 0.5F);
    style.SeparatorTextBorderSize = 1.0F;
    style.SeparatorTextPadding = ImVec2(0.0F, 4.0F);
    style.DockingSeparatorSize = 1.0F;
    style.SelectableTextAlign = ImVec2(0.0F, 0.5F);
    style.AntiAliasedLines = true;
    style.AntiAliasedFill = true;

    ImVec4* c = style.Colors;
    const ImVec4 clear(0.0F, 0.0F, 0.0F, 0.0F);
    c[ImGuiCol_Text] = im(t.text);
    c[ImGuiCol_TextDisabled] = im(t.text_disabled);
    c[ImGuiCol_WindowBg] = im(t.window);
    c[ImGuiCol_ChildBg] = clear;
    c[ImGuiCol_PopupBg] = with_alpha(t.window, 0.98F);
    c[ImGuiCol_Border] = im(t.border);
    c[ImGuiCol_BorderShadow] = clear;
    c[ImGuiCol_FrameBg] = im(t.control);
    c[ImGuiCol_FrameBgHovered] = im(t.control_hover);
    c[ImGuiCol_FrameBgActive] = im(t.control_active);
    c[ImGuiCol_TitleBg] = im(t.title);
    c[ImGuiCol_TitleBgActive] = im(t.title);
    c[ImGuiCol_TitleBgCollapsed] = im(t.title);
    c[ImGuiCol_MenuBarBg] = im(t.title);
    c[ImGuiCol_ScrollbarBg] = clear;
    c[ImGuiCol_ScrollbarGrab] = im(t.control_hover);
    c[ImGuiCol_ScrollbarGrabHovered] = im(t.control_active);
    c[ImGuiCol_ScrollbarGrabActive] = im(t.accent_active);
    c[ImGuiCol_CheckMark] = im(t.accent);
    c[ImGuiCol_SliderGrab] = with_alpha(t.accent, 0.85F);
    c[ImGuiCol_SliderGrabActive] = im(t.accent_hover);
    c[ImGuiCol_Button] = im(t.control);
    c[ImGuiCol_ButtonHovered] = im(t.control_hover);
    c[ImGuiCol_ButtonActive] = im(t.control_active);
    c[ImGuiCol_Header] = im(t.control);
    c[ImGuiCol_HeaderHovered] = im(t.control_hover);
    c[ImGuiCol_HeaderActive] = im(t.control_active);
    c[ImGuiCol_Separator] = im(t.separator);
    c[ImGuiCol_SeparatorHovered] = with_alpha(t.accent, 0.6F);
    c[ImGuiCol_SeparatorActive] = im(t.accent);
    c[ImGuiCol_ResizeGrip] = clear;
    c[ImGuiCol_ResizeGripHovered] = with_alpha(t.accent, 0.5F);
    c[ImGuiCol_ResizeGripActive] = im(t.accent);
    c[ImGuiCol_InputTextCursor] = im(t.accent);
    c[ImGuiCol_TabHovered] = im(t.control_hover);
    c[ImGuiCol_Tab] = im(t.title);
    c[ImGuiCol_TabSelected] = with_alpha(t.window, 1.0F);
    c[ImGuiCol_TabSelectedOverline] = im(t.accent);
    c[ImGuiCol_TabDimmed] = im(t.title);
    c[ImGuiCol_TabDimmedSelected] = with_alpha(t.window, 1.0F);
    c[ImGuiCol_TabDimmedSelectedOverline] = with_alpha(t.accent, 0.45F);
    c[ImGuiCol_DockingPreview] = with_alpha(t.accent, 0.30F);
    c[ImGuiCol_DockingEmptyBg] = clear;
    c[ImGuiCol_PlotLines] = im(t.accent);
    c[ImGuiCol_PlotLinesHovered] = im(t.accent_hover);
    c[ImGuiCol_PlotHistogram] = im(t.accent);
    c[ImGuiCol_PlotHistogramHovered] = im(t.accent_hover);
    c[ImGuiCol_TableHeaderBg] = im(t.control);
    c[ImGuiCol_TableBorderStrong] = im(t.border);
    c[ImGuiCol_TableBorderLight] = im(t.separator);
    c[ImGuiCol_TableRowBg] = clear;
    c[ImGuiCol_TableRowBgAlt] = ImVec4(1.0F, 1.0F, 1.0F, 0.02F);
    c[ImGuiCol_TextLink] = im(t.accent_secondary);
    c[ImGuiCol_TextSelectedBg] = with_alpha(t.accent, 0.25F);
    c[ImGuiCol_DragDropTarget] = im(t.accent);
    c[ImGuiCol_NavCursor] = im(t.accent);
    c[ImGuiCol_NavWindowingHighlight] = ImVec4(1.0F, 1.0F, 1.0F, 0.6F);
    c[ImGuiCol_NavWindowingDimBg] = ImVec4(0.0F, 0.0F, 0.0F, 0.55F);
    c[ImGuiCol_ModalWindowDimBg] = ImVec4(0.0F, 0.0F, 0.0F, 0.55F);

    ImPlotStyle& plot = ImPlot::GetStyle();
    plot = ImPlotStyle();
    plot.PlotPadding = ImVec2(8.0F, 8.0F);
    plot.LabelPadding = ImVec2(4.0F, 3.0F);
    plot.LegendPadding = ImVec2(8.0F, 6.0F);
    plot.PlotBorderSize = 1.0F;
    ImVec4* p = plot.Colors;
    p[ImPlotCol_FrameBg] = clear;
    p[ImPlotCol_PlotBg] = im(t.inset);
    p[ImPlotCol_PlotBorder] = im(t.border);
    p[ImPlotCol_LegendBg] = with_alpha(t.title, 0.85F);
    p[ImPlotCol_LegendBorder] = im(t.border);
    p[ImPlotCol_LegendText] = im(t.text);
    p[ImPlotCol_TitleText] = im(t.text);
    p[ImPlotCol_InlayText] = im(t.text_muted);
    p[ImPlotCol_AxisText] = im(t.text_muted);
    p[ImPlotCol_AxisGrid] = with_alpha(t.text, 0.07F);
    p[ImPlotCol_AxisTick] = with_alpha(t.text, 0.18F);
    p[ImPlotCol_AxisBg] = clear;
    p[ImPlotCol_AxisBgHovered] = im(t.control_hover);
    p[ImPlotCol_AxisBgActive] = im(t.control_active);
    p[ImPlotCol_Selection] = im(t.accent);
    p[ImPlotCol_Crosshairs] = im(t.text_muted);
}

// Makes an ImGui and ImPlot context pair current for the lifetime of the
// guard, restoring the previous pair afterwards.
class ContextGuard {
public:
    ContextGuard(ImGuiContext* imgui, ImPlotContext* implot)
        : previous_imgui_(ImGui::GetCurrentContext()),
          previous_implot_(ImPlot::GetCurrentContext()) {
        ImGui::SetCurrentContext(imgui);
        ImPlot::SetCurrentContext(implot);
    }
    ContextGuard(const ContextGuard&) = delete;
    ContextGuard& operator=(const ContextGuard&) = delete;
    ContextGuard(ContextGuard&&) = delete;
    ContextGuard& operator=(ContextGuard&&) = delete;
    ~ContextGuard() {
        ImGui::SetCurrentContext(previous_imgui_);
        ImPlot::SetCurrentContext(previous_implot_);
    }

private:
    ImGuiContext* previous_imgui_;
    ImPlotContext* previous_implot_;
};

} // namespace

struct UiContext::Impl {
    ImGuiContext* imgui = nullptr;
    ImPlotContext* implot = nullptr;
    GlContext* gl = nullptr;
    bool opengl_backend = false;
    bool glfw_backend = false;
    bool layout_built = false;
    UiTheme theme;
    std::size_t last_vertices = 0;
};

namespace {

// Creates the ImGui and ImPlot contexts with the theme, fonts and settings
// shared by both flavours. The new contexts are left current.
void initialise_contexts(ImGuiContext*& imgui, ImPlotContext*& implot, UiTheme& theme,
                         const UiOptions& options) {
    IMGUI_CHECKVERSION();
    imgui = ImGui::CreateContext();
    ImGui::SetCurrentContext(imgui);
    implot = ImPlot::CreateContext();
    ImPlot::SetCurrentContext(implot);

    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr; // layout is code-defined and deterministic
    io.LogFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigWindowsMoveFromTitleBarOnly = true;

    theme = options.theme;
    theme.font_size = static_cast<float>(std::clamp(options.font_size_px, 8.0, 48.0));
    ImFontConfig font;
    font.SizePixels = theme.font_size;
    font.OversampleH = 2;
    font.OversampleV = 2;
    io.Fonts->AddFontDefaultVector(&font);
    apply_theme(theme);
}

} // namespace

UiContext::UiContext(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}

UiContext::UiContext(UiContext&& other) noexcept : impl_(std::move(other.impl_)) {}

UiContext& UiContext::operator=(UiContext&& other) noexcept {
    if (this != &other) {
        release();
        impl_ = std::move(other.impl_);
    }
    return *this;
}

UiContext::~UiContext() {
    release();
}

void UiContext::release() noexcept {
    if (!impl_) {
        return;
    }
    Impl& impl = *impl_;
    if (impl.imgui != nullptr) {
        ImGuiContext* const previous_imgui = ImGui::GetCurrentContext();
        ImPlotContext* const previous_implot = ImPlot::GetCurrentContext();
        ImGui::SetCurrentContext(impl.imgui);
        ImPlot::SetCurrentContext(impl.implot);
        if ((impl.opengl_backend || impl.glfw_backend) && impl.gl != nullptr && impl.gl->valid()) {
            // Remember whether a switch is needed and what was current
            // before it, including no context at all, and restore exactly
            // that afterwards.
            const bool switched = !impl.gl->is_current();
            void* const previous = switched ? detail::current_native_context() : nullptr;
            if (switched) {
                impl.gl->make_current();
            }
            if (impl.opengl_backend) {
                ImGui_ImplOpenGL3_Shutdown();
            }
            if (impl.glfw_backend) {
                ImGui_ImplGlfw_Shutdown();
            }
            if (switched) {
                detail::make_native_context_current(previous);
            }
        }
        if (impl.implot != nullptr) {
            ImPlot::DestroyContext(impl.implot);
        }
        ImGui::DestroyContext(impl.imgui);
        ImGui::SetCurrentContext(previous_imgui == impl.imgui ? nullptr : previous_imgui);
        ImPlot::SetCurrentContext(previous_implot == impl.implot ? nullptr : previous_implot);
    }
    impl_.reset();
}

core::Result<UiContext, std::string> UiContext::create_headless(const UiOptions& options) {
    ImGuiContext* const previous_imgui = ImGui::GetCurrentContext();
    ImPlotContext* const previous_implot = ImPlot::GetCurrentContext();
    auto impl = std::make_unique<Impl>();
    initialise_contexts(impl->imgui, impl->implot, impl->theme, options);
    // No renderer: rasterise the font atlas on the CPU once (legacy path of
    // ImGui for backends without texture support).
    ImGui::GetIO().Fonts->Build();
    UiContext ui(std::move(impl));
    ImGui::SetCurrentContext(previous_imgui);
    ImPlot::SetCurrentContext(previous_implot);
    return ui;
}

core::Result<UiContext, std::string> UiContext::create(GlContext& context, const UiOptions& options,
                                                       bool attach_window_input) {
    if (!context.valid()) {
        return core::failure(std::string("UI: the OpenGL context is not valid"));
    }
    void* const window = detail::native_window(context);
    if (window == nullptr) {
        return core::failure(std::string("UI: the OpenGL context has no window"));
    }
    ImGuiContext* const previous_imgui = ImGui::GetCurrentContext();
    ImPlotContext* const previous_implot = ImPlot::GetCurrentContext();
    context.make_current();

    auto impl = std::make_unique<Impl>();
    impl->gl = &context;
    initialise_contexts(impl->imgui, impl->implot, impl->theme, options);
    // From here on the UiContext owns the ImGui contexts: an early return
    // releases them (and any backend already initialised).
    UiContext ui(std::move(impl));
    const auto fail = [&](std::string message) -> core::Result<UiContext, std::string> {
        ui.release();
        ImGui::SetCurrentContext(previous_imgui);
        ImPlot::SetCurrentContext(previous_implot);
        return core::failure(std::move(message));
    };
    if (attach_window_input) {
        if (!ImGui_ImplGlfw_InitForOpenGL(static_cast<GLFWwindow*>(window), true)) {
            return fail("UI: the GLFW platform backend could not be initialised");
        }
        ui.impl_->glfw_backend = true;
    }
    if (!ImGui_ImplOpenGL3_Init("#version 460 core")) {
        return fail("UI: the OpenGL 3 renderer backend could not be initialised");
    }
    ui.impl_->opengl_backend = true;
    ImGui::SetCurrentContext(previous_imgui);
    ImPlot::SetCurrentContext(previous_implot);
    return ui;
}

void UiContext::begin_frame(const UiFrameInput& input) {
    Impl& impl = *impl_;
    ImGui::SetCurrentContext(impl.imgui);
    ImPlot::SetCurrentContext(impl.implot);
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(static_cast<float>(std::max(input.width, 1)),
                            static_cast<float>(std::max(input.height, 1)));
    io.DisplayFramebufferScale = ImVec2(1.0F, 1.0F);
    io.DeltaTime = static_cast<float>(std::max(input.delta_time, 1.0e-4));
    if (impl.opengl_backend) {
        if (impl.gl != nullptr) {
            impl.gl->make_current();
        }
        ImGui_ImplOpenGL3_NewFrame();
    }
    if (impl.glfw_backend) {
        ImGui_ImplGlfw_NewFrame();
    } else if (input.pointer) {
        const UiPointer& pointer = *input.pointer;
        io.AddMousePosEvent(static_cast<float>(pointer.x), static_cast<float>(pointer.y));
        for (int button = 0; button < 3; ++button) {
            io.AddMouseButtonEvent(button, pointer.buttons[static_cast<std::size_t>(button)]);
        }
        if (pointer.wheel != 0.0) {
            io.AddMouseWheelEvent(0.0F, static_cast<float>(pointer.wheel));
        }
    }
    ImGui::NewFrame();
}

UiRect UiContext::dockspace() {
    Impl& impl = *impl_;
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const ImGuiID dock_id = ImGui::GetID("OrreryDockspace");
    if (!impl.layout_built) {
        impl.layout_built = true;
        ImGui::DockBuilderRemoveNode(dock_id);
        ImGui::DockBuilderAddNode(dock_id,
                                  static_cast<ImGuiDockNodeFlags>(ImGuiDockNodeFlags_DockSpace) |
                                      ImGuiDockNodeFlags_PassthruCentralNode);
        ImGui::DockBuilderSetNodePos(dock_id, viewport->WorkPos);
        ImGui::DockBuilderSetNodeSize(dock_id, viewport->WorkSize);
        ImGuiID rest = 0;
        const ImGuiID left =
            ImGui::DockBuilderSplitNode(dock_id, ImGuiDir_Left, 0.23F, nullptr, &rest);
        const ImGuiID right =
            ImGui::DockBuilderSplitNode(rest, ImGuiDir_Right, 0.30F, nullptr, &rest);
        ImGuiID centre = 0;
        const ImGuiID bottom =
            ImGui::DockBuilderSplitNode(rest, ImGuiDir_Down, 0.34F, nullptr, &centre);
        ImGui::DockBuilderDockWindow(std::string(kBodyInspectorWindow).c_str(), left);
        ImGui::DockBuilderDockWindow(std::string(kRenderSettingsWindow).c_str(), right);
        ImGui::DockBuilderDockWindow(std::string(kSystemMonitorWindow).c_str(), bottom);
        ImGui::DockBuilderFinish(dock_id);
    }
    ImGui::DockSpaceOverViewport(dock_id, viewport, ImGuiDockNodeFlags_PassthruCentralNode);
    UiRect area{viewport->WorkPos.x, viewport->WorkPos.y, viewport->WorkSize.x,
                viewport->WorkSize.y};
    if (const ImGuiDockNode* central = ImGui::DockBuilderGetCentralNode(dock_id)) {
        area = {central->Pos.x, central->Pos.y, central->Size.x, central->Size.y};
    }
    return area;
}

void UiContext::end_frame() {
    Impl& impl = *impl_;
    ImGui::SetCurrentContext(impl.imgui);
    ImPlot::SetCurrentContext(impl.implot);
    ImGui::Render();
    ImDrawData* draw_data = ImGui::GetDrawData();
    impl.last_vertices =
        draw_data != nullptr ? static_cast<std::size_t>(draw_data->TotalVtxCount) : 0;
    if (impl.opengl_backend && draw_data != nullptr) {
        ImGui_ImplOpenGL3_RenderDrawData(draw_data);
    }
}

std::optional<UiRect> UiContext::window_rect(std::string_view name) const {
    const ContextGuard guard(impl_->imgui, impl_->implot);
    const ImGuiWindow* window = ImGui::FindWindowByName(std::string(name).c_str());
    if (window == nullptr || !window->WasActive) {
        return std::nullopt;
    }
    return UiRect{window->Pos.x, window->Pos.y, window->Size.x, window->Size.y};
}

bool UiContext::window_docked(std::string_view name) const {
    const ContextGuard guard(impl_->imgui, impl_->implot);
    const ImGuiWindow* window = ImGui::FindWindowByName(std::string(name).c_str());
    return window != nullptr && window->DockIsActive;
}

std::size_t UiContext::last_vertex_count() const noexcept {
    return impl_ ? impl_->last_vertices : 0;
}

bool UiContext::wants_pointer() const {
    const ContextGuard guard(impl_->imgui, impl_->implot);
    return ImGui::GetIO().WantCaptureMouse;
}

bool UiContext::has_renderer() const noexcept {
    return impl_ && impl_->opengl_backend;
}

const UiTheme& UiContext::theme() const noexcept {
    return impl_->theme;
}

} // namespace orrery::render
