#pragma once

#include <cstdint>
#include <optional>
#include <string>

// Time-warp budget of the interactive space simulation (docs/app.md,
// "Time-warp budget"). The physics thread may spend only a share of the
// wall-clock time stepping; when the requested warp would need more, the
// governor lowers the warp actually credited to the accumulator to the
// highest 1-2-5 level that fits, and raises it again once the load allows.
//
// Hysteresis keeps the warp from flickering: it drops only after the load
// has stayed above the limit for `lower_after_ns`, and rises only after the
// next level would have stayed below `raise_margin` of the limit for
// `raise_after_ns`. The governor reads no clock: every sample carries its
// own wall and busy time, so a sequence of samples always produces the same
// sequence of warps (tests drive it with a fake clock).
//
// Only the warp changes, never the step: the state after N steps is the same
// whatever warp the governor chose on the way.
namespace orrery::sim {

struct WarpBudget {
    // Share of wall-clock time the physics thread may spend stepping, in
    // [kMinLoadLimit, kMaxLoadLimit].
    double load_limit = 0.6;
    // A higher warp is taken only when it would use at most this share of
    // the limit, in [0.1, 1).
    double raise_margin = 0.75;
    // Wall-clock time the load must stay above the limit before the warp is
    // lowered, and below the margin before it is raised [ns], both >= 0.
    std::int64_t lower_after_ns = 250'000'000;
    std::int64_t raise_after_ns = 1'500'000'000;
    // Smoothing time of the step-cost and load estimates [ns], > 0.
    std::int64_t smoothing_ns = 250'000'000;

    friend bool operator==(const WarpBudget&, const WarpBudget&) = default;
};

inline constexpr double kMinLoadLimit = 0.05;
inline constexpr double kMaxLoadLimit = 0.95;

// Empty when the budget is usable, else the reason.
[[nodiscard]] std::optional<std::string> validate(const WarpBudget& budget);

// One physics tick as the governor sees it.
struct WarpSample {
    std::int64_t wall_ns = 0; // wall-clock time the tick credited
    std::int64_t busy_ns = 0; // wall-clock time spent stepping in it
    std::int64_t steps = 0;   // physics steps taken
};

// The 1-2-5 warp levels (1, 2, 5, 10, 20, 50, ...): the highest level not
// above `warp` (1 for anything below 1), and the lowest level above `warp`.
[[nodiscard]] std::int64_t warp_level_at_or_below(double warp) noexcept;
[[nodiscard]] std::int64_t warp_level_above(std::int64_t warp) noexcept;

class WarpGovernor {
public:
    // `step_ns`: simulated nanoseconds of one physics step (>= 1).
    WarpGovernor(const WarpBudget& budget, std::int64_t requested, std::int64_t step_ns) noexcept;

    // A new requested warp (>= 1). A lower one applies at once; a higher one
    // is granted as far as the measured step cost allows (all of it before
    // any cost is known). Restarts the hysteresis timers.
    void request(std::int64_t warp) noexcept;
    // A new system or step length: the cost estimate is forgotten and the
    // requested warp applies until it is measured again.
    void restart(std::int64_t step_ns) noexcept;
    // Accounts for one tick; returns the warp for the next one. Samples
    // without wall time are ignored; samples without steps update the load
    // but not the cost per step.
    std::int64_t update(const WarpSample& sample) noexcept;

    [[nodiscard]] std::int64_t requested() const noexcept { return requested_; }
    [[nodiscard]] std::int64_t effective() const noexcept { return effective_; }
    // The effective warp is below the requested one.
    [[nodiscard]] bool reduced() const noexcept { return effective_ < requested_; }
    // Smoothed wall-clock cost of one physics step [ns], once measured.
    [[nodiscard]] std::optional<double> step_cost_ns() const noexcept { return cost_ns_; }
    // Smoothed share of wall-clock time spent stepping, once measured.
    [[nodiscard]] std::optional<double> load() const noexcept { return load_; }
    // The warp that would use exactly the load limit, once the cost is known.
    [[nodiscard]] std::optional<double> sustainable_warp() const noexcept;
    [[nodiscard]] const WarpBudget& budget() const noexcept { return budget_; }

private:
    WarpBudget budget_;
    std::int64_t step_ns_ = 1;
    std::int64_t requested_ = 1;
    std::int64_t effective_ = 1;
    std::optional<double> cost_ns_;
    std::optional<double> load_;
    std::int64_t over_ns_ = 0;  // time the load has been above the limit
    std::int64_t under_ns_ = 0; // time a higher level would have fitted
};

} // namespace orrery::sim
