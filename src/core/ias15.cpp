#include "orrery/core/ias15.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace orrery::core {

namespace {

// Gauss-Radau spacings on [0, 1]: h0 = 0 and the seven roots of
// (P7(x) + P8(x)) / (1 + x), x = 2 h - 1 (left Radau rule with 8 nodes).
// Computed with 60-digit arithmetic and written to 26 significant digits, so
// every literal is the correctly rounded double; they agree with the values
// tabulated by Everhart (1985) and Rein and Spiegel (2015).
constexpr std::array<double, 8> spacing = {
    0.0,
    5.6262560536922146465652191e-2,
    1.8024069173689236498757994e-1,
    3.5262471711316963737390777e-1,
    5.4715362633055538300144856e-1,
    7.3421017721541053152321061e-1,
    8.8532094683909576809035976e-1,
    9.7752061356128750189117450e-1,
};

// b_k = sum_{j >= k} c[k][j] g_j, where c[k][j] is the coefficient of s^k in
// prod_{m=1..j} (s - h_m) (expansion of the Newton form
// a = a0 + sum_j g_j s prod_{m=1..j} (s - h_m)). Upper triangular, unit
// diagonal; same 60-digit computation.
constexpr std::array<std::array<double, 7>, 7> c_coef = {{
    {1.0, -5.6262560536922146465652191e-2, 1.0140802830063629986481805e-2,
     -3.5758977292516175949344589e-3, 1.9565654099472210769005671e-3,
     -1.4365302363708915424459553e-3, 1.2717903090268677492943116e-3},
    {0.0, 1.0, -2.3650325227381451145323213e-1, 9.3537695259462065895748461e-2,
     -5.4755386889068686440808430e-2, 4.2158527721268707707297347e-2,
     -3.8760357915906770369904625e-2},
    {0.0, 0.0, 1.0, -5.8912796938698414882713990e-1, 4.1588120008230686168862191e-1,
     -3.6009959650205681228976646e-1, 3.6096224345284598322533981e-1},
    {0.0, 0.0, 0.0, 1.0, -1.1362815957175395318285885e+0, 1.2501507118406910258505441e+0,
     -1.4668842084004269643701553e+0},
    {0.0, 0.0, 0.0, 0.0, 1.0, -1.8704917729329500633517991e+0, 2.9061362593084293014237913e+0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -2.7558127197720458314421588e+0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
}};

// Inverse of c_coef: g_k = sum_{j >= k} d[k][j] b_j.
constexpr std::array<std::array<double, 7>, 7> d_coef = {{
    {1.0, 5.6262560536922146465652191e-2, 3.1654757181708292499904800e-3,
     1.7809776922174338811252792e-4, 1.0020236522329127209567215e-5, 5.6376416393182076103838501e-7,
     3.1718815401761366475854818e-8},
    {0.0, 1.0, 2.3650325227381451145323213e-1, 4.5792985506027918895453873e-2,
     8.4318571535257015444999742e-3, 1.5297840025004658189490080e-3,
     2.7629309098264765931302264e-4},
    {0.0, 0.0, 1.0, 5.8912796938698414882713990e-1, 2.5353406905456926652146160e-1,
     9.7834236532444005365364840e-2, 3.6028553983736459600387074e-2},
    {0.0, 0.0, 0.0, 1.0, 1.1362815957175395318285885e+0, 8.7525466468409109122972459e-1,
     5.7673300027707873135445961e-1},
    {0.0, 0.0, 0.0, 0.0, 1.0, 1.8704917729329500633517991e+0, 2.2485887607691597933926895e+0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.7558127197720458314421588e+0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
}};

// Binomial coefficients C(k + 1, j + 1) for the re-expansion of b about the
// start of the next step (prediction).
constexpr std::array<std::array<double, 7>, 7> binomial = {{
    {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0},
    {0.0, 1.0, 3.0, 6.0, 10.0, 15.0, 21.0},
    {0.0, 0.0, 1.0, 4.0, 10.0, 20.0, 35.0},
    {0.0, 0.0, 0.0, 1.0, 5.0, 15.0, 35.0},
    {0.0, 0.0, 0.0, 0.0, 1.0, 6.0, 21.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 7.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
}};

// Predictor-corrector limits (Rein and Spiegel 2015, section 3.3).
constexpr double corrector_converged = 1.0e-16;
constexpr int max_corrector_iterations = 12;
// A step that grows by more than this factor starts without a b prediction.
constexpr double max_prediction_ratio = 20.0;

// x^(1/7) for finite x > 0 with IEEE +, *, / and the exact frexp/ldexp only,
// so that the controller gives the same step on every platform (std::pow is
// not correctly rounded and differs between C runtimes). x = u 2^(7 q) with
// u in [0.5, 64); Newton's iteration y <- (6 y + u / y^6) / 7 from y = 2,
// above every root in [0.9, 1.82), decreases monotonically to u^(1/7).
double seventh_root(double x) {
    int e = 0;
    const double m = std::frexp(x, &e);
    int q = e / 7;
    int r = e % 7;
    if (r < 0) {
        r += 7;
        --q;
    }
    const double u = std::ldexp(m, r);
    double y = 2.0;
    for (int it = 0; it < 64; ++it) {
        const double y2 = y * y;
        const double y6 = y2 * y2 * y2;
        const double next = (6.0 * y + u / y6) / 7.0;
        if (!(next < y)) {
            break;
        }
        y = next;
    }
    return std::ldexp(y, q);
}

// Kahan addition of `delta` to `sum` with running compensation `comp`
// (comp holds the part of the previous additions lost to rounding, negated).
void compensated_add(double& sum, double& comp, double delta) {
    const double y = delta - comp;
    const double t = sum + y;
    comp = (t - sum) - y;
    sum = t;
}

} // namespace

std::string_view to_string(Ias15Error error) noexcept {
    switch (error) {
        case Ias15Error::invalid_tolerance:
            return "IAS15 tolerance must be finite and positive";
    }
    return "unknown IAS15 error";
}

Result<IAS15, Ias15Error> IAS15::create(double tolerance) {
    if (!std::isfinite(tolerance) || !(tolerance > 0.0)) {
        return failure(Ias15Error::invalid_tolerance);
    }
    return IAS15(tolerance);
}

void IAS15::restart() noexcept {
    for (std::size_t k = 0; k < 7; ++k) {
        std::fill(b_[k].begin(), b_[k].end(), 0.0);
        std::fill(e_[k].begin(), e_[k].end(), 0.0);
        std::fill(b_last_[k].begin(), b_last_[k].end(), 0.0);
        std::fill(e_last_[k].begin(), e_last_[k].end(), 0.0);
    }
    std::fill(comp_x_.begin(), comp_x_.end(), 0.0);
    std::fill(comp_v_.begin(), comp_v_.end(), 0.0);
    last_step_ = 0.0;
    next_step_ = 0.0;
    has_committed_ = false;
}

void IAS15::resize(std::size_t coordinates) {
    for (std::vector<double>* v : {&x0_, &v0_, &a0_, &at_, &comp_x_, &comp_v_}) {
        v->assign(coordinates, 0.0);
    }
    for (std::size_t k = 0; k < 7; ++k) {
        b_[k].assign(coordinates, 0.0);
        g_[k].assign(coordinates, 0.0);
        e_[k].assign(coordinates, 0.0);
        b_last_[k].assign(coordinates, 0.0);
        e_last_[k].assign(coordinates, 0.0);
    }
    const std::size_t n = coordinates / 3;
    trial_position_.resize(n);
    trial_acceleration_.resize(n);
}

bool IAS15::matches_committed(const BodyStore& bodies) const {
    if (!has_committed_ || committed_position_.size() != bodies.size()) {
        return false;
    }
    // Bitwise comparison of the stored doubles (== would equate -0 and +0).
    const auto same = [](double a, double b) {
        return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
    };
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const Vec3<space::Length> p = bodies.position(i);
        const Vec3<space::Velocity> v = bodies.velocity(i);
        const Vec3<space::Length>& cp = committed_position_[i];
        const Vec3<space::Velocity>& cv = committed_velocity_[i];
        if (!same(p.x.value(), cp.x.value()) || !same(p.y.value(), cp.y.value()) ||
            !same(p.z.value(), cp.z.value()) || !same(v.x.value(), cv.x.value()) ||
            !same(v.y.value(), cv.y.value()) || !same(v.z.value(), cv.z.value())) {
            return false;
        }
    }
    return true;
}

// Sets b and e for a step of length `step`. With a previous accepted step of
// length h_last, q = step / h_last and the polynomial of that step is
// re-expanded about its end: e_j = q^(j+1) sum_{k >= j} C(k+1, j+1) b_k.
// The difference between the converged b and the prediction it started from
// is added as a correction (b = e + (b_last - e_last)).
void IAS15::predict_coefficients(double step) {
    const std::size_t n = x0_.size();
    const double q = last_step_ != 0.0 ? step / last_step_ : 0.0;
    if (last_step_ == 0.0 || !(std::abs(q) <= max_prediction_ratio)) {
        for (std::size_t k = 0; k < 7; ++k) {
            std::fill(b_[k].begin(), b_[k].end(), 0.0);
            std::fill(e_[k].begin(), e_[k].end(), 0.0);
        }
        return;
    }
    std::array<double, 7> qp{};
    qp[0] = q;
    for (std::size_t j = 1; j < 7; ++j) {
        qp[j] = qp[j - 1] * q;
    }
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < 7; ++j) {
            // Highest order first, as in a Horner-like fixed order.
            double sum = 0.0;
            for (std::size_t k = 7; k-- > j;) {
                sum += binomial[j][k] * b_last_[k][i];
            }
            e_[j][i] = qp[j] * sum;
            b_[j][i] = e_[j][i] + (b_last_[j][i] - e_last_[j][i]);
        }
    }
}

bool IAS15::evaluate(const BodyStore& bodies, const IForceModel& forces, std::vector<double>& out) {
    forces.accelerations(bodies, trial_position_, trial_acceleration_);
    ++force_evaluations_;
    for (std::size_t i = 0; i < trial_acceleration_.size(); ++i) {
        const Vec3<space::Acceleration>& a = trial_acceleration_[i];
        if (!isfinite(a)) {
            return false;
        }
        out[3 * i] = a.x.value();
        out[3 * i + 1] = a.y.value();
        out[3 * i + 2] = a.z.value();
    }
    return true;
}

IAS15::Attempt IAS15::attempt(const BodyStore& bodies, const IForceModel& forces, double step) {
    const std::size_t n = x0_.size();
    predict_coefficients(step);

    for (std::size_t i = 0; i < n / 3; ++i) {
        trial_position_[i] =
            Vec3<space::Length>{space::Length(x0_[3 * i]), space::Length(x0_[3 * i + 1]),
                                space::Length(x0_[3 * i + 2])};
    }
    if (!evaluate(bodies, forces, a0_)) {
        return {};
    }
    // g from the predicted b.
    for (std::size_t j = 0; j < 7; ++j) {
        for (std::size_t i = 0; i < n; ++i) {
            double sum = 0.0;
            for (std::size_t k = 7; k-- > j;) {
                sum += d_coef[j][k] * b_[k][i];
            }
            g_[j][i] = sum;
        }
    }

    double corrector_error = 1.0e300;
    double corrector_error_last = 2.0;
    int iterations = 0;
    while (true) {
        if (corrector_error < corrector_converged) {
            break;
        }
        if (iterations > 2 && corrector_error_last <= corrector_error) {
            break; // no longer decreasing: rounding noise
        }
        if (iterations >= max_corrector_iterations) {
            break;
        }
        corrector_error_last = corrector_error;
        ++iterations;
        double max_db6 = 0.0;
        double max_a = 0.0;
        for (std::size_t s = 1; s < 8; ++s) {
            const double h = spacing[s];
            // Predicted position at t0 + h step.
            const auto predicted = [&](std::size_t i) {
                const double poly =
                    a0_[i] / 2.0 +
                    h * (b_[0][i] / 6.0 +
                         h * (b_[1][i] / 12.0 +
                              h * (b_[2][i] / 20.0 +
                                   h * (b_[3][i] / 30.0 +
                                        h * (b_[4][i] / 42.0 +
                                             h * (b_[5][i] / 56.0 + h * (b_[6][i] / 72.0)))))));
                const double displacement = step * h * (v0_[i] + step * h * poly);
                return space::Length(x0_[i] + (displacement - comp_x_[i]));
            };
            for (std::size_t body = 0; body < n / 3; ++body) {
                trial_position_[body] = Vec3<space::Length>{
                    predicted(3 * body), predicted(3 * body + 1), predicted(3 * body + 2)};
            }
            if (!evaluate(bodies, forces, at_)) {
                return {};
            }
            // New divided difference g_{s-1} and the matching update of b.
            const std::size_t gk = s - 1;
            for (std::size_t i = 0; i < n; ++i) {
                double value = (at_[i] - a0_[i]) / h;
                for (std::size_t j = 1; j < s; ++j) {
                    value = (value - g_[j - 1][i]) / (h - spacing[j]);
                }
                const double delta = value - g_[gk][i];
                g_[gk][i] = value;
                for (std::size_t k = 0; k <= gk; ++k) {
                    b_[k][i] += c_coef[k][gk] * delta;
                }
                if (s == 7) {
                    max_db6 = std::max(max_db6, std::abs(delta));
                    max_a = std::max(max_a, std::abs(at_[i]));
                }
            }
        }
        if (max_a > 0.0) {
            corrector_error = max_db6 / max_a;
        } else {
            corrector_error = max_db6 == 0.0 ? 0.0 : 1.0e300;
        }
    }

    // Step-size control on the converged b6 (global criterion).
    double max_b6 = 0.0;
    double max_a = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        max_b6 = std::max(max_b6, std::abs(b_[6][i]));
        max_a = std::max(max_a, std::abs(at_[i]));
    }
    const double error = max_a > 0.0 ? max_b6 / max_a : 0.0;
    double next = 0.0;
    if (std::isnormal(error)) {
        next = step * seventh_root(tolerance_ / error);
    } else {
        next = step / safety_factor;
    }
    if (!std::isfinite(next)) {
        next = step / safety_factor;
    }
    if (std::abs(next) < safety_factor * std::abs(step)) {
        ++rejected_steps_;
        return {true, false, next};
    }
    if (std::abs(next) > std::abs(step) / safety_factor) {
        next = step / safety_factor;
    }

    // Accept: advance x0 and v0 to the end of the step.
    for (std::size_t i = 0; i < n; ++i) {
        const double dx =
            step * (v0_[i] +
                    step * (a0_[i] / 2.0 + b_[0][i] / 6.0 + b_[1][i] / 12.0 + b_[2][i] / 20.0 +
                            b_[3][i] / 30.0 + b_[4][i] / 42.0 + b_[5][i] / 56.0 + b_[6][i] / 72.0));
        const double dv =
            step * (a0_[i] + b_[0][i] / 2.0 + b_[1][i] / 3.0 + b_[2][i] / 4.0 + b_[3][i] / 5.0 +
                    b_[4][i] / 6.0 + b_[5][i] / 7.0 + b_[6][i] / 8.0);
        compensated_add(x0_[i], comp_x_[i], dx);
        compensated_add(v0_[i], comp_v_[i], dv);
    }
    for (std::size_t k = 0; k < 7; ++k) {
        std::swap(b_last_[k], b_[k]);
        std::swap(e_last_[k], e_[k]);
    }
    last_step_ = step;
    ++accepted_steps_;
    return {true, true, next};
}

Result<space::Time, IntegrationError> IAS15::step(SimState& state, space::Time dt,
                                                  const IForceModel& forces) {
    return advance(state, dt, forces, nullptr);
}

Result<space::Time, IntegrationError> IAS15::step_observed(SimState& state, space::Time dt,
                                                           const IForceModel& forces,
                                                           IStepObserver& observer) {
    return advance(state, dt, forces, &observer);
}

Result<space::Time, IntegrationError> IAS15::advance(SimState& state, space::Time dt,
                                                     const IForceModel& forces,
                                                     IStepObserver* observer) {
    if (!isfinite(dt)) {
        return failure(IntegrationError::invalid_time_step);
    }
    const BodyStore& bodies = state.bodies();
    const std::size_t count = bodies.size();
    const double target = dt.value();

    if (x0_.size() != 3 * count) {
        resize(3 * count);
        restart();
    } else if (!matches_committed(bodies)) {
        restart();
    }
    // A change of direction invalidates the step size and the prediction.
    if ((next_step_ > 0.0 && target < 0.0) || (next_step_ < 0.0 && target > 0.0)) {
        restart();
    }
    for (std::size_t i = 0; i < count; ++i) {
        const Vec3<space::Length> p = bodies.position(i);
        const Vec3<space::Velocity> v = bodies.velocity(i);
        x0_[3 * i] = p.x.value();
        x0_[3 * i + 1] = p.y.value();
        x0_[3 * i + 2] = p.z.value();
        v0_[3 * i] = v.x.value();
        v0_[3 * i + 1] = v.y.value();
        v0_[3 * i + 2] = v.z.value();
    }

    const double t0 = state.time().value();
    double done = 0.0;
    double proposal = next_step_ != 0.0 ? next_step_ : target;
    if (observer != nullptr) {
        segment_start_.resize(count);
        segment_end_.resize(count);
    }
    const auto positions = [this, count](std::vector<Vec3<space::Length>>& out) {
        for (std::size_t i = 0; i < count; ++i) {
            out[i] = Vec3<space::Length>{space::Length(x0_[3 * i]), space::Length(x0_[3 * i + 1]),
                                         space::Length(x0_[3 * i + 2])};
        }
    };
    while (count > 0 && done != target) {
        const double remaining = target - done;
        const bool clipped = std::abs(proposal) >= std::abs(remaining);
        // Between one and two proposals left: two equal halves, so that no
        // sliver of a step remains at the end.
        const bool halve = !clipped && std::abs(remaining) < 2.0 * std::abs(proposal);
        const double h = clipped ? remaining : (halve ? remaining / 2.0 : proposal);
        // A step the controller proposes must advance the time representation
        // (otherwise the loop could not terminate); the final remainder of
        // the requested interval may be arbitrarily short.
        if (!(std::abs(h) > 0.0) || (!clipped && (t0 + done) + h == t0 + done)) {
            restart();
            return failure(IntegrationError::step_size_underflow);
        }
        if (observer != nullptr) {
            positions(segment_start_);
        }
        const Attempt result = attempt(bodies, forces, h);
        if (!result.ok) {
            restart();
            return failure(IntegrationError::non_finite_state);
        }
        if (!result.accepted) {
            proposal = result.next_step;
            continue;
        }
        const double begin = done;
        done = clipped ? target : done + h;
        // A step shortened to land on the target says little about the
        // natural step: keep the larger of the two proposals.
        proposal = clipped && std::abs(proposal) > std::abs(result.next_step) ? proposal
                                                                              : result.next_step;
        if (observer != nullptr) {
            positions(segment_end_);
            const bool stop = observer->on_segment(bodies, segment_start_, segment_end_,
                                                   space::Time(begin), space::Time(done));
            if (stop) {
                break;
            }
        }
    }
    if (count > 0) {
        next_step_ = proposal;
    }

    // Commit all or nothing.
    for (std::size_t i = 0; i < 3 * count; ++i) {
        if (!std::isfinite(x0_[i]) || !std::isfinite(v0_[i])) {
            restart();
            return failure(IntegrationError::non_finite_state);
        }
    }
    // A stop before the target advances the time by the integrated part only.
    const space::Time new_time =
        count == 0 || done == target ? state.time() + dt : state.time() + space::Time(done);
    if (!isfinite(new_time)) {
        restart();
        return failure(IntegrationError::non_finite_state);
    }
    BodyStore& store = state.bodies();
    committed_position_.resize(count);
    committed_velocity_.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        const Vec3<space::Length> p{space::Length(x0_[3 * i]), space::Length(x0_[3 * i + 1]),
                                    space::Length(x0_[3 * i + 2])};
        const Vec3<space::Velocity> v{space::Velocity(v0_[3 * i]), space::Velocity(v0_[3 * i + 1]),
                                      space::Velocity(v0_[3 * i + 2])};
        // Cannot fail: every component was checked above.
        if (!store.set_position(i, p) || !store.set_velocity(i, v)) {
            restart();
            return failure(IntegrationError::non_finite_state);
        }
        committed_position_[i] = p;
        committed_velocity_[i] = v;
    }
    has_committed_ = count > 0;
    state.set_time(new_time);
    return new_time;
}

} // namespace orrery::core
