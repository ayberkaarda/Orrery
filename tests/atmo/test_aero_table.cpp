#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "orrery/aero/aero_solver.h"
#include "orrery/atmo/aero_table.h"
#include "orrery/geom/primitives.h"

using orrery::atmo::AeroCoefficientTable;
using orrery::atmo::AtmoErrorCode;
using orrery::atmo::CdSample;
using orrery::core::si::Area;

namespace {

AeroCoefficientTable literal_table() {
    auto table = AeroCoefficientTable::create({{0.5, 0.8}, {1.2, 1.4}, {5.0, 1.3}, {20.0, 1.25}},
                                              Area(12.0));
    REQUIRE(table);
    return table.value();
}

// Solver stand-in: Cd = 1 + Mach / 100, finishing after `steps` steps;
// throws from setup() when asked to.
class FakeSolver final : public orrery::aero::IAeroSolver {
public:
    FakeSolver(int steps, bool fail) : steps_(steps), fail_(fail) {}

    [[nodiscard]] orrery::aero::SolverKind kind() const noexcept override {
        return orrery::aero::SolverKind::panel;
    }
    void setup(const orrery::aero::AeroGeometry& /*geometry*/,
               const orrery::aero::FlowConditions& conditions) override {
        if (fail_) {
            throw std::invalid_argument("fake solver refuses");
        }
        mach_ = conditions.mach();
        taken_ = 0;
    }
    bool step() override {
        ++taken_;
        result_.cd = 1.0 + mach_ / 100.0;
        result_.converged = taken_ >= steps_;
        return result_.converged;
    }
    [[nodiscard]] const orrery::aero::AeroResult& result() const noexcept override {
        return result_;
    }

private:
    int steps_;
    bool fail_;
    int taken_ = 0;
    double mach_ = 0.0;
    orrery::aero::AeroResult result_;
};

orrery::atmo::CdSamplingRequest sphere_request() {
    orrery::atmo::CdSamplingRequest request;
    request.geometry.reference = {std::numbers::pi * 0.25, 1.0, {}};
    request.freestream.temperature = 247.0;
    request.freestream.density = 3.1e-4;
    return request;
}

} // namespace

TEST_CASE("Cd table from literal data: interpolation and clamping", "[atmo][aero_table]") {
    const AeroCoefficientTable table = literal_table();
    CHECK(table.reference_area().value() == 12.0);
    CHECK(table.min_mach() == 0.5);
    CHECK(table.max_mach() == 20.0);
    // Exact at the samples.
    CHECK(table.cd(1.2) == 1.4);
    CHECK_FALSE(table.lookup(1.2).clamped);
    // Linear between them.
    CHECK(std::abs(table.cd(0.85) - 1.1) < 1e-15);
    CHECK(std::abs(table.cd(12.5) - 1.275) < 1e-15);
    // End values held outside the range, flagged.
    CHECK(table.lookup(0.1).cd == 0.8);
    CHECK(table.lookup(0.1).clamped);
    CHECK(table.lookup(35.0).cd == 1.25);
    CHECK(table.lookup(35.0).clamped);
    CHECK(table.lookup(std::nan("")).clamped);
}

TEST_CASE("Cd table rejects invalid data", "[atmo][aero_table]") {
    const auto code = [](const std::vector<CdSample>& samples, double area) {
        auto t = AeroCoefficientTable::create(samples, Area(area));
        REQUIRE_FALSE(t);
        return t.error().code;
    };
    CHECK(code({}, 1.0) == AtmoErrorCode::invalid_table);
    CHECK(code({{1.0, 1.0}, {1.0, 1.1}}, 1.0) == AtmoErrorCode::invalid_table);
    CHECK(code({{2.0, 1.0}, {1.0, 1.1}}, 1.0) == AtmoErrorCode::invalid_table);
    CHECK(code({{1.0, 0.0}}, 1.0) == AtmoErrorCode::invalid_table);
    CHECK(code({{-1.0, 1.0}}, 1.0) == AtmoErrorCode::invalid_table);
    CHECK(code({{1.0, std::nan("")}}, 1.0) == AtmoErrorCode::invalid_table);
    CHECK(code({{1.0, 1.0}}, 0.0) == AtmoErrorCode::invalid_table);
    auto constant = AeroCoefficientTable::constant(1.3, Area(2.0));
    REQUIRE(constant);
    CHECK(constant.value().cd(0.0) == 1.3);
    CHECK(constant.value().cd(30.0) == 1.3);
    CHECK(AeroCoefficientTable{}.cd(3.0) == 1.0);
}

TEST_CASE("Cd sampling drives a solver per Mach number", "[atmo][aero_table]") {
    orrery::atmo::CdSamplingRequest request = sphere_request();
    request.mach_numbers = {0.2, 2.0, 10.0};
    const orrery::atmo::AeroSolverFactory factory = [](orrery::aero::SolverKind) {
        return std::make_unique<FakeSolver>(3, false);
    };
    auto sampled = orrery::atmo::sample_cd_table(request, factory);
    REQUIRE(sampled);
    const auto& result = sampled.value();
    REQUIRE(result.points.size() == 3);
    CHECK(result.table.samples().size() == 3);
    CHECK(std::abs(result.table.cd(10.0) - 1.1) < 1e-12);
    CHECK(result.table.reference_area().value() == request.geometry.reference.area);
    CHECK(result.points[0].steps == 3);
    CHECK(result.points[0].converged);
    // The regime selector's recommendation is recorded: Mach 10 is Newtonian,
    // Mach 2 has no valid solver (outside every envelope).
    CHECK(result.points[2].solver == orrery::aero::SolverKind::newtonian);
    CHECK(result.points[2].within_envelope);
    CHECK_FALSE(result.points[1].within_envelope);

    request.max_steps = 2;
    auto capped = orrery::atmo::sample_cd_table(request, factory);
    REQUIRE(capped);
    CHECK(capped.value().points[0].steps == 2);
    CHECK_FALSE(capped.value().points[0].converged);
}

TEST_CASE("Cd sampling reports unavailable and failing solvers", "[atmo][aero_table]") {
    orrery::atmo::CdSamplingRequest request = sphere_request();
    request.solver_override = orrery::aero::SolverKind::lattice_boltzmann;
    auto unavailable =
        orrery::atmo::sample_cd_table(request, orrery::atmo::default_aero_solver_factory());
    REQUIRE_FALSE(unavailable);
    CHECK(unavailable.error().code == AtmoErrorCode::solver_unavailable);

    const orrery::atmo::AeroSolverFactory throwing = [](orrery::aero::SolverKind) {
        return std::make_unique<FakeSolver>(1, true);
    };
    auto failed = orrery::atmo::sample_cd_table(sphere_request(), throwing);
    REQUIRE_FALSE(failed);
    CHECK(failed.error().code == AtmoErrorCode::solver_failed);

    orrery::atmo::CdSamplingRequest bad = sphere_request();
    bad.mach_numbers = {5.0, 5.0};
    auto rejected = orrery::atmo::sample_cd_table(bad, throwing);
    REQUIRE_FALSE(rejected);
    CHECK(rejected.error().code == AtmoErrorCode::invalid_parameter);
}

// Apollo command module at its hypersonic trim attitude (20 degrees between
// the axis and the free stream), five Mach numbers through the regime
// selector and modified Newtonian theory. Reference: Cd about 1.3 (Hillje,
// NASA TN D-5399, 1969), the same criterion as the aero validation set.
TEST_CASE("Cd table built from the aerodynamic solver for the Apollo capsule",
          "[atmo][aero_table][solver]") {
    const orrery::geom::CapsuleShape shape = orrery::geom::CapsuleShape::apollo();
    orrery::atmo::CdSamplingRequest request;
    request.geometry.mesh =
        orrery::geom::capsule_profile(shape).mesh(orrery::geom::RevolutionResolution{96, 0.02});
    // Heat shield forward: (x, y, z) -> (z, y, -x), a proper rotation.
    for (orrery::geom::Vector3& v : request.geometry.mesh.vertices) {
        v = orrery::geom::Vector3{v.z, v.y, -v.x};
    }
    const double radius = 0.5 * shape.max_diameter;
    request.geometry.reference = {std::numbers::pi * radius * radius, shape.max_diameter, {}};
    request.freestream.temperature = 247.0;
    request.freestream.density = 3.1e-4;
    request.freestream.angle_of_attack = 20.0 * std::numbers::pi / 180.0;

    auto sampled = orrery::atmo::sample_cd_table(
        request, orrery::atmo::default_aero_solver_factory(shape.nose_radius));
    REQUIRE(sampled);
    const auto& result = sampled.value();
    REQUIRE(result.table.samples().size() == 5);
    for (const orrery::atmo::CdSamplingPoint& point : result.points) {
        INFO("Mach " << point.mach << ": Cd " << point.cd);
        CHECK(point.solver == orrery::aero::SolverKind::newtonian);
        CHECK(point.within_envelope);
        CHECK(point.warnings.empty());
        CHECK(std::abs(point.cd - 1.3) < 0.13);
    }
    // Cp_max grows slightly with Mach towards its limit: Cd is non-decreasing.
    for (std::size_t i = 1; i < result.points.size(); ++i) {
        CHECK(result.points[i].cd >= result.points[i - 1].cd - 1e-12);
    }
    CHECK(result.table.lookup(30.0).clamped);
}
