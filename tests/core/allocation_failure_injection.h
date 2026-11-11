#pragma once

// Allocation fault injection for test executables.
//
// allocation_failure_injection.cpp replaces every replaceable global
// allocation and deallocation function (scalar, array, nothrow, sized and
// std::align_val_t forms) in the executable that links it. All of them
// count against the same budget while an AllocationFailureScope is alive on
// the calling thread; outside such a scope nothing ever fails.

namespace orrery::test {

// Arms fault injection on the current thread for its lifetime: the first
// `allowed_allocations` global allocations succeed and the next one fails
// (throwing forms throw std::bad_alloc, nothrow forms return nullptr). The
// failure fires at most once; injection is disarmed when the scope ends,
// however it ends. Scopes do not nest.
class AllocationFailureScope {
public:
    explicit AllocationFailureScope(long long allowed_allocations) noexcept;
    ~AllocationFailureScope();
    AllocationFailureScope(const AllocationFailureScope&) = delete;
    AllocationFailureScope& operator=(const AllocationFailureScope&) = delete;

    // Ends injection early; later allocations on this thread always succeed.
    void disarm() noexcept;

    // True once the injected failure has fired inside this scope.
    [[nodiscard]] bool fired() const noexcept;
};

} // namespace orrery::test
