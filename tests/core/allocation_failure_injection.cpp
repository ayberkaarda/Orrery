#include "allocation_failure_injection.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>

// Replacement of the global allocation functions ([new.delete]).
//
// Every allocation function obtains its block from std::malloc and every
// deallocation function returns it with std::free, so each block is allocated
// and released by one allocator family whichever form the caller uses:
//
//  * All 8 allocation and all 12 deallocation forms are replaced. A partial
//    set leaves the toolchain's own versions of the others in place, and those
//    pair with ours: libstdc++'s std::get_temporary_buffer (used by
//    std::stable_sort) allocates with the nothrow form and releases with the
//    sized form, which AddressSanitizer reports as alloc-dealloc-mismatch
//    (operator new vs free) when only the plain forms are replaced.
//  * Over-aligned forms place the block inside a larger malloc block and keep
//    the malloc pointer just before it; they are only ever paired with each
//    other because the language selects the aligned deallocation form for
//    exactly the objects created by the aligned allocation form.
//  * The functions are defined in their own translation unit and are never
//    inlined. GCC's -Wmismatched-new-delete otherwise inlines std::free from
//    the replacement into a caller whose pointer it sees come from
//    ::operator new and reports a mismatch that is not one.
//
// AddressSanitizer intercepts std::malloc and std::free, so it still checks
// every block (bounds, use after free, leaks) and sees matched pairs.

#if defined(_MSC_VER) && !defined(__clang__)
#define ORRERY_TEST_NOINLINE __declspec(noinline)
#elif defined(__clang__)
#define ORRERY_TEST_NOINLINE __attribute__((noinline))
#else
#define ORRERY_TEST_NOINLINE __attribute__((noipa))
#endif

namespace {

// Allocations still allowed to succeed on this thread; negative when
// injection is disarmed. Constant-initialized, so usable before main.
thread_local long long allocations_before_failure = -1;
thread_local bool failure_fired = false;

// Counts one allocation against the armed budget; true when it must fail.
bool injected_failure() noexcept {
    if (allocations_before_failure < 0) {
        return false;
    }
    if (allocations_before_failure == 0) {
        allocations_before_failure = -1;
        failure_fired = true;
        return true;
    }
    --allocations_before_failure;
    return false;
}

void* allocate(std::size_t size) {
    if (injected_failure()) {
        throw std::bad_alloc();
    }
    if (void* memory = std::malloc(size == 0 ? 1 : size)) {
        return memory;
    }
    throw std::bad_alloc();
}

void deallocate(void* memory) noexcept {
    std::free(memory);
}

void* allocate_aligned(std::size_t size, std::align_val_t alignment) {
    const auto align = static_cast<std::size_t>(alignment);
    if (injected_failure()) {
        throw std::bad_alloc();
    }
    const std::size_t overhead = align + sizeof(void*);
    if (size > SIZE_MAX - overhead) {
        throw std::bad_alloc();
    }
    void* raw = std::malloc(size + overhead);
    if (raw == nullptr) {
        throw std::bad_alloc();
    }
    const auto base = reinterpret_cast<std::uintptr_t>(raw);
    const std::uintptr_t first = base + sizeof(void*);
    const std::uintptr_t mask = align - 1;
    const std::uintptr_t aligned = (first + mask) & ~mask;
    auto* block = static_cast<unsigned char*>(raw) + (aligned - base);
    std::memcpy(block - sizeof(void*), static_cast<const void*>(&raw), sizeof(void*));
    return block;
}

void deallocate_aligned(void* memory) noexcept {
    if (memory == nullptr) {
        return;
    }
    void* raw = nullptr;
    std::memcpy(static_cast<void*>(&raw), static_cast<unsigned char*>(memory) - sizeof(void*),
                sizeof(void*));
    std::free(raw);
}

} // namespace

namespace orrery::test {

AllocationFailureScope::AllocationFailureScope(long long allowed_allocations) noexcept {
    failure_fired = false;
    allocations_before_failure = allowed_allocations < 0 ? 0 : allowed_allocations;
}

AllocationFailureScope::~AllocationFailureScope() {
    disarm();
}

void AllocationFailureScope::disarm() noexcept {
    allocations_before_failure = -1;
}

bool AllocationFailureScope::fired() const noexcept {
    return failure_fired;
}

} // namespace orrery::test

// Allocation functions.

ORRERY_TEST_NOINLINE void* operator new(std::size_t size) {
    return allocate(size);
}

ORRERY_TEST_NOINLINE void* operator new[](std::size_t size) {
    return allocate(size);
}

ORRERY_TEST_NOINLINE void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
    try {
        return allocate(size);
    } catch (...) {
        return nullptr;
    }
}

ORRERY_TEST_NOINLINE void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
    try {
        return allocate(size);
    } catch (...) {
        return nullptr;
    }
}

ORRERY_TEST_NOINLINE void* operator new(std::size_t size, std::align_val_t alignment) {
    return allocate_aligned(size, alignment);
}

ORRERY_TEST_NOINLINE void* operator new[](std::size_t size, std::align_val_t alignment) {
    return allocate_aligned(size, alignment);
}

ORRERY_TEST_NOINLINE void* operator new(std::size_t size, std::align_val_t alignment,
                                        const std::nothrow_t&) noexcept {
    try {
        return allocate_aligned(size, alignment);
    } catch (...) {
        return nullptr;
    }
}

ORRERY_TEST_NOINLINE void* operator new[](std::size_t size, std::align_val_t alignment,
                                          const std::nothrow_t&) noexcept {
    try {
        return allocate_aligned(size, alignment);
    } catch (...) {
        return nullptr;
    }
}

// Deallocation functions.

ORRERY_TEST_NOINLINE void operator delete(void* memory) noexcept {
    deallocate(memory);
}

ORRERY_TEST_NOINLINE void operator delete[](void* memory) noexcept {
    deallocate(memory);
}

ORRERY_TEST_NOINLINE void operator delete(void* memory, std::size_t) noexcept {
    deallocate(memory);
}

ORRERY_TEST_NOINLINE void operator delete[](void* memory, std::size_t) noexcept {
    deallocate(memory);
}

ORRERY_TEST_NOINLINE void operator delete(void* memory, const std::nothrow_t&) noexcept {
    deallocate(memory);
}

ORRERY_TEST_NOINLINE void operator delete[](void* memory, const std::nothrow_t&) noexcept {
    deallocate(memory);
}

ORRERY_TEST_NOINLINE void operator delete(void* memory, std::align_val_t) noexcept {
    deallocate_aligned(memory);
}

ORRERY_TEST_NOINLINE void operator delete[](void* memory, std::align_val_t) noexcept {
    deallocate_aligned(memory);
}

ORRERY_TEST_NOINLINE void operator delete(void* memory, std::size_t, std::align_val_t) noexcept {
    deallocate_aligned(memory);
}

ORRERY_TEST_NOINLINE void operator delete[](void* memory, std::size_t, std::align_val_t) noexcept {
    deallocate_aligned(memory);
}

ORRERY_TEST_NOINLINE void operator delete(void* memory, std::align_val_t,
                                          const std::nothrow_t&) noexcept {
    deallocate_aligned(memory);
}

ORRERY_TEST_NOINLINE void operator delete[](void* memory, std::align_val_t,
                                            const std::nothrow_t&) noexcept {
    deallocate_aligned(memory);
}
