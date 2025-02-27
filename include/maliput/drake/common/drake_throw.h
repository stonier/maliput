#pragma once

#define MALIPUT_USED

#include <type_traits>

#include "maliput/drake/common/drake_assert.h"

/// @file
/// Provides a convenient wrapper to throw an exception when a condition is
/// unmet.  This is similar to an assertion, but uses exceptions instead of
/// ::abort(), and cannot be disabled.

namespace maliput::drake {
namespace internal {
// Throw an error message.
[[noreturn]] void Throw(const char* condition, const char* func, const char* file, int line);

template <bool>
constexpr void DrakeThrowUnlessWasUsedWithRawPointer() {}
template <>
[
    [deprecated("\nDRAKE DEPRECATED: When using MALIPUT_DRAKE_THROW_UNLESS on a raw"
                " pointer, always write out MALIPUT_DRAKE_THROW_UNLESS(foo != nullptr), do not write"
                " MALIPUT_DRAKE_THROW_UNLESS(foo) and rely on implicit pointer-to-bool conversion."
                "\nThe deprecated code will be removed from Drake on or after 2021-12-01.")]] constexpr void
DrakeThrowUnlessWasUsedWithRawPointer<true>() {}

}  // namespace internal
}  // namespace maliput::drake

/// Evaluates @p condition and iff the value is false will throw an exception
/// with a message showing at least the condition text, function name, file,
/// and line.
///
/// The condition must not be a pointer, where we'd implicitly rely on its
/// nullness. Instead, always write out "!= nullptr" to be precise.
///
/// Correct: `MALIPUT_DRAKE_THROW_UNLESS(foo != nullptr);`
/// Incorrect: `MALIPUT_DRAKE_THROW_UNLESS(foo);`
///
/// Because this macro is intended to provide a useful exception message to
/// users, we should err on the side of extra detail about the failure. The
/// meaning of "foo" isolated within error message text does not make it
/// clear that a null pointer is the proximate cause of the problem.
#define MALIPUT_DRAKE_THROW_UNLESS(condition)                                                                    \
  do {                                                                                                           \
    typedef ::maliput::drake::assert::ConditionTraits<typename std::remove_cv_t<decltype(condition)>> Trait;     \
    static_assert(Trait::is_valid, "Condition should be bool-convertible.");                                     \
    ::maliput::drake::internal::DrakeThrowUnlessWasUsedWithRawPointer<std::is_pointer_v<decltype(condition)>>(); \
    if (!Trait::Evaluate(condition)) {                                                                           \
      ::maliput::drake::internal::Throw(#condition, __func__, __FILE__, __LINE__);                               \
    }                                                                                                            \
  } while (0)
