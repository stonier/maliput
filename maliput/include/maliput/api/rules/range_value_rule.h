#pragma once

#include <string>
#include <vector>

#include "maliput/api/regions.h"
#include "maliput/api/rules/rule.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace rules {

/// Describes a numeric range based rule.
///
/// Ranges are closed and continuous, defined by a minimum and maximum quantity.
/// When only one extreme is formally defined, the other should take a
/// semantically correct value. For example, if a speed limit only specifies a
/// maximum value, the minimum value is typically zero.
class RangeValueRule : public Rule {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RangeValueRule);

  /// Defines a range for a RangeValueRule.
  struct Range : public Rule::State {
    MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Range);

    /// Default constructor.
    Range() = default;

    /// Creates a Rule::State.
    /// @param severity A non-negative quantity that specifies the
    ///                 level of enforcement. The smaller it is, the
    ///                 more strictly the rule is enforced.
    /// @param related_rules Contains groups of related rules.
    /// @param related_unique_ids Contains groups of related unique ids.
    /// @param description Semantics of the range quantity.
    /// @param min Minimum value of the range.
    /// @param max Maximum value of the range.
    Range(int severity, RelatedRules related_rules, RelatedUniqueIds related_unique_ids, std::string description,
          double min, double max)
        : Rule::State(severity, related_rules, related_unique_ids), description(description), min(min), max(max) {}

    bool operator==(const Range& other) const {
      return min == other.min && max == other.max && description == other.description && Rule::State::operator==(other);
    }
    bool operator!=(const Range& other) const { return !(*this == other); }

    /// Returns true when this range is _less_ than `other`.
    ///
    /// It compares severity, description, min and max in the described order.
    ///
    /// Used for strict ordering in collections such as `std::set`.
    bool operator<(const Range& other) const;

    std::string description;  ///< Semantics of the range quantity.
    double min{};             ///< Minimum value of the range.
    double max{};             ///< Maximum value of the range.
  };

  /// Constructs a range based Rule.
  ///
  /// @param id The Rule ID.
  /// @param type_id The Rule Type ID.
  /// @param zone LaneSRoute to which this rule applies.
  /// @param ranges A vector of possible ranges that this rule could enforce.
  ///               The actual range that's enforced at any given time is
  ///               determined by a RangeValueRuleStateProvider. This vector
  ///               must have at least one Range, and each Range must respect
  ///               that its min <= max and be unique.
  /// @throws maliput::common::assertion_error When `ranges` is empty.
  /// @throws maliput::common::assertion_error When any Range within `ranges`
  ///         violates `min <= max` condition.
  /// @throws maliput::common::assertion_error When there are duplicated Range
  ///         in `ranges`.
  RangeValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                 const std::vector<Range>& ranges);

  const std::vector<Range>& states() const { return states_; }

 private:
  std::vector<Range> states_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
