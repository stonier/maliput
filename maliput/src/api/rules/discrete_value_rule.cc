#include "maliput/api/rules/discrete_value_rule.h"

#include <algorithm>

namespace maliput {
namespace api {
namespace rules {

DiscreteValueRule::DiscreteValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                                     const std::vector<DiscreteValue>& values)
    : Rule(id, type_id, zone), values_(values) {
  MALIPUT_THROW_UNLESS(!values_.empty());
  for (const DiscreteValue& value : values_) {
    ValidateRelatedRules(value.related_rules);
    ValidateRelatedUniqueIds(value.related_unique_ids);
    ValidateSeverity(value.severity);
    MALIPUT_THROW_UNLESS(std::count(values_.begin(), values_.end(), value) == 1);
  }
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
