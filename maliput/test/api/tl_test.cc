#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"

#include "maliput/api/type_specific_identifier.h"

namespace experiments {
namespace test {
namespace {

struct UniqueId;

class Bulb {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Bulb)

  using Id = maliput::api::TypeSpecificIdentifier<Bulb>;

  Bulb(const Id& id, const UniqueId& unique_id);

  const Id& id() const { return id_; }

  const UniqueId& unique_id() const { return *unique_id_; }

  bool operator==(const Bulb& other) const { return id_ == other.id_ && unique_id_ == other.unique_id_; }

 private:
  Id id_;
  std::unique_ptr<UniqueId> unique_id_;
};

class BulbGroup {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BulbGroup)

  using Id = maliput::api::TypeSpecificIdentifier<BulbGroup>;

  BulbGroup(const Id& id, std::unique_ptr<Bulb> bulb) : id_(id), bulb_(std::move(bulb)) {}

  const Id& id() const { return id_; }

  const Bulb* bulb() const { return bulb_.get(); }

  const Bulb& bulb_ref() const { return *bulb_; }

 private:
  Id id_;
  std::unique_ptr<Bulb> bulb_;
};

struct UniqueId {
  Bulb::Id bulb_id;
  BulbGroup::Id bulb_group_id;

  std::string to_string() const {
    return bulb_id.string() + "-" + bulb_group_id.string();
  }

  bool operator==(const UniqueId& other) const {
    return bulb_id == other.bulb_id && bulb_group_id == other.bulb_group_id;
  }
};

Bulb::Bulb(const Id& id, const UniqueId& unique_id) : id_(id) {
  unique_id_.reset(new UniqueId(unique_id));
}

GTEST_TEST(TypeInteraction, CreationAndUsage) {
  const Bulb::Id kBulbId{"Bulb"};
  const BulbGroup::Id kBulbGroupId{"BulbGroup"};
  const UniqueId kUniqueId{kBulbId, kBulbGroupId};

  auto bulb = std::make_unique<Bulb>(kBulbId, kUniqueId);
  const Bulb* bulb_ptr = bulb.get();

  auto bulb_group = std::make_unique<BulbGroup>(kBulbGroupId, std::move(bulb));

  EXPECT_EQ(bulb_ptr->id(), kBulbId);
  EXPECT_EQ(bulb_ptr->unique_id(), kUniqueId);
  EXPECT_EQ(bulb_group->id(), kBulbGroupId);
  EXPECT_EQ(bulb_group->bulb(), bulb_ptr);
  EXPECT_EQ(bulb_group->bulb_ref(), *bulb_ptr);
}

}  // namespace
}  // namespace test
}  // namespace experiments
