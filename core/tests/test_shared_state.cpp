#include <memory>

#include "metaport/shared_state.hpp"
#include "test.hpp"

using namespace metaport;

TEST(shared_block_roundtrip) {
    auto block = std::make_unique<SharedBlock>();
    initSharedBlock(block.get());
    TrackingFrame f;
    f.head.position = {1, 2, 3};
    f.controllers[kRight].trigger = 0.75f;
    publishFrame(block.get(), f);
    TrackingFrame out;
    EXPECT_TRUE(readFrame(block.get(), &out));
    EXPECT_NEAR(out.head.position.y, 2, 0);
    EXPECT_NEAR(out.controllers[kRight].trigger, 0.75, 0);
    EXPECT_TRUE(block->seq.load() % 2 == 0);
}

TEST(shared_block_rejects_bad_magic) {
    auto block = std::make_unique<SharedBlock>();
    initSharedBlock(block.get());
    block->magic = 0;
    TrackingFrame out;
    EXPECT_TRUE(!readFrame(block.get(), &out));
}

TEST(predict_pose_is_capped) {
    PoseSample s;
    s.linearVelocity = {1, 0, 0};
    s.timeNs = 0;
    Pose p = predictPose(s, 1'000'000'000, 0.05f);  // 1 s ahead, capped at 50 ms
    EXPECT_NEAR(p.position.x, 0.05, 1e-6);
}
