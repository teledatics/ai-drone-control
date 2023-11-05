#include "face_tracker.hpp"
#include "gtest/gtest.h"
#include <gtest/gtest_prod.h>
#include <iostream>

class FaceTrackerTest : public ::testing::Test {
  public:
    FaceTrackerTest() = default;

  protected:
    virtual void SetUp() override {
    }
    virtual void TearDown() {
    }
    std::unique_ptr<FaceTrackerPF> ft_;
};

TEST_F(FaceTrackerTest, HistogramCountTest) {
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
