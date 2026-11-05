// Copyright 2026
#include <gtest/gtest.h>
#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

namespace small_gicp_relocalization
{
class SmallGicpMappingCommitTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("mapping_mode", true)});
    node_ = std::make_shared<SmallGicpRelocalizationNode>(options);
    node_->odom_instance_id_ = 101;
    node_->odom_epoch_ = 1;
    node_->localization_epoch_ = 1;
    node_->growing_map_->seed({{0.0, 0.0, 0.0}}, Eigen::Isometry3d::Identity());
    node_->growing_pending_sequence_ = 7;
    node_->growing_pending_scan_ns_ = node_->now().nanoseconds() - 100000000;
    node_->growing_pending_transform_.translation().x() = 0.04;
    for (int y = 0; y < 10; ++y) {
      for (int z = 0; z < 10; ++z) {
        node_->growing_pending_source_.push_back({1.0, y * 0.1, z * 0.1});
      }
    }
  }
  void acceptStatus(bool valid = true, const std::string & child = "odom")
  {
    auto status = std::make_shared<ats_navigation_interfaces::msg::LocalizationStatus>();
    status->header.frame_id = "map";
    status->header.stamp = node_->now();
    status->state = status->STATE_TRACKING;
    status->epoch = 1;
    status->odom_instance_id = 101;
    status->odom_epoch = 1;
    status->observation_sequence = 7;
    status->last_observation_stamp = rclcpp::Time(node_->growing_pending_scan_ns_);
    status->map_to_odom_valid = valid;
    status->map_to_odom.header = status->header;
    status->map_to_odom.child_frame_id = child;
    status->map_to_odom.transform.rotation.w = 1.0;
    node_->localizationStatusCallback(status);
  }
  double committedX() const
  {return node_->growing_fused_transform_.translation().x();}
  double maximumPointX() const
  {
    double value = 0.0;
    for (const auto & point : node_->growing_map_->points()) {value = std::max(value, point[0]);}
    return value;
  }
  std::shared_ptr<SmallGicpRelocalizationNode> node_;
};

// A 4 cm candidate is below fusion's 5 cm update threshold. The observation
// is accepted but its transform is not applied: the committed TF stays identity.
TEST_F(SmallGicpMappingCommitTest, AcceptedSequenceDoesNotMeanCandidateTransformWasApplied)
{
  acceptStatus();
  EXPECT_NEAR(committedX(), 0.0, 1e-12);
  EXPECT_NEAR(maximumPointX(), 1.0, 1e-12);
}

TEST_F(SmallGicpMappingCommitTest, InvalidOrWrongFrameCommitCannotEnterTarget)
{
  acceptStatus(false);
  EXPECT_NEAR(maximumPointX(), 0.0, 1e-12);
  acceptStatus(true, "wrong_odom");
  EXPECT_NEAR(maximumPointX(), 0.0, 1e-12);
}
}  // namespace small_gicp_relocalization
