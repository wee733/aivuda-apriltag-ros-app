// SPDX-License-Identifier: Apache-2.0
#include <charconv>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>
#include <utility>

#include "isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace aivuda
{
class TagScaleFilterNode final : public rclcpp::Node
{
  using Detections = isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;
  using Transforms = tf2_msgs::msg::TFMessage;

public:
  explicit TagScaleFilterNode(const rclcpp::NodeOptions & options)
  : Node("tag_scale_filter", options)
  {
    const double base = declare_parameter("base_size", 0.289);
    const double fallback = declare_parameter("default_size_m", 0.289);
    family_ = declare_parameter<std::string>("tag_family", "tag36h11");
    prefix_ = family_ + ":";
    const auto mode = declare_parameter<std::string>("filter_mode", "configured_only");
    const auto ids = declare_parameter("tag_ids", std::vector<int64_t>{});
    const auto sizes = declare_parameter("tag_sizes", std::vector<double>{});
    if (!valid_size(base) || !valid_size(fallback) || ids.size() != sizes.size() ||
      (mode != "all_detected" && (mode != "configured_only" || ids.empty())))
    {
      throw std::invalid_argument("Invalid tag sizes, ID/size list lengths, or filter mode");
    }
    if (mode == "all_detected") {
      default_scale_ = fallback / base;
    }
    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] < 0 || ids[i] > 65535 || !valid_size(sizes[i]) ||
        !scales_.emplace(ids[i], sizes[i] / base).second)
      {
        throw std::invalid_argument("Invalid or duplicate tag rule");
      }
    }
    detections_ = create_publisher<Detections>("/tag_detections", 1);
    tf_ = create_publisher<Transforms>("/tf", 100);
    detection_input_ = create_subscription<Detections>("/apriltag/internal/detections", 10,
      [this](Detections::UniquePtr msg) {
        auto & tags = msg->detections;
        retain_scaled(tags, [this](auto & tag) {
          return tag.family == family_ && rescale(tag.id, tag.pose.pose.pose.position);
        });
        detections_->publish(std::move(msg));
      });
    tf_input_ = create_subscription<Transforms>("/apriltag/internal/tf", 100,
      [this](Transforms::UniquePtr msg) {
        auto & transforms = msg->transforms;
        retain_scaled(transforms, [this](auto & tf) {
          const std::string_view frame(tf.child_frame_id);
          if (frame.substr(0, prefix_.size()) != prefix_) {
            return false;
          }
          const auto digits = frame.substr(prefix_.size());
          unsigned id = 0;
          const char * end = digits.data() + digits.size();
          const auto parsed = std::from_chars(digits.data(), end, id);
          return parsed.ec == std::errc{} && parsed.ptr == end && id <= 65535 &&
                 rescale(id, tf.transform.translation);
        });
        if (!transforms.empty()) {
          tf_->publish(std::move(msg));
        }
      });
  }

private:
  template<typename Items, typename Scale>
  static void retain_scaled(Items & items, Scale scale)
  {
    auto out = items.begin();
    for (auto it = items.begin(); it != items.end(); ++it) {
      if (scale(*it)) {
        if (out != it) {
          *out = std::move(*it);
        }
        ++out;
      }
    }
    items.erase(out, items.end());
  }

  static bool valid_size(double size) { return size > 0.0 && size <= 10.0; }

  template<typename Vector>
  bool rescale(int64_t id, Vector & p) const
  {
    if (id < 0 || id > 65535) {
      return false;
    }
    const auto found = scales_.find(id);
    if (found == scales_.end() && !default_scale_) {
      return false;
    }
    const double scale = found == scales_.end() ? *default_scale_ : found->second;
    p.x *= scale;
    p.y *= scale;
    p.z *= scale;
    return true;
  }

  std::string family_, prefix_;
  std::unordered_map<int64_t, double> scales_;
  std::optional<double> default_scale_;
  rclcpp::Publisher<Detections>::SharedPtr detections_;
  rclcpp::Publisher<Transforms>::SharedPtr tf_;
  rclcpp::Subscription<Detections>::SharedPtr detection_input_;
  rclcpp::Subscription<Transforms>::SharedPtr tf_input_;
};
}  // namespace aivuda

RCLCPP_COMPONENTS_REGISTER_NODE(aivuda::TagScaleFilterNode)
