#ifndef WAVEMAP_ROS_INPUTS_POINTCLOUD_INPUT_H_
#define WAVEMAP_ROS_INPUTS_POINTCLOUD_INPUT_H_

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <wavemap/core/utils/time/stopwatch.h>

#include "wavemap_ros/inputs/input_base.h"
#include "wavemap_ros/utils/pointcloud_undistorter.h"

#ifdef LIVOX_AVAILABLE
#include <livox_ros_driver2/CustomMsg.h>
#endif

namespace wavemap {
struct PointcloudTopicType : public TypeSelector<PointcloudTopicType> {
  using TypeSelector<PointcloudTopicType>::TypeSelector;

  enum Id : TypeId { kPointCloud2, kOuster, kLivox };

  static constexpr std::array names = {"PointCloud2", "ouster", "livox"};
};

/**
 * Config struct for the pointcloud input handler.
 */
struct PointcloudInputConfig
    : public ConfigBase<PointcloudInputConfig, 11, PointcloudTopicType> {
  //! Name of the ROS topic to subscribe to.
  std::string topic_name = "scan";
  //! Message type of the ROS topic to subscribe to.
  PointcloudTopicType topic_type = PointcloudTopicType::kPointCloud2;
  //! Queue length to use when subscribing to the ROS topic.
  int topic_queue_length = 10;

  //! Time period used to control the rate at which to retry getting the sensor
  //! pose when ROS TF lookups fail.
  Seconds<FloatingPoint> processing_retry_period = 0.05f;
  //! Maximum amount of time to wait for the sensor pose to become available
  // when ROS TF lookups fail.
  Seconds<FloatingPoint> max_wait_for_pose = 1.f;

  //! The frame_id to use to look up the sensor pose using ROS TFs.
  //! Note that setting this is optional, when left blank the header.frame_id of
  //! the measurement's msg is used.
  std::string sensor_frame_id;
  //! Time offset to apply to the header.stamp of the measurement's msg when
  //! looking up its pose using ROS TFs. Can be used when the time offset is
  //! known (e.g. through calibration) but not corrected by the sensor's driver.
  Seconds<FloatingPoint> time_offset = 0.f;
  //! Whether to undistort each pointcloud based on the sensor's motion while it
  //! was captured. We strongly recommend turning this on, unless the robot's
  //! odometry is very jerky or the sensor's driver already performs motion
  //! undistortion.
  bool undistort_motion = false;
  //! Number of intermediate poses to sample from ROS TFs when performing motion
  //! undistortion.
  int num_undistortion_interpolation_intervals_per_cloud = 100;

  //! Name of the topic on which to publish the range image generated from the
  //! pointclouds. Useful for debugging, to see how well the projection model
  //! matches the LiDAR. Disabled if not set.
  std::string projected_range_image_topic_name;
  //! Name of the topic on which to republish the motion-undistorted
  //! pointclouds. Useful to share the undistorted pointclouds with other ROS
  //! nodes and for debugging. Disabled if not set.
  std::string undistorted_pointcloud_topic_name;

  static MemberMap memberMap;

  // Conversion to InputHandler base config
  operator InputBaseConfig() const {  // NOLINT
    return {topic_name, topic_queue_length, processing_retry_period};
  }

  bool isValid(bool verbose) const override;
};

class PointcloudInput : public InputBase {
 public:
  PointcloudInput(const PointcloudInputConfig& config,
                  std::shared_ptr<Pipeline> pipeline,
                  std::vector<std::string> integrator_names,
                  std::shared_ptr<TfTransformer> transformer,
                  std::string world_frame, ros::NodeHandle nh,
                  ros::NodeHandle nh_private);

  InputType getType() const override { return InputType::kPointcloud; }
  PointcloudTopicType getTopicType() const { return config_.topic_type; }

  void callback(const sensor_msgs::PointCloud2& pointcloud_msg);
#ifdef LIVOX_AVAILABLE
  void callback(const livox_ros_driver2::CustomMsg& pointcloud_msg);
#endif

  template <typename RegistrarT>
  static bool registerCallback(PointcloudTopicType type, RegistrarT registrar);

 private:
  const PointcloudInputConfig config_;

  Stopwatch integration_timer_;

  PointcloudUndistorter pointcloud_undistorter_;

  ros::Subscriber pointcloud_sub_;
  std::queue<undistortion::StampedPointcloud> pointcloud_queue_;
  void processQueue() override;

  static bool hasField(const sensor_msgs::PointCloud2& msg,
                       const std::string& field_name);

  void publishProjectedRangeImageIfEnabled(
      const ros::Time& stamp, const PosedPointcloud<>& posed_pointcloud);
  image_transport::Publisher projected_range_image_pub_;

  void publishUndistortedPointcloudIfEnabled(
      const ros::Time& stamp, const PosedPointcloud<>& undistorted_pointcloud);
  ros::Publisher undistorted_pointcloud_pub_;
};
}  // namespace wavemap

#include "wavemap_ros/inputs/impl/pointcloud_input_impl.h"

#endif  // WAVEMAP_ROS_INPUTS_POINTCLOUD_INPUT_H_
