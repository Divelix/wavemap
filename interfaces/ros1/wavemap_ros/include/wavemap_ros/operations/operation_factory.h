#ifndef WAVEMAP_ROS_OPERATIONS_OPERATION_FACTORY_H_
#define WAVEMAP_ROS_OPERATIONS_OPERATION_FACTORY_H_

#include <memory>
#include <string>

#include <ros/ros.h>
#include <wavemap/map/map_base.h>
#include <wavemap/utils/thread_pool.h>

#include "wavemap_ros/operations/operation_base.h"
#include "wavemap_ros/utils/tf_transformer.h"

namespace wavemap {
class OperationFactory {
 public:
  static std::unique_ptr<OperationBase> create(
      const param::Value& params, std::string world_frame,
      MapBase::Ptr occupancy_map, std::shared_ptr<TfTransformer> transformer,
      std::shared_ptr<ThreadPool> thread_pool, ros::NodeHandle nh_private,
      std::optional<OperationType> default_operation_type = std::nullopt);

  static std::unique_ptr<OperationBase> create(
      OperationType operation_type, const param::Value& params,
      std::string world_frame, MapBase::Ptr occupancy_map,
      std::shared_ptr<TfTransformer> transformer,
      std::shared_ptr<ThreadPool> thread_pool, ros::NodeHandle nh_private);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_OPERATIONS_OPERATION_FACTORY_H_
