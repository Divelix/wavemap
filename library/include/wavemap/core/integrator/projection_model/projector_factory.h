#ifndef WAVEMAP_CORE_INTEGRATOR_PROJECTION_MODEL_PROJECTOR_FACTORY_H_
#define WAVEMAP_CORE_INTEGRATOR_PROJECTION_MODEL_PROJECTOR_FACTORY_H_

#include "wavemap/core/integrator/projection_model/projector_base.h"

namespace wavemap {
class ProjectorFactory {
 public:
  static ProjectorBase::Ptr create(
      const param::Value& params,
      std::optional<ProjectorType> default_projector_type = std::nullopt);

  static ProjectorBase::Ptr create(ProjectorType projector_type,
                                   const param::Value& params);
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_INTEGRATOR_PROJECTION_MODEL_PROJECTOR_FACTORY_H_
