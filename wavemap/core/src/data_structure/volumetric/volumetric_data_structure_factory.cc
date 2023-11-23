#include "wavemap/data_structure/volumetric/volumetric_data_structure_factory.h"

#include "wavemap/data_structure/volumetric/hashed_blocks.h"
#include "wavemap/data_structure/volumetric/hashed_chunked_wavelet_octree.h"
#include "wavemap/data_structure/volumetric/hashed_wavelet_octree.h"
#include "wavemap/data_structure/volumetric/volumetric_octree.h"
#include "wavemap/data_structure/volumetric/wavelet_octree.h"

namespace wavemap {
VolumetricDataStructureBase::Ptr VolumetricDataStructureFactory::create(
    const param::Value& params,
    std::optional<VolumetricDataStructureType> default_data_structure_type) {
  if (const auto type = VolumetricDataStructureType::from(params); type) {
    return create(type.value(), params);
  }

  if (default_data_structure_type.has_value()) {
    LOG(WARNING) << "Default type \""
                 << default_data_structure_type.value().toStr()
                 << "\" will be created instead.";
    return create(default_data_structure_type.value(), params);
  }

  LOG(ERROR) << "No default was set. Returning nullptr.";
  return nullptr;
}

VolumetricDataStructureBase::Ptr VolumetricDataStructureFactory::create(
    VolumetricDataStructureType data_structure_type,
    const param::Value& params) {
  switch (data_structure_type.toTypeId()) {
    case VolumetricDataStructureType::kHashedBlocks: {
      if (const auto config = VolumetricDataStructureConfig::from(params);
          config) {
        return std::make_shared<HashedBlocks>(config.value());
      } else {
        LOG(ERROR) << "Hashed blocks volumetric data structure config could "
                      "not be loaded.";
        return nullptr;
      }
    }
    case VolumetricDataStructureType::kOctree: {
      if (const auto config = VolumetricOctreeConfig::from(params); config) {
        return std::make_shared<VolumetricOctree>(config.value());
      } else {
        LOG(ERROR)
            << "Octree volumetric data structure config could not be loaded.";
        return nullptr;
      }
    }
    case VolumetricDataStructureType::kWaveletOctree: {
      if (const auto config = WaveletOctreeConfig::from(params); config) {
        return std::make_shared<WaveletOctree>(config.value());
      } else {
        LOG(ERROR) << "Wavelet octree volumetric data structure config could "
                      "not be loaded.";
        return nullptr;
      }
    }
    case VolumetricDataStructureType::kHashedWaveletOctree: {
      if (const auto config = HashedWaveletOctreeConfig::from(params); config) {
        return std::make_shared<HashedWaveletOctree>(config.value());
      } else {
        LOG(ERROR) << "Hashed wavelet octree volumetric data structure config "
                      "could not be loaded.";
        return nullptr;
      }
    }
    case VolumetricDataStructureType::kHashedChunkedWaveletOctree: {
      if (const auto config = HashedChunkedWaveletOctreeConfig::from(params);
          config) {
        return std::make_shared<HashedChunkedWaveletOctree>(config.value());
      } else {
        LOG(ERROR) << "Hashed chunked wavelet octree volumetric data structure "
                      "config could not be loaded.";
        return nullptr;
      }
    }
    default:
      LOG(ERROR) << "Attempted to create data structure with unknown type ID: "
                 << data_structure_type.toTypeId() << ". Returning nullptr.";
      return nullptr;
  }
}
}  // namespace wavemap
