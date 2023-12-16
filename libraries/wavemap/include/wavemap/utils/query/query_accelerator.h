#ifndef WAVEMAP_UTILS_QUERY_QUERY_ACCELERATOR_H_
#define WAVEMAP_UTILS_QUERY_QUERY_ACCELERATOR_H_

#include <limits>

#include "wavemap/data_structure/ndtree_block_hash.h"
#include "wavemap/data_structure/spatial_hash.h"
#include "wavemap/indexing/index_hashes.h"
#include "wavemap/map/hashed_wavelet_octree.h"

namespace wavemap {
// Base template
template <typename DataStructureT>
class QueryAccelerator {};

// Template deduction guide
template <typename T>
QueryAccelerator(T type) -> QueryAccelerator<T>;

// Query accelerator for vanilla spatial hashes
template <typename BlockDataT, int dim>
class QueryAccelerator<SpatialHash<BlockDataT, dim>> {
 public:
  static constexpr int kDim = dim;

  explicit QueryAccelerator(SpatialHash<BlockDataT, dim>& spatial_hash)
      : spatial_hash_(spatial_hash) {}

  BlockDataT* getBlock(const Index<dim>& block_index);
  template <typename... DefaultArgs>
  BlockDataT& getOrAllocateBlock(const Index<dim>& block_index,
                                 DefaultArgs&&... args);

 private:
  SpatialHash<BlockDataT, dim>& spatial_hash_;

  Index<dim> last_block_index_ =
      Index3D::Constant(std::numeric_limits<IndexElement>::max());
  BlockDataT* last_block_ = nullptr;
};

// Query accelerator for ndtree block hashes
template <typename CellDataT, int dim>
class QueryAccelerator<NdtreeBlockHash<CellDataT, dim>> {
 public:
  static constexpr int kDim = dim;
  using BlockType = typename NdtreeBlockHash<CellDataT, dim>::Block;

  explicit QueryAccelerator(NdtreeBlockHash<CellDataT, dim>& ndtree_block_hash)
      : ndtree_block_hash_(ndtree_block_hash) {}

  BlockType* getBlock(const Index<dim>& block_index);
  template <typename... DefaultArgs>
  BlockType& getOrAllocateBlock(const Index<dim>& block_index,
                                DefaultArgs&&... args);

  // TODO(victorr): Implement accelerated cell accessors

 private:
  NdtreeBlockHash<CellDataT, dim>& ndtree_block_hash_;

  Index<dim> last_block_index_ =
      Index3D::Constant(std::numeric_limits<IndexElement>::max());
  BlockType* last_block_ = nullptr;
};

// Query accelerator for hashed wavelet octrees
template <>
class QueryAccelerator<HashedWaveletOctree> {
 public:
  static constexpr int kDim = HashedWaveletOctree::kDim;

  explicit QueryAccelerator(const HashedWaveletOctree& map)
      : block_map_(map.getHashMap()), tree_height_(map.getTreeHeight()) {}

  FloatingPoint getCellValue(const Index3D& index) {
    return getCellValue(OctreeIndex{0, index});
  }

  FloatingPoint getCellValue(const OctreeIndex& index);

 private:
  using Coefficients = HaarCoefficients<FloatingPoint, kDim>;
  using Transform = HaarTransform<FloatingPoint, kDim>;
  using BlockIndex = typename HashedWaveletOctree::BlockIndex;
  using NodeType = NdtreeNode<typename Coefficients::Details, kDim>;

  const HashedWaveletOctree::BlockHashMap& block_map_;
  const IndexElement tree_height_;

  std::array<const NodeType*, morton::kMaxTreeHeight<3>> node_stack_{};
  std::array<FloatingPoint, morton::kMaxTreeHeight<3>> value_stack_{};

  BlockIndex block_index_ =
      BlockIndex ::Constant(std::numeric_limits<IndexElement>::max());
  MortonIndex morton_code_ = std::numeric_limits<MortonIndex>::max();
  IndexElement height_ = tree_height_;

  BlockIndex computeBlockIndexFromIndex(const OctreeIndex& node_index) const {
    const BlockIndex index = convert::nodeIndexToMinCornerIndex(node_index);
    return int_math::div_exp2_floor(index, tree_height_);
  }

  friend class QueryAcceleratorTest_Equivalence_Test;
};
}  // namespace wavemap

#include "wavemap/utils/query/impl/query_accelerator_inl.h"

#endif  // WAVEMAP_UTILS_QUERY_QUERY_ACCELERATOR_H_
